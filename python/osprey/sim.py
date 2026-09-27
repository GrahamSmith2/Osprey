"""Fixed-step RK4 simulation with a discrete, fixed-rate controller.

Ported from model/simOsprey.m.

Why fixed-step RK4 and not an adaptive solver: the controller is a discrete
device with a zero-order hold and a compute delay. An adaptive solver steps over
controller ticks and smooths that away, and digital phase lag is first-order in
achievable bandwidth.

Three things live in the stepper rather than the derivative, because none of
them is differentiable:
  1. the ventilation latch — held fixed across all four RK4 stages;
  2. cable backlash — hysteresis between servo output and blade angle;
  3. the controller zero-order hold and command-delay queue.

Controllers are either a constant command (open loop) or any object with a
``step(t, x) -> (delta_cmd, T_cmd_port, T_cmd_stbd)`` method (closed loop).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .dynamics import eom


@dataclass
class Fault:
    """Plant fault. The controller is NOT told — it sees only its sensors.
    kind: 'rudder_jam' (value = blade angle, rad) | 'motor_out' (value 1 port,
    2 stbd) | 'ventilation' (forced). Sensor faults such as GPS dropout are
    injected in the sensor model instead."""
    kind: str
    t0: float
    value: float = 0.0


@dataclass
class SimResult:
    t: np.ndarray
    x: np.ndarray
    delta_cmd: np.ndarray
    delta_blade: np.ndarray
    vent: np.ndarray
    beta: np.ndarray
    N_munk: np.ndarray
    N_rud: np.ndarray
    N_hull: np.ndarray
    tau_servo: np.ndarray
    sigma: np.ndarray
    T_max: np.ndarray
    dt: float
    f_ctrl: float
    extra: dict = field(default_factory=dict)

    u = property(lambda s: s.x[:, 0])
    v = property(lambda s: s.x[:, 1])
    r = property(lambda s: s.x[:, 2])
    X = property(lambda s: s.x[:, 3])
    Y = property(lambda s: s.x[:, 4])
    psi = property(lambda s: s.x[:, 5])
    T_p = property(lambda s: s.x[:, 6])
    T_s = property(lambda s: s.x[:, 7])
    delta = property(lambda s: s.x[:, 8])

    def turn_radius(self):
        """Steady turn radius from the final sample [m]."""
        return math.hypot(self.u[-1], self.v[-1]) / max(abs(self.r[-1]), 1e-9)


def simulate(x0, tf, controller, P, *, dt=0.005, f_ctrl=50.0, delay_n=1,
             fault: Fault | None = None, legacy_zero_delay=False,
             stop_when=None) -> SimResult:
    """Integrate for tf seconds.

    delay_n: compute delay in controller ticks. A command computed at tick k is
    applied from tick k + delay_n, as on a real autopilot.

    legacy_zero_delay reproduces the MATLAB queue, which for delay_n = 1 applied
    commands with NO delay despite documenting a one-sample delay (PORTING.md).

    stop_when(t, x) -> bool ends the run early (e.g. course finished)."""
    n_sub = max(round((1 / f_ctrl) / dt), 1)
    dt = (1 / f_ctrl) / n_sub
    n_tick = math.ceil(tf * f_ctrl)
    x = [float(v) for v in x0]

    closed = hasattr(controller, "step")
    if closed:
        init = (x[8], x[6], x[7])            # hold the initial actuator state
    elif isinstance(controller, dict):
        init = (controller["delta_cmd"], controller["T_cmd_port"], controller["T_cmd_stbd"])
    else:
        init = tuple(controller)
    qlen = max(delay_n, 1) if legacy_zero_delay else delay_n + 1
    queue = [init] * qlen

    vent = 0
    blade = x[8]
    b = P.A.backlash
    L = {k: [] for k in ("t", "x", "dc", "db", "vent", "beta", "Nm", "Nr", "Nh",
                          "tau", "sig", "Tm")}

    def log(t, x, cmd, D):
        L["t"].append(t); L["x"].append(list(x)); L["dc"].append(cmd[0])
        L["db"].append(blade); L["vent"].append(vent); L["beta"].append(D["beta"])
        L["Nm"].append(D["N_munk"]); L["Nr"].append(D["N_rud"]); L["Nh"].append(D["N_hull"])
        L["tau"].append(D["tau_servo"]); L["sig"].append(D["sigma"]); L["Tm"].append(D["T_max"])

    for it in range(n_tick):
        t_tick = it * n_sub * dt
        if closed:
            queue.pop(0)
            queue.append(tuple(controller.step(t_tick, list(x))))
        cmd = queue[0]

        for j in range(n_sub):
            t = t_tick + j * dt
            # backlash: blade moves only once the servo has taken up the free play
            if b > 0:
                if x[8] - blade > b / 2:
                    blade = x[8] - b / 2
                elif x[8] - blade < -b / 2:
                    blade = x[8] + b / 2
            else:
                blade = x[8]

            cmd_f = cmd
            if fault is not None and t >= fault.t0:
                if fault.kind == "rudder_jam":
                    blade = fault.value
                elif fault.kind == "motor_out":
                    cmd_f = (cmd[0], 0.0, cmd[2]) if fault.value == 1 else (cmd[0], cmd[1], 0.0)
                elif fault.kind == "ventilation":
                    vent = 2

            f1, D = eom(t, x, cmd_f, vent, P, blade, diag=True)
            log(t, x, cmd, D)
            h = dt / 2
            f2 = eom(t + h, [a + h * k for a, k in zip(x, f1)], cmd_f, vent, P, blade)
            f3 = eom(t + h, [a + h * k for a, k in zip(x, f2)], cmd_f, vent, P, blade)
            f4 = eom(t + dt, [a + dt * k for a, k in zip(x, f3)], cmd_f, vent, P, blade)
            x = [a + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
                 for a, k1, k2, k3, k4 in zip(x, f1, f2, f3, f4)]

            vent = D["vent_state"]
            if fault is not None and fault.kind == "ventilation" and t >= fault.t0:
                vent = 2

        if stop_when is not None and stop_when(t_tick + n_sub * dt, x):
            break

    # final sample, with its own diagnostics so S.<field>[-1] is never a stale zero
    t_end = (it + 1) * n_sub * dt
    _, D = eom(t_end, x, cmd, vent, P, blade, diag=True)
    log(t_end, x, cmd, D)

    a = lambda k: np.asarray(L[k], dtype=float)
    return SimResult(t=a("t"), x=a("x"), delta_cmd=a("dc"), delta_blade=a("db"),
                     vent=a("vent"), beta=a("beta"), N_munk=a("Nm"), N_rud=a("Nr"),
                     N_hull=a("Nh"), tau_servo=a("tau"), sigma=a("sig"),
                     T_max=a("Tm"), dt=dt, f_ctrl=f_ctrl)


def trim_state(U, P):
    """Initial state in straight running at speed U with thrust trimmed to drag."""
    Tt = P.HT.lookup(U)[1] / 2
    return [U, 0, 0, 0, 0, 0, Tt, Tt, 0], Tt

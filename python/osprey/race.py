"""Fly a Lua controller around the PEP course and score it against the TRUE track.

Shared by ``python -m osprey run``, the Monte Carlo and the failure-mode study,
so every race number in the docs is measured the same way.

Scoring notes (each one fixed a real bug in the MATLAB version, see
../docs/HANDOFF.md §7):
  * progress advances by leg index, never nearest leg, or a closed circuit
    loses the finish;
  * only samples before the finish are scored, or a boat that has finished
    reads as off course;
  * "steady tracking" is the middle half of each straight, so the rounding
    overshoot (set by turn geometry) and the tracking error (set by the
    controller) are reported apart.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np

from .course import MILE, Progress, two_mark_oval
from .lua_bridge import LuaController
from .params import DEG, G, KGCM
from .sim import Fault, simulate

MPH = 0.44704
CONTROLLERS = Path(__file__).resolve().parent.parent / "controllers"
AUTOPILOT = CONTROLLERS / "oval_autopilot.lua"


@dataclass(frozen=True)
class Course:
    """Two marks, rounded at `radius`, run as an oval from a standing start."""
    separation: float = 0.25 * MILE
    radius: float = 25.0
    distance: float = 2 * MILE

    def waypoints(self):
        return two_mark_oval(separation=self.separation, radius=self.radius,
                             distance=self.distance)

    @property
    def marks(self):
        """(north, east) of the north and south marks (the rounding centres)."""
        return (self.separation, self.radius), (0.0, self.radius)


def controller_params(P, cruise, fit=None):
    """Constants a Lua controller may legitimately know about the vehicle,
    derived from the DESIGN parameter set P (the nominal boat), never from the
    plant being flown. cruise in m/s."""
    from .nomoto import identify_nomoto
    from .propulsion import prop_max_thrust
    fit = fit or identify_nomoto(P)
    R = P.R
    kn = 0.5 * P.E.rho_w * R.A_r * R.CL_alpha * abs(P.x_r)
    T_c = P.HT.lookup(cruise)[1] / 2
    return {
        "OSP_NOMOTO_K": fit.K_prime, "OSP_NOMOTO_T": fit.T_prime, "OSP_LOA": P.V.LOA,
        "OSP_RUD_KN": kn, "OSP_RUD_MAX_DEG": R.delta_max / DEG,
        "OSP_STEER_LIM_DEG": R.delta_vent_on / DEG,
        "OSP_THR_N": prop_max_thrust(0.0, P), "OSP_THR_U0": P.A.J0_prop * P.A.n_max * P.V.D_prop,
        "OSP_YP": P.V.y_p, "OSP_WC_RATE": 4.0, "OSP_RATE_MAX": 0.6, "OSP_GPS_LOSS_MS": 500,
        "CRUISE_SPEED": cruise,
        "CRUISE_THROTTLE": 100 * T_c / prop_max_thrust(cruise, P),
    }


def race_time_limit(course, cruise):
    """Generous limit: three times the time at cruise speed, plus a minute."""
    return 3.0 * course.distance / max(cruise, 1.0) + 60.0


def top_speed(P):
    """Loaded top speed [m/s]: where twice the thrust ceiling meets resistance."""
    from .propulsion import prop_max_thrust
    for i in range(1, 4500):
        u = 0.01 * i
        if 2 * prop_max_thrust(u, P) < P.HT.lookup(u)[1]:
            return u
    return 45.0


@dataclass
class RaceResult:
    finished: bool
    lost: bool
    t_finish: float | None
    metrics: dict
    S: object = field(repr=False)
    ctl: object = field(repr=False)
    wpts: list = field(repr=False)


def run_race(P, script=AUTOPILOT, *, params, course=Course(), fault: Fault | None = None,
             gps_dropout=None, seed=1, rate=50.0, dt=0.01, tmax=600.0, start_speed=0.0,
             budget=100_000, lua="lua54", echo=False, lost_xt=150.0) -> RaceResult:
    """Fly `script` on plant P. `params` are the controller's constants (from
    controller_params on the design boat). The run ends 2 s after the finish,
    5 s after a disarm, or as soon as the boat is more than `lost_xt` metres
    off the course line (scored as lost)."""
    wpts, lap = course.waypoints()
    ctl = LuaController(script, P, wpts, params=params, rate_hz=rate, seed=seed,
                        gps_dropout=gps_dropout, budget=budget, lua_version=lua, echo=echo)
    prog = Progress(wpts)
    st = {"fin": None, "lost": False}

    def stop_when(t, x):
        _, xt = prog.update(x[3], x[4])
        if prog.finished and st["fin"] is None:
            st["fin"] = t
        if st["fin"] is not None:
            return t > st["fin"] + 2
        if abs(xt) > lost_xt or not all(math.isfinite(v) for v in x):
            st["lost"] = True
            return True
        return (not ctl.armed) and ctl.disarm_t is not None and t > ctl.disarm_t + 5

    x0 = [start_speed, 0, 0, 0, 0, 0, 0, 0, 0]
    if start_speed > 0:
        x0[6] = x0[7] = P.HT.lookup(start_speed)[1] / 2
    S = simulate(x0, tmax, ctl, P, dt=dt, f_ctrl=rate, fault=fault, stop_when=stop_when)
    m = score(S, ctl, wpts, lap, course, P)
    # "lost" means left the course or diverged. Running out of time is not lost:
    # a slow boat is still on the course (an earlier version conflated the two,
    # which made low cruise speeds look worse than high ones).
    lost = st["lost"]
    m["lost"] = lost
    m["timed_out"] = (m["t_finish"] is None and not lost and ctl.disarm_t is None)
    return RaceResult(finished=m["t_finish"] is not None, lost=lost, t_finish=m["t_finish"],
                      metrics=m, S=S, ctl=ctl, wpts=wpts)


def score(S, ctl, wpts, lap, course, P):
    """Race metrics from the TRUE track."""
    sc = Progress(wpts)
    xt, prog, seg, fin = [], [], [], None
    for t, n, e in zip(S.t, S.X, S.Y):
        p, e_ = sc.update(n, e)
        if fin is None and sc.finished:
            fin = float(t)
        if fin is None:
            xt.append(e_); prog.append(p); seg.append(sc.j)
    k = len(xt)
    xt = np.abs(np.asarray(xt)) if xt else np.zeros(1)
    prog = np.asarray(prog) if prog else np.zeros(1)
    seg = np.asarray(seg, dtype=int) if seg else np.zeros(1, dtype=int)
    Lleg = np.asarray(sc.L)
    s_on = prog - np.asarray(sc.cum)[seg]
    straight = Lleg[seg] > 4 * course.radius
    mid = straight & (s_on >= 0.25 * Lleg[seg]) & (s_on <= 0.75 * Lleg[seg])
    xs = xt[mid] if mid.any() else np.array([np.inf])   # never reached a straight: lost

    # per-lap RMS over COMPLETE laps: does error accumulate from lap to lap?
    # (2 miles is 3.35 laps; the part lap is mostly straight and not comparable)
    lap_idx = np.floor(prog / lap).astype(int)
    n_full = int(sc.total // lap)
    lap_rms = [float(np.sqrt(np.mean(xt[lap_idx == i] ** 2))) for i in range(n_full)
               if (lap_idx == i).sum() > 50]

    # distance from each mark while rounding it (the boat must stay clear)
    N, E = S.X[:k], S.Y[:k]
    (nn, ne), (sn, se) = course.marks
    rn = N >= nn
    rs = N <= sn
    rs[: int(2 / S.dt)] = False                          # ignore the start line
    dn = np.hypot(N[rn] - nn, E[rn] - ne)
    ds = np.hypot(N[rs] - sn, E[rs] - se)
    d_all = np.concatenate([dn, ds]) if (rn.any() or rs.any()) else np.array([course.radius])

    lim = P.R.delta_vent_on / P.R.delta_max
    L = [r for r in ctl.log if r[10]]
    steer = np.asarray([r[1] for r in L]) if L else np.zeros(1)
    tick = 1.0 / ctl.rate_hz
    t_end = fin if fin is not None else float(S.t[-1])
    moving = S.t > 5.0

    return dict(
        t_finish=fin,
        avg_mph=(sc.total / fin / MPH) if fin else float("nan"),
        progress_frac=sc.best / sc.total,
        xt_max=float(xt.max()), xt_rms=float(np.sqrt(np.mean(xt ** 2))),
        xt_p95=float(np.percentile(xt, 95)),
        straight_max=float(xs.max()), straight_rms=float(np.sqrt(np.mean(xs ** 2))),
        lap_rms=lap_rms,
        mark_min=float(d_all.min()), mark_max=float(d_all.max()),
        rounding_wide=float(d_all.max() - course.radius),
        speed_mean_mph=float(S.u[S.t <= t_end].mean() / MPH),
        speed_max_mph=float(S.u.max() / MPH),
        speed_min_moving=float(S.u[moving].min()) if moving.any() else float("nan"),
        lat_g=float(np.abs(S.u * S.r).max() / G),
        yaw_rate_peak_dps=float(np.degrees(np.abs(S.r).max())),
        rudder_peak_deg=float(np.degrees(np.abs(S.delta_blade).max())),
        clamp_frac=float(np.mean(np.abs(steer) >= 0.98 * lim)),
        vent_events=int(np.sum(np.diff(S.vent) > 0.5)),
        servo_nominal_kgcm=float(S.tau_servo.max() / KGCM),
        servo_cop_offset_kgcm=servo_cop_offset(S, P) / KGCM,
        t_motor_steer=tick * sum(1 for r in L if abs(r[2] - r[3]) > 1e-9),
        t_rudder_steer=tick * sum(1 for r in L if abs(r[1]) > 1e-9),
        ticks_both=sum(1 for r in L if abs(r[2] - r[3]) > 1e-9 and abs(r[1]) > 1e-9),
        disarm_t=ctl.disarm_t, lua_error=ctl.error,
    )


def plot_race(res, P, path):
    """Track, speed, rudder and motor outputs for one race."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    S, ctl, wpts = res.S, res.ctl, res.wpts
    fig, ax = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    wn, we = zip(*wpts)
    ax[0, 0].plot(we, wn, "--", color="0.6", lw=1, label="course")
    ax[0, 0].plot(S.Y, S.X, lw=1.2, label="boat")
    ax[0, 0].set(xlabel="east [m]", ylabel="north [m]", title="track")
    ax[0, 0].axis("equal"); ax[0, 0].legend(loc="best")
    ax[0, 1].plot(S.t, S.u / MPH, lw=1)
    ax[0, 1].set(xlabel="time [s]", ylabel="surge speed [mph]", title="speed")
    ax[1, 0].plot(S.t, np.degrees(S.delta_blade), lw=0.8, label="blade")
    lim = math.degrees(P.R.delta_vent_on)
    for s in (1, -1):
        ax[1, 0].axhline(s * lim, color="C3", ls=":", lw=1)
    ax[1, 0].set(xlabel="time [s]", ylabel="rudder [deg]", title="rudder (red = ventilation onset)")
    lg = np.asarray([(r[0], r[2], r[3]) for r in ctl.log])
    if len(lg):
        ax[1, 1].plot(lg[:, 0], lg[:, 1], lw=0.8, label="port")
        ax[1, 1].plot(lg[:, 0], lg[:, 2], lw=0.8, label="starboard")
        ax[1, 1].legend(loc="best")
    ax[1, 1].set(xlabel="time [s]", ylabel="throttle [-1..1]", title="motor outputs")
    for a_ in ax.flat:
        a_.grid(alpha=0.3)
    fig.savefig(path, dpi=110)
    plt.close(fig)


def servo_cop_offset(S, P, off=0.10, every=5):
    """Peak servo torque [N*m] replaying the blade history with the centre of
    pressure `off` chords aft of nominal. The nominal CoP sits on the 0.25c
    stock, so the nominal hinge moment is ~0 by construction."""
    from .rudder import rudder_forces
    Pw = replace(P, R=replace(P.R, x_cp_frac=P.R.x_stock_frac + off))
    return max(rudder_forces(S.delta_blade[i], S.u[i], S.v[i], S.r[i], int(S.vent[i]), Pw)[3]["tau_servo"]
               for i in range(0, len(S.t), every))

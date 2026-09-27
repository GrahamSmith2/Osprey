"""First-order Nomoto identification and the derived gain schedule.

Ported from analysis/identifyNomoto.m and control/gainSchedule.m. Derivation is
in those files; the short version:

    T * rdot + r = K * delta,     K = K' U / L,     T = T' L / U

K' and T' are fitted from step-rudder responses of the nonlinear model. The
SCALING (K ~ U, T ~ 1/U) survives the +/-5x hull-coefficient uncertainty; the
values of K' and T' do not, which is why they should be re-identified on the
water.

Inner yaw-rate PI with pole cancellation (Ti = T) crosses over at
wc_r = Kp_r K / T, so

    Kp_r = wc_r T / K  ~ 1/U^2       Ki_r = wc_r / K  ~ 1/U

(angle domain: rate error -> rudder angle). The outer heading P gain is wc_r/4,
independent of speed. A single-loop heading-to-rudder PID would instead scale
as Kp ~ 1/U — both are correct for their own loop structure.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .params import DEG
from .sim import simulate, trim_state


@dataclass
class NomotoFit:
    speeds: list
    K: list
    T: list
    fit_rms: list
    K_prime: float
    T_prime: float
    K_spread: float
    T_spread: float
    L: float

    @property
    def scaling_holds(self):
        return self.K_spread < 0.35 and self.T_spread < 0.35


def _golden_log(f, lo, hi, iters=80):
    a, b = math.log(lo), math.log(hi)
    gr = (math.sqrt(5) - 1) / 2
    c, d = b - gr * (b - a), a + gr * (b - a)
    for _ in range(iters):
        if f(math.exp(c)) < f(math.exp(d)):
            b = d
        else:
            a = c
        c, d = b - gr * (b - a), a + gr * (b - a)
    return math.exp((a + b) / 2)


def identify_nomoto(P, speeds=(4, 6, 8, 10, 13), delta_step=5 * DEG,
                    t_win=6.0, dt=0.002) -> NomotoFit:
    """Step-rudder tests on the nonlinear model. The step must stay BELOW
    ventilation onset, or the 'linear' model is fitted to a latched plant."""
    if delta_step >= P.R.delta_vent_on:
        raise ValueError("step is at or past ventilation onset")
    L = P.V.LOA
    Ks, Ts, rms = [], [], []
    for U in speeds:
        x0, Tt = trim_state(U, P)
        S = simulate(x0, t_win, dict(delta_cmd=delta_step, T_cmd_port=Tt, T_cmd_stbd=Tt),
                     P, dt=dt)
        t, r = S.t, S.r
        r_ss = float(np.mean(r[t > 0.7 * t_win]))
        Ks.append(r_ss / delta_step)
        cost = lambda T: float(np.sum((r - r_ss * (1 - np.exp(-t / T))) ** 2))
        T = _golden_log(cost, 0.02, 20)
        Ts.append(T)
        pred = r_ss * (1 - np.exp(-t / T))
        rms.append(float(np.sqrt(np.mean((r - pred) ** 2)) / max(abs(r_ss), 1e-12)))
    Kp = [k * L / u for k, u in zip(Ks, speeds)]
    Tp = [tt * u / L for tt, u in zip(Ts, speeds)]
    Km, Tm = float(np.mean(Kp)), float(np.mean(Tp))
    return NomotoFit(list(speeds), Ks, Ts, rms, Km, Tm,
                     (max(Kp) - min(Kp)) / abs(Km), (max(Tp) - min(Tp)) / abs(Tm), L)


def gain_schedule(U, fit: NomotoFit, P, wc_r=4.0, f_ctrl=50.0, delay_n=1):
    """Angle-domain cascade gains at speed U. wc_r is capped by the transport
    delay budget (servo lag + ZOH + compute delay): 30 deg of phase to delay."""
    tau_delay = P.A.tau_servo_lag + 0.5 / f_ctrl + delay_n / f_ctrl
    ceiling = 0.52 / tau_delay
    wc = min(wc_r, 0.8 * ceiling)
    U = max(U, 0.5)
    K = fit.K_prime * U / fit.L
    T = fit.T_prime * fit.L / U
    Kp_r = wc * T / K
    Kp_psi = wc / 4
    return dict(U=U, K=K, T=T, wc_r=wc, wc_ceiling=ceiling, tau_delay=tau_delay,
                Kp_r=Kp_r, Ki_r=Kp_r / T, Kp_psi=Kp_psi,
                Ki_psi=0.033 * Kp_psi ** 2, Kd_psi=0.0)

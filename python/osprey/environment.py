"""Wind side force and wave-induced yaw moment.

Ported from model/envForces.m. Current is NOT handled here: it is a change of
frame, applied in the dynamics by computing every hydrodynamic force from the
WATER-relative velocity. Wind is a genuine force.
"""
from __future__ import annotations

import math

DEG = math.pi / 180.0


def env_forces(u, v, r, psi, t, P):
    E, V = P.E, P.V
    psi_to = E.psi_wind + math.pi
    wn, we = E.V_wind * math.cos(psi_to), E.V_wind * math.sin(psi_to)
    c, s = math.cos(psi), math.sin(psi)
    wu, wv = c * wn + s * we, -s * wn + c * we
    u_a, v_a = u - wu, v - wv
    V_rel = math.hypot(u_a, v_a)
    beta_w = math.atan2(-v_a, max(abs(u_a), 0.05))
    CY = E.CY_beta * max(min(beta_w, 30 * DEG), -30 * DEG)
    q_a = 0.5 * E.rho_a * V_rel * V_rel
    Y_aero = q_a * V.A_lateral * CY
    # Aero CoP forward of the CG: destabilising, turns the bow further off the wind
    N_aero = Y_aero * E.x_cp_aero * V.LOA
    sgn = 1.0 if u_a > 0 else (-1.0 if u_a < 0 else 0.0)
    X_aero = -q_a * V.A_lateral * 0.1 * sgn * abs(math.cos(beta_w))

    # Deterministic band-limited wave moment: a derivative function must return
    # the same value when evaluated twice at the same t, or RK4 breaks.
    N_wave = 0.0
    if E.wave_N_amp > 0:
        acc = 0.0
        for k in range(8):
            f = E.wave_f_lo + (E.wave_f_hi - E.wave_f_lo) * k / 7
            acc += math.sin(2 * math.pi * f * t + 2 * math.pi * k / 8 * 3.7)
        N_wave = E.wave_N_amp * acc / math.sqrt(8)

    return X_aero, Y_aero, N_aero + N_wave

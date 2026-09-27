"""Rudder: lift, drag, ventilation, hinge moment, servo demand, cavitation.

Ported from model/rudderForces.m.

Sign convention (fixed here once): delta > 0 produces a POSITIVE yaw moment,
a turn to starboard. The rudder is aft of the CG so x_r < 0; making Y_rud
negative for positive delta then makes N_rud = x_r * Y_rud positive.

Three non-linearities dominate the tuning:
  1. Ventilation — a latched, hysteretic lift loss. Past ~10 deg the yaw moment
     FALLS with more deflection and stays down until the blade unloads.
  2. Cavitation — reported, not modelled. Past sigma ~0.5 this is void.
  3. Stall — late on a low-AR blade, and ventilation bounds the useful range
     long before stall does.
"""
from __future__ import annotations

import math

DEG = math.pi / 180.0


def _smoothsat(t):
    t = 0.0 if t < 0 else (1.0 if t > 1 else t)
    return t * t * (3 - 2 * t)


def rudder_forces(delta, u, v, r, vent_state, P):
    """delta is the BLADE angle (after servo dynamics and backlash); u, v are
    water-relative. vent_state: 0 wetted, 1 ventilated (latched), 2 forced
    ventilated (fault injection — bypasses the latch)."""
    R, E, A = P.R, P.E, P.A
    AR_eff = R.k_surface * (R.h_sub / R.chord)
    CL_alpha = 1.8 * math.pi * AR_eff / (1.8 + math.sqrt(AR_eff * AR_eff + 4)) * R.CLa_fac

    # The blade sees commanded angle PLUS the local flow angle from the boat's
    # own sway and yaw. This term is the rudder's contribution to yaw damping.
    u_r = max(abs(u), 0.05)
    v_r = v + r * P.x_r
    beta_r = math.atan2(v_r, u_r)
    alpha = delta + beta_r
    aa = abs(alpha)

    # Ventilation with hysteresis
    a_on = R.delta_vent_on
    a_off = max(R.delta_vent_on - R.vent_hysteresis, 1 * DEG)
    if vent_state == 2:
        vent_new, w = 2, 1.0
    else:
        vent_new = vent_state
        if aa > a_on:
            vent_new = 1
        if aa < a_off:
            vent_new = 0
        w = _smoothsat((aa - a_off) / R.vent_blend) if vent_new else 0.0
    k_vent = 1 - w * (1 - R.k_vent_loss)

    # Lift and drag, late stall
    f_stall = 1 - _smoothsat((aa - 25 * DEG) / (10 * DEG)) * 0.6
    CL = CL_alpha * alpha * f_stall * k_vent
    CD = R.CD0 + CL * CL / (math.pi * AR_eff * R.e_oswald)
    q = 0.5 * E.rho_w * u_r * u_r
    A_r = R.h_sub * R.chord
    F_N = q * A_r * CL
    F_D = q * A_r * CD

    Y = -F_N
    X = -F_D
    N = P.x_r * Y

    # Hinge moment: force x (CoP - STOCK) lever — NOT force x servo horn radius
    e_arm = (R.x_cp_frac - R.x_stock_frac) * R.chord
    M_h = F_N * e_arm
    T_cable = M_h / A.r_rudder_arm
    tau_servo = abs(T_cable) * A.r_servo_horn / A.eta_cable

    # Cavitation number at mid-span depth
    h_mid = R.h_sub / 2
    sigma = (E.p_atm + E.rho_w * E.g * h_mid - E.p_vap) / max(q, 1.0)

    return X, Y, N, dict(F_N=F_N, F_D=F_D, CL=CL, CD=CD, CL_alpha=CL_alpha,
                         AR_eff=AR_eff, alpha=alpha, beta_r=beta_r,
                         vent_state=vent_new, k_vent=k_vent, M_h=M_h,
                         tau_servo=tau_servo, e_arm=e_arm, sigma=sigma)


def peak_servo_torque(P, U, delta_limit=None, n=351):
    """Peak steering-servo torque [N*m] sweeping the rudder from 0 out to
    delta_limit (default: the mechanical stop) at speed U, with the ventilation
    latch tracking as the blade swings out.

    The peak is NOT at ventilation onset in general. Lift drops past onset, but
    ventilated lift keeps growing with angle and, at the nominal 50% loss,
    exceeds the onset value above ~20 deg and peaks near 26 deg before stall.
    So the servo load depends on how far the rudder is allowed to travel:
    clamped at onset by the autopilot, or free to the stop under manual command.
    The MATLAB sizing evaluated one point, 35 deg ventilated, which is neither
    (PORTING.md). Returns (tau_peak, delta_at_peak)."""
    lim = P.R.delta_max if delta_limit is None else min(delta_limit, P.R.delta_max)
    best, at, vent = 0.0, 0.0, 0
    for i in range(n):
        d = lim * i / (n - 1)
        _, _, _, D = rudder_forces(d, U, 0.0, 0.0, vent, P)
        vent = D["vent_state"]
        if D["tau_servo"] > best:
            best, at = D["tau_servo"], d
    return best, at


def cavitation_speed(P, sigma_crit=0.5):
    """Speed [m/s] above which sigma < sigma_crit and the lift model is void."""
    E, R = P.E, P.R
    num = E.p_atm + E.rho_w * E.g * R.h_sub / 2 - E.p_vap
    return math.sqrt(num / (0.5 * sigma_crit * E.rho_w))

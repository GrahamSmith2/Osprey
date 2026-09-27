"""Rudder sizing: compare blades, sweep span, and check the servo.

Ported from analysis/sizingSweep.m and runSizingStudy.m, with one correction:
servo demand is now the PEAK hinge moment over a full deflection sweep, which
occurs just before ventilation onset. The MATLAB version evaluated a single
point, 35 deg ventilated, and under-read the peak (PORTING.md).

Run:  python -m osprey size
"""
from __future__ import annotations

import math
from dataclasses import replace

from .params import AS_BUILT, PROPOSED, DEG, INCH, KGCM, Rudder
from .rudder import rudder_forces, peak_servo_torque, cavitation_speed
from .sim import simulate, trim_state

MPH = 0.44704


def steady_turn(P, U, delta, t_sim=25.0, dt=0.005):
    """Open-loop steady turn from straight running at U. Returns
    (radius_m, radius_LOA, yaw_rate, final_speed, t90_s)."""
    x0, Tt = trim_state(U, P)
    S = simulate(x0, t_sim, dict(delta_cmd=delta, T_cmd_port=Tt, T_cmd_stbd=Tt), P, dt=dt)
    R = S.turn_radius()
    hit = [t for t, p in zip(S.t, S.psi) if abs(p) >= math.pi / 2]
    return R, R / P.V.LOA, abs(S.r[-1]), S.u[-1], (hit[0] if hit else float("nan"))


SERVO_SPEEDS_MPH = (30, 50, 70)          # 50 = planned race cruise


def servo_table(P, speeds_mph=SERVO_SPEEDS_MPH, stocks=(0.0, 0.15, 0.25), delta_limit=None):
    """Peak servo torque [kg*cm] by speed and stock position (fraction of chord
    aft of the leading edge). delta_limit None = full mechanical travel.

    The 0.15c column doubles as a 0.25c ("balanced") stock whose centre of
    pressure is 0.10c off nominal: only the lever (x_cp - x_stock) matters."""
    rows = []
    for mph in speeds_mph:
        row = [mph]
        for st in stocks:
            Pw = replace(P, R=replace(P.R, x_stock_frac=st))
            tau, _ = peak_servo_torque(Pw, mph * MPH, delta_limit)
            row.append(tau / KGCM)
        rows.append(row)
    return rows


def compare(P, rudders=(AS_BUILT, PROPOSED), full_mech=False):
    """Side-by-side metrics for a set of rudders on the same boat."""
    out = []
    base = None
    for rd in rudders:
        Pr = P.with_rudder(rd)
        R = Pr.R
        auth = R.authority
        base = base or auth
        d_use = min(R.delta_vent_on, R.delta_max)
        t5 = steady_turn(Pr, 5.0, d_use)
        t10 = steady_turn(Pr, 10.0, d_use)
        rec = dict(
            name=rd.name, span_mm=R.h_sub * 1000, chord_mm=R.chord * 1000,
            area_in2=R.A_r / INCH ** 2, AR=R.AR_geom, CL_alpha=R.CL_alpha,
            authority=auth / base,
            R5_LOA=t5[1], R10_LOA=t10[1], t90_5=t5[4],
            drag_pct_10=_drag_pct(Pr, 10.0, d_use),
            cav_mps=cavitation_speed(Pr),
            servo_clamped=servo_table(Pr, delta_limit=R.delta_vent_on),
            servo_full=servo_table(Pr),
        )
        if full_mech:
            rec["R5_full_LOA"] = steady_turn(Pr, 5.0, R.delta_max)[1]
        out.append(rec)
    return out


def _drag_pct(P, U, delta):
    _, _, _, D = rudder_forces(delta, U, 0, 0, 0, P)
    return 100 * abs(D["F_D"]) / P.HT.lookup(U)[1]


def span_sweep(P, chord=0.030, spans=(0.060, 0.075, 0.090, 0.110, 0.130, 0.150, 0.180, 0.220),
               delta=35 * DEG):
    """The original sweep: submerged span at fixed chord, turn radius at 5 and
    10 m/s at the mechanical stop (as in runSizingStudy.m)."""
    rows = []
    for h in spans:
        Pr = P.with_rudder(Rudder(name=f"{h*1000:.0f}mm", h_sub=h, chord=chord, span_total=h))
        rows.append((h, steady_turn(Pr, 5.0, delta)[1], steady_turn(Pr, 10.0, delta)[1]))
    return rows


def thesis_servo_check(P):
    """Reconcile the thesis's 66 kg*cm with a correctly formulated hinge moment,
    at their own 80 mph design condition, for the as-built blade."""
    U = 35.8
    thesis = 0.5 * 1010 * U ** 2 * (0.5 * 0.150 * 0.030) * 0.3 * 0.015 / KGCM
    Pa = P.with_rudder(replace(AS_BUILT, h_sub=0.075))
    out = dict(thesis_kgcm=thesis)
    for label, st in (("LE", 0.0), ("0.15c", 0.15), ("balanced", 0.25)):
        Pw = replace(Pa, R=replace(Pa.R, x_stock_frac=st))
        full, at = peak_servo_torque(Pw, U)
        clamped, _ = peak_servo_torque(Pw, U, Pw.R.delta_vent_on)
        _, _, _, D35 = rudder_forces(Pw.R.delta_max, U, 0, 0, 1, Pw)
        out[label] = dict(full_kgcm=full / KGCM, at_deg=at / DEG,
                          clamped_kgcm=clamped / KGCM,
                          matlab_35deg_kgcm=D35["tau_servo"] / KGCM)
    return out

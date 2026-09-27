"""Design studies: the analyses behind results/summary.md.

    python -m osprey report      runs all of them and writes results/python/

Ported from analysis/monteCarlo.m, failureModes.m, roundingStudy.m,
lapAccumulation.m and model/rollEnvelope.m, re-pointed at the current race:
two-mark oval, standing start, 50 mph cruise, the Lua autopilot as the
controller. The MATLAB versions flew a MATLAB autopilot on a circle at 8 m/s.

Every study holds the CONTROLLER fixed at the nominal design and varies only
the PLANT. Re-deriving gains per draw would only confirm that the design
procedure works when you already know the answer.
"""
from __future__ import annotations

import math
import os
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace

import numpy as np

from .params import AS_BUILT, PROPOSED, DEG, G, INCH, UNCERTAINTY, Rudder, build_params, sample_uncertainty
from .propulsion import prop_max_thrust
from .race import AUTOPILOT, MPH, Course, controller_params, race_time_limit, run_race
from .sim import Fault, simulate, trim_state

KT = 0.514444
RACE_WIND = dict(wind_kt=15.0, wind_from=90.0, current=0.5, current_to=180.0)


def race_params(rudder=PROPOSED, S=None, *, top_speed_mph=55.0, wind_kt=0.0, wind_from=90.0,
                current=0.0, current_to=180.0):
    """Plant parameters for a race run: the chosen rudder, propulsion sized for
    `top_speed_mph` (the assumed upgrade), and the given wind and current."""
    P = build_params(S, rudder=rudder, top_speed=top_speed_mph * MPH if top_speed_mph else None)
    return P.with_env(V_wind=wind_kt * KT, psi_wind=wind_from * DEG,
                      V_current=current, psi_current=current_to * DEG)


# ---------------------------------------------------------------------------
# Roll / hooking envelope (quasi-static; the 3-DOF model cannot see either)
# ---------------------------------------------------------------------------
def roll_envelope(u, P, beta=5 * DEG):
    """Safe yaw-rate ceiling from three quasi-static checks, per speed u [m/s]:
    inner-sponson unloading, blow-over (aero lift fraction), hooking.
    Port of model/rollEnvelope.m."""
    from .hull import hull_solve
    V, E = P.V, P.E
    u = max(abs(u), 0.1)
    f_aero = hull_solve(u, V, E, P.m)["f_aero"]
    a_y_crit = E.g * V.y_hull / V.h_cg                   # inner pad unloads
    SF = 0.6                                             # quasi-static: no roll inertia
    r_roll = SF * a_y_crit / u
    beta_hook = 12 * DEG                                 # judgement, ASSUMPTIONS A15
    r_hook = beta_hook * u / max(V.x_cg, 1e-3)
    f_eff = f_aero * (1 + 2 * abs(math.sin(beta)))
    blowover_ok = f_eff < 0.40
    r_safe = min(r_roll, r_hook) if blowover_ok else 0.0
    binding = "blow-over" if not blowover_ok else ("roll" if r_roll <= r_hook else "hooking")
    return dict(u=u, a_y_crit_g=a_y_crit / E.g, r_roll=r_roll, r_hook=r_hook,
                r_safe=r_safe, f_aero=f_aero, f_aero_eff=f_eff, binding=binding)


# ---------------------------------------------------------------------------
# Sizing: why span, not area
# ---------------------------------------------------------------------------
def area_vs_span(P, U=10.0, delta=10 * DEG):
    """Authority and turn radius for blades that grow area by chord vs by span.
    Turn radius is an open-loop steady turn at `delta` (the useful limit)."""
    from .sizing import steady_turn
    h0, c0 = AS_BUILT.h_sub, AS_BUILT.chord
    cases = [
        ("as built", h0, c0),
        ("3x area, same span", h0, 3 * c0),
        ("30x area, same span", h0, 30 * c0),
        ("3.3x area via span", 3.3 * h0, c0),
        ("5 in x 30 mm", 5 * INCH, c0),
        ("5 in x 45 mm (chosen)", 5 * INCH, 0.045),
    ]
    rows, base = [], None
    for name, h, c in cases:
        Pr = P.with_rudder(Rudder(name=name, h_sub=h, chord=c, span_total=h))
        auth = Pr.R.authority
        base = base or auth
        R_loa = steady_turn(Pr, U, min(delta, Pr.R.delta_vent_on))[1]
        rows.append(dict(name=name, span_mm=h * 1e3, chord_mm=c * 1e3,
                         area_x=(h * c) / (h0 * c0), AR=Pr.R.AR_geom, CL_alpha=Pr.R.CL_alpha,
                         authority_x=auth / base, R_LOA=R_loa))
    return rows


# ---------------------------------------------------------------------------
# Turn budget: speed sag and the Munk moment in a steady turn
# ---------------------------------------------------------------------------
def turn_budget(P, speeds_mph=(10, 20, 30, 40, 50), delta=None, t_sim=12.0):
    """Open-loop turn at the useful rudder limit from straight running with
    thrust held at trim. Reports speed lost, lateral g, and the Munk moment
    relative to the rudder moment once settled."""
    delta = P.R.delta_vent_on if delta is None else delta
    rows = []
    for mph in speeds_mph:
        U = mph * MPH
        x0, Tt = trim_state(U, P)
        S = simulate(x0, t_sim, dict(delta_cmd=delta, T_cmd_port=Tt, T_cmd_stbd=Tt), P, dt=0.005)
        k = len(S.t) - 1
        rows.append(dict(mph=mph, speed_loss_pct=100 * (1 - S.u[-1] / U),
                         lat_g=float(np.abs(S.u * S.r).max() / G),
                         radius_m=S.turn_radius(),
                         munk_over_rudder=abs(S.N_munk[k]) / max(abs(S.N_rud[k]), 1e-9),
                         beta_deg=math.degrees(S.beta[k]),
                         sigma=float(S.sigma[k])))
    return rows


# ---------------------------------------------------------------------------
# Jammed rudder vs differential thrust
# ---------------------------------------------------------------------------
def jam_moment_balance(P, speeds_mph=(5, 10, 15, 20, 30, 40, 50), jam=10 * DEG):
    """Yaw moment of a rudder jammed at `jam` (grows as u^2) against the most
    the motors can apply (one full, one idle; nearly speed-independent)."""
    from .rudder import rudder_forces
    rows = []
    for mph in speeds_mph:
        U = mph * MPH
        N_jam = abs(rudder_forces(jam, U, 0, 0, 0, P)[2])
        N_thr = prop_max_thrust(U, P) * P.V.y_p
        rows.append(dict(mph=mph, N_jam=N_jam, N_thrust=N_thr, margin=N_thr - N_jam))
    return rows


# ---------------------------------------------------------------------------
# Monte Carlo: one fixed controller, the whole uncertainty space
# ---------------------------------------------------------------------------
def _mc_one(job):
    i, S, rudder, env, params, top_mph, course, dt, tmax = job
    P = race_params(rudder, S, top_speed_mph=top_mph, **env)
    try:
        res = run_race(P, AUTOPILOT, params=params, course=course, seed=1000 + i, dt=dt,
                       tmax=tmax)
        m = dict(res.metrics)
    except Exception as e:                               # a crashed run is a lost run
        m = dict(lost=True, t_finish=None, error=repr(e))
    m.update(i=i, rudder=rudder.name, draw=S)
    m["passed"] = bool(m.get("t_finish") and not m.get("lost")
                       and m.get("straight_max", 1e9) <= 5.0 and m.get("mark_min", 0) > 5.0)
    m.pop("lap_rms", None)
    return m


def monte_carlo(n=60, rudders=(AS_BUILT, PROPOSED), *, seed=12345, cruise_mph=50.0,
                top_speed_mph=55.0, env=RACE_WIND, course=Course(), dt=0.01, workers=None):
    """Latin-hypercube draws over every declared uncertainty, the SAME draws for
    each rudder, flown with the nominal-design controller. Pass = finished,
    never lost, middle-of-straight error <= 5 m, never within 5 m of a mark."""
    draws = sample_uncertainty("lhs", n, seed)
    envkw = dict(wind_kt=env["wind_kt"], wind_from=env["wind_from"],
                 current=env["current"], current_to=env["current_to"])
    tmax = race_time_limit(course, cruise_mph * MPH)
    jobs = []
    for rd in rudders:
        Pn = race_params(rd, top_speed_mph=top_speed_mph)
        params = controller_params(Pn, cruise_mph * MPH)
        jobs += [(i, S, rd, envkw, params, top_speed_mph, course, dt, tmax)
                 for i, S in enumerate(draws)]
    workers = workers or max(1, (os.cpu_count() or 2) - 1)
    if workers == 1:
        return [_mc_one(j) for j in jobs]
    with ProcessPoolExecutor(max_workers=workers) as ex:
        return list(ex.map(_mc_one, jobs, chunksize=2))


def mc_summary(runs):
    """Roll-up per rudder, and Spearman rank correlation of each uncertain
    parameter with the two error measures (non-lost runs only)."""
    out = {}
    for name in dict.fromkeys(r["rudder"] for r in runs):
        R = [r for r in runs if r["rudder"] == name]
        ok = [r for r in R if not r.get("lost") and r.get("t_finish")]
        col = lambda k: np.asarray([r[k] for r in ok], dtype=float)
        s = dict(n=len(R), passed=np.mean([r["passed"] for r in R]),
                 lost=np.mean([bool(r.get("lost")) for r in R]),
                 timed_out=np.mean([bool(r.get("timed_out")) for r in R]),
                 dnf=np.mean([not r.get("t_finish") for r in R]))
        if ok:
            s.update(straight_rms_med=np.median(col("straight_rms")),
                     straight_max_med=np.median(col("straight_max")),
                     straight_max_p90=np.percentile(col("straight_max"), 90),
                     xt_max_med=np.median(col("xt_max")),
                     wide_med=np.median(col("rounding_wide")),
                     clamp_med=np.median(col("clamp_frac")),
                     clamp_p90=np.percentile(col("clamp_frac"), 90),
                     vent_any=np.mean(col("vent_events") > 0),
                     lat_g_med=np.median(col("lat_g")),
                     servo_p90=np.percentile(col("servo_cop_offset_kgcm"), 90),
                     time_med=np.median(col("t_finish")))
            s["rho_straight"] = _spearman(ok, "straight_rms")
            s["rho_xt"] = _spearman(ok, "xt_max")
        out[name] = s
    return out


def _rank(a):
    r = np.empty(len(a))
    r[np.argsort(a, kind="stable")] = np.arange(len(a))
    return r


def _spearman(runs, metric):
    if len(runs) < 3:
        return {}
    y = _rank(np.asarray([r[metric] for r in runs], dtype=float))
    rho = {}
    for k in UNCERTAINTY:
        x = _rank(np.asarray([r["draw"][k] for r in runs], dtype=float))
        c = np.corrcoef(x, y)[0, 1]
        rho[k] = float(c) if np.isfinite(c) else 0.0
    return dict(sorted(rho.items(), key=lambda kv: -abs(kv[1])))


# ---------------------------------------------------------------------------
# Failure modes
# ---------------------------------------------------------------------------
FAULT_T = 10.0            # mid first straight at full speed (north ~200 m of 402)


def failure_modes(rudder=PROPOSED, *, cruise_mph=50.0, top_speed_mph=55.0, env=RACE_WIND,
                  course=Course(), t_f=FAULT_T):
    """One race per fault, injected at t_f with the boat settled on the first
    straight. Plant faults are not reported to the controller; it sees only
    its sensors."""
    P = race_params(rudder, top_speed_mph=top_speed_mph, **{k: env[k] for k in env})
    params = controller_params(race_params(rudder, top_speed_mph=top_speed_mph), cruise_mph * MPH)
    cases = [
        ("no fault", None, None),
        ("rudder jammed at 10 deg", Fault("rudder_jam", t_f, 10 * DEG), None),
        ("rudder jammed centred", Fault("rudder_jam", t_f, 0.0), None),
        ("port motor out", Fault("motor_out", t_f, 1), None),
        ("ventilation latched", Fault("ventilation", t_f), None),
        ("GPS lost for 5 s", None, (t_f, t_f + 5)),
    ]
    rows = []
    for name, fault, drop in cases:
        res = run_race(P, AUTOPILOT, params=params, course=course, fault=fault, gps_dropout=drop,
                       tmax=race_time_limit(course, cruise_mph * MPH))
        S = res.S
        after = (S.t >= t_f) & (S.t <= t_f + 5)
        k = np.flatnonzero(after)
        psi = np.unwrap(S.psi[k]) if len(k) else np.zeros(1)
        m = res.metrics
        rows.append(dict(name=name, finished=res.finished, lost=res.lost,
                         disarm_t=res.ctl.disarm_t,
                         heading_change_5s=float(np.degrees(np.abs(psi - psi[0]).max())),
                         yaw_rate_peak=float(np.degrees(np.abs(S.r[S.t >= t_f]).max())),
                         lat_g=float(np.abs(S.u[S.t >= t_f] * S.r[S.t >= t_f]).max() / G),
                         t_finish=m["t_finish"], xt_max=m["xt_max"],
                         speed_mean_mph=m["speed_mean_mph"], res=res))
    return rows


# ---------------------------------------------------------------------------
# Rounding radius
# ---------------------------------------------------------------------------
def rounding_study(radii=(25, 40, 60, 80), rudder=PROPOSED, *, cruise_mph=50.0,
                   top_speed_mph=55.0, env=None):
    """Race time, how wide the boat swings, and lateral g, per rounding radius."""
    env = env or dict(wind_kt=0.0, wind_from=90.0, current=0.0, current_to=180.0)
    P = race_params(rudder, top_speed_mph=top_speed_mph, **env)
    params = controller_params(race_params(rudder, top_speed_mph=top_speed_mph), cruise_mph * MPH)
    rows = []
    for R in radii:
        c = Course(radius=float(R))
        res = run_race(P, AUTOPILOT, params=params, course=c,
                       tmax=race_time_limit(c, cruise_mph * MPH))
        m = res.metrics
        rows.append(dict(radius=R, finished=res.finished, t_finish=m["t_finish"],
                         avg_mph=m["avg_mph"], xt_max=m["xt_max"], mark_min=m["mark_min"],
                         mark_max=m["mark_max"], wide=m["rounding_wide"], lat_g=m["lat_g"],
                         straight_max=m["straight_max"], lap_rms=m["lap_rms"]))
    return rows

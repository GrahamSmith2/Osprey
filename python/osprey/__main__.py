"""Command line.

    python -m osprey size                       rudder comparison + servo check
    python -m osprey nomoto [--rudder R]        Nomoto fit and gain schedule
    python -m osprey run SCRIPT.lua [options]   fly a Lua controller on the course
    python -m osprey report [--n 60]            every study -> results/python/

Run from the python/ directory.
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path

from .params import AS_BUILT, PROPOSED, DEG, INCH, KGCM, build_params

RUDDERS = {"as-built": AS_BUILT, "proposed": PROPOSED}
MPH = 0.44704


def cmd_size(a):
    from .sizing import compare, thesis_servo_check, SERVO_SPEEDS_MPH
    from .params import Rudder
    P = build_params()
    mid = Rudder(name="5 in x 30 mm", h_sub=5 * INCH, chord=0.030, span_total=0.150 + 2 * INCH)
    C = compare(P, (AS_BUILT, mid, PROPOSED))
    w = 20
    print(f"\n{'':36s}" + "".join(f"{c['name'][:w]:>{w+2}s}" for c in C))
    rows = [("submerged span [mm]", "span_mm", "{:.0f}"), ("chord [mm]", "chord_mm", "{:.0f}"),
            ("wetted area [in^2]", "area_in2", "{:.2f}"), ("aspect ratio", "AR", "{:.2f}"),
            ("CL_alpha [1/rad]", "CL_alpha", "{:.3f}"),
            ("authority A*CLa (x as-built)", "authority", "{:.2f}x"),
            ("turn radius 5 m/s, 10 deg [LOA]", "R5_LOA", "{:.2f}"),
            ("turn radius 10 m/s, 10 deg [LOA]", "R10_LOA", "{:.2f}"),
            ("90 deg turn at 5 m/s [s]", "t90_5", "{:.2f}"),
            ("rudder drag 10 m/s, 10 deg [% hull]", "drag_pct_10", "{:.1f}"),
            ("cavitation ceiling [m/s]", "cav_mps", "{:.1f}")]
    for label, k, f in rows:
        print(f"{label:36s}" + "".join(f"{f.format(c[k]):>{w+2}s}" for c in C))
    for key, title in (("servo_clamped", "travel clamped at ventilation onset (autopilot)"),
                       ("servo_full", "full mechanical travel (manual / RC)")):
        print(f"\nPeak servo torque, {title}")
        print(f"  [kg*cm, stall 74]  stock position LE / 0.15c / 0.25c")
        for i, mph in enumerate(SERVO_SPEEDS_MPH):
            print(f"  {mph:3d} mph{'':26s}" + "".join(
                f"{' / '.join(f'{v:.0f}' for v in c[key][i][1:]):>{w+2}s}" for c in C))
    print("\n  0.15c is also a 0.25c stock with the centre of pressure 0.10c off nominal.")
    print("  50 and 70 mph are above the 44 mph cavitation ceiling: extrapolated.")
    t = thesis_servo_check(P)
    print(f"\nThesis servo sizing (80 mph): {t['thesis_kgcm']:.1f} kg*cm by drag x horn radius.")
    for k in ("LE", "0.15c", "balanced"):
        d = t[k]
        print(f"  correct hinge moment, stock {k:9s} clamped {d['clamped_kgcm']:5.1f}   "
              f"full travel {d['full_kgcm']:5.1f} kg*cm")


def cmd_nomoto(a):
    from .nomoto import identify_nomoto, gain_schedule
    P = build_params(rudder=RUDDERS[a.rudder])
    fit = identify_nomoto(P)
    print(f"\nNomoto fit, {P.R.name}")
    print(f"  K' = {fit.K_prime:.3f} (spread {100*fit.K_spread:.0f}%)   "
          f"T' = {fit.T_prime:.3f} (spread {100*fit.T_spread:.0f}%)   "
          f"scaling holds: {fit.scaling_holds}")
    print(f"\n  {'U m/s':>6} {'K 1/s':>7} {'T s':>6} {'Kp_r':>8} {'Ki_r':>8} {'Kp_psi':>7}")
    for U in (3, 5, 8, 10, 13):
        g = gain_schedule(U, fit, P)
        print(f"  {U:6.1f} {g['K']:7.3f} {g['T']:6.3f} {g['Kp_r']:8.4f} {g['Ki_r']:8.4f} {g['Kp_psi']:7.3f}")


def cmd_run(a):
    from .course import MILE
    from .lua_bridge import LuaScriptError
    from .race import Course, controller_params, run_race, top_speed
    from .rudder import cavitation_speed
    from .sim import Fault

    P = build_params(rudder=RUDDERS[a.rudder], top_speed=a.top_speed * MPH or None).with_env(
        V_wind=a.wind * 0.514444, psi_wind=a.wind_from * DEG,
        V_current=a.current, psi_current=a.current_to * DEG)
    course = Course(separation=a.separation * MILE, radius=a.radius, distance=a.distance * MILE)
    wpts, lap = course.waypoints()
    from .nomoto import identify_nomoto
    fit = identify_nomoto(P)
    cruise = a.speed * MPH
    params = controller_params(P, cruise, fit)
    for kv in a.param or []:
        k, v = kv.split("=", 1)
        params[k] = float(v)

    fault = None
    if a.fault:
        kind, t0, *rest = a.fault.split(":")
        val = float(rest[0]) if rest else 0.0
        fault = Fault(kind, float(t0), val * DEG if kind == "rudder_jam" else val)
    dropout = tuple(float(s) for s in a.gps_dropout.split(",")) if a.gps_dropout else None

    print(f"\nOsprey  |  {Path(a.script).name}  |  {P.R.name}")
    print(f"course: two marks {a.separation} mi apart, rounding radius {a.radius:.0f} m, "
          f"lap {lap:.0f} m, race {a.distance} mi ({len(wpts)} waypoints), "
          f"{'standing' if a.start_speed == 0 else 'flying'} start")
    print(f"conditions: wind {a.wind} kt from {a.wind_from:.0f} deg, current "
          f"{a.current} m/s to {a.current_to:.0f} deg" + (f", fault {a.fault}" if a.fault else "")
          + (f", GPS dropout {a.gps_dropout} s" if dropout else ""))
    print(f"Nomoto for this rudder: K' {fit.K_prime:.3f}, T' {fit.T_prime:.3f}")
    u_top = top_speed(P)
    prop = (f"sized for {a.top_speed:.0f} mph top speed (ASSUMED upgrade)" if a.top_speed
            else "as logged")
    print(f"propulsion: {prop}; loaded top speed {u_top / MPH:.1f} mph; "
          f"cruise {a.speed:.0f} mph needs {P.HT.lookup(cruise)[1] * cruise / 1e3:.1f} kW "
          f"into the water (both motors, before prop losses)")
    if u_top < 0.98 * cruise:
        print(f"  WARNING: cruise {a.speed:.0f} mph is above this boat's top speed; "
              f"use --top-speed to model the upgrade")
    v_cav = cavitation_speed(P)
    if cruise > v_cav:
        print(f"  NOTE: cruise is above the rudder model's cavitation ceiling "
              f"({v_cav / MPH:.0f} mph); rudder forces there are extrapolated")
    print()

    t0 = time.time()
    try:
        res = run_race(P, a.script, params=params, course=course, fault=fault,
                       gps_dropout=dropout, seed=a.seed, rate=a.rate, dt=a.dt, tmax=a.tmax,
                       start_speed=a.start_speed, budget=a.budget, lua=a.lua, echo=True)
    except LuaScriptError as e:
        print(e)
        return 2
    wall = time.time() - t0
    S, ctl, m = res.S, res.ctl, res.metrics

    print("\n--- result " + "-" * 60)
    if res.finished:
        print(f"FINISHED   {m['t_finish']:7.1f} s   avg {m['avg_mph']:4.1f} mph")
    else:
        print(f"DID NOT FINISH   {100 * m['progress_frac']:.0f}% of the course"
              + ("   (lost: left the course)" if res.lost else ""))
    if ctl.error:
        print(f"LUA ERROR: {ctl.error.strip()}")
    if ctl.disarm_t is not None and not res.finished:
        print(f"disarmed at {ctl.disarm_t:.1f} s")
    print(f"cross-track       max {m['xt_max']:6.2f} m   RMS {m['xt_rms']:5.2f} m   "
          f"straights (middle half) max {m['straight_max']:5.2f} m")
    print(f"mark rounding     closest {m['mark_min']:5.1f} m, widest {m['mark_max']:5.1f} m "
          f"from the mark (rounding radius {a.radius:.0f} m)   peak lateral {m['lat_g']:.2f} g")
    print(f"per-lap RMS       " + "  ".join(f"{v:.2f}" for v in m["lap_rms"]) + " m")
    print(f"speed             mean {m['speed_mean_mph']:5.1f}  max {m['speed_max_mph']:5.1f} mph")
    print(f"rudder            peak {m['rudder_peak_deg']:5.1f} deg   "
          f"at ventilation clamp {100 * m['clamp_frac']:4.1f}% of ticks   "
          f"ventilation events {m['vent_events']}")
    print(f"steering          motors {m['t_motor_steer']:6.1f} s   rudder {m['t_rudder_steer']:6.1f} s   "
          f"both at once {m['ticks_both']} ticks")
    print(f"servo             peak {m['servo_nominal_kgcm']:5.1f} kg*cm nominal CoP, "
          f"{m['servo_cop_offset_kgcm']:5.1f} with CoP 0.10c off the stock (74 stall)")
    print(f"yaw rate          peak {m['yaw_rate_peak_dps']:5.1f} deg/s")
    print(f"messages          {len(ctl.msgs)}   sim {S.t[-1]:.0f} s in {wall:.1f} s wall")
    wpts = res.wpts
    fin = m["t_finish"]

    stem = Path(a.out) if a.out else Path("runs") / Path(a.script).stem
    stem.parent.mkdir(parents=True, exist_ok=True)
    with open(stem.with_suffix(".csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "u", "v", "r", "north", "east", "psi_deg", "T_port", "T_stbd",
                    "delta_deg", "blade_deg", "vent", "tau_servo_kgcm"])
        for i in range(len(S.t)):
            x = S.x[i]
            w.writerow([f"{S.t[i]:.3f}", *(f"{v:.4f}" for v in x[:5]), f"{math.degrees(x[5]):.2f}",
                        f"{x[6]:.2f}", f"{x[7]:.2f}", f"{math.degrees(x[8]):.3f}",
                        f"{math.degrees(S.delta_blade[i]):.3f}", int(S.vent[i]),
                        f"{S.tau_servo[i] / KGCM:.2f}"])
    from .race import plot_race
    plot_race(res, P, stem.with_suffix(".png"))
    print(f"wrote {stem.with_suffix('.csv')} and {stem.with_suffix('.png')}")
    return 0 if (fin is not None and not ctl.error) else 1


def cmd_report(a):
    from .report import build_report
    path = build_report(Path(a.out), n_mc=a.n, workers=a.workers)
    print(f"wrote {path}")


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m osprey", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("size", help="compare rudders and check the servo")
    n = sub.add_parser("nomoto", help="identify K', T' and print the gain schedule")
    n.add_argument("--rudder", choices=RUDDERS, default="proposed")
    r = sub.add_parser("run", help="fly a Lua controller on the course")
    r.add_argument("script")
    r.add_argument("--rudder", choices=RUDDERS, default="proposed")
    r.add_argument("--speed", type=float, default=50.0, help="cruise speed [mph]")
    r.add_argument("--top-speed", type=float, default=55.0,
                   help="size propulsion so the loaded boat tops out here [mph]; "
                        "0 = prop as logged (tops out near 29 mph)")
    r.add_argument("--start-speed", type=float, default=0.0, help="initial speed [m/s]")
    r.add_argument("--wind", type=float, default=0.0, help="wind speed [kt]")
    r.add_argument("--wind-from", type=float, default=90.0, help="direction wind comes FROM [deg]")
    r.add_argument("--current", type=float, default=0.0, help="current [m/s]")
    r.add_argument("--current-to", type=float, default=180.0, help="direction current flows TO [deg]")
    r.add_argument("--separation", type=float, default=0.25, help="mark separation [mi]")
    r.add_argument("--radius", type=float, default=25.0, help="rounding radius [m]")
    r.add_argument("--distance", type=float, default=2.0, help="race distance [mi]")
    r.add_argument("--tmax", type=float, default=900.0, help="time limit [s]")
    r.add_argument("--rate", type=float, default=50.0, help="controller tick [Hz]")
    r.add_argument("--dt", type=float, default=0.01, help="integrator step [s]")
    r.add_argument("--fault", help="rudder_jam:T0:DEG | motor_out:T0:1or2 | ventilation:T0")
    r.add_argument("--gps-dropout", help="T0,T1 seconds")
    r.add_argument("--param", action="append", help="NAME=VALUE for param:get (repeatable)")
    r.add_argument("--seed", type=int, default=1)
    r.add_argument("--budget", type=int, default=100_000, help="Lua instructions per call")
    r.add_argument("--lua", default="lua54", help="lua53 | lua54 (ArduPilot uses 5.4)")
    r.add_argument("--out", help="output file stem (default runs/<script>)")
    rp = sub.add_parser("report", help="run every study and write results/python/")
    rp.add_argument("--out", default=str(Path(__file__).resolve().parents[2] / "results" / "python"))
    rp.add_argument("--n", type=int, default=60, help="Monte Carlo draws per rudder")
    rp.add_argument("--workers", type=int, default=None, help="parallel processes")
    a = ap.parse_args(argv)
    return {"size": cmd_size, "nomoto": cmd_nomoto, "run": cmd_run,
            "report": cmd_report}[a.cmd](a) or 0


if __name__ == "__main__":
    sys.exit(main())

# Modelling Assumptions

Every assumption that drives a number, with the reason it was made and the
**expected error direction** — i.e. which way the real boat should differ.

Numbers are from the Python model (`python -m osprey report` →
`results/python/report.md`) unless marked MATLAB. Every uncertain value lives in
`UNCERTAINTY` in `python/osprey/params.py` as a range. Ranked impact is at the
bottom, from the Monte Carlo rank correlations.

---

### A1 — Yaw inertia from a radius-of-gyration rule of thumb
**Assumption.** `Iz = m*(0.25*LOA)^2 = 12.6 kg·m²`, k_zz = 25% of LOA.
**Why.** Iz has not been measured. 0.20–0.30 LOA is the usual band for small
craft.
**Error direction.** Likely **under-estimated**. Osprey is a catamaran: mass is
concentrated in two sponsons outboard of the centreline, and the batteries and
motors sit near the ends of the tunnel. Both push k_zz up. Under-estimating Iz
makes the boat look more agile than it is, which would **under-size** the
rudder.
**Handling.** Swept ±50%. A bifilar pendulum test collapses this in an hour —
see the on-water/bench test list in `results/summary.md`.

---

### A2 — Hull steady state is Savitsky (1964), not the report's Appendix A.1
**Assumption.** Wetted area, trim and resistance come from an independent
implementation of the Savitsky prismatic planing method plus a
displacement-mode branch, blended over 6–10 m/s.
**Why.** The design report's Appendix A.1 was not available. Rather than invent
its contents, a standard published method was implemented and anchored to the
report's checkable outputs.
**Error direction.** Savitsky is for a **prismatic monohull planing surface**.
It does not know about the tunnel, sponson interference, or the step. For a
catamaran running two narrow pads it should be **roughly right on lift and
optimistic on drag** (it misses tunnel spray drag and sponson interference).
**Handling.** Anchored to the 13.4 m/s power data point: 2324 W against ~2400 W
logged, ~3%. (MATLAB reported 2534 W, ~6%, because it used the loaded mass for a
no-payload run.) If Appendix A.1 becomes available, replace `hull_solve` in
`python/osprey/hull.py`; the returned dict is the only interface.

**Known defect.** Resistance does not go to zero at zero speed (~95 N at rest,
~25 N from 0.6–5 m/s): the planing drag term `L·tan(trim)` is applied in
displacement mode too. Kept for MATLAB parity. It makes a standing start
**pessimistic**. See `python/PORTING.md`.

---

### A3 — Running trim is solved, not fixed at the design 1.5°
**Assumption.** Trim is solved per speed, with mean wetted length capped at
0.95·LOA. Below ~18 m/s the hull is pinned at full wetted length and trims
bow-up (3.3° at 2 m/s → 1.5° at 20 m/s).
**Why.** Holding trim at 1.5° at all speeds produced λ ≈ 17–20, i.e. a wetted
length of 2.4–2.9 m on a 2.13 m boat. Physically impossible. The reported 1.5°
is the **design-point** trim at 35.8 m/s, where the tunnel already carries 36%
of the weight.
**Error direction.** The solved low-speed trim is a **lower bound**. Real hulls
hit higher hump trim than a quasi-static lift balance predicts, so hump
resistance here is probably **optimistic**.
**Handling.** Affects the displacement/planing transition speed, which sets
where differential thrust must take over from the rudder.

---

### A4 — No free-surface image effect on the rudder (k_surface = 1.0)
**Assumption.** `AR_eff = k_surface · (h_sub/c)` with nominal k_surface = 1.0.
**Why.** A **deeply submerged** foil with an end plate behaves like a wing of
~2× its geometric aspect ratio, because the plate/free surface acts as a
reflecting plane. A **surface-piercing** rudder gets no such benefit: the free
surface is a pressure-release boundary, and once the blade ventilates it is
literally open to atmosphere.
**Error direction.** If the blade stays wetted at low deflection, effective AR
is higher than assumed and the rudder is **more** effective than modelled — so
this assumption is **conservative for sizing** (it will size the rudder up).
**Handling.** Swept 1.0–2.0.

---

### A5 — Ventilation is a latched, hysteretic 50% lift loss
**Assumption.** Lift is linear to `delta_vent_on` (nominal 10°, swept 6–15°),
then falls to 50% of the attached value. Re-wetting only below
`delta_vent_on − 4°`.
**Why.** Surface-piercing rudders entrain air down the suction side past a
critical angle; the resulting cavity is self-sustaining, so recovery is
hysteretic.
**Error direction.** Ventilation onset on a sharp-edged, unfaired blade like the
C5000 is likely **earlier** than 10°, and the loss **deeper** than 50%.
**Consequence already observed:** for the 75 mm blade at 13.4 m/s the yaw moment
peaks at 10° (64.4 N·m) and *falls* to 38.7 N·m at 12°. Control authority is
**non-monotonic in deflection** — commanding more rudder gives less turn. This
is the mechanism that will produce integral windup and limit cycling in a
naively tuned PID, and why the autopilot clamps the rudder at onset.
**Second consequence (servo):** ventilated lift keeps growing with angle, so
with full travel the hinge moment peaks near **26°**, not at onset. The MATLAB
servo sizing read one point, 35° ventilated, and under-read the peak.

---

### A6 — Cavitation is reported, not modelled
**Assumption.** The attached-flow lift model is used at all speeds, with the
cavitation number computed and reported alongside.
**Why.** Modelling supercavitating flow requires a different section and a
different formulation entirely.
**Error direction.** Above σ ≈ 0.5 the model **over-predicts** rudder lift,
badly. σ < 0.5 above **19.9 m/s (44 mph)**; σ = 0.93 at 50 mph and 0.154 at the
35.8 m/s design speed.
**Handling.** Hard-flagged in results. **Sizing conclusions above ~20 m/s are
not supported by this model — and the 50 mph race cruise is above it.** Every
50 mph rudder number is an extrapolation, optimistic on authority.

---

### A7 — Aero side force uses a flat-plate-like CY_beta = 1.2 /rad
**Assumption.** `Y_aero = 0.5·rho_a·V_rel²·A_lat·CY(beta)`, CoP 25% LOA forward
of the CG (destabilising), A_lat = 0.30 m² (**VERIFY from CAD**).
**Why.** No wind-tunnel or CFD data for the hull-plus-canopy in sideslip.
**Error direction.** A tunnel cat with a canopy presents a bluff, high-drag
lateral profile; CY_beta is more likely **higher** than 1.2. Under-estimating it
makes the cross-track error in crosswind look **better** than reality.

---

### A8 — Prop thrust ceiling from a linear KT(J), calibrated to the log
**Assumption.** `KT = KT0·(1 − J/J0)`. KT0 is solved so that, at the logged
13.4 m/s, no payload and the logged 23 150 rpm, thrust equals hull resistance.
`J0 = 1.35` is assumed. First-order spool-up lag 0.10–0.30 s.
**Why.** Graupner K-series 76 mm open-water data not in hand.
**Error direction.** Sets the thrust ceiling, top speed and differential-thrust
authority. It does not touch rudder sizing. With the prop as logged the loaded
boat tops out at **28.8 mph**; the race assumption is A18.

---

### A9 — Rudder tiller arm radius assumed equal to the servo horn (1:1)
**Assumption.** `r_rudder_arm = r_servo_horn = 0.015 m`.
**Why.** **Not supplied.** The servo horn radius was given; the rudder-side arm
was not.
**Error direction.** Unknown, but this scales required servo torque **linearly**
and is therefore a first-order sizing input, not a detail.
**Handling.** Swept 0.010–0.025 m. **Measure this before trusting any servo
margin number.**

---

### A10 — Rudder stock is balanced at 25% chord
**Assumption.** `x_stock_frac = 0.25`, equal to the nominal CoP, giving
near-zero nominal hinge moment.
**Why.** Not specified for the C5000 blade.
**Error direction.** **This is the assumption most likely to be wrong and most
consequential for the servo spec.** For the new 45 mm × 5 in blade at 50 mph,
worst case from straight running (clamped at 10° / full travel): stock at the
leading edge **103 / 132 kg·cm** against a 74 kg·cm stall; stock at 25% chord
with the CoP 0.10c off nominal **41 / 53**; exactly balanced ~0. If the stock
sits *aft* of the CoP the blade is overbalanced and will slam to the stop — a
stability problem, not just a torque problem.
**Handling.** CoP swept 0.15–0.35c against a fixed stock; the sizing analysis
reports both the balanced and leading-edge-stock cases. **Measure the stock
position.**

---

### A11 — Linear hull derivatives from Clarke (1982), area-scaled
**Assumption.** `Yv, Yr, Nv, Nr` from the Clarke regression evaluated per
demihull at the displacement condition and doubled, then scaled by
`SW(u)/SW_disp` as the hull climbs out of the water.
**Why.** Clarke is the standard slender-body regression and at least has the
right *structure* (forces linear in U, proportional to immersed area). The
alternative was inventing numbers with no pedigree at all.
**Error direction.** Unknown in sign, large in magnitude. Clarke is fitted at
Fn < 0.3 on displacement monohulls; this is a Fn 2.9–7.8 planing catamaran. The
area-scaling proxy is itself an assumption — real planing hulls lose lateral
force faster than wetted area alone suggests, because what remains is a flat
pad with little lateral projection. So the derivatives here are probably
**over-estimated at high speed**, making the boat look more directionally
damped than it is.
**Handling.** Swept ±5× (log-uniform). No conclusion is reported unless it
survives that sweep.

---

### A12 — The Munk moment is NOT small (finding, not assumption)
The prompt anticipated a small Munk moment for a planing hull. The model says
otherwise, and this is now a recorded result rather than an assumption:

At 5° sideslip, against full useful rudder (10°, ventilation onset). The ratio
does not depend on speed: both moments go as U².

| Added-mass case | X_udot/m | Y_vdot/m | old 75 mm blade (MATLAB) | new 45 mm × 5 in blade |
|---|---|---|---|---|
| surge high / sway low | 0.15 | 0.05 | −108% | **−40%** |
| nominal | 0.05 | 0.15 | +108% | **+40%** |
| surge low / sway high | 0.02 | 0.40 | +409% | **+150%** |

Consequences:
1. The new blade cuts the ratio by 2.7× (its authority), but in the high-sway
   corner the Munk moment still **exceeds full useful rudder**.
2. **Its SIGN is not determined** by the current data — it flips depending on
   whether surge or sway added mass dominates. A destabilising Munk moment
   makes the boat directionally divergent in sideslip.
3. **At 50 mph this is what loses boats.** In the Monte Carlo, replayed lost
   draws show the rudder hard over at the 10° clamp while the boat turns
   steadily the *other* way (`results/summary.md` §4b). Hull damping is linear
   in speed while the Munk and rudder moments go as U², so the hull's share of
   the yaw balance shrinks as speed rises.

In a settled open-loop 10° turn the Munk moment is 0.6× the rudder moment at
10–20 mph and 1.6× above 30 mph, where sideslip grows from 3.4° to 5.3°. That is
a qualitatively different plant from the one the control design was expected to
target.

**This is the highest-value measurement to collapse.** A single towing-tank or
on-water sideslip test that pins Y_vdot would resolve both the magnitude and
the sign.

---

### A13 — Speed sag invalidates mid-manoeuvre linearisation (finding)
Open-loop turns at the useful limit (10°) from straight running, thrust held at
trim, lose **21% to 35%** of forward speed between 10 and 50 mph, past the 15%
validity threshold at every speed tested. (MATLAB, 35° turns, old blade:
14–34%.) Since the gain schedule schedules *on measured u*, the controller
is chasing a moving operating point throughout any hard turn.

**Consequence for the design:** the gain schedule must be validated against the
*swept* speed range encountered during a manoeuvre, not just the entry speed,
and commanded yaw rate should be limited to keep sag inside the band where the
schedule is valid.

---

### A14 — CG height above the planing surface assumed at 0.12 m
**Assumption.** `h_cg = 0.12 m`. **Not supplied — VERIFY from CAD.**
**Why.** It is the lever converting turn lateral acceleration into roll load
transfer, so it alone sets the sponson-unloading limit
`a_y_crit = g·y_hull/h_cg = 2.49 g`.
**Error direction.** If the real CG sits higher (batteries on deck rather than
in the sponsons), `a_y_crit` falls proportionally and the envelope tightens.
**Handling.** At 8 m/s this was non-binding by ~3×. **At 50 mph it is nearly
binding:** the autopilot pulls a peak **1.35 g** in the roundings, 54% of the
2.49 g unloading limit and 90% of the 1.50 g quasi-static safe limit (safety
factor 0.6). The autopilot's 0.6 rad/s yaw-rate cap is what keeps it there; an
open-loop 10° turn at 50 mph reaches 2.45 g. A CG 20% higher than assumed makes
the race roundings exceed the safe limit. **Measure `h_cg`.**

---

### A15 — Hooking threshold set at 12° sideslip
**Assumption.** The outer sponson trips at β > 12°.
**Why.** There is no usable theory for sponson tripping at Fn 2.9–7.8. This is
a judgement call from planing-craft practice, not a derived quantity.
**Error direction.** Unknown. It binds only below ~7 m/s, where the roll limit
is looser.
**Handling.** Flagged. It is the weakest number in `studies.roll_envelope`.

---

### A16 — Derivative on heading is unusable; damping must come from the inner loop
**Finding, recorded as a design constraint.** Placing a useful lead zero at
`wc_psi/3` in the outer heading loop needs `Kd_psi ≈ 3`. The fused IMU/mag
heading carries 1.5° of noise; differentiated at the 50 Hz loop rate that is
**1.85 rad/s of noise**, so `Kd_psi = 3` would inject **5.6 rad/s of command
noise against a useful `r_cmd` of ~0.5 rad/s**.

`Kd_psi` is therefore set to **zero**, and all loop damping comes from the inner
rate loop closing on the IMU gyro (0.004 rad/s noise — **463× cleaner**).
This is the concrete justification for the cascade architecture over a
single-loop heading PID.

Separately (found in the MATLAB version, carried into the Lua script), the
textbook integral corner at `wc/10` proved too fast: it produced a lightly
damped ~20 s mode (30° step overshooting to 41° and ringing for half a minute),
because during the step the **Munk moment reaches 135% of the rudder moment**
and acts as negative damping. Backing the corner off to `wc/30`
(`KI_PSI = 0.033·KP_PSI²` in `oval_autopilot.lua`) fixed it. The plant is not
the clean Nomoto model the gains were derived from.

---

### A17 — The gain schedule lives in the unit conversion, not a gain table
**Finding.** With the moment-domain cascade in `oval_autopilot.lua`, the inner
proportional gain `Kp_N = wc·T′·L²·KN/K′` is **speed-invariant**
(76.6 N·m/(rad/s) for the new blade). All the `u²` speed dependence is in the
conversion from yaw moment to actuator command, which uses measured speed:
`δ = N / (KN·U²)` for the rudder, `Δthrottle = N / (2·T_max(U)·y_p)` for the
motors.

Consequences:

1. **The `1/U` law is right for a single-loop heading→rudder PID**
   (`Kp = wc/K ∝ 1/U`). It is **not** the law for this cascade, where the
   angle-domain gain goes as `1/U²` and the moment-domain gain is flat. Both
   are correct; they belong to different loop structures.
2. **The same gain drives either actuator**, so the 10 mph handover between
   motors and rudder needs no retuning and the integrator carries across.
3. **The script does its own scheduling**, so ArduPilot Rover's lack of native
   gain scheduling does not matter as long as the script drives the outputs.
4. **Large steps hide all of this.** A 30° step saturates the rudder at the
   ventilation clamp, so the response is authority-limited and gain barely
   matters. Any gain comparison must be run small-signal or it measures nothing.

(MATLAB, old blade, blended allocator: fixed gains frozen at 15 mph degraded 3×
at 5 mph — 34.6% vs 10.4% overshoot — and were fine at 30 mph. Not re-run for
the Lua script, which always schedules.)

---

### A18 — Propulsion scaled to a 55 mph top speed (race runs only)
**Assumption.** For race runs, `build_params(top_speed=55 mph)` scales KT0 so
the loaded boat's thrust ceiling equals its resistance at 55 mph. That gives
~14% thrust margin at the 50 mph cruise.
**Why.** The team plans to race at 50 mph; the prop as logged tops out at
28.8 mph loaded. 50 mph needs ~336 N and ~7.5 kW into the water, against the
~2 kW per motor logged. This stands in for the ESC/prop/power upgrade.
**Error direction.** The whole thrust curve scales, including static thrust, so
**launch acceleration is optimistic** (0 → 49 mph in ~4 s). Top-end behaviour
is set by the target, so it is right by construction if the upgrade delivers.

---

### A19 — Standing start
**Assumption.** Every race starts from rest on the start line, heading up the
first straight. The MATLAB Monte Carlo and fault runs started at cruise speed.
**Why.** PEP is a standing start [TEAM].
**Error direction.** Launch is where differential-thrust steering (below
10 mph) is used at all; it lasts ~0.8 s here because of A18. With a realistic
launch it lasts longer. A18 and the zero-speed resistance defect (A2) pull in
opposite directions.

---

### A20 — Steering split at 10 mph, rudder clamped at 10° (team decision)
**Decision, not assumption, recorded for traceability.** The autopilot uses
differential thrust only below 10 mph and the rudder only at and above, handing
back to the motors below 9 mph. The rudder is clamped at ventilation onset,
nominally 10°.
**Consequence.** Above 10 mph there is **no second steering actuator**: a
rudder jam there is not survivable without a fault mode that slows below
10 mph. Below ~22 mph the motors could out-muscle a rudder jammed at 10°.

---

### A21 — Yaw-rate command capped at 0.6 rad/s
**Assumption.** `OSP_RATE_MAX = 0.6 rad/s` in the Lua script.
**Why.** Inherited default; it sits just inside the quasi-static roll limit at
50 mph (0.656 rad/s, A14).
**Error direction.** At 50 mph it sets the tightest turn to ~37 m radius, so on
a 25 m rounding the boat swings up to ~34 m wide. Raising it tightens the
rounding and spends the remaining roll margin; lowering it widens the rounding.

---

## Assumptions ranked by impact

From the 50 mph Monte Carlo (`results/summary.md` §4b): rank correlations with
the worst distance off the line, and the parameters of the draws that lost the
boat. Chosen rudder, 60 draws.

1. **A11** hull derivatives — `Nv` (ρ = −0.55) and `Nr` (+0.31) are the
   strongest correlates; lost draws have low sway damping `Yv` (median 0.49×)
   and the launch spin-outs all have strong `Nv` (1.6–4.9×).
2. **A12** sway added mass `Y_v̇` (+0.26) — the Munk moment's size and sign.
3. **A10 / A9** stock position and tiller arm — decide whether the servo works
   at all with the new blade. Not in the Monte Carlo's failure count, because
   the model has no servo stall; they are first for the hardware.
4. **A14** CG height — the 50 mph roundings use 90% of the roll margin.
5. **A1** yaw inertia (−0.24; lost draws 1.17×).
6. **A5** ventilation onset (+0.21) — bounds the useful rudder.
7. **A6** cavitation ceiling — the race is past it; optimistic on authority.
8. **A18** propulsion scaling — makes the launch harsher than a real one.
9. **A4** free-surface image factor — conservative in the safe direction.
10. **A2 / A3** hull method and trim — set the regime-transition speed.

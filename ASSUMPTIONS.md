# Modelling Assumptions

Every assumption that drives a number, with the reason it was made and the
**expected error direction** — i.e. which way the real boat should differ.

Ranked impact on the rudder-sizing conclusion is at the bottom, and is
re-derived from the sensitivity study once the sizing sweep runs.

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
**Handling.** Anchored to the 13.4 m/s power data point (agrees to ~6%). If
Appendix A.1 becomes available, replace the body of `hullSteadyState.m`; the
returned struct is the only interface.

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
**Consequence already observed:** yaw moment at 13.4 m/s peaks at 10° (64.4 N·m)
and *falls* to 38.7 N·m at 12°. Control authority is **non-monotonic in
deflection** — commanding more rudder gives less turn. This is the mechanism
that will produce integral windup and limit cycling in a naively tuned PID.

---

### A6 — Cavitation is reported, not modelled
**Assumption.** The attached-flow lift model is used at all speeds, with the
cavitation number computed and reported alongside.
**Why.** Modelling supercavitating flow requires a different section and a
different formulation entirely.
**Error direction.** Above σ ≈ 0.5 the model **over-predicts** rudder lift,
badly. σ < 0.5 above **19.8 m/s (44 mph)**; σ = 0.154 at the 35.8 m/s design
speed.
**Handling.** Hard-flagged in results. **Sizing conclusions above ~20 m/s are
not supported by this model.**

---

### A7 — Aero side force uses a flat-plate-like CY_beta = 1.2 /rad
**Assumption.** `Y_aero = 0.5·rho_a·V_rel²·A_lat·CY(beta)`, CoP 25% LOA forward
of the CG (destabilising), A_lat = 0.30 m² (**VERIFY from CAD**).
**Why.** No wind-tunnel or CFD data for the hull-plus-canopy in sideslip.
**Error direction.** A tunnel cat with a canopy presents a bluff, high-drag
lateral profile; CY_beta is more likely **higher** than 1.2. Under-estimating it
makes the cross-track error in crosswind look **better** than reality.

---

### A8 — Prop thrust from a generic linear KT(J) fit
**Assumption.** `KT = 0.14 − 0.16·J`, first-order spool-up lag 0.10–0.30 s.
**Why.** Graupner K-series 76 mm open-water data not in hand.
**Error direction.** Affects the **differential-thrust authority curve and the
rudder/thrust crossover speed only**. It does not touch the rudder sizing band,
which is set by rudder physics. Treat the crossover speed as ±30%.

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
consequential for the servo spec.** If the stock is at the leading edge the
lever becomes 0.25c and hinge moment rises from ~0 to 4.75 N·m (48.5 kg·cm) at
35.8 m/s / 15°. If the stock sits *aft* of the CoP the blade is overbalanced and
will slam to the stop — a stability problem, not just a torque problem.
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

At u = 13.4 m/s and 5° sideslip, against 64.4 N·m of rudder moment at
ventilation onset:

| Added-mass case | X_udot | Y_vdot | N_munk | vs rudder |
|---|---|---|---|---|
| Xudot hi / Yvdot lo | −6.63 | −2.21 | **−69.4 N·m** | 108% |
| nominal | −2.21 | −6.63 | **+69.4 N·m** | 108% |
| Xudot lo / Yvdot hi | −0.88 | −17.68 | **+263.9 N·m** | 409% |

Two consequences:
1. The Munk moment **equals or exceeds full useful rudder authority** across
   most of the declared range.
2. **Its SIGN is not determined** by the current data — it flips depending on
   whether surge or sway added mass dominates. A destabilising Munk moment
   makes the boat directionally divergent in sideslip; a stabilising one does
   the opposite.

In the settled 8° turn at 8 m/s the yaw balance is essentially *rudder against
Munk*, with hull damping contributing under 3% of the budget. That is a
qualitatively different plant from the one the control design was expected to
target.

**This is the highest-value measurement to collapse.** A single towing-tank or
on-water sideslip test that pins Y_vdot would resolve both the magnitude and
the sign.

---

### A13 — Speed sag invalidates mid-manoeuvre linearisation (finding)
Open-loop full-rudder (35°) turns from steady state lose **13.7% to 33.8%** of
forward speed, exceeding the 15% validity threshold at four of five tested
entry speeds. Since the gain schedule schedules *on measured u*, the controller
is chasing a moving operating point throughout any hard turn.

**Consequence for the design:** the gain schedule must be validated against the
*swept* speed range encountered during a manoeuvre, not just the entry speed,
and commanded yaw rate should be limited to keep sag inside the band where the
schedule is valid.

---

## Assumptions ranked by impact on the rudder-sizing conclusion

*Placeholder ordering, from the sensitivity runs completed so far. To be
regenerated from the Monte Carlo sensitivity study once the sizing sweep runs.*

1. **A5** ventilation onset and depth — bounds the *maximum useful deflection*,
   which is what actually sets the required area (not stall).
2. **A1** yaw inertia — sets the turn-rate response and therefore the minimum
   area from the low-speed turn-rate spec.
3. **A6** cavitation ceiling — voids the whole conclusion above 20 m/s.
4. **A10 / A9** stock position and tiller arm — set the servo torque spec, but
   not the area band.
5. **A4** free-surface image factor — conservative in the safe direction.
6. **A2 / A3** hull method and trim — set the regime-transition speed, second
   order for area.
7. **A8** prop KT — affects the allocator crossover only.

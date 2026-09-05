# Osprey USV — Rudder Sizing & Heading Control Simulation

<https://github.com/GrahamSmith2/Osprey>

MATLAB simulation supporting rudder sizing, servo specification, and heading /
cross-track controller design for **Osprey**, a 7 ft twin-inboard electric
planing catamaran USV intended to run a 2-mile autonomous course.

Plain MATLAB. No Simulink, no OOP, no required toolboxes. Entry point:
`run_all.m`.

---

## THE CENTRAL CAVEAT — READ THIS BEFORE USING ANY NUMBER

At 30 mph this hull runs at Froude number **Fn ≈ 2.9**; at the 80 mph design
speed, **Fn ≈ 7.8**.

Every standard manoeuvring-derivative regression — Clarke, Norrbin, Inoue, and
the rest of the ship-hydrodynamics literature — is fitted to **slender
displacement hulls at Fn < 0.3**. This boat operates one to one-and-a-half
orders of magnitude outside that domain, on a planing catamaran with a
ram-air tunnel, which is not the hull form those regressions describe either.

**Any hull derivative in this repo taken from those correlations is a
placeholder with order-of-magnitude uncertainty, not an estimate.** They are
swept ±5× in `params/uncertainty.m` for exactly this reason.

### What this simulation therefore cannot do

It **cannot tell you what your gains should be.** Any specific Kp/Ki/Kd it
prints is conditional on hull derivatives that are unknown to within 5×.

### What it can do, and is built to do

1. **A rudder area that works across the whole plausible parameter space** —
   sized from both ends (minimum by turn-rate spec at the lowest autonomous
   speed, maximum by drag, hinge moment, ventilation onset, and the blow-over
   envelope at top speed). A feasible band, not a single number.
2. **A servo specification with margin**, computed from a correct hinge-moment
   formulation rather than force × horn radius.
3. **The structure of the gain schedule and its scaling law** — that
   `Kp ∝ 1/U` and `Td ∝ 1/U` follows from Nomoto `K ∝ U`, `T ∝ 1/U`, and that
   result is robust even when `K'` and `T'` themselves are not known.
4. **Which experiment to run** to collapse the remaining uncertainty.

Every deliverable is designed to survive being wrong by 5× on the hull
derivatives. If a conclusion in `results/summary.md` does not survive that, it
is labelled as not surviving it.

---

## Second caveat: the cavitation ceiling

The rudder cavitation number is

```
sigma = (p_atm + rho*g*h - p_v) / (0.5*rho*U^2)
```

For this blade at mid-span depth, **sigma drops below 0.5 at U ≈ 19.8 m/s
(44 mph)** and reaches **0.154 at the 35.8 m/s design speed**.

**The attached-flow lift model in `model/rudderForces.m` is not valid above
about 20 m/s.** That is 55% of the design top speed. Results above it are
printed but flagged. Sizing the rudder for 80 mph operation requires a
supercavitating or ventilated-wedge section, which is a different blade and a
different model, not a bigger version of this one.

---

## Validation status

The model is anchored to the two independent numbers the design report supplies:

| Check | Reported | Model | Status |
|---|---|---|---|
| Aero lift fraction at 35.8 m/s | 36% of weight | 36.0% | calibrated to this (not a check) |
| Effective power at 13.4 m/s | ~2400 W (45 A × 44.4 V × 2, ~60% chain eff.) | 2534 W | **independent, ~6%** |
| Cavitation number at 35.8 m/s | ≈ 0.15 | 0.154 | **independent** |

The 13.4 m/s power check is the meaningful one: nothing in the hull resistance
model was fitted to it.

---

## Repository layout

```
run_all.m              single entry point
params/                every physical constant. Nothing is hardcoded downstream.
  params_vessel.m      measured geometry and mass properties
  params_env.m         fluid properties, disturbance defaults
  params_rudder.m      baseline MHZ Mystic C5000 blade
  params_actuators.m   servo, cable linkage, ESC/motor/prop
  uncertainty.m        EVERY unknown, as a range. The most important file here.
  sampleUncertainty.m  nominal / LHS / corner sampling, no toolbox
  buildParams.m        assembles the one struct every model file reads
model/
  hullSteadyState.m    wetted area, lift split, resistance vs speed
  rudderForces.m       lift, drag, ventilation, hinge moment, cavitation
docs/
results/               figures (PNG), summary.md, autopilot_params.txt
tests/                 sanity checks that must pass before any result is trusted
ASSUMPTIONS.md         every modelling assumption, its reason, and its expected
                       error direction
```

## Requirements

MATLAB R2023b+. No toolboxes. Where a Control System Toolbox function would
help (`margin`, `bode`), a plain-MATLAB fallback is used unless
`license('test', ...)` passes.

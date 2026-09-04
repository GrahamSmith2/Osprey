# Osprey USV — Rudder Sizing & Heading Control: Results Summary

Generated from `run_all.m`. Every number below is conditional on the
assumptions in `../ASSUMPTIONS.md`, and the important ones are named inline.

**Read `../README.md` first.** The hull derivatives underpinning all of this are
swept ±5× because the standard regressions are being used an order of magnitude
outside their validity domain (Fn 2.9–7.8 vs Fn < 0.3). Nothing here is a
prediction of the boat's trajectory.

---

## 1. Headline: the rudder should be sized on SPAN, not area

This is the single most important result, and it was not obvious going in.

Rudder authority is set by the product **A_r · CL_α**, not by area alone. Growing
area at fixed span means growing chord, which collapses the aspect ratio, and
`CL_α` falls almost as fast as `A_r` rises:

| change | chord | AR | CL_α [1/rad] | A_r·CL_α | turn radius @10 m/s |
|---|---|---|---|---|---|
| baseline | 30 mm | 2.50 | 2.83 | 1.00× | 9.21 LOA |
| 3× area, same span | 90 mm | 0.83 | 1.19 | 1.26× | 7.85 LOA |
| **30× area, same span** | 900 mm | 0.08 | 0.12 | **1.32×** | **7.64 LOA** |
| 3.3× area via **span** | 30 mm | 8.33 | 4.54 | **5.36×** | **3.53 LOA** |

**Thirty times the area at fixed span buys 32% more authority. Growing the span
instead buys 436% for one tenth the area increase.**

Practical consequence: the sweep the brief specified (area 0.5×→3×) would have
concluded the rudder cannot be usefully improved. That conclusion would have
been an artifact of holding span fixed.

---

## 2. The sizing band — and why it is currently empty

Span sweep at the as-built 30 mm chord. Spec used: turn radius ≤ 5 LOA and
t90 ≤ 5 s at 5 m/s; servo ≤ 50% of stall; rudder drag ≤ 6% of hull resistance.

| span [mm] | A_r/A₀ | AR | R @5 m/s [LOA] | t90 [s] | drag [%] | servo [% stall]¹ | feasible |
|---|---|---|---|---|---|---|---|
| 60 | 0.80 | 2.00 | 12.99 | 9.79 | 1.9 | 13 | no (turn) |
| **75 (as built)** | 1.00 | 2.50 | **9.53** | 7.42 | 2.5 | 19 | **no (turn)** |
| 90 | 1.20 | 3.00 | 7.56 | 6.05 | 3.1 | 25 | no (turn) |
| 110 | 1.47 | 3.67 | 6.09 | 5.01 | 3.7 | 34 | no (turn) |
| 130 | 1.73 | 4.33 | 5.27 | 4.42 | 4.3 | 44 | no (turn, marginal) |
| 150 | 2.00 | 5.00 | 4.76 | 4.05 | 4.9 | 53 | no (servo) |
| 180 | 2.40 | 6.00 | 4.25 | 3.69 | 5.6 | 68 | no (servo) |
| 220 | 2.93 | 7.33 | 3.79 | 3.37 | 6.5 | 87 | no (servo, drag) |

¹ worst case: rudder stock at the leading edge (unbalanced).

**The band is empty.** The turn requirement needs span ≥ 150 mm; the servo
requirement (at 2× margin, unbalanced stock) allows ≤ ~140 mm. The two bounds
cross by about 10 mm.

### That result is decided by ONE unmeasured number

Servo torque at 20 m/s and 35° deflection, as a function of stock position:

| span [mm] | stock @ 0.00c | 0.10c | 0.15c | 0.20c | 0.25c |
|---|---|---|---|---|---|
| 150 | 53% | 32% | 21% | 11% | **0%** |
| 180 | 68% | 41% | 27% | 14% | **0%** |
| 220 | 87% | 52% | 35% | 17% | **0%** |

So the band depends entirely on where the stock sits:

| assumption | feasible band |
|---|---|
| stock at LE, 2.0× servo margin | **EMPTY** — bounds cross |
| stock at LE, 1.5× servo margin | span 150–180 mm (A_r 1.5×–2.4×) |
| stock at 0.15c, 2.0× margin | span 150–220 mm |
| stock balanced at 0.25c | span 150–220 mm, **drag-limited, servo never binds** |

**Recommendation.** Target **span 150–180 mm at the existing 30 mm chord**
(A_r = 4.5–5.4 × 10⁻³ m², i.e. 2.0–2.4× the as-built area), and **balance the
stock at 15–25% chord**. Balancing is nearly free and removes the binding
constraint entirely. Without it, the servo margin is the thing that fails.

**Is the existing MHZ Mystic C5000 in the band? No.** At 75 mm submerged span it
misses the turn requirement by roughly 2× (9.53 LOA against a 5 LOA spec) and is
outside the band at every assumption set above.

### Caveat on the spec itself
`R_turn ≤ 5 LOA at 5 m/s` is **my** placeholder, not yours — the PEP course
geometry and buoy turn radii were never supplied. The band scales directly with
it, so substitute the real requirement before ordering anything. The table above
gives the mapping from spec to span.

---

## 3. Servo requirement — recheck of the report's 66 kg·cm

Baseline blade, 35° deflection.

| condition | F_N [N] | lever | M_hinge [N·m] | servo demand |
|---|---|---|---|---|
| 20 m/s, stock @ LE | 157.0 | +7.5 mm | 1.18 | **14.1 kg·cm** |
| 35.8 m/s, stock @ LE | 502.9 | +7.5 mm | 3.77 | **45.2 kg·cm** |
| 35.8 m/s, stock @ 0.25c | 502.9 | 0 | 0 | **0 kg·cm** |

**Verdict: the report's 66 kg·cm was CONSERVATIVE, by roughly 46%.**

The most likely source of the discrepancy is using the **servo horn radius as
the hinge-moment lever**:

```
F_N × r_horn = 502.9 × 0.015 = 7.54 N·m = 76.9 kg·cm
```

which lands very close to 66 kg·cm for slightly different force assumptions.
That formulation is dimensionally the force-to-torque conversion *at the servo*,
not the hinge moment *at the stock*. The correct lever is the distance from the
centre of pressure to the stock axis, `(x_cp − x_stock) · c`, which for a
25%-balanced blade is near zero and for an LE stock is 0.25c = 7.5 mm.

**Two caveats before treating 45.2 kg·cm as the answer:**
1. It assumes a **1:1 tiller-arm to servo-horn ratio** (ASSUMPTIONS #A9). That
   ratio scales the requirement **linearly** and was never supplied.
2. 35.8 m/s is **above the cavitation validity ceiling** (see §5), so that row
   is an extrapolation. The 20 m/s row is the defensible one.

**Servo rate.** Datasheet no-load speed was not supplied (`<<FILL>>`). The
sizing assumed 0.10 s/60° ⇒ 600°/s. A stop-to-stop slew in 0.5 s requires
140°/s, so there is ample margin at the assumed value — but confirm it, because
the rate limit is what actually caps achievable derivative gain.

---

## 4. Gain schedule

Nomoto identification on the nonlinear model (5° step, below ventilation onset):

**K′ = 0.506 (±11% across speed), T′ = 1.084 (±15%)**, fit RMS < 5%.
The `K ∝ U`, `T ∝ 1/U` scaling holds, which is the result that survives the ±5×
hull-derivative uncertainty.

### Derived gains (cascade, wc_r = 4.0 rad/s)

| U [m/s] | K [1/s] | T [s] | Kp_N [N·m/(rad/s)] | Ki_N | Kp_ψ | Kd_ψ |
|---|---|---|---|---|---|---|
| 3.0 | 0.711 | 0.771 | 80.28 | 104.1 | 1.00 | 0 |
| 5.0 | 1.184 | 0.462 | 80.28 | 173.6 | 1.00 | 0 |
| 6.7 | 1.587 | 0.345 | 80.28 | 232.7 | 1.00 | 0 |
| 10.0 | 2.369 | 0.231 | 80.28 | 347.2 | 1.00 | 0 |
| 13.0 | 3.080 | 0.178 | 80.28 | 451.3 | 1.00 | 0 |

Inner crossover 4.0 rad/s against a delay-imposed ceiling of 7.43 rad/s
(70 ms of servo lag + ZOH + one-sample compute delay). Outer crossover 1.0 rad/s.
Achieved step response: **overshoot flat at −3% and t90 ≈ 2.0–2.5 s from 4 to
13 m/s** — i.e. the schedule does deliver speed-invariant behaviour.

### Three corrections to the brief's expectations

1. **`Kp ∝ 1/U` is right for a single-loop heading→rudder PID** (verified:
   `single_Kp · U` = 4.221, constant). It is **not** the law for this cascade,
   where the angle-domain gain goes as `1/U²` and the moment-domain gain
   `Kp_N` is **speed-invariant**. Both are correct for their architectures.
2. **`Kd` on heading is unusable.** A useful lead zero needs `Kd_ψ ≈ 3`; the
   1.5° heading noise differentiated at 50 Hz is 1.85 rad/s, so that gain would
   inject 5.6 rad/s of command noise against a 0.5 rad/s useful command. Damping
   comes from the inner rate loop on the gyro instead (463× cleaner signal).
   **This is the concrete justification for the cascade over a single-loop PID.**
3. **Fixed gains do not fail the way expected.** Small-signal 5° step, gains
   frozen at 15 mph:

   | U | scheduled | fixed @15 mph |
   |---|---|---|
   | 2.2 m/s (5 mph) | 10.4% | **34.6%** |
   | 6.7 m/s (15 mph) | 10.9% | 10.9% |
   | 13.4 m/s (30 mph) | 11.7% | 11.0% |

   Degradation is 3× at the **low** end, not the high end, and never unstable.
   The differential-thrust allocator has no `u²` dependence and props up exactly
   the end that would otherwise be gain-starved. A rudder-only boat would fail.

   **Methodological note:** a 30° step saturates the rudder at the ventilation
   clamp, making the response authority-limited so gain barely matters, and the
   2° magnetometer bias swamps a 5° step. Any gain comparison must be run
   small-signal and bias-free or it measures nothing.

---

## 5. Validity limits — where this model stops being usable

**Cavitation.** σ falls below 0.5 at **19.8 m/s (44 mph)** and reaches 0.154 at
the 35.8 m/s design speed (independently reproducing the brief's ≈0.15 figure).
**The attached-flow lift model is void above ~20 m/s**, which is 55% of the
design top speed. Sizing a rudder for 80 mph needs a supercavitating or
ventilated-wedge section — a different blade and a different model.

**Ventilation.** Control authority is **non-monotonic in deflection**. At
13.4 m/s the yaw moment peaks at 10° (64.4 N·m) and falls to 38.7 N·m at 12°,
latched until the blade unloads below 6°. The allocator therefore clamps
commanded deflection at ventilation onset, not at the 35° mechanical stop: the
authority beyond onset is not real.

**Speed sag.** Full-rudder turns bleed **14–34%** of forward speed (up to 55% at
20 m/s entry), breaking the brief's 15% threshold at nearly every condition.
Since gains schedule on measured `u`, the controller chases a moving operating
point throughout any hard turn.

**Munk moment.** At nominal it is **108% of rudder authority** at 5° sideslip,
and up to **409%** at the range edge — **with its sign undetermined**, since it
flips depending on whether surge or sway added mass dominates. In a settled 8°
turn the yaw balance is essentially rudder against Munk, with hull damping under
3% of the budget. The plant is not the well-damped object the control design
assumed.

**Roll envelope is NOT binding.** Sponson unloading occurs at 2.49 g lateral;
the boat reaches only **15–36% of `r_max_safe`** at full rudder. Blow-over only
flags above ~35 m/s with sideslip. Rudder authority binds everywhere first.

---

## 6. Powertrain finding (outside the rudder scope, but it changes the problem)

Reaching the 35.8 m/s design speed requires 281 N of thrust ⇒ 10.1 kW effective
⇒ ~16 kW electrical at the 63% chain efficiency implied by your own log. That is
**~180 A per motor against the 45 A logged**.

This argument does **not** depend on prop assumptions — only on the hull
resistance model, which validated to ~6% against the logged 13.4 m/s run.
**The 80 mph design point appears roughly 4× short on installed power.**

If real top speed is nearer 13 m/s, the cavitation ceiling stops binding
entirely and the rudder sizing problem becomes strictly easier — the band in §2
is then governed by the low-speed turn requirement alone.

---

## 7. The minimum on-water test set

Ranked by how much uncertainty each collapses per hour of boat time.

| # | Test | Instrumentation | Duration | Identifies |
|---|---|---|---|---|
| 1 | **Bifilar pendulum swing** (on the bench, not the water) | stopwatch, two parallel lines | 1 h | **Iz** to ~5%. Kills the ±50% sweep on the single largest unknown. |
| 2 | **Zig-zag (20°/20°) at 3 speeds** (5, 8, 12 m/s) | IMU @100 Hz, GPS @5 Hz, rudder feedback pot | 2 h | **K′ and T′** directly, and their speed scaling. This is the classic Nomoto ID manoeuvre. |
| 3 | **Steady turn circles**, 5°/10°/15°/20° rudder, both directions, 2 speeds | same + speed log | 2 h | Turn radius vs deflection ⇒ **the ventilation onset angle and the depth of the authority loss** (#A5), plus confirms the non-monotonicity. |
| 4 | **Pull test on the tiller** at 3 speeds, straight-line | inline load cell on the steering cable | 1 h | **Hinge moment and the stock position** (#A10, #A9) — the one measurement that decides the sizing band in §2. |
| 5 | **Drift/sideslip run**: hold rudder, let the boat settle, measure crab angle | GPS COG vs IMU heading | 1 h | **Y_v̇ ⇒ the Munk moment magnitude AND sign** (#A12). |
| 6 | **Coast-down from top speed** | GPS @10 Hz | 0.5 h | Hull resistance curve — validates `hullSteadyState` beyond the single 13.4 m/s point. |

Tests 1, 2 and 4 alone would collapse most of the uncertainty that currently
makes the sizing band ambiguous.

---

## 8. Assumptions ranked by impact on the sizing conclusion

1. **#A10 stock chordwise position** — decides whether the feasible band exists
   at all. *Measure this first.*
2. **#A5 ventilation onset and depth** — bounds maximum useful deflection, which
   sets required area.
3. **#A1 yaw inertia** — sets low-speed turn response and therefore the minimum.
4. **#A9 tiller arm radius** — scales servo torque linearly.
5. **#A6 cavitation ceiling** — voids everything above 20 m/s.
6. **#A12 Munk moment sign** — determines whether the plant is directionally
   divergent; affects controller robustness more than area.
7. **#A11 hull derivatives (±5×)** — large, but the sizing band is set mostly by
   rudder physics, which is why the band survives the sweep at all.
8. **#A14/#A15 roll envelope** — currently non-binding by 3×.

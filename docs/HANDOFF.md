# Osprey USV Simulation — Handoff

**For:** an engineer taking over the simulation and/or writing the flight code.
Assumes you are comfortable with software but **not** that you know planing-hull
hydrodynamics. Everything domain-specific is explained where it matters.

**Repo:** `C:\Osprey` (git, `main`, no remote yet)
**Language:** MATLAB R2023b+, plain scripts and functions. **No Simulink, no
OOP, no toolboxes.** Where a Control System Toolbox function would help, a
plain-MATLAB equivalent is written instead. Verified: the repo runs on a MATLAB
install with no Statistics Toolbox (`tiedrank`, `corr`, `prctile`, `interp1`
on the hot path are all hand-rolled — see `analysis/rankCorr.m`).

---

## 1. What this is, in one paragraph

Osprey is a 7 ft, 44 kg twin-motor electric planing catamaran USV that must run
a 2-mile autonomous course (four 0.5-mile circular laps) at the ASNE/ONR **PEP26
Autonomy Division** competition. This repo is a 3-DOF manoeuvring simulation
built to answer three design questions: **how big should the rudder be**, **what
servo is needed**, and **what is the structure of the heading-control gain
schedule**. It is explicitly *not* built to predict the boat's trajectory
accurately — the hull hydrodynamic coefficients are unknown to within ±5×, and
the whole thing is structured so that conclusions survive that.

**Read `README.md` before trusting any number.** The short version: standard
ship manoeuvring-derivative regressions are fitted at Froude number < 0.3; this
boat runs at Fn 2.9–7.8. Every hull derivative here is a placeholder swept ±5×.

---

## 2. Run it

```matlab
addpath(genpath('C:\Osprey'));
run_all          % ~285 s: tests, sizing sweep, failure modes, course, params
run_tests        % ~60 s: 44 sanity checks, run these before trusting anything
```

`run_all.m` is the only entry point. Every other file does one thing and says so
in a header comment. Outputs land in `results/` (PNG figures, `summary.md`,
`autopilot_params.txt`).

**`results/summary.md` is the deliverable.** It carries every conclusion with
its caveats. This handoff is about the *code*; that document is about the
*findings*.

---

## 3. Architecture

Data flows one way. There are no globals, no persistent state except two
deliberate caches (noted below), and no file I/O inside the simulation loop.

```
params/                 ->  P (one struct)  ->  model/  ->  state derivative
  params_*.m                                     control/ ->  commands
  uncertainty.m                                  sensors/ ->  measurements
  sampleUncertainty.m
  buildParams.m
```

### The `P` struct is the single contract

`buildParams(S, overrides)` assembles **everything** the model needs into one
struct. Model functions take `P` and nothing else — they never call `params_*.m`
directly. That is what makes a 500-run Monte Carlo possible: vary `S`, and the
entire model follows.

```matlab
P = buildParams();                                   % nominal
P = buildParams(sampleUncertainty('lhs',500,1)(i));  % one Monte Carlo draw
P = buildParams([], struct('R_A_r', 4.5e-3));        % override a parameter
```

Sub-structs: `P.V` vessel, `P.E` environment, `P.R` rudder, `P.A` actuators,
`P.S` the uncertainty draw (kept attached for traceability), `P.HT` a
precomputed hull table (see §6).

Override keys are underscore-joined paths: `'R_A_r'` → `P.R.A_r`.

### Simulation loop

```
simOsprey(x0, tf, ctrlfun, P, opts)
  └─ fixed-step RK4, dt = 0.02 s (500 Hz available at dt = 0.002)
     ├─ controller tick at 50 Hz (ZOH) with one-sample compute delay
     ├─ eom3dof(t, x, ctrl, vent_state, P, delta_blade)  -> xdot
     │    ├─ hullSteadyState   wetted area, resistance  (table lookup)
     │    ├─ hullForces        sway/yaw damping, turn drag
     │    ├─ rudderForces      lift, ventilation, hinge moment
     │    ├─ propForces        thrust -> surge + differential yaw
     │    └─ envForces         wind side force, wave yaw moment
     └─ three things live in the STEPPER, not the derivative (see below)
```

**State vector (9):** `[u; v; r; X; Y; psi; T_port; T_stbd; delta]`
— body-frame surge/sway/yaw rate, earth position, heading, two thrust states
(ESC spool-up lag), one servo state.

### Why fixed-step RK4 and not `ode45`

The controller is a discrete, fixed-rate device with a one-sample compute delay.
`ode45` chooses its own step and will step over a controller tick, silently
smoothing the ZOH and the delay into something the real autopilot never does.
Digital phase lag is a first-order effect on achievable bandwidth, so it has to
be simulated as what it is.

### Three things deliberately live in the stepper

These are all **non-differentiable**, so they cannot live inside a state
derivative without corrupting the integrator:

1. **Ventilation latch** — a discrete state, held **fixed across all four RK4
   stages** and updated once per step. If it flipped mid-stage the four stages
   would disagree about which physical branch they are on.
2. **Cable backlash** — hysteresis between servo output `x(9)` and the blade
   angle the water actually sees. `eom3dof` takes `delta_blade` as an optional
   6th argument for this reason.
3. **Controller ZOH and the command delay queue.**

### Control cascade

```
waypoints -> losGuidance      -> psi_cmd   (crab-corrected)
          -> heading PID      -> r_cmd     (limited by rollEnvelope)
          -> yaw-rate PI      -> N_cmd     [N*m, moment units]
          -> allocate         -> delta_cmd + differential thrust
```

`makeAutopilot(P, NM, cfg)` returns a closure usable as `ctrlfun`. Controller
state lives in the `mem` struct that `simOsprey` threads through — no globals.

---

## 4. Conventions you must not break

- **SI internally, always.** Conversions happen at the boundary, in `params/`.
  Every quantity carries its unit in a trailing comment.
- **Body frame at the CG.** x forward, y starboard, z down. `psi` positive
  clockwise from north.
- **Lever arms are positive forward.** The rudder is aft, so `P.x_r < 0`.
- **Added mass is stored NEGATIVE** (SNAME convention), so `(m - X_udot)` reads
  as "mass plus added mass". This keeps the equations identical to the textbook
  rather than littered with ad-hoc sign flips.
- **Positive rudder deflection produces positive yaw moment** (turn to
  starboard). The sign bookkeeping is done **once**, in `rudderForces.m`.
  Everything else follows from it.
- **Anything unknown goes in `params/uncertainty.m` as a RANGE**, never in a
  model file as a constant. This is the most important rule in the repo.

---

## 5. The test suite

`tests/run_tests.m` — 44 assertions, no framework, no toolbox. **They must all
pass before any result is trusted**, and `run_all.m` aborts if they don't.

They are not smoke tests; several caught real bugs:

- **T9** straight line: zero rudder, zero disturbance → `r ≡ 0` to machine
  precision.
- **T10b** steady-turn centripetal closure: `−(m − X_u̇)·u·r + ΣY = 0` to 1e-4
  relative. This is the single best check that the equations are consistent.
- **T11** numerical Jacobian **convergence** (halving the step changes it by
  <1e-6). *Note:* the original spec asked for an analytic Jacobian agreeing to
  1e-6. There isn't one, deliberately — the model contains `min`/`max`
  saturation, `abs`, `sign`, a bisection solve, and a latched branch. It is
  piecewise-smooth, so a closed-form Jacobian would be fiction over part of the
  state space. Convergence is the property that actually catches the failure
  mode.
- **T12** current is a **frame shift, not a force** — with a pure current and
  zero thrust the boat drifts at exactly the current speed with no yaw.
- **T14** the gain-schedule scaling exponents (`Kp_r ∝ 1/U²`, `Kp_N` flat).

---

## 6. Performance notes

A 500-draw Monte Carlo was originally ~35 min. It is now ~17. What was done:

- **`model/buildHullTable.m`** — `hullSteadyState` solves two nested bisections,
  and `eom3dof` calls it four times per RK4 step (~16,000 solves per 20 s run).
  Every quantity it returns is a smooth function of `|u|` for fixed parameters,
  so it is tabulated once at `buildParams` time (0.02 m/s grid) and interpolated
  thereafter. Agreement 1e-5, verified.
- **`hullSteadyState(u, P, 'lite')`** — the dynamics read only `SW` and
  `R_total`; building the other 11 fields 20,000 times a run was pure cost.
- `simOsprey` called `eom3dof` **twice per step with identical arguments** (once
  for logging, once as RK4 stage 1). Now captured once and reused.
- `makeAutopilot` grew its telemetry struct array element-by-element — O(n²).
  Now preallocated columns.
- Bisection cut from 80 halvings to 40 (80 resolved a range of 30 to 2.5e-23).

**If you add anything to the inner loop, profile it.** `eom3dof` is called
~4× per step × 50 steps/s × run length.

---

## 7. Software gotchas already found and fixed

Listed because the same class of bug will recur if you extend this. All of these
produced *plausible-looking wrong answers*, not crashes.

| bug | symptom | root cause |
|---|---|---|
| Resistance applied as `−R_total` regardless of direction | boat accelerated the wrong way under a following current | drag must carry the sign of water-relative surge |
| Final log sample never populated | steady-state checks read a zeroed rudder angle and reported **inverted moment signs** | off-by-one in the logging loop |
| Rate-loop gain applied in angle units, multiplied by `Iz`, then divided by `dN/dδ` in the allocator | overshoot climbed 9% → 60% across the speed range | double-counted a factor of `u²` |
| Cross-track scored the boat sailing *past* the finish | median error 4 m → 19 m; every config looked equally bad | 30 s run covers 240 m of a 213 m course |
| Course progress computed by *nearest leg* | a boat that **completed** the course scored 353 m of "cross-track error" | wrong on a closed circuit — progress collapses when the boat returns to leg 1 |
| `sin(8π)` ≠ 0 | finish never detected on the circle | exact loop-closing test appended a duplicate point, creating a 1e-13 m leg; `prog >= L_course` then failed by one ulp |
| Corner mask = `2 × median(leg length)` | `NaN` straight-leg error | evaluates to 800 m on a course with 400 m legs |
| Forced-ventilation fault cured itself instantly | fault had no effect | `rudderForces` re-evaluated the latch and cleared it; needed a sentinel value |
| Fault injected at t=12 s | corner transient and fault response superimposed | 12 s lands exactly on a waypoint |

**The pattern:** almost every one was a *measurement* bug, not a physics bug.
When a result looks dramatic, check the metric before believing it.

---

## 8. What is done, and what is not

**Done and validated:** parameters + uncertainty framework; hull steady state
(validated to ~6% against the one logged data point); rudder model with
ventilation and cavitation; 3-DOF EOM; actuator dynamics; sensor models; roll
envelope; full control cascade; Nomoto system ID; rudder sizing sweep; servo
spec; gain schedule; Monte Carlo robustness; failure modes; full 4-lap course
sim; ArduPilot Rover parameter export.

**Not done / open:**

- **No hardware-in-the-loop path.** If you want to test the real autopilot
  against this plant, you'd need to expose `eom3dof` over a socket or via MAVLink
  SITL. Nothing in the design prevents it; it just doesn't exist.
- **`autopilot_params.txt` has never been validated on hardware.** Start at 50%
  of the P gains.
- **ArduPilot Rover has no native gain scheduling** on the steering rate loop.
  The `1/U²` schedule needs a Lua script, a constant cruise speed, or acceptance
  of ~3× worse low-speed overshoot. The table is in the params file.
- **Rule 21 compliance is not implemented.** The competition requires the kill
  switch to engage on missing or corrupted navigation data. The sim currently
  dead-reckons through a GPS dropout. See `params/params_competition.m`.
- **The Monte Carlo does not reach 95% pass** at any rudder size or bandwidth.
  That is a finding, not a defect — see `summary.md` §4b.

---

## 9. UNKNOWN MECHANICAL THINGS — the list you asked for

Every one of these is a physical measurement someone needs to take. They are
ordered by **how much the answer changes**. Several are an hour's work with a
ruler or a load cell and would collapse more uncertainty than any amount of
further simulation.

Code references: `ASSUMPTIONS.md` entry IDs, and the `<<FILL>>` / `VERIFY`
markers in `params/`.

### Tier 1 — blocks a design decision right now

| # | Unknown | Why it matters | How to get it | Effort |
|---|---|---|---|---|
| 1 | **Rudder stock chordwise position** (A10) | *Decides whether the servo spec closes at all.* Stock at the leading edge → 45 kg·cm at 35.8 m/s. Stock balanced at 25% chord → ~0. Same blade, same speed. If the stock sits **aft** of the centre of pressure the blade is overbalanced and will slam to the stop — a stability problem, not just a torque one. | Measure where the shaft axis intersects the blade, as a fraction of chord aft of the leading edge. Callipers. | 10 min |
| 2 | **Rudder tiller arm radius** (A9) | Scales required servo torque **linearly**. The servo horn radius (0.015 m) was given; the rudder-side arm was not. Currently assumed 1:1. | Measure from the stock axis to the cable attachment point. | 10 min |
| 3 | **Yaw inertia `Izz`** (A1) | The single largest unknown in the model, swept ±50%. Drives turn response and therefore the whole low-speed sizing argument. Likely **under**-estimated: mass is in two outboard sponsons. | **Bifilar pendulum**: hang the boat from two parallel lines, twist it, time 20 oscillations. Standard formula. | 1 h, bench |
| 4 | **Servo no-load speed, deadband, PWM update rate** (`params_actuators.m` FILL) | The rate limit is what actually caps achievable derivative gain. Currently assumed 0.10 s/60°. | AGFRC A81FHM HV datasheet, or bench-test with a protractor and a phone camera. | 30 min |

### Tier 2 — changes numbers, not decisions

| # | Unknown | Why it matters | How to get it | Effort |
|---|---|---|---|---|
| 5 | **CG height above the planing surface `h_cg`** (A14) | Sets the whole roll/blow-over envelope (`a_y_crit = g·y_hull/h_cg`, currently 2.49 g). Non-binding by ~3× at present, so it would have to be badly wrong to matter. | CAD, or balance the boat on a knife edge. | 30 min |
| 6 | **Submerged rudder fraction at planing trim** (`params_rudder.m` VERIFY) | Assumed 50% (0.075 m of a 0.150 m blade). **This is the sizing variable** — see §1 of `summary.md`. If the real submerged span differs, the whole sizing conclusion shifts with it. | Photograph the transom at speed, or measure the waterline on the blade after a run. | 1 run |
| 7 | **Lateral projected area above waterline `A_lateral`** (A7) | Drives the crosswind disturbance directly. Assumed 0.30 m². | CAD projection, or photograph the boat side-on against a scale. | 20 min |
| 8 | **Prop lateral separation `y_p`** (`params_vessel.m` VERIFY) | Sets differential-thrust authority, which is what rescues low-speed control and a rudder jam. Assumed 0.598 m. Should equal `2 × y_hull` — worth checking they agree. | Tape measure. | 5 min |
| 9 | **Graupner K-series 76 mm prop `KT(J)` data**, especially the zero-thrust advance ratio `J0` (A8) | `J0` is **load-bearing**: at 0.9 the boat cannot exceed 26.4 m/s, so the 35.8 m/s design point would be unreachable. Currently assumed 1.35. Affects the allocator crossover speed (±30%). | Manufacturer open-water data, or a static thrust test plus a top-speed run. | varies |
| 10 | **`x_cg` travel range on the payload rails** (A6/uncertainty) | Assumed 25–35% LOA forward of the transom. | Measure the rail end stops. | 10 min |

### Tier 3 — needs on-water testing (see `summary.md` §7)

| # | Unknown | Note |
|---|---|---|
| 11 | **Hull manoeuvring derivatives `Yv, Yr, Nv, Nr`** (A11) | Swept ±5×. The Monte Carlo shows **yaw damping `Nr` is the dominant driver of tracking failure**. A 20°/20° zig-zag at three speeds identifies `K′` and `T′` directly. |
| 12 | **Added mass `Y_v̇`** (A12) | Determines the Munk moment's **magnitude and sign**. Currently the sign is undetermined — the model cannot say whether the boat is directionally divergent in sideslip. A drift/sideslip run measures it. |
| 13 | **Ventilation onset angle and depth of loss** (A5) | Bounds maximum useful deflection, which is what the allocator clamps to. Steady turn circles at 5/10/15/20° rudder will show the non-monotonic authority directly. |
| 14 | **Hull resistance curve** (A2) | Only one data point exists (13.4 m/s, matched to ~6%). A coast-down from top speed gives the whole curve. |
| 15 | **Hinge moment vs speed and deflection** | Settles #1 and #2 empirically regardless of geometry. Inline load cell on the steering cable. |

### Tier 4 — design questions, not measurements

| # | Question | Note |
|---|---|---|
| 16 | **Is the 35.8 m/s (80 mph) design speed reachable at all?** | Independent of prop assumptions, the hull model says it needs ~10.1 kW effective ⇒ ~16 kW electrical at the 63% chain efficiency implied by your own log ⇒ **~180 A per motor against the 45 A logged**. The powertrain looks ~4× short. Worth reconciling with whatever the design report assumed. |
| 17 | **Design report Appendix A.1** (A2) | The hull sizing routine was never available; Savitsky (1964) was implemented independently instead and anchored to the logged data point. If A.1 turns up, `hullSteadyState.m` is the only file to replace — the returned struct is the whole interface. |

---

## 10. Running SITL with a virtual flight controller in Mission Planner

This is the natural next step, and it is the only way to test the **real
autopilot code** rather than the simplified cascade in `control/`. There are two
levels of ambition and they cost very different amounts.

### Level 1 — stock ArduRover SITL, no MATLAB (half a day)

Get Mission Planner talking to a virtual Rover with ArduPilot's own built-in
physics. The plant is a generic skid-steer ground rover, **not this boat**, so
nothing it says about tracking performance is meaningful. What it *does* test is
everything above the physics:

- Mission upload from Mission Planner, waypoint sequencing, `WP_RADIUS` behaviour
- Mode switching, arming, pre-arm checks, the failsafe tree
- Parameter handling — you can load `results/autopilot_params.txt` and see what
  ArduPilot rejects or clamps
- **Rule 21 compliance**: the competition requires the kill switch to engage on
  missing or corrupted navigation data. This is where you build and test that
  logic (GPS failsafe → `FS_GCS_ENABLE`, `FS_EKF_ACTION`, or a Lua script).

```bash
# WSL or Linux; Mission Planner on Windows connects over UDP
sim_vehicle.py -v Rover -f rover --console --map --out=udp:<windows-ip>:14550
```

Then in Mission Planner: *Connect → UDP → port 14550*.

Honestly, **do this first regardless**. Most of the flight-code work is here,
and none of it needs the MATLAB model.

### Level 2 — this MATLAB model as ArduPilot's physics backend (1–2 weeks)

ArduPilot SITL supports an **external physics backend over UDP using a JSON
protocol**, which is exactly the hook needed. ArduPilot sends servo PWM; your
simulator replies with vehicle state. That puts the real ArduRover navigation
and steering controllers in the loop against the plant in `model/`.

```bash
sim_vehicle.py -v Rover -f rover --model JSON:127.0.0.1 --console --map
```

#### The protocol (verified against ArduPilot master)

**ArduPilot → physics, UDP port 9002**, binary little-endian:

```c
uint16 magic = 18458;    // 29569 if SERVO_32_ENABLE = 1
uint16 frame_rate;       // = SIM_RATE_HZ
uint32 frame_count;
uint16 pwm[16];          // microseconds, 1000-2000
```

**Physics → ArduPilot**, newline-delimited plain-text JSON. Required fields:

| field | units | frame |
|---|---|---|
| `timestamp` | s | absolute physics time |
| `imu.gyro` `[roll,pitch,yaw]` | rad/s | body |
| `imu.accel_body` `[x,y,z]` | m/s² | body, **specific force** (includes −g) |
| `position` `[n,e,d]` | m | earth NED |
| `velocity` `[n,e,d]` | m/s | earth NED |
| `attitude` `[roll,pitch,yaw]` | rad | — (or `quaternion`) |

Optional and useful here: `windvane.direction` / `windvane.speed`,
`velocity_wind`, `battery.voltage` / `battery.current`, `rc.rc_1`…`rc_12`.

Minimal example ArduPilot accepts:

```json
{"timestamp":2500,"imu":{"gyro":[0,0,0],"accel_body":[0,0,0]},"position":[0,0,0],"attitude":[0,0,0],"velocity":[0,0,0]}
```

#### MATLAB-side networking — one gotcha, already resolved

**`udpport` is Instrument Control Toolbox, and this machine does not have it**
(`license('test','instr_control')` returns 0). The repo's no-toolbox rule holds,
so use Java, which is in base MATLAB:

```matlab
sock = java.net.DatagramSocket(9002);
sock.setSoTimeout(100);                       % ms
buf  = zeros(1, 1024, 'int8');
rx   = java.net.DatagramPacket(buf, 1024);
sock.receive(rx);
raw  = typecast(rx.getData(), 'uint8');
hdr  = typecast(raw(1:8), 'uint16');          % hdr(1) = 18458 magic
pwm  = typecast(raw(9:40), 'uint16');         % 16 channels
```

**Both halves of this are verified working on this machine** — a Java UDP
loopback and the `typecast` decode of the servo packet (magic 18458,
frame_rate 50) were tested directly. No toolbox needed.

#### The real work: impedance mismatches

The protocol is the easy part. These are what will actually take the time:

1. **This is a 3-DOF model; ArduPilot expects 6-DOF state.** There is no roll or
   pitch anywhere in `eom3dof`. Simplest honest approach: send
   `attitude = [0, 0, psi]` and accept that SITL cannot see the blow-over /
   sponson-unloading failure modes — which `model/rollEnvelope.m` covers
   separately as a quasi-static check. Do **not** synthesise a fake roll unless
   you are prepared to defend it.

2. **`accel_body` is specific force, not acceleration.** For a level boat:

   ```
   accel_body = [ udot - v*r ,  vdot + u*r ,  -9.80665 ]
   ```

   `udot`/`vdot` come straight out of `eom3dof`. Getting the −g wrong is the
   classic way to make ArduPilot's EKF diverge on the first arm.

3. **Frame mapping is direct but check it.** The repo already uses X = north,
   Y = east, `psi` clockwise from north, z down. So
   `position = [X, Y, 0]`, `velocity = [Xdot, Ydot, 0]`, `gyro = [0, 0, r]`.

4. **PWM → physical inputs.** Decode per Rover's `SERVOn_FUNCTION`. Default is
   `SERVO1_FUNCTION = 26` (GroundSteering) and `SERVO3_FUNCTION = 70`
   (Throttle); map 1000–2000 µs linearly to `±P.R.delta_max` and to
   ±full thrust.

5. **⚠ ArduPilot has no equivalent of `control/allocate.m`.** The repo's
   allocator sends the yaw demand to the rudder first and overflows the
   shortfall into differential thrust — that is what rescues low-speed control
   and (below 5 m/s) a rudder jam. Rover gives you *either* a steering servo
   plus common throttle, *or* skid steering (`SERVO1_FUNCTION = 73`
   ThrottleLeft, `SERVO3_FUNCTION = 74` ThrottleRight) — not a blended
   rudder-plus-differential allocator. **Reproducing the allocator needs a Lua
   script or a custom mixer, and this is the single biggest gap between the
   simulated controller and what ArduPilot will actually do.** Decide the
   architecture before tuning anything.

6. **Lockstep saves you.** ArduPilot SITL waits for the JSON reply, so MATLAB
   does **not** need to run in real time — a slow physics step just slows the
   simulated clock. Set `SIM_RATE_HZ` at or above the vehicle loop rate (Rover
   defaults to 50 Hz, which matches this repo's `dt = 0.02` exactly). Use
   `SIM_SPEEDUP` if you want it faster than wall-clock.

7. **Restructure the loop, don't reuse `simOsprey`.** `simOsprey` owns its own
   time loop and calls a controller. Under SITL the *controller owns the clock*
   and calls you. Write a thin `sitlBridge.m` that holds `x` and `P`, and on
   each received packet does one RK4 step of `eom3def` and replies. Keep the
   ventilation latch and backlash handling from `simOsprey` — they are stepper
   state and will be silently lost otherwise.

#### What Level 2 buys you that the MATLAB cascade cannot

- The **real ArduPilot L1 navigation controller**, not the LOS guidance in
  `control/losGuidance.m`
- Whether the **`1/U²` steering-gain schedule** can actually be implemented on
  Rover, which has no native gain scheduling — this is where you test the Lua
  script
- Real mission execution: four 0.5-mile laps as an actual waypoint mission, with
  real waypoint acceptance behaviour
- Failsafe interaction with the Rule 21 GPS-loss kill

#### What it still will not tell you

Nothing about roll, blow-over or sponson unloading (no roll DOF). Nothing about
real sensor error characteristics — ArduPilot SITL synthesises its own sensors
from the truth state you send, so `sensors/sensorModel.m` is bypassed entirely.
And nothing about the hull derivatives, which remain uncertain by ±5× no matter
whose controller is driving.

#### Effort

| task | estimate |
|---|---|
| Level 1: stock SITL + Mission Planner + mission upload | 0.5 day |
| UDP bridge, packet decode, JSON encode | 1 day |
| `sitlBridge.m` — state ownership, RK4 step, latch/backlash carry-over | 1–2 days |
| PWM mapping and Rover servo-function configuration | 0.5 day |
| Allocator architecture decision + Lua mixer (item 5) | 2–4 days |
| Debugging EKF convergence, frames and units | 2–3 days |

ArduPilot ships reference implementations in `libraries/SITL/examples/JSON/`
(Python and C++). Crib the Python one — it is the closest structural match to
what `sitlBridge.m` needs to do.

---

## 11. Where to start

1. Run `run_tests`. If anything fails, stop and fix that first.
2. Read `results/summary.md` — especially §2 (which carries a **superseded**
   warning) and §4d (the real course).
3. Read `ASSUMPTIONS.md`. Every number that drives a conclusion has an entry
   with its reason and its expected error direction.
4. Chase Tier 1 of §9. Items 1 and 2 are twenty minutes with callipers and they
   decide whether the existing servo is adequate.
5. Stand up **Level 1 SITL** (§10). It is half a day, needs nothing from the
   MATLAB model, and most of the flight-code work lives there — including the
   Rule 21 GPS-loss kill, which is a competition requirement and is currently
   not implemented anywhere.

If you only do two things: **measure the rudder stock position** (§9 item 1) and
**decide the rudder-vs-differential-thrust allocation architecture** (§10 item
5). Those two block more downstream work than anything else in this repo.

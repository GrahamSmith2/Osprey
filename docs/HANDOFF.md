# Osprey USV Simulation — Handoff

**For:** an engineer taking over the simulation and/or writing the flight code.
Assumes you are comfortable with software but **not** that you know planing-hull
hydrodynamics. Everything domain-specific is explained where it matters.

**Repo:** https://github.com/GrahamSmith2/Osprey — local working copy at
`C:\Osprey`, branch `main`
**Language:** Python 3.11 (numpy, lupa, matplotlib, pytest). The autopilot is a
**Lua 5.4 script** using ArduPilot's scripting API. The original MATLAB model is
still in the repo, **frozen**, as the reference the parity tests check against
(`python/PORTING.md`).

---

## 1. What this is, in one paragraph

Osprey is a 7 ft, 44 kg twin-motor electric planing catamaran USV that must run
a 2-mile autonomous course (two marks 0.25 mile apart, run as an oval, from a
standing start) at the ASNE/ONR **PEP Autonomy Division**. This repo is a 3-DOF
manoeuvring simulation built to answer three design questions — **how big
should the rudder be**, **what servo is needed**, and **what is the structure of
the heading-control gain schedule** — and a test bench that flies the real Lua
autopilot script against the simulated boat. It is explicitly *not* built to
predict the boat's trajectory accurately — the hull hydrodynamic coefficients
are unknown to within ±5×, and the whole thing is structured so that
conclusions survive that.

**Read `README.md` before trusting any number.** The short version: standard
ship manoeuvring-derivative regressions are fitted at Froude number < 0.3; this
boat runs at Fn 2.9–7.8. Every hull derivative here is a placeholder swept ±5×.

---

## 2. Run it

```bash
cd python
```

```bash
python -m pytest tests -q
```

```bash
python -m osprey run controllers/oval_autopilot.lua
```

```bash
python -m osprey report
```

- `pytest` — 49 checks, ~15 s. **Run these before trusting anything.**
- `run` — one race, ~2 s. Prints a scorecard; writes `runs/<script>.csv` and
  `.png`. `--help` lists wind, current, faults, GPS dropout, rudder, speed.
- `report` — every study, ~10 min on 8 cores. Writes `results/python/report.md`,
  `monte_carlo.csv` and figures.
- `size`, `nomoto` — the rudder comparison and the Nomoto fit on their own.

**`results/summary.md` is the deliverable.** It carries every conclusion with
its caveats. This handoff is about the *code*; that document is about the
*findings*. `results/python/report.md` is the machine-written source of every
number in it.

---

## 3. Architecture

Data flows one way. No globals, no file I/O inside the simulation loop.

```
osprey/params.py   ->  Params (one frozen object)  ->  osprey/dynamics.py  ->  state derivative
  Vessel, Environment,                                   hull.py, rudder.py,
  Rudder, Actuators,                                     propulsion.py, environment.py
  UNCERTAINTY, sample_uncertainty, build_params
                                                         sim.py        fixed-step RK4 + stepper state
                                                         sensors.py    what the controller sees
                                                         lua_bridge.py runs a Lua script as the controller
                                                         race.py       course, scoring, controller constants
                                                         studies.py    Monte Carlo, faults, envelopes
                                                         report.py     writes results/python/
```

### `Params` is the single contract

`build_params(S, rudder=..., top_speed=...)` assembles **everything** the model
needs into one frozen dataclass. Model functions take `P` and nothing else. That
is what makes a Monte Carlo trivial: vary `S`, and the entire model follows.

```python
P = build_params()                                          # nominal, as-built rudder
P = build_params(rudder=PROPOSED, top_speed=55 * MPH)       # the race boat
P = build_params(sample_uncertainty("lhs", 60, 12345)[i])   # one Monte Carlo draw
P = P.with_env(V_wind=7.7, psi_wind=math.pi / 2)            # same boat, different day
P = P.with_rudder(Rudder("test", h_sub=0.1, chord=0.03, span_total=0.1))
```

Sub-objects: `P.V` vessel, `P.E` environment, `P.R` rudder, `P.A` actuators,
`P.S` the uncertainty draw (kept attached for traceability), `P.HT` a
precomputed hull table (§6).

### Simulation loop

```
simulate(x0, tf, controller, P, dt=0.005, f_ctrl=50, delay_n=1, fault=None, stop_when=None)
  └─ fixed-step RK4
     ├─ controller tick at 50 Hz (zero-order hold), one-tick compute delay
     ├─ eom(t, x, cmd, vent, P, delta_blade) -> xdot
     │    ├─ hull_forces      sway/yaw damping, resistance, turn drag  (table lookup)
     │    ├─ rudder_forces    lift, ventilation, hinge moment, cavitation number
     │    ├─ prop_forces      thrust -> surge + differential yaw
     │    └─ env_forces       wind side force, wave yaw moment
     └─ three things live in the STEPPER, not the derivative (below)
```

**State vector (9):** `[u, v, r, X, Y, psi, T_port, T_stbd, delta]` —
body-frame surge/sway/yaw rate, earth position (north, east), heading, two
thrust states (ESC spool-up lag), one servo state.

A controller is anything with `step(t, x) -> (delta_cmd, T_port, T_stbd)`.
`LuaController` is one; a constant command dict is the open-loop case.

### Why fixed-step RK4 and not an adaptive solver

The controller is a discrete, fixed-rate device with a one-tick compute delay.
An adaptive solver chooses its own step and will step over a controller tick,
silently smoothing the zero-order hold and the delay into something the real
autopilot never does. Digital phase lag is a first-order effect on achievable
bandwidth, so it has to be simulated as what it is.

### Three things deliberately live in the stepper

These are all **non-differentiable**, so they cannot live inside a state
derivative without corrupting the integrator:

1. **Ventilation latch** — a discrete state, held **fixed across all four RK4
   stages** and updated once per step.
2. **Cable backlash** — hysteresis between servo output `x[8]` and the blade
   angle the water sees. `eom` takes `delta_blade` for this reason.
3. **Controller zero-order hold and the command delay queue.**

### The Lua bridge

`LuaController(script, P, waypoints, params=...)` runs the script in an
embedded Lua 5.4 (lupa), behind a prelude that implements the ArduPilot API
subset in `python/LUA_API.md`. The script:

- reads **sensors, not truth** — `sensors.py`: GPS 5 Hz, 150 ms late, 1 m
  noise; gyro noise with a drifting bias; heading with a 2° bias;
- writes normalised outputs through `SRV_Channels` (rudder 26, port 73,
  starboard 74, both 70);
- schedules itself by returning `update, ms`;
- gets vehicle constants through `param:get` (`race.controller_params`, always
  from the **nominal** boat, never the plant being flown);
- runs under an instruction budget; a runtime error **disarms** the boat.

### Control cascade (`controllers/oval_autopilot.lua`)

```
waypoints -> LOS guidance        -> course cmd, crab-corrected -> heading cmd
          -> heading PI           -> yaw-rate cmd (capped at 0.6 rad/s)
          -> yaw-rate PI          -> yaw-moment demand [N*m]   (moment domain)
          -> ONE actuator by GPS speed:
               < 10 mph   differential thrust, rudder centred
               >= 10 mph  rudder only, clamped at ventilation onset (10 deg)
               hands back to the motors below 9 mph (hysteresis)
speed PI -> common throttle;  GPS lost > 500 ms -> disarm (PEP Rule 21);  finish -> disarm
```

Working in moment units is what makes this simple: the rate-loop gain
`Kp_N = wc·T′·L²·KN/K′` is **speed-invariant**, and the same gain drives either
actuator, so the integrator carries straight across the handover.

---

## 4. Conventions you must not break

- **SI internally, always.** Conversions happen at the boundary (`params.py`,
  the CLI). Every quantity carries its unit in a trailing comment.
- **Body frame at the CG.** x forward, y starboard, z down. `psi` positive
  clockwise from north.
- **Lever arms are positive forward.** The rudder is aft, so `P.x_r < 0`.
- **Added mass is stored NEGATIVE** (SNAME convention), so `(m - X_udot)` reads
  as "mass plus added mass".
- **Positive rudder deflection produces positive yaw moment** (turn to
  starboard). The sign bookkeeping is done **once**, in `rudder.py`. More port
  thrust than starboard also turns to starboard.
- **Anything unknown goes in `UNCERTAINTY` as a RANGE**, never in a model file
  as a constant. This is the most important rule in the repo.
- **Controllers see sensors, not state.** Nothing in `lua_bridge.py` may hand a
  script the true state.

---

## 5. The test suite

`python/tests/` — 49 pytest checks. **They must all pass before any result is
trusted.** Three files:

- `test_physics.py` — the physics sanity checks carried over from
  `tests/run_tests.m`. Several caught real bugs:
  - straight line: zero rudder, zero disturbance → `r ≡ 0` to machine precision;
  - steady-turn closure: `−(m − X_u̇)·u·r + ΣY = 0`, and the yaw budget including
    the Munk moment sums to `(Iz − N_ṙ)·ṙ`. The single best check that the
    equations are consistent;
  - numerical Jacobian **convergence** (halving the step changes it by < 1e-6).
    There is no analytic Jacobian, deliberately: the model has saturations,
    `abs`, a bisection solve and a latched branch, so a closed form would be
    fiction over part of the state space;
  - current is a **frame shift, not a force**;
  - gain-schedule exponents (`Kp_r ∝ 1/U²`);
  - the servo peak is **not** at ventilation onset.
- `test_parity.py` — the port reproduces recorded MATLAB outputs (hull at 8
  speeds, rudder forces, prop calibration, steady turn, Nomoto, span sweep).
- `test_lua.py` — the scripting interface (scheduling, sandbox, errors, budget,
  sensors not truth) and the autopilot: finishes at 50 mph, motors below 10 mph
  and rudder above with no overlap, error does not grow lap to lap, Rule 21 kill
  within 500 ms, heading hold settles on target minus the sensor bias.

---

## 6. Performance notes

- **The hull table.** `hull_solve` does two nested bisections, and `eom` is
  called four times per RK4 step. Every quantity it returns is a smooth function
  of `|u|` for fixed parameters, so it is tabulated once in `build_params`
  (0.02 m/s grid, 0–45 m/s) and interpolated.
- **One race ≈ 2 s** at `dt = 0.01`, dominated by `eom` and the Lua call each
  tick. **The Monte Carlo is process-parallel** (`ProcessPoolExecutor`); 120
  races take ~3 min on 8 cores. Each worker builds its own `Params` and Lua
  runtime. `report.plot_mc_csv` redraws the Monte Carlo figure from the CSV
  without re-running it.
- **If you add anything to the inner loop, profile it.** `eom` runs
  ~4 × 100 × race-seconds times per race.

---

## 7. Software gotchas already found and fixed

Listed because the same class of bug will recur if you extend this. Almost all
produced *plausible-looking wrong answers*, not crashes.

**From the MATLAB version** (the scoring in `race.py` encodes the fixes):

| bug | symptom | root cause |
|---|---|---|
| Resistance applied as `−R_total` regardless of direction | boat accelerated the wrong way under a following current | drag must carry the sign of water-relative surge |
| Final log sample never populated | steady-state checks read a zeroed rudder angle and reported **inverted moment signs** | off-by-one in the logging loop |
| Rate-loop gain applied in angle units, multiplied by `Iz`, then divided by `dN/dδ` | overshoot climbed 9% → 60% across the speed range | double-counted a factor of `u²` |
| Cross-track scored the boat sailing *past* the finish | median error 4 m → 19 m | fixed-length run outlasted the course |
| Course progress computed by *nearest leg* | a boat that **completed** the course scored 353 m of "cross-track error" | wrong on a closed circuit |
| Exact loop-closing test | finish never detected | a duplicate point created a 1e-13 m leg |
| Forced-ventilation fault cured itself instantly | fault had no effect | the rudder model re-evaluated the latch; needs a sentinel value (`vent = 2`) |
| Fault injected on a waypoint | corner transient and fault response superimposed | inject mid-straight (`studies.FAULT_T`) |

**Found by the port** (details in `python/PORTING.md`):

| bug | symptom | root cause |
|---|---|---|
| Command queue one element short | MATLAB applied commands with **zero** delay while documenting one tick | queue length `max(n, 1)` instead of `n + 1` |
| Prop calibrated at loaded mass | +9.0% thrust in every MATLAB run after the hull table was added | the calibration read the loaded-mass table |
| Servo sized at 35° ventilated | under-read the peak hinge moment; "thesis conservative by 46%" was wrong | the peak is below onset (clamped) or near 26° (full travel) |
| `(u > 0) - (u < 0)` as a sign function | `TypeError` on numpy floats | numpy booleans do not subtract; use an explicit branch |
| `g.__osp` inside a class | the Lua bridge table never reached Lua | Python name-mangles `__osp` to `_LuaController__osp`; use `g["__osp"]` |
| Lua `load` result unpacked as a pair | crash on every successful compile | `load` returns one value on success, two on failure |

**The pattern:** almost every one was a *measurement* bug, not a physics bug.
When a result looks dramatic, check the metric before believing it. (The 50 mph
Monte Carlo losses in `summary.md` §4b were checked this way: replayed draws
show the rudder hard over and the boat turning the wrong way, not a scoring
artefact.)

---

## 8. What is done, and what is not

**Done:** parameters + uncertainty framework; hull steady state (validated to
~3% against the one logged data point); rudder model with ventilation and
cavitation number; 3-DOF equations of motion; actuator dynamics; sensor models;
roll/hooking envelope; Nomoto system ID; rudder sizing comparison; servo spec;
Lua scripting bridge; the race autopilot including the **PEP Rule 21 GPS-loss
kill**; Monte Carlo robustness; failure modes; full 2-mile race from a standing
start.

**Not done / open:**

- **Nothing has run on hardware.** The Lua script uses ArduPilot names, but the
  subset and `sim:waypoints()` need porting (`python/LUA_API.md`, last section),
  and it has never run in ArduPilot SITL.
- **No fault response above 10 mph.** By design the motors do not steer above
  10 mph, so a jammed rudder there is unrecoverable (`summary.md` §4c). A
  detector plus "slow below 10 mph and steer on the motors" is the obvious mode.
- **At 50 mph a large share of plausible boats cannot make the roundings**
  (`summary.md` §4b). That is a finding about the uncertainty, and it makes the
  on-water sideslip and zig-zag tests the critical path.
- **Model defects carried over for parity**: hull resistance is ~95 N at zero
  speed; servo deadband is declared but not simulated (`PORTING.md`).
- **Propulsion at 50 mph is assumed**, not modelled from hardware: the prop as
  logged tops out at 28.8 mph loaded.

---

## 9. UNKNOWN MECHANICAL THINGS — the list you asked for

Every one of these is a physical measurement someone needs to take. They are
ordered by **how much the answer changes**. Several are an hour's work with a
ruler or a load cell and would collapse more uncertainty than any amount of
further simulation.

Code references: `ASSUMPTIONS.md` entry IDs, and the `UNVERIFIED` / `NOT
SUPPLIED` / `PLACEHOLDER` comments in `python/osprey/params.py`.

### Tier 1 — blocks a design decision right now

| # | Unknown | Why it matters | How to get it | Effort |
|---|---|---|---|---|
| 1 | **Rudder stock chordwise position** (A10) | *Decides whether the servo spec closes at all.* For the new 45 mm × 5 in blade at 50 mph: stock at the leading edge → ~103 kg·cm, past the 74 kg·cm stall. Stock balanced at 25% chord → ~0 nominal, 41 kg·cm if the centre of pressure is 0.10c off. Same blade, same speed (`python -m osprey size`). If the stock sits **aft** of the centre of pressure the blade is overbalanced and will slam to the stop — a stability problem, not just a torque one. | Measure where the shaft axis intersects the blade, as a fraction of chord aft of the leading edge. Callipers. | 10 min |
| 2 | **Rudder tiller arm radius** (A9) | Scales required servo torque **linearly**. The servo horn radius (0.015 m) was given; the rudder-side arm was not. Currently assumed 1:1. | Measure from the stock axis to the cable attachment point. | 10 min |
| 3 | **Yaw inertia `Izz`** (A1) | The single largest unknown in the model, swept ±50%. Likely **under**-estimated: mass is in two outboard sponsons. | **Bifilar pendulum**: hang the boat from two parallel lines, twist it, time 20 oscillations. Standard formula. | 1 h, bench |
| 4 | **Servo no-load speed, deadband, PWM update rate** (`Actuators`, PLACEHOLDER) | The rate limit is what actually caps achievable derivative gain. Currently assumed 0.10 s/60°. | AGFRC A81FHM HV datasheet, or bench-test with a protractor and a phone camera. | 30 min |

### Tier 2 — changes numbers, not decisions

| # | Unknown | Why it matters | How to get it | Effort |
|---|---|---|---|---|
| 5 | **CG height above the planing surface `h_cg`** (A14) | Sets the whole roll envelope (`a_y_crit = g·y_hull/h_cg`, currently 2.49 g). **At 50 mph the autopilot pulls ~1.35 g in the roundings**, so this is now close to binding. | CAD, or balance the boat on a knife edge. | 30 min |
| 6 | **Submerged rudder depth on plane** (`AS_BUILT.h_sub`) | Assumed 3 in for the as-built blade. **This is the sizing variable**; the new blade is specified as 2 in more than whatever this really is. | Photograph the transom at speed, or measure the waterline on the blade after a run. | 1 run |
| 7 | **Lateral projected area above waterline `A_lateral`** (A7) | Drives the crosswind disturbance directly. Assumed 0.30 m². | CAD projection, or photograph the boat side-on against a scale. | 20 min |
| 8 | **Prop lateral separation `prop_sep`** (VERIFY) | Sets differential-thrust authority, which is all the steering there is below 10 mph. Assumed 0.598 m. | Tape measure. | 5 min |
| 9 | **Prop `KT(J)` data**, especially the zero-thrust advance ratio `J0` (A8) | Sets the thrust ceiling and top speed. The prop as logged gives 28.8 mph loaded; 50 mph needs ~336 N and ~7.5 kW into the water. | Manufacturer data for the new prop, or a static thrust test plus a top-speed run. | varies |
| 10 | **`x_cg` travel range on the payload rails** | Assumed 25–35% LOA forward of the transom. | Measure the rail end stops. | 10 min |

### Tier 3 — needs on-water testing (see `results/summary.md` §7)

| # | Unknown | Note |
|---|---|---|
| 11 | **Hull manoeuvring derivatives `Yv, Yr, Nv, Nr`** (A11) | Swept ±5×. The 50 mph Monte Carlo loses boats with **low yaw and sway damping**. A 20°/20° zig-zag at three speeds identifies `K′` and `T′` directly. |
| 12 | **Added mass `Y_v̇`** (A12) | Determines the Munk moment's **magnitude and sign**. Currently undetermined. A drift/sideslip run measures it. |
| 13 | **Ventilation onset angle and depth of loss** (A5) | Bounds maximum useful deflection, which is what the autopilot clamps to. Steady turn circles at 5/10/15/20° rudder show it directly. |
| 14 | **Hull resistance curve** (A2) | Only one data point exists (13.4 m/s, matched to ~3%). A coast-down from top speed gives the whole curve. |
| 15 | **Hinge moment vs speed and deflection** | Settles #1 and #2 empirically regardless of geometry. Inline load cell on the steering cable. |

### Tier 4 — design questions, not measurements

| # | Question | Note |
|---|---|---|
| 16 | **What propulsion delivers 50 mph?** | The hull model says ~336 N and ~7.5 kW into the water at 50 mph loaded; the logged run drew ~2 kW per motor. The simulation assumes an upgrade that tops out at 55 mph. |
| 17 | **Design report Appendix A.1** (A2) | The thesis hull-sizing scripts. Savitsky (1964) was implemented independently instead and anchored to the logged data point. `hull_solve` in `hull.py` is the only function to replace; the returned dict is the whole interface. |

---

## 10. Running SITL with a virtual flight controller in Mission Planner

This is the natural next step, and it is the only way to test the **real
autopilot firmware** rather than the simulated prelude. The autopilot being a
Lua script makes this much more direct than it was: the same script can run on
SITL. There are two levels of ambition.

### Level 1 — stock ArduRover SITL running the Lua script (1 day)

Get Mission Planner talking to a virtual Rover with ArduPilot's own built-in
physics. The plant is a generic ground rover, **not this boat**, so nothing it
says about tracking performance is meaningful. What it *does* test is
everything above the physics:

- The script itself on real ArduPilot: `SCR_ENABLE = 1`, copy
  `oval_autopilot.lua` into SITL's `scripts/` folder, reboot
- The porting list in `python/LUA_API.md`: replace `sim:waypoints()` with
  `mission:get_item()`, register the `OSP_*` constants with `param:add_table`
- Mission upload from Mission Planner, arming, pre-arm checks, the failsafe tree
- **Rule 21**: the script disarms on GPS loss; check it against
  `FS_EKF_ACTION` and the GCS failsafe so they do not fight

```bash
sim_vehicle.py -v Rover -f rover --console --map --out=udp:<windows-ip>:14550
```

Then in Mission Planner: *Connect → UDP → port 14550*.

**Do this first regardless.** Most of the flight-code work is here.

### Level 2 — this Python model as ArduPilot's physics backend (1 week)

ArduPilot SITL supports an **external physics backend over UDP using a JSON
protocol**. ArduPilot sends servo PWM; your simulator replies with vehicle
state. That puts real ArduRover firmware and the real Lua script in the loop
against the plant in `python/osprey/`. ArduPilot's reference backend
(`libraries/SITL/examples/JSON/`) is itself Python, so this is a thin wrapper.

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

Minimal example ArduPilot accepts:

```json
{"timestamp":2500,"imu":{"gyro":[0,0,0],"accel_body":[0,0,0]},"position":[0,0,0],"attitude":[0,0,0],"velocity":[0,0,0]}
```

Python side, standard library only:

```python
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", 9002))
data, addr = sock.recvfrom(1024)
magic, rate, frame = struct.unpack_from("<HHI", data)   # magic == 18458
pwm = struct.unpack_from("<16H", data, 8)
sock.sendto((json.dumps(state) + "\n").encode(), addr)
```

#### The real work: impedance mismatches

1. **This is a 3-DOF model; ArduPilot expects 6-DOF state.** Send
   `attitude = [0, 0, psi]` and accept that SITL cannot see roll or hooking —
   `studies.roll_envelope` covers that separately as a quasi-static check.
2. **`accel_body` is specific force, not acceleration.** For a level boat:
   `[udot - v*r, vdot + u*r, -9.80665]`, with `udot`/`vdot` from `eom`. Getting
   the −g wrong is the classic way to make the EKF diverge on the first arm.
3. **Frame mapping is direct.** X = north, Y = east, `psi` clockwise from north,
   z down: `position = [X, Y, 0]`, `velocity = [Xdot, Ydot, 0]`,
   `gyro = [0, 0, r]`.
4. **PWM → physical inputs.** Decode per `SERVOn_FUNCTION` exactly as
   `lua_bridge.py` does: 26 → rudder, 73/74 → port/starboard thrust, 1100 /
   1500 / 1900 µs = −1 / 0 / +1.
5. **Restructure the loop, don't reuse `simulate`.** `simulate` owns its clock
   and calls a controller. Under SITL the *firmware owns the clock* and calls
   you. Write a bridge that holds `x` and `P`, and per packet does one RK4 step
   of `eom` and replies. **Keep the ventilation latch and backlash from
   `sim.py`** — they are stepper state and will be silently lost otherwise.
6. **Lockstep saves you.** SITL waits for the reply, so the model need not run
   in real time. Set `SIM_RATE_HZ` at or above the Rover loop rate.

#### What it still will not tell you

Nothing about roll, blow-over or sponson unloading. Nothing about real sensor
error — SITL synthesises its own sensors from the state you send, bypassing
`sensors.py`. And nothing about the hull derivatives, which stay uncertain by
±5× whoever is driving.

#### Effort

| task | estimate |
|---|---|
| Level 1: stock SITL + Mission Planner + the script on it | 1 day |
| UDP bridge, packet decode, JSON encode | 0.5 day |
| Bridge state ownership, RK4 step, latch/backlash carry-over | 1 day |
| PWM mapping and Rover servo-function configuration | 0.5 day |
| Debugging EKF convergence, frames and units | 2–3 days |

---

## 11. Where to start

1. Run `python -m pytest tests -q`. If anything fails, stop and fix that first.
2. Read `results/summary.md` — especially §4b (the 50 mph Monte Carlo) and §4c
   (faults).
3. Read `ASSUMPTIONS.md`. Every number that drives a conclusion has an entry
   with its reason and its expected error direction.
4. Chase Tier 1 of §9. Items 1 and 2 are twenty minutes with callipers and they
   decide whether the servo is adequate for the new blade.
5. Stand up **Level 1 SITL** (§10) with the Lua script on it.

If you only do two things: **measure the rudder stock position** (§9 item 1)
and **get the sideslip and zig-zag data** (§9 items 11–12). The first decides
the servo; the second decides whether 50 mph is controllable at all.

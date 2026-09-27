# MATLAB → Python port

The MATLAB model (`../model`, `../params`, `../analysis`, `../control`,
`../tests`, `../run_all.m`) is **frozen**. It is kept as the reference the
parity tests check against. New work goes in `python/`.

## What maps to what

| MATLAB | Python |
|---|---|
| `params/*.m`, `uncertainty.m`, `sampleUncertainty.m`, `buildParams.m` | `osprey/params.py` |
| `model/hullSteadyState.m`, `buildHullTable.m`, `hullForces.m` | `osprey/hull.py` |
| `model/rudderForces.m` | `osprey/rudder.py` |
| `model/propForces.m`, `propMaxThrust.m` | `osprey/propulsion.py` |
| `model/envForces.m` | `osprey/environment.py` |
| `model/eom3dof.m` | `osprey/dynamics.py` |
| `model/simOsprey.m` | `osprey/sim.py` |
| `model/rollEnvelope.m` | `osprey/studies.py: roll_envelope` |
| `sensors/sensorModel.m` | `osprey/sensors.py` |
| `analysis/identifyNomoto.m`, `control/gainSchedule.m` | `osprey/nomoto.py` |
| `analysis/sizingSweep.m`, `runSizingStudy.m` | `osprey/sizing.py` |
| `analysis/courseGeometry.m` (two-mark oval only), `courseSim.m` | `osprey/course.py`, `osprey/race.py` |
| `analysis/monteCarlo.m`, `failureModes.m`, `roundingStudy.m`, `lapAccumulation.m` | `osprey/studies.py` |
| `control/makeAutopilot.m`, `losGuidance.m`, `allocate.m` | `controllers/oval_autopilot.lua` (a Lua script, run by `osprey/lua_bridge.py`) |
| `analysis/writeAutopilotParams.m` | not ported: the controller is now a Lua script, not an ArduPilot parameter set |
| `run_all.m` | `python -m osprey report` |
| `tests/run_tests.m` | `tests/test_physics.py`, `tests/test_parity.py`, `tests/test_lua.py` |

Circle and triangle course geometries were placeholders and were not ported.

## Parity

`tests/test_parity.py` checks the port against numbers recorded from MATLAB
before the port. All agree to the precision MATLAB printed:

| quantity | points checked |
|---|---|
| hull trim, wetted area, resistance | 8 speeds, 2–35.8 m/s |
| aero lift fraction at 35.8 m/s | 0.36 |
| rudder CL, normal force, yaw moment | 2°, 5°, 8°, 10° at 13.4 m/s |
| ventilated rudder | 12° latched |
| prop calibration KT0, thrust ceiling at 13.4 m/s | |
| steady turn at 8°, 8 m/s: u, v, r, radius | |
| Nomoto K′, T′ | |
| span sweep turn radii | 4 spans |

These use MATLAB's rudder (75 mm submerged, 50% of the blade), not today's.

Every check in `run_tests.m` has a Python counterpart (T1, T3–T12, T14, and
T16 with `heading_hold.lua`). T2 tested an area-override path that no longer
exists; T13 is covered by the Nomoto parity test; T15 tested the MATLAB
allocator, replaced by the speed-split test on the Lua autopilot.

## Bugs found in the MATLAB model

Found while porting. **Not fixed in MATLAB** (frozen). Fixed in Python, with a
switch that reproduces the old behaviour for the parity tests.

1. **The command delay was zero.** `simOsprey.m` documented a one-tick compute
   delay, but its queue was `max(delay_n, 1)` long, so a command computed at a
   tick was applied at that same tick. The MATLAB controller results are
   slightly optimistic on phase margin. Python applies one tick
   (`simulate(..., legacy_zero_delay=True)` reproduces MATLAB).
2. **Prop calibrated at the wrong mass.** After the hull table was added, the
   KT0 calibration read the table built at the *loaded* mass instead of solving
   at the *dry* mass of the logged run. Every MATLAB run after that had 9.0% too
   much thrust. The same mix-up is in the README validation check: 2534 W is the
   loaded-mass figure; the no-payload run the log describes gives 2324 W, closer
   to the ~2400 W logged. (`build_params(legacy_kt0=True)` reproduces it.)
3. **Servo sized at the wrong point.** MATLAB evaluated the hinge moment at one
   point, 35° ventilated. That is not the peak. With travel clamped at
   ventilation onset the peak is just below onset. With full travel, ventilated
   lift keeps growing with angle and peaks near 26°. For the thesis's 80 mph
   case with a leading-edge stock: MATLAB 45.2 kg·cm; correct 64.6 clamped,
   82.7 full travel. So "the thesis's 66 kg·cm is conservative by 46%" was
   wrong; `results/summary.md` §3 carries the correction.

## Known defects carried over, not fixed

Kept so the parity tests stay meaningful. Fixing them changes MATLAB-matched
numbers, so each needs its own decision.

- **Hull resistance is ~95 N at zero speed.** The planing drag term
  `L_hydro · tan(trim)` is applied in displacement mode too, and below 0.5 m/s
  the trim sits at its 12° cap. It is ~25 N from 0.6–5 m/s, where trim is
  ~3.3°. Effects: a standing start is pessimistic; a boat drifting in a current
  chatters about zero water speed, so `test_current_is_a_frame_shift` needs
  dt = 0.002. The right fix is to blend `R_induced` with the planing fraction.
- **Servo deadband and PWM rate** are declared in `Actuators` but not simulated
  (same in MATLAB).

## Changed on purpose

- **Controller.** The MATLAB autopilot sent yaw demand to the rudder first and
  overflowed to differential thrust. `oval_autopilot.lua` uses **one**
  actuator at a time: motors below 10 mph, rudder at 10 mph and above (team
  decision, 2026-09-26). The moment-domain rate loop is unchanged, so the same
  gain drives either.
- **Race.** Standing start (MATLAB's Monte Carlo and fault runs started at
  cruise), 50 mph cruise (MATLAB used 8 m/s), two-mark oval (MATLAB mostly used
  a placeholder circle).
- **Propulsion.** `build_params(top_speed=...)` scales the thrust curve so the
  loaded boat can reach a given top speed, standing in for the ESC/prop
  upgrade. The prop as logged tops out at 28.8 mph loaded.
- **Rudder.** The as-built blade is 3 in submerged (MATLAB: 75 mm), and the
  chosen blade is 45 mm × 5 in.

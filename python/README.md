# Osprey — Python model and Lua autopilot runner

Python port of the MATLAB rudder sizing and manoeuvring model in `../model` and
`../analysis`, plus a runner that flies a **Lua control script** around the PEP
two-mark oval. The script uses ArduPilot's scripting API names, so what works
here has a path onto the real autopilot.

```bash
pip install -r requirements.txt
```

```bash
python -m osprey size
```

```bash
python -m osprey run controllers/oval_autopilot.lua
```

Run from this `python/` folder. Python 3.11, tested with numpy 2.4, lupa 2.x
(Lua 5.4), matplotlib 3.11, pytest 9.

## Commands

| command | what it does |
|---|---|
| `size` | Compares as-built, 5 in × 30 mm and the chosen 45 mm × 5 in rudder: authority, turn radius, drag, cavitation ceiling, servo torque by stock position. Reconciles the thesis's 66.8 kg·cm. |
| `nomoto [--rudder R]` | Identifies Nomoto K′, T′ for a rudder and prints the gain schedule. |
| `run SCRIPT.lua [options]` | Flies the script around the course and prints a scorecard: finish time, cross-track, closest and widest approach to each mark, lap-by-lap RMS, lateral g, rudder use, servo load, which actuator steered. Writes `runs/<script>.csv` and `.png`. |
| `report [--n 60]` | Runs every study (race, rounding radius, Monte Carlo with a cruise-speed sweep, failure modes, turn physics, roll envelope, powertrain) and writes `../results/python/report.md`, `monte_carlo.csv` and figures. ~10 min on 8 cores. |

Main `run` options (see `--help` for all):

| option | default | |
|---|---|---|
| `--rudder` | `proposed` | `proposed` (45 mm × 5 in) or `as-built` (30 mm × 3 in) |
| `--speed` | `50` | cruise speed, **mph**; passed to the script as `CRUISE_SPEED` in m/s |
| `--top-speed` | `55` | scale propulsion so the loaded boat tops out here, mph. `0` = prop as logged |
| `--start-speed` | `0` | m/s. PEP is a standing start; leave at 0 |
| `--radius` / `--separation` / `--distance` | 25 m / 0.25 mi / 2 mi | course |
| `--wind` / `--wind-from` | 0 kt / 90° | steady wind |
| `--current` / `--current-to` | 0 m/s / 180° | uniform current |
| `--fault` | | `rudder_jam:T0:DEG`, `motor_out:T0:1or2`, `ventilation:T0` |
| `--gps-dropout` | | `T0,T1` seconds |
| `--param NAME=VALUE` | | override anything the script reads with `param:get` |

## Assumptions to know before reading a result

- **Propulsion at 50 mph is assumed.** The prop is calibrated to the logged
  13.4 m/s run, and with payload it tops out at **28.8 mph**. Holding 50 mph needs
  ~336 N, i.e. 7.5 kW into the water. `--top-speed 55` scales the whole thrust
  curve to stand in for the ESC/prop upgrade. Static thrust scales with it, so
  launch acceleration is optimistic.
- **50 mph is past the rudder model's 44 mph cavitation ceiling.** Rudder forces
  there are extrapolated.
- **3-DOF only**: no roll, no chine trip. At 50 mph the autopilot pulls ~1.3 g in
  the roundings; the model cannot say whether the hull survives that.
- **Hull resistance is 95 N at zero speed** (inherited MATLAB defect, see
  `PORTING.md`). Launch from rest is pessimistic on this count.
- **The script sees sensors, not truth**: GPS at 5 Hz, 1 m noise, 150 ms late;
  gyro noise with drifting bias; heading with a 2° bias.
- Everything in `../ASSUMPTIONS.md` still applies. Hull derivatives are
  placeholders swept ±5×.

## Controllers

- `controllers/oval_autopilot.lua` — the race autopilot. LOS guidance → heading
  PI → yaw-rate PI in moment units → **differential thrust only below 10 mph,
  rudder only at 10 mph and above** (hands back below 9 mph), speed PI, PEP Rule
  21 kill on GPS loss, disarm at the finish.
- `controllers/heading_hold.lua` — minimal example: hold a heading, fixed
  throttle.

Writing your own: `LUA_API.md`.

## Tests

```bash
python -m pytest tests -q
```

49 checks, ~15 s. `test_parity.py` checks the port against recorded MATLAB
outputs, `test_physics.py` the physics sanity checks from
`../tests/run_tests.m`, and `test_lua.py` the scripting interface and the race
autopilot.

## Layout

```
osprey/params.py      every constant, the rudders, the uncertainty ranges
osprey/hull.py        Savitsky planing + displacement blend, sway/yaw damping
osprey/rudder.py      rudder forces, ventilation, hinge moment, cavitation
osprey/propulsion.py  thrust ceiling, calibration, top-speed sizing
osprey/dynamics.py    3-DOF equations of motion
osprey/sim.py         fixed-step RK4 with controller hold, delay, backlash, faults
osprey/sensors.py     GPS / gyro / heading models
osprey/course.py      two-mark oval, progress and cross-track
osprey/nomoto.py      Nomoto identification and gain schedule
osprey/sizing.py      rudder comparison and servo tables
osprey/lua_bridge.py  runs a Lua script as the controller
osprey/lua/prelude.lua  the ArduPilot-style API seen by scripts
osprey/race.py        one race: course, scoring against the true track, plots
osprey/studies.py     Monte Carlo, failure modes, rounding radius, roll envelope
osprey/report.py      `report`: every study -> ../results/python/
```

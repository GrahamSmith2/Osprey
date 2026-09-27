# Lua control API

A control script is an ordinary ArduPilot-style Lua script. It runs under Lua
5.4, the version ArduPilot uses, and it is scheduled the ArduPilot way: the
chunk returns `update, ms`, and each call to `update` returns the next function
and delay. Return nothing to stop the script.

```lua
function update()
  local r = ahrs:get_gyro():z()
  SRV_Channels:set_output_norm(26, -0.5 * r)     -- rudder
  SRV_Channels:set_output_norm(70, 0.4)          -- both motors
  return update, 20                              -- again in 20 ms
end
return update, 20
```

The sim ticks the controller at `--rate` (50 Hz). A shorter delay than one tick
runs at one tick, with a warning.

## Reading the boat

All angles are radians, except the one ArduPilot itself gives in degrees.

| call | returns |
|---|---|
| `millis()` / `micros()` | time since start as `uint32_t`; use `:tofloat()` for maths |
| `ahrs:get_yaw()` | heading, rad, −π..π, 0 = north, positive clockwise (biased) |
| `ahrs:get_gyro()` | `Vector3f`; `:z()` is yaw rate, rad/s (noise + drifting bias) |
| `ahrs:get_roll()`, `get_pitch()` | 0 (3-DOF model) |
| `ahrs:get_relative_position_NED_home()` | `Vector3f` north/east of the start, m, or **nil without GPS fix** |
| `ahrs:get_velocity_NED()` | `Vector3f`, m/s, or nil without fix |
| `ahrs:groundspeed_vector()` | `Vector2f` north/east velocity, m/s |
| `gps:status(0)` | 3 = 3D fix, 1 = no fix; compare with `gps.GPS_OK_FIX_3D` |
| `gps:ground_speed(0)` | m/s |
| `gps:ground_course(0)` | **degrees**, 0..360, as ArduPilot |
| `gps:num_sats(0)` | 14 with fix, 0 without |

Position and velocity come from GPS: 5 Hz, 150 ms late, 1 m noise. Heading and
yaw rate come from the IMU. Nothing gives the script the true state.

## Driving the boat

Normalised outputs are −1..1. Servo function numbers follow ArduPilot:

| actuator | function | channel | plant |
|---|---|---|---|
| rudder | 26 GroundSteering or 94 Script1 | SERVO1 | angle = norm × mechanical stop (35°) |
| port motor | 73 ThrottleLeft or 95 Script2 | SERVO3 | thrust = norm × available thrust at current speed |
| starboard motor | 74 ThrottleRight or 96 Script3 | SERVO4 | same; reverse limited to 30% |
| both motors | 70 Throttle | | sets port and starboard |

| call | |
|---|---|
| `SRV_Channels:set_output_norm(fn, v)` | set an output, −1..1 |
| `SRV_Channels:set_output_pwm(fn, pwm)` | same in PWM; 1100 / 1500 / 1900 = −1 / 0 / 1 |
| `SRV_Channels:get_output_pwm(fn)` | current PWM |
| `SRV_Channels:find_channel(fn)` | channel index for a function (0-based) |
| `SRV_Channels:set_output_pwm_chan_timeout(ch, pwm, ms)` | override a channel for `ms`, then revert to 0 |

Outputs hold until changed. Positive rudder turns the boat to starboard. More
port thrust than starboard also turns to starboard.

## Everything else

| call | |
|---|---|
| `gcs:send_text(severity, text)` | message to the console and the run log; 0 = emergency … 6 = info |
| `param:get(name)` | a value from the runner, or nil (see below) |
| `param:set(name, v)` | store a value |
| `arming:is_armed()`, `arm()`, `disarm()` | disarmed = zero outputs |
| `Vector3f()`, `Vector2f()` | vectors with `:x()`, `:x(v)`, `+ - *`, `:length()`, `:normalize()` |
| `sim:waypoints()` | **simulator only**: the course as `{ {n=, e=}, ... }`, m from the start |
| `print(...)` | debugging only; ArduPilot has no `print` |

## Parameters the runner provides

`python -m osprey run` fills these for the selected rudder and cruise speed.
Override any of them with `--param NAME=VALUE`.

| name | meaning |
|---|---|
| `CRUISE_SPEED` | cruise speed, m/s (`--speed` converted from mph) |
| `CRUISE_THROTTLE` | throttle %, feed-forward at cruise |
| `OSP_NOMOTO_K`, `OSP_NOMOTO_T` | Nomoto K′, T′ identified for this rudder |
| `OSP_LOA` | length overall, m |
| `OSP_RUD_KN` | rudder yaw moment = `KN · U² · δ`, N·m per (m/s)²·rad |
| `OSP_RUD_MAX_DEG` | rudder angle at output 1.0 (mechanical stop) |
| `OSP_STEER_LIM_DEG` | ventilation onset; do not command past this |
| `OSP_THR_N` | thrust per motor at full throttle and zero speed, N |
| `OSP_THR_U0` | speed at which the prop's thrust runs out, m/s |
| `OSP_YP` | motor lateral offset from centreline, m |
| `OSP_WC_RATE`, `OSP_RATE_MAX` | yaw-rate loop crossover, rad/s; yaw-rate command cap, rad/s |
| `OSP_GPS_LOSS_MS` | GPS-loss grace before the Rule 21 kill |

`oval_autopilot.lua` also reads `OSP_SPLIT_MPH` (10), `OSP_SPLIT_HYST_MPH` (1)
and `OSP_DTHR_MAX` (0.5) if given.

## Sandbox

`io`, `os`, `require`, `dofile`, `loadfile`, `package` and `debug` are removed,
as on ArduPilot. Each call has an instruction budget (`--budget`, default
100 000). An infinite loop fails with an error instead of hanging the sim. A
runtime error stops the script and **disarms** the boat.

## Moving a script to the autopilot

Replace `sim:waypoints()` with `mission:get_item()` and convert each Location to
a north/east offset from home. Replace `param:get` defaults with
`param:add_table` / `param:add_param`. Delete any `print`. The rest of the calls
above have the same names on ArduPilot. Check the ArduPilot scripting docs for
the firmware version on the boat before trusting that.

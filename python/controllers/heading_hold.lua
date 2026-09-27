-- heading_hold.lua
-- The smallest useful Osprey control script: hold one heading at a fixed
-- throttle. Start here to see the API; oval_autopilot.lua is the full thing.
--
-- Run:  python -m osprey run controllers/heading_hold.lua --tmax 60

local TARGET  = math.rad(param:get("HOLD_HEADING_DEG") or 20)  -- where to point
local RUD_MAX = math.rad(35)   -- servo throw at output 1.0
local LIM     = math.rad(10)   -- never command past ventilation onset
local KP      = 0.8            -- rudder rad per rad of heading error
local KD      = 0.3            -- rudder rad per rad/s of yaw rate (damping)
local THR     = 0.45           -- common throttle, 0..1

local last_report = millis()

local function wrap(a) return (a + math.pi) % (2 * math.pi) - math.pi end

function update()
  local err = wrap(TARGET - ahrs:get_yaw())
  local r   = ahrs:get_gyro():z()                  -- yaw rate from the IMU
  local rud = math.max(-LIM, math.min(LIM, KP * err - KD * r))

  SRV_Channels:set_output_norm(26, rud / RUD_MAX)  -- 26 = GroundSteering
  SRV_Channels:set_output_norm(70, THR)            -- 70 = Throttle, both motors

  if millis() - last_report > 5000 then
    last_report = millis()
    gcs:send_text(6, string.format("hdg err %5.1f deg, speed %4.1f m/s",
                                   math.deg(err), gps:ground_speed(0)))
  end
  return update, 20                                -- call again in 20 ms
end

return update, 100                                 -- first call after 100 ms

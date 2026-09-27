-- Osprey simulator: ArduPilot-style scripting API for Lua control scripts.
--
-- Loaded before the user's script. Everything here is backed by the Python
-- bridge through the private table __osp. The binding NAMES follow ArduPilot's
-- scripting API so a script developed here has a path onto the real
-- autopilot; the SUBSET implemented is listed in python/LUA_API.md. Anything
-- under `sim` exists only in the simulator.

local osp = __osp

---------------------------------------------------------------------------
-- uint32_t: what ArduPilot's millis() returns. Arithmetic and comparisons
-- work against numbers and other uint32_t; use :tofloat() for maths.
---------------------------------------------------------------------------
local U32 = {}
U32.__index = U32
local function u32(v) return setmetatable({ v_ = math.floor(v) % 4294967296 }, U32) end
local function uv(a) if type(a) == "table" then return a.v_ else return a end end
U32.__add = function(a, b) return u32(uv(a) + uv(b)) end
U32.__sub = function(a, b) return u32(uv(a) - uv(b)) end
U32.__mul = function(a, b) return u32(uv(a) * uv(b)) end
U32.__div = function(a, b) return u32(uv(a) // uv(b)) end
U32.__idiv = U32.__div
U32.__mod = function(a, b) return u32(uv(a) % uv(b)) end
U32.__lt = function(a, b) return uv(a) < uv(b) end
U32.__le = function(a, b) return uv(a) <= uv(b) end
U32.__eq = function(a, b) return uv(a) == uv(b) end
U32.__tostring = function(a) return tostring(a.v_) end
function U32:tofloat() return self.v_ + 0.0 end
function U32:toint() return math.tointeger(self.v_) end
function uint32_t(v) return u32(v or 0) end

function millis() return u32(osp.time_ms()) end
function micros() return u32(osp.time_ms() * 1000) end

---------------------------------------------------------------------------
-- Vector3f / Vector2f: v:x() reads, v:x(val) writes, as in ArduPilot.
---------------------------------------------------------------------------
local V3 = {}
V3.__index = V3
local function v3(x, y, z) return setmetatable({ x_ = x or 0.0, y_ = y or 0.0, z_ = z or 0.0 }, V3) end
function V3:x(v) if v ~= nil then self.x_ = v end return self.x_ end
function V3:y(v) if v ~= nil then self.y_ = v end return self.y_ end
function V3:z(v) if v ~= nil then self.z_ = v end return self.z_ end
function V3:length() return math.sqrt(self.x_ ^ 2 + self.y_ ^ 2 + self.z_ ^ 2) end
function V3:length_squared() return self.x_ ^ 2 + self.y_ ^ 2 + self.z_ ^ 2 end
function V3:dot(o) return self.x_ * o.x_ + self.y_ * o.y_ + self.z_ * o.z_ end
function V3:normalize()
  local l = self:length()
  if l > 0 then self.x_, self.y_, self.z_ = self.x_ / l, self.y_ / l, self.z_ / l end
end
function V3:copy() return v3(self.x_, self.y_, self.z_) end
V3.__add = function(a, b) return v3(a.x_ + b.x_, a.y_ + b.y_, a.z_ + b.z_) end
V3.__sub = function(a, b) return v3(a.x_ - b.x_, a.y_ - b.y_, a.z_ - b.z_) end
V3.__unm = function(a) return v3(-a.x_, -a.y_, -a.z_) end
V3.__mul = function(a, b)
  if type(a) == "number" then return v3(a * b.x_, a * b.y_, a * b.z_) end
  return v3(a.x_ * b, a.y_ * b, a.z_ * b)
end
V3.__tostring = function(a) return string.format("(%.3f, %.3f, %.3f)", a.x_, a.y_, a.z_) end
function Vector3f() return v3() end

local V2 = {}
V2.__index = V2
local function v2(x, y) return setmetatable({ x_ = x or 0.0, y_ = y or 0.0 }, V2) end
function V2:x(v) if v ~= nil then self.x_ = v end return self.x_ end
function V2:y(v) if v ~= nil then self.y_ = v end return self.y_ end
function V2:length() return math.sqrt(self.x_ ^ 2 + self.y_ ^ 2) end
function V2:angle() return math.atan(self.y_, self.x_) end
function V2:dot(o) return self.x_ * o.x_ + self.y_ * o.y_ end
function V2:normalize()
  local l = self:length()
  if l > 0 then self.x_, self.y_ = self.x_ / l, self.y_ / l end
end
function V2:copy() return v2(self.x_, self.y_) end
V2.__add = function(a, b) return v2(a.x_ + b.x_, a.y_ + b.y_) end
V2.__sub = function(a, b) return v2(a.x_ - b.x_, a.y_ - b.y_) end
V2.__unm = function(a) return v2(-a.x_, -a.y_) end
V2.__mul = function(a, b)
  if type(a) == "number" then return v2(a * b.x_, a * b.y_) end
  return v2(a.x_ * b, a.y_ * b)
end
V2.__tostring = function(a) return string.format("(%.3f, %.3f)", a.x_, a.y_) end
function Vector2f() return v2() end

---------------------------------------------------------------------------
-- ahrs: attitude and navigation solution. 3-DOF model: roll = pitch = 0.
-- Heading and yaw rate come from the IMU; position and velocity from GPS
-- (5 Hz, latency, noise). Position calls return nil without a GPS fix.
---------------------------------------------------------------------------
ahrs = {}
function ahrs:get_yaw() return osp.yaw() end
ahrs.get_yaw_rad = ahrs.get_yaw
function ahrs:get_roll() return 0.0 end
function ahrs:get_pitch() return 0.0 end
ahrs.get_roll_rad = ahrs.get_roll
ahrs.get_pitch_rad = ahrs.get_pitch
function ahrs:get_gyro() return v3(0.0, 0.0, osp.gyro_z()) end
function ahrs:healthy() return true end
function ahrs:get_velocity_NED()
  if not osp.gps_fix() then return nil end
  local n, e = osp.vel()
  return v3(n, e, 0.0)
end
function ahrs:groundspeed_vector()
  local n, e = osp.vel()
  return v2(n, e)
end
function ahrs:get_relative_position_NED_home()
  if not osp.gps_fix() then return nil end
  local n, e = osp.pos()
  return v3(n, e, 0.0)
end

---------------------------------------------------------------------------
-- gps
---------------------------------------------------------------------------
gps = { NO_GPS = 0, NO_FIX = 1, GPS_OK_FIX_2D = 2, GPS_OK_FIX_3D = 3 }
function gps:num_sensors() return 1 end
function gps:primary_sensor() return 0 end
function gps:status(_) if osp.gps_fix() then return 3 else return 1 end end
function gps:num_sats(_) if osp.gps_fix() then return 14 else return 0 end end
function gps:ground_speed(_) return osp.gspeed() end
function gps:ground_course(_) return osp.gcourse_deg() end   -- degrees, as ArduPilot

---------------------------------------------------------------------------
-- SRV_Channels: outputs. Function numbers mapped to Osprey's actuators:
--   rudder          26 GroundSteering  or 94 Script1   -> SERVO1
--   port motor      73 ThrottleLeft    or 95 Script2   -> SERVO3
--   starboard motor 74 ThrottleRight   or 96 Script3   -> SERVO4
--   both motors     70 Throttle
-- Normalised range -1..1. PWM assumed 1100 / 1500 / 1900 (min / trim / max).
---------------------------------------------------------------------------
SRV_Channels = {}
function SRV_Channels:set_output_norm(fn, v) osp.set_out(fn, v) end
function SRV_Channels:set_output_pwm(fn, pwm) osp.set_out_pwm(fn, pwm) end
function SRV_Channels:get_output_pwm(fn) return osp.get_out_pwm(fn) end
function SRV_Channels:find_channel(fn) return osp.find_chan(fn) end
function SRV_Channels:set_output_pwm_chan_timeout(ch, pwm, ms) osp.set_chan_pwm_timeout(ch, pwm, ms) end

---------------------------------------------------------------------------
-- gcs, param, arming
---------------------------------------------------------------------------
gcs = {}
function gcs:send_text(sev, txt) osp.send_text(sev, tostring(txt)) end

param = {}
function param:get(name) return osp.param_get(name) end
function param:set(name, v) return osp.param_set(name, v) end
param.set_and_save = param.set

arming = {}
function arming:is_armed() return osp.is_armed() end
function arming:arm() return osp.arm() end
function arming:disarm() return osp.disarm() end

---------------------------------------------------------------------------
-- sim: SIMULATOR ONLY. No equivalent on the autopilot — on hardware, read the
-- mission with mission:get_item() and convert Locations to local offsets.
---------------------------------------------------------------------------
sim = {}
function sim:waypoints() return osp.waypoints() end

-- print() is not available on ArduPilot (use gcs:send_text); kept for debugging.
print = function(...)
  local t = table.pack(...)
  for i = 1, t.n do t[i] = tostring(t[i]) end
  osp.print(table.concat(t, "\t"))
end

---------------------------------------------------------------------------
-- Run a callback under an instruction budget, like ArduPilot's scheduler,
-- so an infinite loop fails here rather than hanging the sim.
---------------------------------------------------------------------------
local sethook, traceback = debug.sethook, debug.traceback
local pack, unpack = table.pack, table.unpack
function __osp_call(fn, budget)
  sethook(function()
    error("script exceeded its instruction budget of " .. budget .. " per call", 2)
  end, "", budget)
  local r = pack(xpcall(fn, traceback))
  sethook()
  return unpack(r, 1, r.n)
end

-- Remove what ArduPilot's scripting environment does not provide.
io, os, require, dofile, loadfile, package, debug = nil, nil, nil, nil, nil, nil, nil

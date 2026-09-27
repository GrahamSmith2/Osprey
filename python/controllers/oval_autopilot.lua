-- oval_autopilot.lua
-- Autonomy for the PEP two-mark oval: follow the waypoint list from start to
-- finish, then stop. Port of the MATLAB cascade in control/.
--
--   waypoints -> LOS guidance (speed-adaptive lookahead, crab-corrected)
--             -> heading P+I            -> yaw-rate command (capped)
--             -> yaw-rate PI, MOMENT domain -> yaw-moment demand
--             -> ONE steering actuator, chosen by GPS speed:
--                  below 10 mph  differential thrust only, rudder centred
--                  10 mph and up rudder only (clamped at ventilation onset),
--                                both motors on the same throttle
--   speed PI on the common throttle; kill on GPS loss (PEP Rule 21).
--
-- The switch has hysteresis: the rudder takes over when speed reaches 10 mph
-- and hands back only below 9 mph, so GPS speed noise near 10 mph cannot make
-- it flip every fix.
--
-- Why the rate loop works in moment units: rudder authority goes as U^2, so an
-- angle-domain gain must be scheduled as 1/U^2 and blows up at the start line.
-- In moment units the gain is speed-invariant and the SAME gain drives either
-- actuator, so nothing needs retuning at the handover and the integrator
-- carries straight across.
--
-- Run:  python -m osprey run controllers/oval_autopilot.lua

---------------------------------------------------------------------------
-- Vehicle constants. The runner fills these in for the selected rudder; on
-- the real boat they would be script parameters (param:add_table).
---------------------------------------------------------------------------
local function P(name, default)
  local v = param:get(name)
  if v == nil then return default end
  return v
end

local K_PRIME   = P("OSP_NOMOTO_K", 0.506)          -- Nomoto K', identified per rudder
local T_PRIME   = P("OSP_NOMOTO_T", 1.084)          -- Nomoto T'
local LOA       = P("OSP_LOA", 2.134)               -- [m]
local RUD_KN    = P("OSP_RUD_KN", 2.1)              -- rudder moment: N = KN * U^2 * delta
local RUD_MAX   = math.rad(P("OSP_RUD_MAX_DEG", 35))    -- servo throw at output 1.0
local STEER_LIM = math.rad(P("OSP_STEER_LIM_DEG", 10))  -- ventilation onset: never exceed
local THR_N     = P("OSP_THR_N", 120)               -- [N] thrust per motor, full throttle, U = 0
local THR_U0    = P("OSP_THR_U0", 39.6)             -- [m/s] speed where prop thrust runs out
local YP        = P("OSP_YP", 0.299)                -- [m] motor lateral offset
local CRUISE_V  = P("CRUISE_SPEED", 22.35)          -- [m/s] 50 mph
local CRUISE_TH = P("CRUISE_THROTTLE", 45) / 100    -- feed-forward throttle at cruise
local MPH       = 0.44704
local SPLIT_V   = P("OSP_SPLIT_MPH", 10) * MPH      -- [m/s] rudder at and above, motors below
local SPLIT_HYS = P("OSP_SPLIT_HYST_MPH", 1) * MPH  -- [m/s] motors resume this far below
local DTHR_MAX  = P("OSP_DTHR_MAX", 0.5)            -- max throttle split, each side of common
local WC        = P("OSP_WC_RATE", 4.0)             -- [rad/s] inner-loop crossover
local R_MAX     = P("OSP_RATE_MAX", 0.6)            -- [rad/s] yaw-rate command ceiling
local GPS_LOSS  = P("OSP_GPS_LOSS_MS", 500)         -- [ms] grace before the Rule 21 kill

local RATE_MS = 20

-- Gains derived from the Nomoto model (python/osprey/nomoto.py), not tuned by hand
local KP_PSI = WC / 4
local KI_PSI = 0.033 * KP_PSI ^ 2
local KP_N   = WC * T_PRIME * LOA ^ 2 * RUD_KN / K_PRIME   -- [N*m per rad/s], speed-invariant

---------------------------------------------------------------------------
local wpts = sim:waypoints()          -- sim-only; on hardware read the mission
local leg = 1
local I_psi, I_N, I_u = 0.0, 0.0, 0.0
local psi_cmd = 0.0
local sat = false
local rudder_mode = false
local last_ms = nil
local lost_since = nil
local last_report = millis()

local function clamp(x, lo, hi) return math.max(lo, math.min(hi, x)) end
local function wrap(a) return (a + math.pi) % (2 * math.pi) - math.pi end

-- Advance through the legs; returns course command, cross-track, finished
local function guidance(n, e, U)
  while leg < #wpts - 1 do
    local a, b = wpts[leg], wpts[leg + 1]
    local th = math.atan(b.e - a.e, b.n - a.n)
    local L = math.sqrt((b.n - a.n) ^ 2 + (b.e - a.e) ^ 2)
    local s = (n - a.n) * math.cos(th) + (e - a.e) * math.sin(th)
    if s > L then leg = leg + 1 else break end
  end
  local a, b = wpts[leg], wpts[leg + 1]
  local th = math.atan(b.e - a.e, b.n - a.n)
  local L = math.sqrt((b.n - a.n) ^ 2 + (b.e - a.e) ^ 2)
  local s  = (n - a.n) * math.cos(th) + (e - a.e) * math.sin(th)
  local xt = -(n - a.n) * math.sin(th) + (e - a.e) * math.cos(th)
  local La = math.max(3 * U, 2 * LOA)                  -- lookahead scales with speed
  return th + math.atan(-xt, La), xt, (leg == #wpts - 1 and s > L)
end

-- Pick the steering actuator from speed, with hysteresis
local function update_mode(U)
  if rudder_mode then
    if U < SPLIT_V - SPLIT_HYS then
      rudder_mode = false
      gcs:send_text(6, string.format("steering: motors (%.1f mph)", U / MPH))
    end
  elseif U >= SPLIT_V then
    rudder_mode = true
    gcs:send_text(6, string.format("steering: rudder (%.1f mph)", U / MPH))
  end
end

-- Yaw moment the active actuator can deliver at this speed
local function capacity(U)
  if rudder_mode then return RUD_KN * U * U * STEER_LIM end
  return 2 * THR_N * math.max(1 - U / THR_U0, 0) * YP * DTHR_MAX
end

-- Turn a yaw-moment demand into a rudder angle OR a throttle split, never both.
-- Returns rudder [rad], throttle split [-], saturated?
local function allocate(N_cmd, U)
  if rudder_mode then
    local kn = RUD_KN * U * U                           -- U >= 9 mph here, never zero
    local delta = clamp(N_cmd / kn, -STEER_LIM, STEER_LIM)
    return delta, 0.0, math.abs(N_cmd) > kn * STEER_LIM
  end
  local Tmax = THR_N * math.max(1 - U / THR_U0, 0)
  local want = (Tmax > 1e-6) and N_cmd / (2 * Tmax * YP) or 0.0
  local dthr = clamp(want, -DTHR_MAX, DTHR_MAX)
  return 0.0, dthr, math.abs(want) > DTHR_MAX
end

function update()
  local now = millis()
  local dt = 0.02
  if last_ms ~= nil then dt = math.max((now - last_ms):tofloat() * 0.001, 1e-3) end
  last_ms = now

  -- PEP Rule 21: kill on missing or corrupted navigation data
  local pos = ahrs:get_relative_position_NED_home()
  if gps:status(0) < gps.GPS_OK_FIX_3D or pos == nil then
    lost_since = lost_since or now
    if (now - lost_since):tofloat() > GPS_LOSS then
      gcs:send_text(2, "GPS lost: disarming (PEP Rule 21)")
      arming:disarm()
      return                                            -- script ends
    end
  else
    lost_since = nil
  end

  local psi = ahrs:get_yaw()
  local r   = ahrs:get_gyro():z()
  local U   = gps:ground_speed(0)

  -- Guidance (hold the last heading command through a short GPS gap)
  if pos ~= nil then
    local chi_cmd, xt, finished = guidance(pos:x(), pos:y(), U)
    if finished then
      gcs:send_text(5, "Finish: disarming")
      arming:disarm()
      return
    end
    local crab = 0.0
    if U > 1.0 then crab = wrap(math.rad(gps:ground_course(0)) - psi) end
    psi_cmd = wrap(chi_cmd - crab)                      -- course command -> heading command
  end

  -- Speed loop -> common throttle
  local e_u = CRUISE_V - U
  I_u = clamp(I_u + 0.06 * e_u * dt, -0.5, 0.5)
  local thr = clamp(CRUISE_TH + 0.3 * e_u + I_u, 0.0, 1.0)

  -- Heading loop -> yaw-rate command (derivative comes from the rate loop)
  local e_psi = wrap(psi_cmd - psi)
  local r_raw = KP_PSI * e_psi + I_psi
  local r_cmd = clamp(r_raw, -R_MAX, R_MAX)
  if math.abs(r_raw) <= R_MAX and not sat then
    I_psi = clamp(I_psi + KI_PSI * e_psi * dt, -0.3, 0.3)
  end

  -- Yaw-rate PI in moment units -> allocation
  local Uf = math.max(U, 0.5)
  local KI_N = KP_N * Uf / (T_PRIME * LOA)
  local e_r = r_cmd - r
  local N_cmd = KP_N * e_r + I_N
  update_mode(U)
  local delta, dthr
  delta, dthr, sat = allocate(N_cmd, U)
  if not sat then
    I_N = I_N + KI_N * e_r * dt
  else
    I_N = I_N * 0.95                                   -- unwind while saturated
  end
  local I_lim = capacity(U)
  I_N = clamp(I_N, -I_lim, I_lim)

  -- Steering has priority over speed while the motors steer: pull the common
  -- throttle in so both sides of the split stay between 0 and 1.
  if dthr ~= 0.0 then thr = clamp(thr, math.abs(dthr), 1 - math.abs(dthr)) end

  SRV_Channels:set_output_norm(26, delta / RUD_MAX)
  SRV_Channels:set_output_norm(73, thr + dthr)          -- port
  SRV_Channels:set_output_norm(74, thr - dthr)          -- starboard

  if (now - last_report):tofloat() > 30000 then
    last_report = now
    gcs:send_text(6, string.format("leg %d/%d  %.1f mph  %s  rudder %.1f deg  split %.2f",
                                   leg, #wpts - 1, U / MPH, rudder_mode and "rudder" or "motors",
                                   math.deg(delta), dthr))
  end
  return update, RATE_MS
end

gcs:send_text(6, string.format("oval_autopilot: %d waypoints, cruise %.0f mph, Kp_N %.1f",
                               #wpts, CRUISE_V / MPH, KP_N))
return update, RATE_MS

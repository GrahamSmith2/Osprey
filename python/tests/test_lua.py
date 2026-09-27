"""The Lua control interface: API behaviour, scheduling, faults and failure."""
from pathlib import Path

import pytest

from osprey import build_params, simulate
from osprey.course import Progress, two_mark_oval
from osprey.lua_bridge import LuaController, LuaScriptError
from osprey.sensors import SensorSpec

HERE = Path(__file__).parent
CTRL = HERE.parent / "controllers"


@pytest.fixture(scope="module")
def P():
    return build_params()


def run(src, P, tmp_path, t=2.0, **kw):
    f = tmp_path / "s.lua"
    f.write_text(src)
    wpts, _ = two_mark_oval()
    c = LuaController(f, P, wpts, echo=False, **kw)
    S = simulate([5, 0, 0, 0, 0, 0, 0, 0, 0], t, c, P, dt=0.01)
    return c, S


def test_outputs_reach_the_plant(P, tmp_path):
    c, S = run("""
        function update()
          SRV_Channels:set_output_norm(26, 0.2)
          SRV_Channels:set_output_norm(73, 0.5)
          SRV_Channels:set_output_norm(74, 0.5)
          return update, 20
        end
        return update, 20""", P, tmp_path)
    assert S.delta_cmd[-1] == pytest.approx(0.2 * P.R.delta_max)
    assert S.r[-1] > 0                            # positive rudder turns to starboard


def test_millis_and_scheduling(P, tmp_path):
    c, _ = run("""
        local calls = 0
        function update()
          calls = calls + 1
          if millis():tofloat() >= 1000 then
            gcs:send_text(6, "calls " .. calls)
            return
          end
          return update, 100
        end
        return update, 100""", P, tmp_path)
    said = [m[2] for m in c.msgs if m[2].startswith("calls")]
    assert said == ["calls 10"]                   # 100 ms period -> 10 calls in 1 s
    assert not c.alive


def test_heading_hold_converges_to_target_minus_sensor_bias(P):   # T16
    """The script holds 20 deg MEASURED; the heading sensor reads 2 deg high,
    so the true heading settles near 18. It cannot see its own bias."""
    import math
    from osprey import trim_state
    wpts, _ = two_mark_oval()
    x0, _ = trim_state(8.0, P)
    c = LuaController(CTRL / "heading_hold.lua", P, wpts, echo=False)
    S = simulate(x0, 40, c, P, dt=0.01)
    assert abs(math.degrees(S.psi[-1]) - 18.0) < 2.0
    assert S.vent.max() == 0                             # clamp keeps it wetted


def test_script_sees_sensors_not_truth(P, tmp_path):
    c, _ = run("""
        function update()
          local p = ahrs:get_relative_position_NED_home()
          gcs:send_text(6, string.format("%.6f", p:x()))
          return update, 20
        end
        return update, 1000""", P, tmp_path, t=1.2)
    reported = float(c.msgs[-1][2])
    assert reported != pytest.approx(c.sens._buf[-1][0], abs=1e-6)   # GPS is noisy and late


def test_runtime_error_disarms(P, tmp_path):
    c, S = run("""
        function update() local x = nil; return x.y end
        return update, 20""", P, tmp_path)
    assert c.error and "attempt to index" in c.error
    assert not c.armed


def test_infinite_loop_hits_budget(P, tmp_path):
    c, _ = run("""
        function update() while true do end end
        return update, 20""", P, tmp_path, budget=10_000)
    assert c.error and "instruction budget" in c.error


def test_compile_error_is_reported(P, tmp_path):
    f = tmp_path / "bad.lua"
    f.write_text("function update( return end")
    with pytest.raises(LuaScriptError):
        LuaController(f, P, [(0, 0), (1, 0)], echo=False)


def test_no_filesystem_access(P, tmp_path):
    c, _ = run("""
        function update()
          gcs:send_text(6, tostring(io) .. tostring(os) .. tostring(require))
          return
        end
        return update, 20""", P, tmp_path)
    assert c.msgs[0][2] == "nilnilnil"


def test_bridge_is_not_reachable_from_scripts(P, tmp_path):
    c, _ = run("""
        function update() gcs:send_text(6, tostring(__osp) .. tostring(__osp_call)); return end
        return update, 20""", P, tmp_path)
    assert c.msgs[0][2] == "nilnil"


MPH = 0.44704


@pytest.fixture(scope="module")
def race():
    """The planned race: proposed rudder, propulsion upgraded to a 55 mph top
    speed, 50 mph cruise, standing start."""
    from osprey import PROPOSED
    from osprey.race import controller_params, run_race
    P = build_params(rudder=PROPOSED, top_speed=55 * MPH)
    return run_race(P, params=controller_params(P, 50 * MPH))


def test_oval_autopilot_finishes_at_50_mph(race):
    assert race.finished and not race.lost and race.ctl.error is None
    assert race.S.u.max() > 49 * MPH
    assert race.metrics["mark_min"] > 10.0              # never near the mark itself


def test_race_error_does_not_accumulate_over_laps(race):
    laps = race.metrics["lap_rms"]                      # complete laps only
    assert len(laps) == 3 and max(laps) < 1.2 * min(laps)


def test_motors_steer_below_10_mph_and_rudder_above(race):
    c = race.ctl
    ticks = [r for r in c.log if r[10]]                 # armed
    slow = [r for r in ticks if r[8] < 9 * MPH]         # below the hysteresis band
    fast = [r for r in ticks if r[8] >= 10 * MPH]
    assert slow and fast
    assert all(r[1] == 0.0 for r in slow)               # rudder centred
    assert any(r[2] != r[3] for r in slow)              # and the motors do steer
    assert all(r[2] == r[3] for r in fast)              # no throttle split
    assert not any(r[1] != 0.0 and r[2] != r[3] for r in ticks)


def test_logged_prop_cannot_reach_50_mph():
    from osprey.propulsion import prop_max_thrust
    P = build_params()
    u = 50 * MPH
    assert 2 * prop_max_thrust(u, P) < 0.5 * P.HT.lookup(u)[1]


def test_gps_loss_triggers_rule_21_kill(P):
    from osprey.race import controller_params
    wpts, _ = two_mark_oval()
    c = LuaController(CTRL / "oval_autopilot.lua", P, wpts, echo=False,
                      gps_dropout=(30.0, 40.0), params=controller_params(P, 8.0))
    simulate([0] * 9, 35, c, P, dt=0.01)
    assert not c.armed
    assert 30.4 < c.disarm_t < 30.7                     # 500 ms grace
    assert any("Rule 21" in m[2] for m in c.msgs)

"""Run a Lua control script against the simulated boat.

The script is written against an ArduPilot-style scripting API (see
python/LUA_API.md): it reads sensors through ``ahrs`` and ``gps``, drives the
rudder and motors through ``SRV_Channels``, and schedules itself the ArduPilot
way, by returning ``update, <ms>`` from each call.

The controller sees SENSORS, never truth: noisy 5 Hz GPS with latency, a gyro
with drifting bias, a biased heading. Outputs are normalised (-1..1) and mapped
onto the plant: rudder = norm x mechanical stop; motor thrust = norm x the
prop's available thrust at the current speed (reverse limited to 30%).
"""
from __future__ import annotations

import math
from pathlib import Path

from lupa import LuaError

from .propulsion import prop_max_thrust
from .sensors import SensorModel, SensorSpec

PRELUDE = Path(__file__).with_name("lua") / "prelude.lua"

# servo function -> actuator
STEER, PORT, STBD = "steer", "port", "stbd"
FUNC_MAP = {26: STEER, 94: STEER, 73: PORT, 95: PORT, 74: STBD, 96: STBD}
CHAN_MAP = {0: STEER, 2: PORT, 3: STBD}            # SERVO1, SERVO3, SERVO4
CHAN_OF = {STEER: 0, PORT: 2, STBD: 3}


def _runtime(version):
    import importlib
    mod = importlib.import_module(f"lupa.{version}")
    return mod.LuaRuntime(unpack_returned_tuples=True)


class LuaScriptError(RuntimeError):
    pass


class LuaController:
    """Closed-loop controller for ``osprey.sim.simulate`` that executes a Lua
    script. Construct, then pass as the controller."""

    def __init__(self, script: str | Path, P, waypoints, *, params=None,
                 rate_hz=50.0, seed=1, sensors: SensorSpec | None = None,
                 gps_dropout=None, budget=100_000, lua_version="lua54",
                 echo=True):
        self.P = P
        self.path = Path(script)
        self.rate_hz = rate_hz
        self.budget = int(budget)
        self.echo = echo
        self.sens = SensorModel(sensors, seed=seed, gps_dropout=gps_dropout)
        self.wpts = list(waypoints)
        self.params = dict(params or {})
        self.out = {STEER: 0.0, PORT: 0.0, STBD: 0.0}
        self.expire = {}                     # actuator -> time output reverts
        self.armed = True
        self.disarm_t = None
        self.t = 0.0
        self.msgs = []                       # (t, severity, text)
        self.log = []                        # per-tick record
        self.alive = True
        self.end_reason = None
        self.error = None
        self._fast_warned = False

        L = self.lua = _runtime(lua_version)
        g = L.globals()
        # Item access, not attributes: inside a class, `g.__osp` is name-mangled
        # by Python to `g._LuaController__osp` and never reaches Lua.
        g["__osp"] = L.table_from({
            "time_ms": lambda: int(round(self.t * 1000)),
            "yaw": lambda: self.sens.psi,
            "gyro_z": lambda: self.sens.r,
            "gps_fix": lambda: self.sens.fix,
            "pos": lambda: (self.sens.n, self.sens.e),
            "vel": lambda: (self.sens.vn, self.sens.ve),
            "gspeed": lambda: self.sens.speed,
            "gcourse_deg": lambda: math.degrees(self.sens.cog) % 360.0,
            "set_out": self._set_out,
            "set_out_pwm": lambda fn, pwm: self._set_out(fn, (pwm - 1500) / 400),
            "get_out_pwm": self._get_out_pwm,
            "find_chan": lambda fn: CHAN_OF.get(FUNC_MAP.get(int(fn))),
            "set_chan_pwm_timeout": self._set_chan_timeout,
            "send_text": self._send_text,
            "param_get": lambda name: self.params.get(str(name)),
            "param_set": self._param_set,
            "is_armed": lambda: self.armed,
            "arm": self._arm,
            "disarm": self._disarm,
            "waypoints": lambda: L.table_from(
                [L.table_from({"n": float(n), "e": float(e)}) for n, e in self.wpts]),
            "print": lambda s: self._send_text(7, s),
        })
        L.execute(PRELUDE.read_text(encoding="utf-8"))
        self._call = g["__osp_call"]
        g["__osp_call"] = None               # scripts cannot reach the bridge
        g["__osp"] = None

        src = self.path.read_text(encoding="utf-8")
        res = L.globals().load(src, "@" + self.path.name, "t")
        # Lua's load returns the chunk alone on success, (nil, message) on failure
        chunk, err = (res[0], res[1]) if isinstance(res, tuple) else (res, None)
        if chunk is None:
            raise LuaScriptError(f"{self.path.name} failed to compile:\n{err}")
        self.fn = chunk
        self.next_t = 0.0

    # ----- bindings ------------------------------------------------------
    def _set_out(self, fn, v):
        a = FUNC_MAP.get(int(fn))
        if a is None and int(fn) == 70:                  # common throttle
            self.out[PORT] = self.out[STBD] = max(-1.0, min(1.0, float(v)))
            return
        if a is not None:
            self.out[a] = max(-1.0, min(1.0, float(v)))
            self.expire.pop(a, None)

    def _get_out_pwm(self, fn):
        a = FUNC_MAP.get(int(fn))
        return None if a is None else int(round(1500 + 400 * self.out[a]))

    def _set_chan_timeout(self, ch, pwm, ms):
        a = CHAN_MAP.get(int(ch))
        if a is not None:
            self.out[a] = max(-1.0, min(1.0, (float(pwm) - 1500) / 400))
            self.expire[a] = self.t + float(ms) / 1000

    def _send_text(self, sev, text):
        self.msgs.append((self.t, int(sev), str(text)))
        if self.echo:
            print(f"  [{self.t:7.2f} s] {text}")

    def _param_set(self, name, v):
        self.params[str(name)] = float(v)
        return True

    def _arm(self):
        self.armed = True
        return True

    def _disarm(self):
        if self.armed:
            self.armed = False
            self.disarm_t = self.t
        return True

    # ----- scheduler -----------------------------------------------------
    def _run_due(self):
        if not self.alive or self.t + 1e-9 < self.next_t:
            return
        try:
            res = self._call(self.fn, self.budget)
        except LuaError as e:                    # error inside the bridge itself
            return self._die(str(e))
        res = res if isinstance(res, tuple) else (res,)
        if not res[0]:
            return self._die(str(res[1]))
        nxt = res[1] if len(res) > 1 else None
        delay = res[2] if len(res) > 2 else None
        if nxt is None:
            self.alive = False
            self.end_reason = "script returned no function; it has ended (ArduPilot would stop it too)"
            self._send_text(4, "Lua: " + self.end_reason)
            return
        d_ms = float(delay) if delay is not None else 0.0
        min_ms = 1000.0 / self.rate_hz
        if d_ms < min_ms and not self._fast_warned:
            self._fast_warned = True
            self._send_text(4, f"Lua: requested {d_ms:g} ms but the sim ticks at "
                               f"{min_ms:g} ms; running at {min_ms:g} ms")
        self.fn = nxt
        self.next_t = self.t + max(d_ms, min_ms) / 1000

    def _die(self, msg):
        self.alive = False
        self.error = msg
        self._send_text(0, "Lua error, script stopped: " + msg.strip().splitlines()[0])
        self._disarm()

    # ----- controller interface -----------------------------------------
    def step(self, t, x):
        self.t = t
        self.sens.update(t, x)
        for a, te in list(self.expire.items()):
            if t >= te:
                self.out[a] = 0.0
                del self.expire[a]
        self._run_due()

        if not self.armed:
            cmd = (0.0, 0.0, 0.0)
        else:
            Tmax = prop_max_thrust(max(x[0], 0.0), self.P)
            thr = lambda v: v * Tmax if v >= 0 else v * 0.3 * Tmax
            cmd = (self.out[STEER] * self.P.R.delta_max, thr(self.out[PORT]), thr(self.out[STBD]))
        self.log.append((t, self.out[STEER], self.out[PORT], self.out[STBD],
                         self.sens.psi, self.sens.r, self.sens.n, self.sens.e,
                         self.sens.speed, self.sens.fix, self.armed))
        return cmd

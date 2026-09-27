"""GPS and IMU measurements from the true state.

Ported (simplified) from sensors/sensorModel.m. A controller should never see
truth: GPS arrives at 5 Hz, 150 ms late, with metre-level noise; the gyro has a
drifting bias; heading carries a magnetometer/installation bias.

Heading is not course over ground. The IMU reports where the bow points, GPS
reports where the boat is going; in current or crosswind they differ by the crab
angle, and a cross-track loop that confuses them quietly biases downstream.
"""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass

import numpy as np

DEG = math.pi / 180


@dataclass
class SensorSpec:
    gps_rate: float = 5.0            # [Hz]   (PLACEHOLDER — module not specified)
    gps_sigma: float = 1.0           # [m]
    gps_latency: float = 0.15        # [s]
    gps_vel_sigma: float = 0.10      # [m/s]
    cog_min_speed: float = 1.0       # [m/s] COG meaningless below this
    gyro_sigma: float = 0.004        # [rad/s]
    gyro_bias_rw: float = 2e-4       # [rad/s/sqrt(s)]
    gyro_bias_0: float = 0.002       # [rad/s]
    head_sigma: float = 1.5 * DEG    # [rad]
    head_bias: float = 2.0 * DEG     # [rad]
    noise: bool = True


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class SensorModel:
    """Call ``update(t, x)`` once per controller tick; read the fields after."""

    def __init__(self, spec: SensorSpec | None = None, seed: int = 1,
                 gps_dropout: tuple | None = None):
        self.s = spec or SensorSpec()
        self.rng = np.random.default_rng(seed)
        self.gps_dropout = gps_dropout
        self.bias = self.s.gyro_bias_0 if self.s.noise else 0.0
        self._t_prev = 0.0
        self._t_last_fix = -1e9
        self._buf_t, self._buf = [], []
        # latest outputs
        self.r = 0.0
        self.psi = 0.0
        self.n = self.e = 0.0
        self.vn = self.ve = 0.0
        self.speed = 0.0
        self.cog = 0.0
        self.fix = True

    def _nz(self, sigma):
        return float(self.rng.normal(0, sigma)) if self.s.noise else 0.0

    def update(self, t, x):
        u, v, r, X, Y, psi = x[:6]
        dt = max(t - self._t_prev, 0.0)
        self._t_prev = t
        s = self.s

        if s.noise:
            self.bias += s.gyro_bias_rw * math.sqrt(dt) * float(self.rng.normal())
        self.r = r + self.bias + self._nz(s.gyro_sigma)
        self.psi = _wrap(psi + (s.head_bias if s.noise else 0.0) + self._nz(s.head_sigma))

        c, sn = math.cos(psi), math.sin(psi)
        vn, ve = u * c - v * sn, u * sn + v * c
        self._buf_t.append(t)
        self._buf.append((X, Y, vn, ve))
        if len(self._buf_t) > 2000:
            del self._buf_t[:1000], self._buf[:1000]

        dropped = (self.gps_dropout is not None
                   and self.gps_dropout[0] <= t <= self.gps_dropout[1])
        self.fix = not dropped
        if dropped:
            return
        if t - self._t_last_fix >= 1 / s.gps_rate - 1e-9:
            self._t_last_fix = t
            tf = t - s.gps_latency                 # the fix describes where the boat WAS
            i = bisect.bisect_left(self._buf_t, tf)
            if i <= 0:
                row = self._buf[0]
            elif i >= len(self._buf_t):
                row = self._buf[-1]
            else:
                t0, t1 = self._buf_t[i - 1], self._buf_t[i]
                w = (tf - t0) / (t1 - t0) if t1 > t0 else 0.0
                a, b = self._buf[i - 1], self._buf[i]
                row = tuple(p + w * (q - p) for p, q in zip(a, b))
            self.n = row[0] + self._nz(s.gps_sigma)
            self.e = row[1] + self._nz(s.gps_sigma)
            self.vn = row[2] + self._nz(s.gps_vel_sigma)
            self.ve = row[3] + self._nz(s.gps_vel_sigma)
            self.speed = math.hypot(self.vn, self.ve)
            if self.speed > s.cog_min_speed:
                self.cog = math.atan2(self.ve, self.vn)

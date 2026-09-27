"""Race course geometry, progress tracking and cross-track scoring.

The PEP course per the team: two marks 0.25 statute mile apart, run as an oval
(up one side, 180 deg around each mark, same direction). The 2-mile race is
therefore ~3.5 laps and finishes part-way down a straight.

Progress is tracked by ADVANCING LEG INDEX, never nearest-leg: nearest-leg is
wrong on a closed circuit, because progress collapses when the boat comes back
around to leg 1 and the finish is never detected. (That bug once scored a boat
that had finished as 353 m off course.)
"""
from __future__ import annotations

import math

MILE = 1609.344
RACE = 2 * MILE


def two_mark_oval(separation=0.25 * MILE, radius=25.0, distance=RACE, per_turn=12):
    """Waypoints [(north, east), ...] covering `distance` along the path, ending
    exactly at the finish. Starts at the first mark's line heading north."""
    lap = []
    lap.append((0.0, 0.0))
    lap.append((separation, 0.0))
    for k in range(1, per_turn + 1):                    # round the north mark
        th = -math.pi / 2 + math.pi * k / per_turn
        lap.append((separation + radius * math.cos(th), radius + radius * math.sin(th)))
    lap.append((0.0, 2 * radius))
    for k in range(1, per_turn + 1):                    # round the south mark
        th = math.pi / 2 + math.pi * k / per_turn
        lap.append((radius * math.cos(th), radius + radius * math.sin(th)))
    lap_len = sum(math.dist(a, b) for a, b in zip(lap, lap[1:]))
    pts = [lap[0]]
    run = 0.0
    while run < distance - 1e-9:
        for a, b in zip(lap, lap[1:]):
            d = math.dist(a, b)
            if run + d >= distance:
                f = (distance - run) / d
                pts.append((a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])))
                run = distance
                break
            pts.append(b)
            run += d
    return pts, lap_len


class Progress:
    """Sequential along-course progress and signed cross-track error."""

    def __init__(self, wpts):
        self.w = wpts
        self.L = [math.dist(a, b) for a, b in zip(wpts, wpts[1:])]
        self.cum = [0.0]
        for d in self.L:
            self.cum.append(self.cum[-1] + d)
        self.total = self.cum[-1]
        self.j = 0
        self.best = 0.0

    def update(self, n, e):
        """Advance the leg index as the boat passes each leg end. Returns
        (progress_m, cross_track_m)."""
        w, nl = self.w, len(self.L)
        while self.j < nl - 1:
            (n0, e0), (n1, e1) = w[self.j], w[self.j + 1]
            a = math.atan2(e1 - e0, n1 - n0)
            s = (n - n0) * math.cos(a) + (e - e0) * math.sin(a)
            if s > self.L[self.j]:
                self.j += 1
            else:
                break
        (n0, e0), (n1, e1) = w[self.j], w[self.j + 1]
        a = math.atan2(e1 - e0, n1 - n0)
        s = (n - n0) * math.cos(a) + (e - e0) * math.sin(a)
        xt = -(n - n0) * math.sin(a) + (e - e0) * math.cos(a)
        prog = self.cum[self.j] + min(max(s, 0.0), self.L[self.j])
        self.best = max(self.best, prog)
        return self.best, xt

    @property
    def finished(self):
        return self.best >= self.total - 1e-6

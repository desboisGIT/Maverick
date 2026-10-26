"""Reactive clearance guard (pure Python, no ROS imports).

Runs next to the skills on the host, at about 10 Hz, on the latest clearance report of the voxel
map (maverick_mapping.clearance). It never creates motion of its own beyond "hold here" and a
small step away from an obstacle, and both leave through the same goto goals as the skills, i.e.
through the flight client's safety gate. Skills plan around obstacles with margins; the guard is
what happens when the drone ends up closer than planned anyway (tracking error, overshoot of a
waypoint, an obstacle mapped late, a graze beside a half-mapped box).

  slow  an occupied cell within `slow_m` in the direction of travel: cap the speed
  stop  within `stop_m` in the direction of travel: hold here, the skill replans
  nudge within `nudge_m` in any direction: step away from it (up to `nudge_max_m`), also when
        hovering, where the stationary graze beside furniture happened (S3 tuning seeds 20-59)

All distances are map-space (body centre to the centre of the nearest occupied cell). The depth
camera sits 0.07 m ahead of the body centre and does not see closer than 0.15 m, so an obstacle
straight ahead is invisible below about 0.22 m: the first version (stop at 0.20 m) fired only
after the contact (tuning seeds 24, 26, 27), hence the larger distances. The stop cone is narrow
(an obstacle that would be hit, lateral offset below the body radius) so that door frames passed
at 0.25 m do not stop the drone.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

XY = tuple[float, float]


@dataclass(frozen=True)
class GuardConfig:
    stop_m: float = 0.30
    stop_fan_rad: float = math.radians(35.0)  # half width of the cone in which `stop_m` applies
    slow_m: float = 0.50
    slow_fan_rad: float = math.radians(50.0)
    slow_speed: float = 0.25
    nudge_m: float = 0.20
    nudge_target_m: float = 0.26
    nudge_max_m: float = 0.12
    # braking: the drone coasts about 0.3 m from 0.8 m/s (setpoint stepped back, attitude lag), so
    # both distances grow with the speed towards the obstacle; a stop needs the drone to be closing
    # in (a drone at rest next to an obstacle must be free to move away: a hard stop there froze
    # tuning seed 26 beside a wall for 15 s), except inside `hard_stop_m`
    k_stop_s: float = 0.35
    k_slow_s: float = 0.6
    min_closing_mps: float = 0.08
    hard_stop_m: float = 0.20
    keepout_m: float = 0.20  # radius of the keepout the skill planner gets around a stop's obstacle
    keepout_s: float = 40.0
    min_speed: float = 0.12  # below this the velocity heading is noise; use the goal's heading
    max_age_s: float = 0.8  # a report older than this is ignored (no guard, the skills' margins)
    hold_s: float = 1.2  # how long a stop holds before the skill's goal is sent again


@dataclass(frozen=True)
class Decision:
    action: str  # "ok" | "slow" | "stop" | "nudge"
    d_fwd: float
    d_min: float
    heading: float | None = None
    speed_cap: float | None = None
    away: XY | None = None  # unit vector away from the obstacle (nudge)
    step: float = 0.0  # nudge distance, m
    bearing: float | None = None  # bearing of the nearest obstacle in the stop cone (stop)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def bearing_of(k: int, n: int) -> float:
    return -math.pi + (k + 0.5) * 2 * math.pi / n


class ClearanceGuard:
    def __init__(self, cfg: GuardConfig | None = None) -> None:
        self.cfg = cfg or GuardConfig()

    def heading(self, velocity: XY | None, to_goal: XY | None) -> float | None:
        """Direction of travel: the velocity if the drone moves, else towards the current goal."""
        if velocity is not None and math.hypot(*velocity) >= self.cfg.min_speed:
            return math.atan2(velocity[1], velocity[0])
        if to_goal is not None and math.hypot(*to_goal) > 0.1:
            return math.atan2(to_goal[1], to_goal[0])
        return None

    def evaluate(
        self, sectors: list[float], heading: float | None, v_toward: float = 0.0
    ) -> Decision:
        """`v_toward`: speed along `heading`, m/s (the drone's own, from poses)."""
        c = self.cfg
        n = len(sectors)
        d_min = min(sectors)
        d_fwd = d_slow = d_min
        bearing = None
        v = max(v_toward, 0.0)
        if heading is not None:

            def cone(fan: float) -> tuple[float, float | None]:
                half = fan + math.pi / n  # a sector counts if any part of it is in the cone
                ks = [k for k in range(n) if abs(_wrap(bearing_of(k, n) - heading)) <= half]
                if not ks:
                    return d_min, None
                k = min(ks, key=lambda k: sectors[k])
                return sectors[k], bearing_of(k, n)

            (d_fwd, bearing), (d_slow, _) = cone(c.stop_fan_rad), cone(c.slow_fan_rad)
            closing = v >= c.min_closing_mps
            if d_fwd < c.hard_stop_m or (d_fwd < c.stop_m + c.k_stop_s * v and closing):
                return Decision("stop", d_fwd, d_min, heading, bearing=bearing)
        if d_min < c.nudge_m:
            ax = ay = 0.0
            for k, d in enumerate(sectors):
                if d < c.nudge_target_m:
                    w = c.nudge_target_m - d
                    b = bearing_of(k, n)
                    ax -= w * math.cos(b)
                    ay -= w * math.sin(b)
            norm = math.hypot(ax, ay)
            if norm > 1e-6:
                step = min(c.nudge_max_m, c.nudge_target_m - d_min)
                return Decision(
                    "nudge", d_fwd, d_min, heading, away=(ax / norm, ay / norm), step=step
                )
        if heading is not None and d_slow < c.slow_m + c.k_slow_s * v:
            return Decision("slow", d_slow, d_min, heading, speed_cap=c.slow_speed)
        return Decision("ok", d_fwd, d_min, heading)


class VelocityEstimate:
    """Horizontal velocity from the last pose samples (window `window_s`)."""

    def __init__(self, window_s: float = 0.6) -> None:
        self.window = window_s
        self.samples: list[tuple[float, float, float]] = []

    def add(self, t: float, xy: XY) -> None:
        self.samples.append((t, xy[0], xy[1]))
        while len(self.samples) > 2 and t - self.samples[0][0] > self.window:
            self.samples.pop(0)

    def velocity(self) -> XY | None:
        if len(self.samples) < 2:
            return None
        a, b = self.samples[0], self.samples[-1]
        dt = b[0] - a[0]
        if dt < 0.2:
            return None
        return ((b[1] - a[1]) / dt, (b[2] - a[2]) / dt)

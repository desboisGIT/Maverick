"""Host safety gate (pure Python, no ROS). See docs/s1-plan.md, "Host flight client".

Everything that can reach PX4 passes through here. A request is rejected, never silently
clamped: every rejection is logged with a reason and counted. The speed limit is enforced by
stepping the streamed setpoint towards a goal, so no setpoint jumps further than max_jump_m.

Positions are PX4 local NED metres (z negative is up).
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field

Vec3 = tuple[float, float, float]

REASON_NON_FINITE = "non_finite"
REASON_GEOFENCE = "geofence"
REASON_JUMP = "jump"
REASON_SPEED = "speed_limit"

log = logging.getLogger("maverick_flight.gate")


@dataclass(frozen=True)
class GateConfig:
    geofence_min: Vec3 = (-5.0, -5.0, -4.0)  # NED; z = -4 is 4 m up
    geofence_max: Vec3 = (5.0, 5.0, 0.3)
    max_jump_m: float = 1.0
    max_speed_mps: float = 1.0


@dataclass(frozen=True)
class Rejection:
    reason: str
    detail: str


@dataclass
class SafetyGate:
    config: GateConfig = field(default_factory=GateConfig)
    # The geofence box is relative to this point (the take-off point: x, y and ground z), so
    # it means "this far from where the mission started" whatever the estimator origin is.
    # PX4's origin drifts between flights on a reused stack (~0.16 m down per flight in SITL).
    origin: Vec3 = (0.0, 0.0, 0.0)
    rejections: dict[str, int] = field(default_factory=dict)
    last_rejection: Rejection | None = None

    def check_position(self, p: Vec3) -> Rejection | None:
        if not all(math.isfinite(v) for v in p):
            return Rejection(REASON_NON_FINITE, f"position {p}")
        lo, hi = self.config.geofence_min, self.config.geofence_max
        for axis, v, o, a, b in zip("xyz", p, self.origin, lo, hi, strict=True):
            if not a <= v - o <= b:
                return Rejection(
                    REASON_GEOFENCE, f"{axis}={v:.2f} ({v - o:+.2f} from start) outside [{a}, {b}]"
                )
        return None

    def check_goal(self, p: Vec3, speed: float | None = None) -> Rejection | None:
        """A goto goal: finite, inside the geofence, requested speed within the limit."""
        rej = self.check_position(p)
        if rej is not None:
            return rej
        if speed is not None and not (
            math.isfinite(speed) and 0.0 < speed <= self.config.max_speed_mps
        ):
            return Rejection(REASON_SPEED, f"speed {speed} > {self.config.max_speed_mps}")
        return None

    def check_setpoint(self, current: Vec3, candidate: Vec3) -> Rejection | None:
        """A setpoint about to be streamed: finite, in the geofence, no jump from the last one."""
        rej = self.check_position(candidate)
        if rej is not None:
            return rej
        jump = math.dist(current, candidate)
        if jump > self.config.max_jump_m:
            return Rejection(REASON_JUMP, f"jump {jump:.2f} m > {self.config.max_jump_m} m")
        return None

    def step_towards(
        self, current: Vec3, goal: Vec3, dt: float, speed: float | None = None
    ) -> Vec3:
        """Next setpoint: at most speed*dt from `current` towards `goal` (never overshoots)."""
        v = min(
            speed if speed is not None else self.config.max_speed_mps, self.config.max_speed_mps
        )
        max_step = min(max(v * dt, 0.0), self.config.max_jump_m)
        dist = math.dist(current, goal)
        if dist <= max_step or dist == 0.0:
            return goal
        k = max_step / dist
        return (
            current[0] + (goal[0] - current[0]) * k,
            current[1] + (goal[1] - current[1]) * k,
            current[2] + (goal[2] - current[2]) * k,
        )

    def reject(self, rej: Rejection, source: str = "") -> None:
        self.rejections[rej.reason] = self.rejections.get(rej.reason, 0) + 1
        self.last_rejection = rej
        log.warning("rejected %s: %s (%s)", source or "request", rej.reason, rej.detail)

    @property
    def rejected_total(self) -> int:
        return sum(self.rejections.values())

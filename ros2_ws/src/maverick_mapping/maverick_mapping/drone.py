"""Which drone the stack flies, and the numbers that follow from its size (pure Python).

One switch: MAVERICK_DRONE=small|x500 (default small), read by the node launch files. The drone
stack gets its body radius from here, never from the world manifest or Gazebo ground truth.
Planner margins keep the logic of the S3 x500 tuning (docs/s3-plan.md): hard radius = body radius
+ margin, door "squeezes" between a tight radius (body + 0.07) and the hard radius, fallbacks at
body + 0.04 and body + 0.01.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

from .planner import PlannerConfig

DEFAULT_DRONE = "small"


@dataclass(frozen=True)
class DroneConfig:
    name: str
    body_radius: float  # circumscribed radius incl. guards, m
    camera_forward_m: float  # camera mount x (camera_mount.json)
    margin: float
    start_free_radius: float
    unknown_radius: float
    trail_radius: float
    goal_clear: float  # memory: navigation goals keep this far from obstacles
    connect_clear: float  # memory: connectivity of the start to the goal room
    wall_clear: float  # memory: preferred goal distance from the room edge
    mount_dir: str  # sim/models/<mount_dir>/camera_mount.json

    def planner(self) -> PlannerConfig:
        r = self.body_radius
        return PlannerConfig(
            robot_radius=r,
            margin=self.margin,
            tight_radius=round(r + 0.07, 3),
            tight_fallbacks=(round(r + 0.04, 3), round(r + 0.01, 3)),
            start_free_radius=self.start_free_radius,
            unknown_radius=self.unknown_radius,
            trail_radius=self.trail_radius,
        )


DRONES = {
    # x500 values: exactly what the S3 mocap results were produced with
    "x500": DroneConfig("x500", 0.39, 0.12, 0.22, 0.45, 0.3, 0.2, 0.45, 0.3, 1.1, "maverick"),
    # 3.5 inch ducted quad: 0.30 m across with guards
    "small": DroneConfig(
        "small", 0.15, 0.07, 0.20, 0.27, 0.25, 0.12, 0.30, 0.2, 0.8, "maverick_small"
    ),
}


def drone_config(name: str | None = None) -> DroneConfig:
    name = (name or os.environ.get("MAVERICK_DRONE") or DEFAULT_DRONE).lower()
    if name not in DRONES:
        raise ValueError(f"unknown drone {name!r}, expected one of {sorted(DRONES)}")
    return DRONES[name]

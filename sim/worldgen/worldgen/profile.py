"""Drone size profiles: how much room the generated flat must give the drone."""

from __future__ import annotations

import os
from dataclasses import dataclass


@dataclass(frozen=True)
class DroneProfile:
    name: str
    door_widths_dm: tuple[int, ...]
    open_min_dm: int  # narrowest open-plan gap
    corridor: float  # free corridor width kept between furniture, m
    zone_depth: float  # clear floor in front of every opening, m
    open_zone_half: float  # half width of the kept-clear part of an open-plan gap
    spawn_clearance: float
    min_sides: dict[str, float]  # per room type, m

    @property
    def corridor_radius(self) -> float:
        return self.corridor / 2

    @property
    def entry_depth(self) -> float:
        return self.zone_depth * 0.75

    @property
    def door_range(self) -> tuple[float, float]:
        return min(self.door_widths_dm) / 10, max(self.door_widths_dm) / 10

    @property
    def min_shared_dm(self) -> int:
        return min(self.door_widths_dm) + 4


# 3.5 inch ducted quad, 0.30 m across with guards (circumscribed radius 0.15 m).
SMALL = DroneProfile(
    "small",
    (8, 9, 10),
    12,
    0.8,
    0.8,
    0.5,
    1.0,
    {"living": 3.2, "kitchen": 2.0, "bathroom": 2.0, "bedroom": 2.4, "office": 2.0, "hallway": 1.2},
)

# PX4 x500: circumscribed radius about 0.39 m, and it yaws during 360 degree scans.
X500 = DroneProfile(
    "x500",
    (11, 12, 13),
    14,
    1.2,
    1.0,
    0.7,
    1.2,
    {"living": 3.4, "kitchen": 2.4, "bathroom": 2.2, "bedroom": 2.8, "office": 2.4, "hallway": 1.5},
)

PROFILES = {p.name: p for p in (SMALL, X500)}
# One switch for the whole repo: MAVERICK_DRONE=small|x500 (default small).
DEFAULT_PROFILE = os.environ.get("MAVERICK_DRONE", "small").lower()
if DEFAULT_PROFILE not in PROFILES:
    raise ValueError(f"MAVERICK_DRONE={DEFAULT_PROFILE!r}: expected one of {sorted(PROFILES)}")

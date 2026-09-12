"""Programme: which rooms a flat has and how big they want to be."""

from __future__ import annotations

import random
from dataclasses import dataclass

from .profile import DroneProfile

FLAT_TYPES = ("studio", "1-bed", "2-bed", "3-bed")
_FLAT_WEIGHTS = (0.15, 0.30, 0.30, 0.25)
BEDROOMS = {"studio": 0, "1-bed": 1, "2-bed": 2, "3-bed": 3}


@dataclass(frozen=True)
class RoomSpec:
    type: str
    area: float  # target area, m2
    min_side: float
    min_area: float
    max_aspect: float


# type -> (min_side, min_area, max_aspect). The living room needs a 2 x 2 m spawn pocket.
RULES: dict[str, tuple[float, float, float]] = {
    "living": (3.2, 12.0, 2.6),
    "kitchen": (2.0, 5.0, 3.0),
    "bathroom": (2.0, 3.0, 2.5),
    "bedroom": (2.4, 7.0, 2.4),
    "office": (2.0, 4.5, 2.6),
    "hallway": (1.2, 3.0, 7.0),
}


@dataclass(frozen=True)
class Programme:
    flat_type: str
    rooms: tuple[RoomSpec, ...]  # ordered so neighbours in the list tend to be neighbours in space
    open_kitchen: bool

    @property
    def total_area(self) -> float:
        return sum(r.area for r in self.rooms)


def _spec(rng: random.Random, prof: DroneProfile, kind: str, lo: float, hi: float) -> RoomSpec:
    _, min_area, aspect = RULES[kind]
    return RoomSpec(kind, round(rng.uniform(lo, hi), 1), prof.min_sides[kind], min_area, aspect)


def make_programme(rng: random.Random, prof: DroneProfile) -> Programme:
    flat = rng.choices(FLAT_TYPES, _FLAT_WEIGHTS)[0]
    living = kitchen = bath = None
    beds: list[RoomSpec] = []
    hall = office = None
    if flat == "studio":
        living = _spec(rng, prof, "living", 18, 26)
        kitchen = _spec(rng, prof, "kitchen", 6, 9)
        bath = _spec(rng, prof, "bathroom", 4, 5.5)
        hall = _spec(rng, prof, "hallway", 3, 4.5) if rng.random() < 0.4 else None
    elif flat == "1-bed":
        living = _spec(rng, prof, "living", 16, 24)
        kitchen = _spec(rng, prof, "kitchen", 7, 10)
        bath = _spec(rng, prof, "bathroom", 4, 6)
        beds = [_spec(rng, prof, "bedroom", 10, 15)]
        hall = _spec(rng, prof, "hallway", 4, 7) if rng.random() < 0.7 else None
    elif flat == "2-bed":
        living = _spec(rng, prof, "living", 18, 26)
        kitchen = _spec(rng, prof, "kitchen", 8, 12)
        bath = _spec(rng, prof, "bathroom", 4.5, 7)
        beds = [_spec(rng, prof, "bedroom", 11, 15), _spec(rng, prof, "bedroom", 9, 13)]
        hall = _spec(rng, prof, "hallway", 4, 7) if rng.random() < 0.85 else None
        office = _spec(rng, prof, "office", 6, 9) if rng.random() < 0.25 else None
    else:
        living = _spec(rng, prof, "living", 20, 30)
        kitchen = _spec(rng, prof, "kitchen", 9, 13)
        bath = _spec(rng, prof, "bathroom", 5, 7)
        beds = [
            _spec(rng, prof, "bedroom", 12, 16),
            _spec(rng, prof, "bedroom", 10, 14),
            _spec(rng, prof, "bedroom", 9, 12),
        ]
        hall = _spec(rng, prof, "hallway", 6, 9)
        office = _spec(rng, prof, "office", 6, 9) if rng.random() < 0.25 else None

    # Order: kitchen next to living, then a shuffle of the private rooms with the hallway
    # inserted near the living end. Neighbours in the list tend to become neighbours in space.
    private: list[RoomSpec] = beds + ([office] if office else []) + [bath]
    rng.shuffle(private)
    if hall:
        private.insert(rng.randint(0, min(2, len(private))), hall)
    order = [kitchen, living] + private
    if rng.random() < 0.5:
        order.reverse()
    open_kitchen = rng.random() < (0.7 if flat == "studio" else 0.45)
    return Programme(flat, tuple(order), open_kitchen)

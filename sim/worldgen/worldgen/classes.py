"""Stable semantic class table: name -> integer label (0 = background / unlabelled).

Structural classes come first, then furniture classes in a fixed order. Never reorder or
insert: ids are used by the simulated segmentation detector. Append new classes at the end.
"""

from __future__ import annotations

STRUCTURAL = ("floor", "wall", "ceiling", "door", "window", "decor")
FURNITURE = (
    "kitchen_counter",
    "fridge",
    "oven",
    "dining_table",
    "chair",
    "sofa",
    "coffee_table",
    "tv_unit",
    "bed",
    "nightstand",
    "wardrobe",
    "desk",
    "office_chair",
    "bathtub",
    "shower",
    "toilet",
    "sink",
    "shelves",
    "plant",
    "rug",
)

CLASSES: dict[str, int] = {name: i for i, name in enumerate(STRUCTURAL + FURNITURE, start=1)}
BACKGROUND = 0

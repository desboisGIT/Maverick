"""Rule-based room typing from the objects inside a room (plus its shape, for hallways)."""

from __future__ import annotations

from dataclasses import dataclass, field

# class -> {room type: evidence weight}. A sink is weak evidence for both kitchen and bathroom.
WEIGHTS: dict[str, dict[str, float]] = {
    "fridge": {"kitchen": 3.0},
    "oven": {"kitchen": 3.0},
    "kitchen_counter": {"kitchen": 2.5},
    "dining_table": {"kitchen": 1.0, "living": 0.5},
    "chair": {"kitchen": 0.3},
    "sink": {"bathroom": 1.5, "kitchen": 0.6},
    "toilet": {"bathroom": 3.0},
    "shower": {"bathroom": 3.0},
    "bathtub": {"bathroom": 3.0},
    "bed": {"bedroom": 4.5},
    "nightstand": {"bedroom": 1.5},
    "wardrobe": {"bedroom": 1.5},
    "sofa": {"living": 3.0},
    "tv_unit": {"living": 2.5},
    "coffee_table": {"living": 2.0},
    "rug": {"living": 0.4},
    "desk": {"office": 1.5},
    "office_chair": {"office": 2.0},
    "shelves": {"living": 0.2, "office": 0.3},
    "plant": {"living": 0.1, "hallway": 0.2},
}
TYPES = ("living", "kitchen", "bedroom", "bathroom", "hallway", "office")


@dataclass
class TypingConfig:
    min_score: float = 1.0  # below this the room stays "unknown"
    repeat_bonus: float = 0.25  # each extra object of a class adds this fraction of its weight
    repeat_cap: float = 1.5  # ... up to this multiple
    hallway_aspect: float = 1.5  # elongated: long side / short side
    hallway_width: float = 2.1  # m, widest corridor
    hallway_score: float = 2.5
    hallway_max_other: float = 1.5  # ... only if the objects say little else
    open_plan_min: float = 2.5  # both kitchen and living scores at least this: open plan
    conf_norm: float = 1.0  # confidence = best / (best + second + conf_norm)
    weights: dict = field(default_factory=lambda: WEIGHTS)


@dataclass
class RoomType:
    type: str
    confidence: float
    scores: dict[str, float]
    secondary: str | None = None  # e.g. "living" for an open-plan kitchen typed "kitchen"


def score_room(
    objects: list[tuple[str, float]],
    elongation: float = 1.0,
    width: float = 99.0,
    cfg: TypingConfig | None = None,
) -> RoomType:
    """Type a room from [(class, confidence)] of its confirmed objects and its shape.

    elongation: long / short side of the room; width: short side in metres.
    """
    cfg = cfg or TypingConfig()
    scores = dict.fromkeys(TYPES, 0.0)
    by_class: dict[str, list[float]] = {}
    for label, conf in objects:
        by_class.setdefault(label, []).append(conf)
    for label, confs in by_class.items():
        w = cfg.weights.get(label)
        if not w:
            continue
        mult = min(1.0 + cfg.repeat_bonus * (len(confs) - 1), cfg.repeat_cap)
        c = min(1.0, max(confs) + 0.15)  # a confirmed class is mostly right even at low conf
        for t, wt in w.items():
            scores[t] += wt * mult * c
    other = max((v for t, v in scores.items() if t != "hallway"), default=0.0)
    if (
        elongation >= cfg.hallway_aspect
        and width <= cfg.hallway_width
        and other <= (cfg.hallway_max_other)
    ):
        scores["hallway"] += cfg.hallway_score
    ranked = sorted(scores.items(), key=lambda kv: -kv[1])
    (best_t, best), (_, second) = ranked[0], ranked[1]
    if best < cfg.min_score:
        return RoomType("unknown", 0.0, scores)
    conf = best / (best + second + cfg.conf_norm)
    secondary = None
    if {best_t, ranked[1][0]} == {"kitchen", "living"} and min(
        scores["kitchen"], scores["living"]
    ) >= cfg.open_plan_min:
        secondary = ranked[1][0]
    return RoomType(best_t, float(conf), {k: round(v, 3) for k, v in scores.items()}, secondary)

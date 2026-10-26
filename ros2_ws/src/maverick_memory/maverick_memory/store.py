"""Persistent object store: associate per-frame observations into objects with stable ids."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .lifting import ObjectObservation

ALIASES = {
    "refrigerator": "fridge",
    "couch": "sofa",
    "tv": "tv_unit",
    "tv_stand": "tv_unit",
    "stove": "oven",
    "cooker": "oven",
    "counter": "kitchen_counter",
    "table": "dining_table",
    "toilet_seat": "toilet",
    "bathroom_sink": "sink",
    "wash_basin": "sink",
    "bookshelf": "shelves",
    "shelf": "shelves",
    "potted_plant": "plant",
    "armchair": "chair",
    "bananas": "banana",
    "loaf": "bread",
    "bread_loaf": "bread",
    "human": "person",
    "man": "person",
    "woman": "person",
}


def canonical_label(label: str) -> str:
    s = label.strip().lower().replace(" ", "_")
    return ALIASES.get(s, s)


@dataclass
class StoredObject:
    id: str
    label: str
    lo: np.ndarray
    hi: np.ndarray
    conf_miss: float  # prod(1 - w * c_i); confidence = 1 - conf_miss
    n_obs: int
    first_seen: float
    last_seen: float
    n_full: int = 0  # observations that were not truncated by the image border
    room_id: str | None = None
    status: str = "present"  # present | unverified | gone
    extra: dict = field(default_factory=dict)

    @property
    def position(self) -> np.ndarray:
        return 0.5 * (self.lo + self.hi)

    @property
    def size(self) -> np.ndarray:
        return self.hi - self.lo

    @property
    def confidence(self) -> float:
        return float(min(0.99, 1.0 - self.conf_miss))


def _vol(lo: np.ndarray, hi: np.ndarray) -> float:
    return float(np.prod(np.maximum(hi - lo, 1e-3)))


def _overlap_min_frac(lo1, hi1, lo2, hi2) -> float:
    """Intersection volume / volume of the smaller box (thin boxes get 1 mm of thickness)."""
    lo, hi = np.maximum(lo1, lo2), np.minimum(hi1, hi2)
    if np.any(hi < lo):
        return 0.0
    return _vol(lo, hi) / min(_vol(lo1, hi1), _vol(lo2, hi2))


@dataclass
class StoreConfig:
    min_obs: int = 3  # observations before an object is reported (confirmed)
    gate_base: float = 0.25  # xy gate = gate_base + gate_size * max(size_xy of both), m
    gate_size: float = 0.5
    gate_min: float = 0.35
    gate_max: float = 1.4
    overlap_assoc: float = 0.3  # or: boxes overlap by this fraction of the smaller one
    merge_overlap: float = 0.5  # merge two same-class objects overlapping this much
    obs_weight: float = 0.6  # confidence accumulation: miss *= 1 - w * c
    prune_after: float = 8.0  # s: unconfirmed objects not seen this long are dropped
    max_size: float = 4.0  # observations larger than this (a wall, a false box) are ignored


class ObjectStore:
    def __init__(self, cfg: StoreConfig | None = None) -> None:
        self.cfg = cfg or StoreConfig()
        self.objects: dict[str, StoredObject] = {}
        self._next = 0

    # --- update -------------------------------------------------------------------------------
    def _gate(self, o: StoredObject, obs: ObjectObservation) -> float:
        c = self.cfg
        s = max(float(o.size[:2].max()), float((obs.hi - obs.lo)[:2].max()))
        return float(np.clip(c.gate_base + c.gate_size * s, c.gate_min, c.gate_max))

    def _match(self, obs: ObjectObservation) -> StoredObject | None:
        best, best_d = None, np.inf
        ctr = obs.center
        for o in self.objects.values():
            if o.label != obs.label:
                continue
            d = float(np.linalg.norm((o.position - ctr)[:2]))
            ok = d < self._gate(o, obs) or (
                _overlap_min_frac(o.lo, o.hi, obs.lo, obs.hi) >= self.cfg.overlap_assoc
            )
            if ok and d < best_d:
                best, best_d = o, d
        return best

    def match(self, obs: ObjectObservation) -> StoredObject | None:
        """The stored object an observation of this class belongs to (None: no association)."""
        obs.label = canonical_label(obs.label)
        return self._match(obs)

    def add(self, obs: ObjectObservation) -> StoredObject | None:
        """Associate one observation; returns the object it updated or created (None if ignored)."""
        c = self.cfg
        obs.label = canonical_label(obs.label)
        if float((obs.hi - obs.lo).max()) > c.max_size:
            return None
        o = self._match(obs)
        w = c.obs_weight * float(np.clip(obs.confidence, 0.0, 1.0))
        if o is None:
            self._next += 1
            o = StoredObject(
                f"obj_{self._next}",
                obs.label,
                obs.lo.copy(),
                obs.hi.copy(),
                1.0 - w,
                1,
                obs.t,
                obs.t,
                0 if obs.truncated else 1,
            )
            self.objects[o.id] = o
            return o
        o.lo = np.minimum(o.lo, obs.lo)
        o.hi = np.maximum(o.hi, obs.hi)
        o.conf_miss *= 1.0 - w
        o.n_obs += 1
        o.n_full += 0 if obs.truncated else 1
        o.last_seen = max(o.last_seen, obs.t)
        o.status = "present"
        return o

    def add_frame(self, observations: list[ObjectObservation], now: float | None = None) -> None:
        for obs in sorted(observations, key=lambda x: -x.confidence):
            self.add(obs)
        self.merge_duplicates()
        if now is not None:
            self.prune(now)

    def merge_duplicates(self) -> int:
        """Merge same-class objects whose boxes overlap (grown partial views meeting)."""
        merged = 0
        again = True
        while again:
            again = False
            objs = sorted(self.objects.values(), key=lambda o: int(o.id.split("_")[1]))
            for i, a in enumerate(objs):
                for b in objs[i + 1 :]:
                    if a.label != b.label:
                        continue
                    if _overlap_min_frac(a.lo, a.hi, b.lo, b.hi) >= self.cfg.merge_overlap:
                        a.lo, a.hi = np.minimum(a.lo, b.lo), np.maximum(a.hi, b.hi)
                        a.conf_miss *= b.conf_miss
                        a.n_obs += b.n_obs
                        a.n_full += b.n_full
                        a.first_seen = min(a.first_seen, b.first_seen)
                        a.last_seen = max(a.last_seen, b.last_seen)
                        a.extra = {**b.extra, **a.extra}
                        del self.objects[b.id]
                        merged += 1
                        again = True
                        break
                if again:
                    break
        return merged

    def prune(self, now: float) -> int:
        """Drop objects with too little support (never confirmed and not seen for a while)."""
        c = self.cfg
        dead = [
            k
            for k, o in self.objects.items()
            if o.n_obs < c.min_obs and now - o.last_seen > c.prune_after
        ]
        for k in dead:
            del self.objects[k]
        return len(dead)

    # --- queries ------------------------------------------------------------------------------
    def confirmed(self) -> list[StoredObject]:
        return [o for o in self.objects.values() if o.n_obs >= self.cfg.min_obs]

    def get(self, object_id: str) -> StoredObject | None:
        return self.objects.get(object_id)

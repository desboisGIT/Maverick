"""Simulated detector on the segmentation camera: class-id image + depth -> detections.

The S3 simulated detector (plan, "Simulated detector"): Gazebo's segmentation camera labels every
pixel with a worldgen class id (sim/worldgen/worldgen/classes.py). This turns it into per-object
2D detections (connected components of one class, split at depth discontinuities) and applies the
same noise model as SimDetector (misses, false positives, box jitter, confidence, mask dropout).
Oracle-quality perception with real occlusion: report results as such. Replaceable by a real
detector that outputs the same `Detection` list.
"""

from __future__ import annotations

import numpy as np

from .lifting import Detection
from .rooms import label4
from .sim_detector import DetectorNoise

# Copy of worldgen.classes (ids are stable, append-only). test_ros_glue checks it against worldgen.
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
SMALL = ("banana", "bread", "person")  # S4b, appended: small objects and people
SMALL_MIN_PIXELS = 6  # a banana is a few pixels at 2 m: the furniture minimum would drop it
CLASS_IDS = {name: i for i, name in enumerate(STRUCTURAL + FURNITURE + SMALL, start=1)}
ID_TO_NAME = {i: n for n, i in CLASS_IDS.items()}
FURNITURE_IDS = frozenset(CLASS_IDS[n] for n in FURNITURE + SMALL)
SMALL_IDS = frozenset(CLASS_IDS[n] for n in ("banana", "bread"))


def depth_edges(depth: np.ndarray, jump: float = 0.25, rel: float = 0.08) -> np.ndarray:
    """Pixels next to a depth jump larger than `jump` + rel * depth (object boundaries)."""
    d = np.where(np.isfinite(depth), depth, 0.0)
    e = np.zeros(d.shape, bool)
    bx = np.abs(d[:, 1:] - d[:, :-1]) > jump + rel * np.minimum(d[:, 1:], d[:, :-1])
    by = np.abs(d[1:, :] - d[:-1, :]) > jump + rel * np.minimum(d[1:, :], d[:-1, :])
    e[:, 1:] |= bx
    e[:, :-1] |= bx
    e[1:, :] |= by
    e[:-1, :] |= by
    return e


def _regrow(lab: np.ndarray, rim: np.ndarray) -> np.ndarray:
    """Give the removed boundary pixels (`rim`) the label of a labelled 4-neighbour (one step)."""
    out = lab.copy()
    pad = np.pad(lab, 1)
    for nb in (pad[:-2, 1:-1], pad[2:, 1:-1], pad[1:-1, :-2], pad[1:-1, 2:]):
        take = rim & (out == 0) & (nb > 0)
        out[take] = nb[take]
    return out


class SemanticDetector:
    def __init__(self, noise: DetectorNoise | None = None, seed: int = 0) -> None:
        self.noise = noise or DetectorNoise()
        self.rng = np.random.default_rng(seed)

    def components(
        self, semantic: np.ndarray, depth: np.ndarray, min_pixels: int
    ) -> list[tuple[str, np.ndarray]]:
        """(label, mask) of every furniture component with at least `min_pixels` pixels."""
        edges = depth_edges(depth)
        out = []
        for cid in np.unique(semantic):
            if int(cid) not in FURNITURE_IDS:
                continue
            m = semantic == cid
            lab, n = label4(m & ~edges)
            lab = _regrow(lab, m & edges)
            for k in range(1, n + 1):
                mk = lab == k
                if int(mk.sum()) >= (
                    min(min_pixels, SMALL_MIN_PIXELS) if int(cid) in SMALL_IDS else min_pixels
                ):
                    out.append((ID_TO_NAME[int(cid)], mk))
        return out

    def detect(self, semantic: np.ndarray, depth: np.ndarray) -> list[Detection]:
        n = self.noise
        h, w = semantic.shape
        min_px = max(2, int(round(n.min_pixels * (h * w) / 4800.0)))
        dets: list[Detection] = []
        for label, m in self.components(semantic, depth, min_px):
            cnt = int(m.sum())
            if self.rng.random() < n.miss_rate:
                continue
            vv, uu = np.nonzero(m)
            u0, u1, v0, v1 = uu.min(), uu.max(), vv.min(), vv.max()
            bw, bh = u1 - u0 + 1, v1 - v0 + 1
            jit = self.rng.normal(size=4) * np.maximum(
                n.box_jitter * np.array([bw, bh, bw, bh]), 0.5
            )
            box = (
                float(np.clip(u0 + jit[0], 0, w - 1)),
                float(np.clip(v0 + jit[1], 0, h - 1)),
                float(np.clip(u1 + jit[2], 0, w - 1)),
                float(np.clip(v1 + jit[3], 0, h - 1)),
            )
            trunc = u0 <= 1 or v0 <= 1 or u1 >= w - 2 or v1 >= h - 2
            conf = self.rng.normal(n.conf_mean, n.conf_sigma) - (0.1 if trunc else 0.0)
            conf *= min(1.0, 0.5 + cnt / (60.0 * h * w / 4800.0))
            mask = None
            if n.use_mask:
                mask = m & (self.rng.random(m.shape) >= n.mask_dropout)
            dets.append(Detection(label, float(np.clip(conf, 0.2, 0.99)), box, mask))
        if self.rng.random() < n.fp_rate:
            s = np.sqrt(h * w / 4800.0)
            bw, bh = self.rng.uniform(6, 24) * s, self.rng.uniform(6, 24) * s
            u0, v0 = self.rng.uniform(0, max(w - bw, 1)), self.rng.uniform(0, max(h - bh, 1))
            pool = FURNITURE + SMALL
            lab = pool[int(self.rng.integers(len(pool)))]
            dets.append(
                Detection(lab, float(self.rng.uniform(0.3, 0.6)), (u0, v0, u0 + bw, v0 + bh), None)
            )
        return dets

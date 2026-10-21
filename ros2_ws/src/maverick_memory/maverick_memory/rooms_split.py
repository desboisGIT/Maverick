"""Split an open-plan region (kitchen and living room in one open space) by furniture clusters.

The wall-gap segmentation of rooms.py cannot separate two rooms that share one open space (or are
joined by a wide opening bounded by counters rather than thin walls). Their furniture still tells
them apart: kitchen furniture (fridge, oven, counter) clusters in the kitchen part, sofa / TV /
coffee table in the living part. Each free cell of the region goes to the type of the nearest
cluster object (distance to its footprint), small leftovers are absorbed by their neighbours, and
the split is kept only when every part is big enough to be a room area. Pure numpy.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .rooms import _grow, label4


@dataclass
class SplitConfig:
    min_region_area: float = 10.0  # m^2, smaller regions are never split
    min_part_area: float = 3.0  # m^2 (free cells), every part must be at least this
    min_fragment: float = 1.0  # m^2, smaller disconnected fragments of a part are absorbed


def split_by_clusters(
    mask: np.ndarray,
    free: np.ndarray,
    seeds: dict[str, list[tuple[float, float, float, float]]],
    res: float,
    origin: tuple[float, float],
    cfg: SplitConfig | None = None,
) -> dict[str, np.ndarray] | None:
    """Partition `mask` (a region's cells) between the types in `seeds`.

    seeds: type -> object footprints (x0, y0, x1, y1) in the map frame, metres. Returns
    {type: bool mask} (disjoint, together exactly `mask`), or None when fewer than two types
    survive the size checks.
    """
    cfg = cfg or SplitConfig()
    types = [t for t, v in seeds.items() if v]
    if len(types) < 2 or mask.sum() * res * res < cfg.min_region_area:
        return None
    nx, ny = mask.shape
    xs = origin[0] + (np.arange(nx) + 0.5) * res
    ys = origin[1] + (np.arange(ny) + 0.5) * res
    dist = np.full((len(types), nx, ny), np.inf)
    for k, t in enumerate(types):
        for x0, y0, x1, y1 in seeds[t]:
            dx = np.maximum(np.maximum(x0 - xs, xs - x1), 0.0)[:, None]
            dy = np.maximum(np.maximum(y0 - ys, ys - y1), 0.0)[None, :]
            dist[k] = np.minimum(dist[k], np.hypot(dx, dy))
    lab = np.where(mask, np.argmin(dist, axis=0) + 1, 0).astype(np.int32)
    # absorb disconnected fragments of a type's cells (a stray sofa across the room)
    frag = int(round(cfg.min_fragment / (res * res)))
    for k in range(1, len(types) + 1):
        comp, n = label4((lab == k) & free)
        if n <= 1:
            continue
        sizes = np.bincount(comp.ravel())[1:]
        main = 1 + int(np.argmax(sizes))
        for c in range(1, n + 1):
            if c != main and sizes[c - 1] < frag:
                lab[comp == c] = 0
    if (lab == 0).any() & mask.any():
        lab = _grow(lab, mask & (lab == 0), nx + ny)
    parts = {}
    for k, t in enumerate(types, start=1):
        m = lab == k
        if (m & free).sum() * res * res >= cfg.min_part_area:
            parts[t] = m
    if len(parts) < 2:
        return None
    if len(parts) < len(types):  # a dropped type's cells go to the nearest remaining part
        keep = np.zeros_like(lab)
        for i, m in enumerate(parts.values(), start=1):
            keep[m] = i
        keep = _grow(keep, mask & (keep == 0), nx + ny)
        parts = {t: keep == i for i, t in enumerate(parts, start=1)}
    return parts

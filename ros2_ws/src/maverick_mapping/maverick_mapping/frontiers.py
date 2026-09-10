"""Frontier detection and clustering on the 2D projection (and a 3D variant)."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

from .voxel_map import FREE, UNKNOWN, VoxelMap

_N8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


@dataclass
class Frontier:
    """A cluster of frontier cells. Cells are (ix, iy) grid indices; xy are metres in map frame."""

    cells: np.ndarray
    centroid: tuple[float, float]
    size: int
    unknown_gain: int = 0
    viewpoint: tuple[int, int] | None = None  # reachable cell (ix, iy) to observe it from
    viewpoint_xy: tuple[float, float] | None = None


def frontier_mask(grid: np.ndarray, eight: bool = False) -> np.ndarray:
    """Free cells adjacent (4- or 8-neighbourhood) to an unknown cell."""
    unk = grid == UNKNOWN
    pad = np.pad(unk, 1, constant_values=False)
    near = pad[:-2, 1:-1] | pad[2:, 1:-1] | pad[1:-1, :-2] | pad[1:-1, 2:]
    if eight:
        near |= pad[:-2, :-2] | pad[:-2, 2:] | pad[2:, :-2] | pad[2:, 2:]
    return (grid == FREE) & near


def cluster_cells(mask: np.ndarray, min_size: int = 3) -> list[np.ndarray]:
    """8-connected components of a boolean mask, dropping those smaller than min_size."""
    cells = {(int(i), int(j)) for i, j in np.argwhere(mask)}
    out = []
    while cells:
        seed = cells.pop()
        comp = [seed]
        q = deque([seed])
        while q:
            ci, cj = q.popleft()
            for di, dj in _N8:
                nb = (ci + di, cj + dj)
                if nb in cells:
                    cells.remove(nb)
                    comp.append(nb)
                    q.append(nb)
        if len(comp) >= min_size:
            out.append(np.array(sorted(comp), dtype=int))
    return out


def detect_frontiers(
    grid: np.ndarray,
    resolution: float,
    origin: tuple[float, float],
    min_size: int = 3,
    exclude: np.ndarray | None = None,
    gain_radius: float = 1.5,
) -> list[Frontier]:
    """Frontier clusters of a 2D occupancy grid, sorted by cell index (deterministic)."""
    mask = frontier_mask(grid)
    if exclude is not None:
        mask &= ~exclude
    unk = grid == UNKNOWN
    r = int(round(gain_radius / resolution))
    out = []
    for cells in cluster_cells(mask, min_size):
        c = cells.mean(axis=0)
        cx = origin[0] + (c[0] + 0.5) * resolution
        cy = origin[1] + (c[1] + 0.5) * resolution
        i0, j0 = np.clip(np.round(c).astype(int) - r, 0, None)
        i1, j1 = np.round(c).astype(int) + r + 1
        gain = int(unk[i0:i1, j0:j1].sum())
        out.append(Frontier(cells, (float(cx), float(cy)), len(cells), gain))
    return out


def frontier_voxels_3d(vmap: VoxelMap, zmin: float = 0.3, zmax: float = 2.0) -> np.ndarray:
    """Centres of observed-free voxels in the band with an unobserved 6-neighbour in the band.

    Useful for openings the 2D projection hides (e.g. a doorway seen at one height only).
    """
    k0, k1 = vmap._k_range(zmin, zmax)
    if k1 < k0:
        return np.zeros((0, 3))
    seen = vmap.seen[:, :, k0 : k1 + 1]
    free = seen & ~vmap.occupied_mask()[:, :, k0 : k1 + 1]
    unk = ~seen
    pad = np.pad(unk, 1, constant_values=False)
    near = (
        pad[:-2, 1:-1, 1:-1]
        | pad[2:, 1:-1, 1:-1]
        | pad[1:-1, :-2, 1:-1]
        | pad[1:-1, 2:, 1:-1]
        | pad[1:-1, 1:-1, :-2]
        | pad[1:-1, 1:-1, 2:]
    )
    idx = np.argwhere(free & near)
    idx[:, 2] += k0
    return vmap.centers(idx)

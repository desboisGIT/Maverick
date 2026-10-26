"""Clearance report around the robot from the voxel map (numpy only, no ROS).

The reactive guard of the skill node (maverick_skills.guard) needs to know, at the map's own rate
(5 Hz, not the planner grid's 1 Hz), how far the nearest occupied cell is in each direction. This
module reduces the obstacle band of the voxel map around the body centre to the nearest occupied
cell per bearing sector.

Distances are from the body centre to the occupied cell's centre. A cell's centre is up to half a
cell (0.05 m) from the surface it stands for, and walls in the map are fat at doors (S3 plan), so
thresholds on these numbers are in map space, not true clearance.
"""

from __future__ import annotations

import math

import numpy as np

from .voxel_map import VoxelMap


def occupied_window(
    vmap: VoxelMap, center: tuple[float, float], radius: float, band: tuple[float, float]
) -> tuple[np.ndarray, tuple[float, float]]:
    """Occupied columns (any occupied voxel in the height `band`) within a square window.

    Returns (bool array [ix, iy], (x, y) of the corner of cell [0, 0]); an empty array if the
    window is outside the map.
    """
    nx, ny, _ = vmap.shape
    if nx == 0:
        return np.zeros((0, 0), bool), (0.0, 0.0)
    res = vmap.res
    i0 = int(math.floor((center[0] - radius - vmap.origin[0]) / res))
    i1 = int(math.floor((center[0] + radius - vmap.origin[0]) / res)) + 1
    j0 = int(math.floor((center[1] - radius - vmap.origin[1]) / res))
    j1 = int(math.floor((center[1] + radius - vmap.origin[1]) / res)) + 1
    i0, i1, j0, j1 = max(i0, 0), min(i1, nx), max(j0, 0), min(j1, ny)
    k0, k1 = vmap._k_range(*band)
    if i1 <= i0 or j1 <= j0 or k1 < k0:
        return np.zeros((0, 0), bool), (0.0, 0.0)
    occ = (vmap.logodds[i0:i1, j0:j1, k0 : k1 + 1] > vmap.occ_thresh).any(axis=2)
    return occ, (float(vmap.origin[0] + i0 * res), float(vmap.origin[1] + j0 * res))


def sector_clearance(
    occ: np.ndarray,
    res: float,
    origin: tuple[float, float],
    center: tuple[float, float],
    rmax: float = 0.8,
    n_sectors: int = 24,
) -> list[float]:
    """Nearest occupied cell centre per bearing sector, metres (`rmax` where there is none).

    Sector k covers bearings [-pi + k w, -pi + (k + 1) w), w = 2 pi / n_sectors, counter-clockwise
    from +x in the map frame.
    """
    out = np.full(n_sectors, rmax)
    if occ.size == 0:
        return [float(v) for v in out]
    cells = np.argwhere(occ)
    if len(cells) == 0:
        return [float(v) for v in out]
    dx = origin[0] + (cells[:, 0] + 0.5) * res - center[0]
    dy = origin[1] + (cells[:, 1] + 0.5) * res - center[1]
    r = np.hypot(dx, dy)
    near = r < rmax
    if not near.any():
        return [float(v) for v in out]
    ang = np.arctan2(dy[near], dx[near])
    k = np.floor((ang + math.pi) / (2 * math.pi / n_sectors)).astype(int) % n_sectors
    np.minimum.at(out, k, r[near])
    return [round(float(v), 3) for v in out]

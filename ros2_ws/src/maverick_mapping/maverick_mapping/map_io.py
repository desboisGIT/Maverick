"""Save and load a VoxelMap as .npz (the referee scores the drone's own map offline).

`save_map` is called by mapping_node's `/maverick/map/dump` service; `load_map` rebuilds a
VoxelMap (same lattice, origin, log-odds, seen mask) for scoring against ground truth.
"""

from __future__ import annotations

import numpy as np

from .voxel_map import VoxelMap


def save_map(vmap: VoxelMap, path: str) -> None:
    np.savez_compressed(
        path,
        logodds=vmap.logodds,
        seen=vmap.seen,
        origin=vmap.origin,
        res=np.float64(vmap.res),
        lattice=np.float64(vmap._lattice),
        occ_thresh=np.float64(vmap.occ_thresh),
        frames=np.int64(vmap.frames),
    )


def load_map(path: str) -> VoxelMap:
    d = np.load(path)
    vmap = VoxelMap(
        float(d["res"]), grid_offset=float(d["lattice"]), occ_thresh=float(d["occ_thresh"])
    )
    vmap.origin = np.asarray(d["origin"], float)
    vmap.logodds = d["logodds"].astype(np.float32)
    vmap.seen = d["seen"].astype(bool)
    vmap.frames = int(d["frames"])
    return vmap

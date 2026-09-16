"""Delta encoding of the occupied voxels for the gateway `map_update` event (numpy only).

Voxel indices are absolute (relative to the map lattice, not to the growing array), so they stay
valid when the VoxelMap grows. Voxel centre = origin + (index + 0.5) * resolution with
origin = (lattice, lattice, lattice), as in docs/gateway-events.md.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .voxel_map import VoxelMap

_M = 4096
_OFF = 2048


def encode(idx: np.ndarray) -> np.ndarray:
    """(n, 3) absolute indices in [-2048, 2047] -> unique sorted int64 codes."""
    i = np.asarray(idx, dtype=np.int64).reshape(-1, 3) + _OFF
    return np.unique((i[:, 0] * _M + i[:, 1]) * _M + i[:, 2])


def decode(codes: np.ndarray) -> np.ndarray:
    c = np.asarray(codes, dtype=np.int64)
    return np.stack([c // (_M * _M), (c // _M) % _M, c % _M], axis=1) - _OFF


def occupied_codes(vmap: VoxelMap) -> np.ndarray:
    if vmap.logodds.size == 0:
        return np.zeros(0, np.int64)
    local = np.argwhere(vmap.occupied_mask())
    shift = np.round((vmap.origin - vmap._lattice) / vmap.res).astype(np.int64)
    return encode(local + shift)


@dataclass
class Delta:
    add: np.ndarray  # (n, 3)
    remove: np.ndarray  # (m, 3)
    reset: bool

    def empty(self) -> bool:
        return not self.reset and len(self.add) == 0 and len(self.remove) == 0

    def to_json(self, resolution: float, origin: float, t: float, seq: int) -> dict:
        return {
            "t": t,
            "seq": seq,
            "resolution": resolution,
            "origin": [origin, origin, origin],
            "reset": self.reset,
            "add": self.add.reshape(-1).astype(int).tolist(),
            "remove": self.remove.reshape(-1).astype(int).tolist(),
        }


class DeltaEncoder:
    """Tracks what the consumers have: `delta(vmap)` is the change since the previous call."""

    def __init__(self) -> None:
        self._prev = np.zeros(0, np.int64)
        self._first = True

    def delta(self, vmap: VoxelMap, reset: bool = False) -> Delta:
        cur = occupied_codes(vmap)
        if reset or self._first:
            self._prev, self._first = cur, False
            return Delta(decode(cur), np.zeros((0, 3), np.int64), True)
        add = np.setdiff1d(cur, self._prev, assume_unique=True)
        rem = np.setdiff1d(self._prev, cur, assume_unique=True)
        self._prev = cur
        return Delta(decode(add), decode(rem), False)


class OccupiedSet:
    """Consumer side: the occupied voxels rebuilt from a stream of deltas."""

    def __init__(self) -> None:
        self.codes = np.zeros(0, np.int64)
        self.resolution = 0.1
        self.origin = [0.05, 0.05, 0.05]

    def apply(self, d: dict) -> None:
        self.resolution = float(d.get("resolution", self.resolution))
        self.origin = list(d.get("origin", self.origin))
        if d.get("reset"):
            self.codes = np.zeros(0, np.int64)
        add = np.asarray(d.get("add", []), dtype=np.int64).reshape(-1, 3)
        rem = np.asarray(d.get("remove", []), dtype=np.int64).reshape(-1, 3)
        if len(rem):
            self.codes = np.setdiff1d(self.codes, encode(rem), assume_unique=True)
        if len(add):
            self.codes = np.union1d(self.codes, encode(add))

    def snapshot(self) -> dict:
        """A `map_update` payload that resets the console and adds every occupied voxel."""
        return {
            "resolution": self.resolution,
            "origin": self.origin,
            "reset": True,
            "add": decode(self.codes).reshape(-1).astype(int).tolist(),
            "remove": [],
        }

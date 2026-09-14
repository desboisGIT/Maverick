"""Pure frame processing for the camera driver: pairing by sim time and downsampling.

numpy only; no Gazebo, no ROS. Depth is metres float32 (Gazebo: inf / nan beyond the clip planes),
semantic is the label image as integer class ids.
"""

from __future__ import annotations

from collections import deque
from typing import Any

import numpy as np

STREAMS = ("rgb", "depth", "semantic")


def depth_to_mm(depth_m: np.ndarray, near: float, far: float) -> np.ndarray:
    """Float metres -> u16 millimetres; 0 for nan, inf, non-positive and out-of-range values."""
    d = np.asarray(depth_m, dtype=np.float32)
    ok = np.isfinite(d) & (d >= near - 1e-3) & (d <= far + 1e-3)
    mm = np.zeros(d.shape, dtype=np.uint16)
    mm[ok] = np.rint(d[ok] * 1000.0).astype(np.uint16)
    return mm


def downsample_depth(depth_mm: np.ndarray, factor: int) -> np.ndarray:
    """Block minimum over valid (non-zero) pixels, 0 if a block has none.

    The nearest return is the safe choice for obstacle mapping and avoids inventing in-between
    depths at object edges.
    """
    h, w = depth_mm.shape
    if h % factor or w % factor:
        raise ValueError(f"{w}x{h} is not divisible by {factor}")
    blocks = depth_mm.reshape(h // factor, factor, w // factor, factor).swapaxes(1, 2)
    blocks = blocks.reshape(h // factor, w // factor, factor * factor)
    big = np.where(blocks == 0, np.uint16(65535), blocks)
    out = big.min(axis=2)
    out[out == 65535] = 0
    return out.astype(np.uint16)


def downsample_semantic(labels: np.ndarray, factor: int) -> np.ndarray:
    """Nearest sample at the block centre (class ids must not be averaged)."""
    h, w = labels.shape
    if h % factor or w % factor:
        raise ValueError(f"{w}x{h} is not divisible by {factor}")
    o = factor // 2
    return np.ascontiguousarray(labels[o::factor, o::factor]).astype(np.uint8)


class Pairer:
    """Pairs one image per stream by simulation time.

    The three Gazebo sensors of the same link render in the same Sensors-system tick, so their
    stamps normally match exactly; `tolerance_ns` allows for sensors updating on neighbouring
    ticks. Complete sets are emitted in time order; anything older than an emitted set is dropped.
    """

    def __init__(self, tolerance_ns: int = 5_000_000, depth: int = 6) -> None:
        self.tol = tolerance_ns
        self._q: dict[str, deque[tuple[int, Any]]] = {s: deque(maxlen=depth) for s in STREAMS}

    def add(self, stream: str, t_ns: int, data: Any) -> tuple[int, dict[str, Any]] | None:
        """Returns (time_ns of the rgb frame, {stream: data}) when a full set is available."""
        self._q[stream].append((t_ns, data))
        # candidates anchored at the newest item of every stream, oldest anchor first
        for t_a, _ in sorted(self._q[stream]):
            picked: dict[str, tuple[int, Any]] = {}
            for s in STREAMS:
                best = None
                for t, d in self._q[s]:
                    if abs(t - t_a) <= self.tol and (
                        best is None or abs(t - t_a) < abs(best[0] - t_a)
                    ):
                        best = (t, d)
                if best is None:
                    break
                picked[s] = best
            else:
                limit = max(t for t, _ in picked.values())
                for s in STREAMS:
                    keep = [(t, d) for t, d in self._q[s] if t > limit]
                    self._q[s].clear()
                    self._q[s].extend(keep)
                return picked["rgb"][0], {s: picked[s][1] for s in STREAMS}
        return None

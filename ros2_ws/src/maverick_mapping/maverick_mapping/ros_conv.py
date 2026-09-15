"""ROS message <-> numpy conversions and stamp synchronisation, without importing rclpy.

Everything takes duck-typed message objects (attribute access only), so it is unit-tested with
SimpleNamespace stand-ins and runs unchanged on real sensor_msgs / geometry_msgs / nav_msgs.
"""

from __future__ import annotations

import math
from collections import deque
from collections.abc import Callable
from typing import Any

import numpy as np

from .voxel_map import CameraIntrinsics

_DTYPES = {
    "16UC1": np.dtype("<u2"),
    "mono16": np.dtype("<u2"),
    "mono8": np.dtype("u1"),
    "8UC1": np.dtype("u1"),
    "32FC1": np.dtype("<f4"),
}


def stamp_to_sec(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def stamp_key(stamp: Any) -> int:
    """Exact integer key (ns) of a header stamp."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def image_to_array(msg: Any) -> np.ndarray:
    """sensor_msgs/Image (single channel) -> (height, width) array of the message dtype."""
    dt = _DTYPES.get(msg.encoding)
    if dt is None:
        raise ValueError(f"unsupported image encoding {msg.encoding!r}")
    if getattr(msg, "is_bigendian", 0) and dt.itemsize > 1:
        dt = dt.newbyteorder(">")
    raw = np.frombuffer(bytes(msg.data), dtype=dt)
    row = int(msg.step) // dt.itemsize if msg.step else int(msg.width)
    arr = raw[: row * int(msg.height)].reshape(int(msg.height), row)[:, : int(msg.width)]
    return arr.astype(dt.newbyteorder("="), copy=False)


def depth_to_metres(msg: Any) -> np.ndarray:
    """Depth Image (16UC1 millimetres, 0 = invalid, or 32FC1 metres) -> float32 m, NaN invalid."""
    a = image_to_array(msg)
    d = a.astype(np.float32) * 1e-3 if a.dtype.kind == "u" else a.astype(np.float32)
    d[~np.isfinite(d) | (d <= 0.0)] = np.nan
    return d


def info_to_intrinsics(msg: Any) -> CameraIntrinsics:
    k = list(msg.k)
    return CameraIntrinsics(int(msg.width), int(msg.height), k[0], k[4], k[2], k[5])


def intrinsics_for(intr: CameraIntrinsics, width: int, height: int) -> CameraIntrinsics:
    """Intrinsics scaled to another image size (same field of view)."""
    if (intr.width, intr.height) == (width, height):
        return intr
    return intr.scaled(width / intr.width, height / intr.height)


def hfov_deg(intr: CameraIntrinsics) -> float:
    return math.degrees(2.0 * math.atan(intr.width / (2.0 * intr.fx)))


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quat(r: np.ndarray) -> tuple[float, float, float, float]:
    """Rotation matrix -> quaternion (x, y, z, w)."""
    t = float(np.trace(r))
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, s / 4)
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2
    q = [0.0] * 4
    q[i] = s / 4
    q[3] = (r[k, j] - r[j, k]) / s
    q[j] = (r[j, i] + r[i, j]) / s
    q[k] = (r[k, i] + r[i, k]) / s
    return (q[0], q[1], q[2], q[3])


def pose_to_tr(msg: Any) -> tuple[np.ndarray, np.ndarray]:
    """geometry_msgs/PoseStamped (camera optical frame in the map frame) -> (t_wc, r_wc)."""
    p, q = msg.pose.position, msg.pose.orientation
    return np.array([p.x, p.y, p.z], float), quat_to_matrix(q.x, q.y, q.z, q.w)


def camera_euler(r_wc: np.ndarray) -> tuple[float, float, float]:
    """(yaw, pitch, roll) of a camera optical-frame rotation, in the gateway convention.

    Optical axes: x right, y down, z forward. yaw ccw from +x, pitch positive nose up, roll
    positive right side down.
    """
    fwd, right = r_wc[:, 2], r_wc[:, 0]
    yaw = math.atan2(fwd[1], fwd[0])
    pitch = math.asin(max(-1.0, min(1.0, float(fwd[2]))))
    roll = -math.asin(max(-1.0, min(1.0, float(right[2]))))
    return yaw, pitch, roll


# --- occupancy grid ---------------------------------------------------------------------------
def grid_to_fields(grid: np.ndarray, res: float, origin: tuple[float, float]) -> dict:
    """[ix, iy] int8 grid -> OccupancyGrid fields (row-major with rows = y, so data = grid.T)."""
    nx, ny = grid.shape
    return {
        "width": int(nx),
        "height": int(ny),
        "resolution": float(res),
        "origin_x": float(origin[0]),
        "origin_y": float(origin[1]),
        "data": np.ascontiguousarray(grid.T, dtype=np.int8).ravel().tolist(),
    }


def occupancy_to_grid(msg: Any) -> tuple[np.ndarray, float, tuple[float, float]]:
    """nav_msgs/OccupancyGrid -> ([ix, iy] int8 grid, resolution, (x, y) origin of cell [0, 0])."""
    w, h = int(msg.info.width), int(msg.info.height)
    data = np.asarray(msg.data, dtype=np.int8).reshape(h, w)
    o = msg.info.origin.position
    return np.ascontiguousarray(data.T), float(msg.info.resolution), (float(o.x), float(o.y))


# --- time synchronisation ---------------------------------------------------------------------
class SyncBuffer:
    """Match messages of several topics by header stamp (the bridge stamps one frame identically).

    push(name, t, item) returns a list with at most one complete set {name: item}, in which every
    item is within `tol` seconds of the one just pushed. Items older than a completed set are
    discarded; each queue is capped at `maxlen`.
    """

    def __init__(self, names: list[str], tol: float = 0.02, maxlen: int = 20) -> None:
        self.names = list(names)
        self.tol = tol
        self.maxlen = maxlen
        self.q: dict[str, deque] = {n: deque(maxlen=maxlen) for n in names}

    def push(self, name: str, t: float, item: Any) -> list[dict[str, Any]]:
        self.q[name].append((t, item))
        picked: dict[str, tuple[float, Any]] = {name: (t, item)}
        for other in self.names:
            if other == name:
                continue
            best = min(self.q[other], key=lambda e: abs(e[0] - t), default=None)
            if best is None or abs(best[0] - t) > self.tol:
                return []
            picked[other] = best
        t_max = max(v[0] for v in picked.values())
        for n in self.names:  # drop the used and everything older
            self.q[n] = deque((e for e in self.q[n] if e[0] > t_max + 1e-9), maxlen=self.maxlen)
        return [{n: picked[n][1] for n in self.names}]


class RateLimiter:
    """Allow an action at most `hz` times per second of the supplied clock."""

    def __init__(self, hz: float, clock: Callable[[], float]) -> None:
        self.period = 1.0 / hz if hz > 0 else 0.0
        self.clock = clock
        self._last = -math.inf

    def ready(self) -> bool:
        now = self.clock()
        if now - self._last >= self.period:
            self._last = now
            return True
        return False

"""Pure core of the visual-odometry adapter (numpy, no ROS).

rtabmap's rgbd_odometry reports the pose of the camera optical frame relative to where it started
(identity there). PX4 wants the pose of the body in its own local frame. The adapter anchors the
odometry once, to the PX4-based camera pose (`/maverick/camera/pose`, map frame ENU, origin = PX4
local origin) at the first valid odometry frame, then maps every later odometry pose through the
same anchor:

    T_map_cam(t)  = T_map_odom * T_odom_cam(t),   T_map_odom = T_map_cam(t0) * inv(T_odom_cam(t0))
    T_map_body(t) = T_map_cam(t) * inv(T_body_cam)

All matrices are 4x4 homogeneous, quaternions are (x, y, z, w) as in ROS messages.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

# rtabmap marks a lost track with a huge covariance (9999) and a null pose.
LOST_COVARIANCE = 999.0


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        raise ValueError("zero quaternion")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quat(r: np.ndarray) -> tuple[float, float, float, float]:
    """(x, y, z, w) with w >= 0."""
    t = r[0, 0] + r[1, 1] + r[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x, y, z = (
            0.25 * s,
            (r[2, 1] - r[1, 2]) / s,
            (r[0, 2] - r[2, 0]) / s,
            (r[1, 0] - r[0, 1]) / s,
        )
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        w, x, y, z = (
            (r[2, 1] - r[1, 2]) / s,
            0.25 * s,
            (r[0, 1] + r[1, 0]) / s,
            (r[0, 2] + r[2, 0]) / s,
        )
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        w, x, y, z = (
            (r[0, 2] - r[2, 0]) / s,
            (r[0, 1] + r[1, 0]) / s,
            0.25 * s,
            (r[1, 2] + r[2, 1]) / s,
        )
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        w, x, y, z = (
            (r[1, 0] - r[0, 1]) / s,
            (r[0, 2] + r[2, 0]) / s,
            (r[1, 2] + r[2, 1]) / s,
            0.25 * s,
        )
    if w < 0:
        x, y, z, w = -x, -y, -z, -w
    return float(x), float(y), float(z), float(w)


def make_T(position, quat_xyzw) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(*quat_xyzw)
    T[:3, 3] = position
    return T


def invert_T(T: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return out


def body_to_camera(r_flu_opt, t_flu) -> np.ndarray:
    """T_body_cam: the optical frame in the FLU body frame (from camera_mount.json)."""
    T = np.eye(4)
    T[:3, :3] = np.asarray(r_flu_opt, float)
    T[:3, 3] = t_flu
    return T


def odom_is_valid(position, covariance_diag_max: float, max_abs: float = 1e4) -> bool:
    """False for rtabmap's lost-track message (null pose, covariance 9999) and non-finite poses."""
    p = np.asarray(position, float)
    return bool(np.all(np.isfinite(p)) and np.all(np.abs(p) < max_abs)) and (
        covariance_diag_max < LOST_COVARIANCE
    )


@dataclass
class VisionPose:
    stamp: float
    T_map_body: np.ndarray
    # twist of the body in its own (FLU) frame, NaN when unknown
    v_body: np.ndarray
    pos_var: float
    ori_var: float
    vel_var: float


class OdomAnchor:
    """Keeps recent PX4-based camera poses and anchors the odometry on the first valid frame."""

    def __init__(self, pose_tol_s: float = 0.02, history: int = 50) -> None:
        self.tol = pose_tol_s
        self._poses: deque[tuple[float, np.ndarray]] = deque(maxlen=history)
        self.T_map_odom: np.ndarray | None = None

    def push_pose(self, stamp: float, T_map_cam: np.ndarray) -> None:
        self._poses.append((stamp, T_map_cam))

    def reset(self) -> None:
        self.T_map_odom = None

    def _pose_at(self, stamp: float) -> np.ndarray | None:
        best = None
        for t, T in self._poses:
            if abs(t - stamp) <= self.tol and (
                best is None or abs(t - stamp) < abs(best[0] - stamp)
            ):
                best = (t, T)
        return None if best is None else best[1]

    def to_map(self, stamp: float, T_odom_cam: np.ndarray) -> np.ndarray | None:
        """T_map_cam for an odometry sample, or None while no anchor can be set yet."""
        if self.T_map_odom is None:
            ref = self._pose_at(stamp)
            if ref is None:
                return None
            self.T_map_odom = ref @ invert_T(T_odom_cam)
        return self.T_map_odom @ T_odom_cam


def jump_too_large(
    T_prev: np.ndarray | None, T_new: np.ndarray, dt: float, vmax: float = 4.0
) -> bool:
    """Reject a pose that moved faster than `vmax` m/s from the previous accepted one."""
    if T_prev is None:
        return False
    d = float(np.linalg.norm(T_new[:3, 3] - T_prev[:3, 3]))
    return d > vmax * max(dt, 0.05)

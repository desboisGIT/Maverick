"""Camera optical-frame pose in the map frame (ENU) from the PX4 estimate and the fixed mount.

Pure Python (no numpy). Conventions:
- PX4 local frame NED, body FRD; VehicleAttitude.q = [w, x, y, z] rotates FRD vectors into NED.
- Map frame ENU (same rule as maverick_mapping.frames: x_enu = y_ned, y_enu = x_ned,
  z_enu = -z_ned; test_camera_pose.py checks both agree). The map origin is the PX4 local origin.
- Mount (camera_mount.json) is the Gazebo sensor frame in the FLU body frame (x forward, y left,
  z up), rpy as in SDF (R = Rz(yaw) Ry(pitch) Rx(roll)).
- Optical frame: z forward, x right, y down (REP 103 camera_optical).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

Mat = tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
Vec = tuple[float, float, float]

DEFAULT_MOUNT_PATH = "/maverick/sim/models/maverick/camera_mount.json"

# FLU <-> FRD is the same diagonal flip in both directions.
_FLIP: Mat = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))
# Optical axes written in the FLU sensor frame, as columns: x_opt = -y, y_opt = -z, z_opt = +x.
_FLU_OPT: Mat = ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0))
# NED -> ENU basis change (proper rotation, det +1).
_NED_TO_ENU: Mat = ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0))


def matmul(a: Mat, b: Mat) -> Mat:
    return tuple(  # type: ignore[return-value]
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3)
    )


def matvec(a: Mat, v: Vec) -> Vec:
    return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))  # type: ignore[return-value]


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> Mat:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def quat_to_matrix(w: float, x: float, y: float, z: float) -> Mat:
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        raise ValueError("zero quaternion")
    w, x, y, z = w / n, x / n, y / n, z / n
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def matrix_to_quat(r: Mat) -> tuple[float, float, float, float]:
    """Returns (w, x, y, z) with w >= 0."""
    t = r[0][0] + r[1][1] + r[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x = 0.25 * s, (r[2][1] - r[1][2]) / s
        y, z = (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2
        w, x = (r[2][1] - r[1][2]) / s, 0.25 * s
        y, z = (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2
        w, x = (r[0][2] - r[2][0]) / s, (r[0][1] + r[1][0]) / s
        y, z = 0.25 * s, (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2
        w, x = (r[1][0] - r[0][1]) / s, (r[0][2] + r[2][0]) / s
        y, z = (r[1][2] + r[2][1]) / s, 0.25 * s
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    return w, x, y, z


@dataclass(frozen=True)
class CameraMount:
    frame_id: str
    xyz_flu: Vec
    rpy_flu: Vec
    horizontal_fov: float
    rgb_size: tuple[int, int]
    depth_size: tuple[int, int]
    clip: tuple[float, float]
    rate_hz: float

    @staticmethod
    def load(path: str | Path = DEFAULT_MOUNT_PATH) -> CameraMount:
        d = json.loads(Path(path).read_text())
        return CameraMount(
            frame_id=d["frame_id"],
            xyz_flu=tuple(float(v) for v in d["xyz_flu"]),  # type: ignore[arg-type]
            rpy_flu=tuple(float(v) for v in d["rpy_flu"]),  # type: ignore[arg-type]
            horizontal_fov=float(d["horizontal_fov"]),
            rgb_size=tuple(d["rgb_size"]),  # type: ignore[arg-type]
            depth_size=tuple(d["depth_size"]),  # type: ignore[arg-type]
            clip=tuple(float(v) for v in d["clip"]),  # type: ignore[arg-type]
            rate_hz=float(d["rate_hz"]),
        )

    def intrinsics(self, width: int, height: int) -> tuple[float, float, float, float]:
        """fx, fy, cx, cy for an image of this size (square pixels, principal point at centre)."""
        f = width / (2.0 * math.tan(self.horizontal_fov / 2.0))
        return f, f, width / 2.0, height / 2.0


def camera_pose_enu(
    position_ned: Vec, q_frd_ned: tuple[float, float, float, float], mount: CameraMount
) -> tuple[Vec, tuple[float, float, float, float]]:
    """Optical frame position (ENU, m) and orientation (w, x, y, z; optical -> ENU)."""
    r_ned_frd = quat_to_matrix(*q_frd_ned)
    r_flu_cam = rpy_to_matrix(*mount.rpy_flu)
    r_frd_opt = matmul(_FLIP, matmul(r_flu_cam, _FLU_OPT))
    r_enu_opt = matmul(_NED_TO_ENU, matmul(r_ned_frd, r_frd_opt))
    t_ned = matvec(r_ned_frd, matvec(_FLIP, mount.xyz_flu))
    p_ned = (position_ned[0] + t_ned[0], position_ned[1] + t_ned[1], position_ned[2] + t_ned[2])
    return matvec(_NED_TO_ENU, p_ned), matrix_to_quat(r_enu_opt)

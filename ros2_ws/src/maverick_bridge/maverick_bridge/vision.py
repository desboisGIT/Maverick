"""Visual-odometry relay rules and frame conversion (pure Python, no ROS imports).

The host sends the body pose in the map frame (ROS ENU, body FLU, quaternion x y z w) as
nav_msgs/Odometry on /maverick/vision/odometry. PX4's external-vision input wants NED, body FRD
and (w, x, y, z). The conversion is a fixed change of basis and the checks are the same as for
setpoints: nothing old or malformed reaches PX4. The relay never repeats or invents a sample, so
when the odometry stops, PX4's EKF falls back to optical flow by itself.

    v_ned = M v_enu,  v_flu = F v_frd  =>  R_ned_frd = M R_enu_flu F,   p_ned = M p_enu
    M swaps x and y and flips z (its own inverse), F = diag(1, -1, -1)
"""

from __future__ import annotations

import math
from dataclasses import dataclass

VISION_MAX_AGE_S = 0.200
VISION_MAX_FUTURE_S = 0.050

Mat = tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]

_M: Mat = ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0))
_F: Mat = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))


def _mm(a: Mat, b: Mat) -> Mat:
    return tuple(  # type: ignore[return-value]
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3)
    )


def _quat_to_matrix(x: float, y: float, z: float, w: float) -> Mat:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        raise ValueError("zero quaternion")
    x, y, z, w = x / n, y / n, z / n, w / n
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def _matrix_to_quat_wxyz(r: Mat) -> tuple[float, float, float, float]:
    t = r[0][0] + r[1][1] + r[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x, y, z = (
            0.25 * s,
            (r[2][1] - r[1][2]) / s,
            (r[0][2] - r[2][0]) / s,
            (r[1][0] - r[0][1]) / s,
        )
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2
        w, x, y, z = (
            (r[2][1] - r[1][2]) / s,
            0.25 * s,
            (r[0][1] + r[1][0]) / s,
            (r[0][2] + r[2][0]) / s,
        )
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2
        w, x, y, z = (
            (r[0][2] - r[2][0]) / s,
            (r[0][1] + r[1][0]) / s,
            0.25 * s,
            (r[1][2] + r[2][1]) / s,
        )
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2
        w, x, y, z = (
            (r[1][0] - r[0][1]) / s,
            (r[0][2] + r[2][0]) / s,
            (r[1][2] + r[2][1]) / s,
            0.25 * s,
        )
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    return w, x, y, z


@dataclass(frozen=True)
class VisionOdometry:
    """What goes into px4_msgs/VehicleOdometry (pose_frame NED, velocity_frame BODY_FRD)."""

    sample_time_s: float  # camera frame stamp, bridge clock (-> timestamp_sample)
    position_ned: tuple[float, float, float]
    q_frd_ned: tuple[float, float, float, float]  # w, x, y, z
    velocity_frd: tuple[float, float, float]  # NaN when the host sent none
    position_var: tuple[float, float, float]  # NED order
    orientation_var: tuple[float, float, float]  # roll, pitch, yaw about the FRD axes
    velocity_var: tuple[float, float, float]


def convert(
    sample_time_s: float,
    position_enu: tuple[float, float, float],
    quat_xyzw_enu_flu: tuple[float, float, float, float],
    velocity_flu: tuple[float, float, float],
    pos_var_enu: tuple[float, float, float],
    ori_var_flu: tuple[float, float, float],
    vel_var_flu: tuple[float, float, float],
) -> VisionOdometry:
    r_ned_frd = _mm(_mm(_M, _quat_to_matrix(*quat_xyzw_enu_flu)), _F)
    px, py, pz = position_enu
    vx, vy, vz = velocity_flu
    pvx, pvy, pvz = pos_var_enu
    return VisionOdometry(
        sample_time_s=sample_time_s,
        position_ned=(py, px, -pz),
        q_frd_ned=_matrix_to_quat_wxyz(r_ned_frd),
        velocity_frd=(vx, -vy, -vz),
        position_var=(pvy, pvx, pvz),
        orientation_var=ori_var_flu,
        velocity_var=vel_var_flu,
    )


@dataclass
class VisionCounters:
    forwarded: int = 0
    dropped_stale: int = 0
    dropped_invalid: int = 0


class VisionRelay:
    def __init__(
        self,
        max_age_s: float = VISION_MAX_AGE_S,
        max_future_s: float = VISION_MAX_FUTURE_S,
    ) -> None:
        self.max_age = max_age_s
        self.max_future = max_future_s
        self.counters = VisionCounters()

    def on_odometry(
        self,
        stamp_s: float,
        now_s: float,
        position_enu: tuple[float, float, float],
        quat_xyzw_enu_flu: tuple[float, float, float, float],
        velocity_flu: tuple[float, float, float],
        pos_var_enu: tuple[float, float, float],
        ori_var_flu: tuple[float, float, float],
        vel_var_flu: tuple[float, float, float],
    ) -> VisionOdometry | None:
        """The converted sample, or None (and a count) if it must not reach PX4.

        Position, attitude and variances must be finite and the quaternion non-zero; a NaN velocity
        is allowed (means "no velocity", PX4 then fuses position and yaw only).
        """
        k = self.counters
        pose_vals = (*position_enu, *quat_xyzw_enu_flu, *pos_var_enu, *ori_var_flu, *vel_var_flu)
        if (
            not all(math.isfinite(v) for v in pose_vals)
            or math.sqrt(sum(q * q for q in quat_xyzw_enu_flu)) < 1e-6
            or any(v < 0.0 for v in (*pos_var_enu, *ori_var_flu, *vel_var_flu))
            or not math.isfinite(stamp_s)
        ):
            k.dropped_invalid += 1
            return None
        age = now_s - stamp_s
        if age > self.max_age or age < -self.max_future:
            k.dropped_stale += 1
            return None
        k.forwarded += 1
        return convert(
            stamp_s,
            position_enu,
            quat_xyzw_enu_flu,
            velocity_flu,
            pos_var_enu,
            ori_var_flu,
            vel_var_flu,
        )

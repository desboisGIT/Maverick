"""Host node `vo_adapter`: rtabmap odometry -> body pose in the map frame.

Publishes /maverick/vision/odometry.

  in  /maverick/vo/odom             nav_msgs/Odometry  rtabmap rgbd_odometry (camera optical frame
                                    relative to its start pose; header.stamp = image stamp)
  in  /maverick/camera/pose         geometry_msgs/PoseStamped  PX4-based camera pose in the map
                                    frame (used once, to anchor the odometry at the first frame)
  out /maverick/vision/odometry     nav_msgs/Odometry  frame `map` (ENU, origin = PX4 local origin),
                                    child `base_link` (FLU body), header.stamp = camera frame stamp;
                                    best-effort; Zenoh carries it to the relay, which converts
                                    it to px4_msgs VehicleOdometry

Nothing is published while rtabmap is lost (flow carries PX4's estimate then) or while no PX4
camera pose matches the first odometry frame. Parameters: send_velocity (false), min_pos_var
(0.0004 m^2), min_ori_var (0.0004 rad^2), min_vel_var (0.01), max_speed (4 m/s jump filter),
mount (camera_mount.json).
"""

from __future__ import annotations

import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from maverick_bridge.camera_pose import DEFAULT_MOUNT_PATH, CameraMount, mount_in_body_flu
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from maverick_vo.core import (
    OdomAnchor,
    body_to_camera,
    invert_T,
    jump_too_large,
    make_T,
    matrix_to_quat,
    odom_is_valid,
)


def stamp_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def msg_T(pose) -> np.ndarray:
    p, q = pose.position, pose.orientation
    return make_T((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


class VoAdapter(Node):
    def __init__(self) -> None:
        super().__init__("vo_adapter")
        p = self.declare_parameter
        self.send_velocity = bool(p("send_velocity", False).value)
        self.min_pos_var = float(p("min_pos_var", 4e-4).value)
        self.min_ori_var = float(p("min_ori_var", 4e-4).value)
        self.min_vel_var = float(p("min_vel_var", 1e-2).value)
        self.max_speed = float(p("max_speed", 4.0).value)
        mount = CameraMount.load(os.environ.get("CAMERA_MOUNT", DEFAULT_MOUNT_PATH))
        r_flu_opt, t_flu = mount_in_body_flu(mount)
        self.T_body_cam = body_to_camera(r_flu_opt, t_flu)
        self.T_cam_body = invert_T(self.T_body_cam)
        self.anchor = OdomAnchor()
        self.last_T: np.ndarray | None = None
        self.last_t = 0.0
        self.n_in = self.n_out = self.n_lost = self.n_jump = self.n_wait = 0
        self.pub = self.create_publisher(
            Odometry, "/maverick/vision/odometry", qos_profile_sensor_data
        )
        self.create_subscription(
            PoseStamped, "/maverick/camera/pose", self._on_pose, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "/maverick/vo/odom", self._on_odom, qos_profile_sensor_data
        )
        self.create_timer(5.0, self._log)

    def _log(self) -> None:
        self.get_logger().info(
            f"vo in {self.n_in} out {self.n_out} lost {self.n_lost} jump {self.n_jump} "
            f"waiting-for-anchor {self.n_wait} anchored {self.anchor.T_map_odom is not None}"
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        self.anchor.push_pose(stamp_s(msg.header.stamp), msg_T(msg.pose))

    def _on_odom(self, msg: Odometry) -> None:
        self.n_in += 1
        cov = np.array(msg.pose.covariance).reshape(6, 6)
        pos = msg.pose.pose.position
        o = msg.pose.pose.orientation
        if not odom_is_valid((pos.x, pos.y, pos.z), float(np.max(np.diag(cov)))) or (
            o.x == o.y == o.z == o.w == 0.0
        ):
            self.n_lost += 1
            return
        t = stamp_s(msg.header.stamp)
        T_map_cam = self.anchor.to_map(t, msg_T(msg.pose.pose))
        if T_map_cam is None:
            self.n_wait += 1
            return
        T_map_body = T_map_cam @ self.T_cam_body
        if jump_too_large(self.last_T, T_map_body, t - self.last_t, self.max_speed):
            self.n_jump += 1
            return
        self.last_T, self.last_t = T_map_body, t

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "map"
        out.child_frame_id = "base_link"
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = (
            float(v) for v in T_map_body[:3, 3]
        )
        qx, qy, qz, qw = matrix_to_quat(T_map_body[:3, :3])
        o2 = out.pose.pose.orientation
        o2.x, o2.y, o2.z, o2.w = qx, qy, qz, qw
        pos_var = max(self.min_pos_var, float(np.max(np.diag(cov)[:3])))
        ori_var = max(self.min_ori_var, float(np.max(np.diag(cov)[3:])))
        for i in range(3):
            out.pose.covariance[i * 6 + i] = pos_var
            out.pose.covariance[(i + 3) * 6 + i + 3] = ori_var
        v = out.twist.twist.linear
        if self.send_velocity:
            vc = msg.twist.twist.linear
            vb = self.T_body_cam[:3, :3] @ np.array([vc.x, vc.y, vc.z])
            v.x, v.y, v.z = (float(c) for c in vb)
            vv = max(
                self.min_vel_var,
                float(np.max(np.diag(np.array(msg.twist.covariance).reshape(6, 6))[:3])),
            )
        else:
            v.x = v.y = v.z = float("nan")
            vv = self.min_vel_var
        for i in range(3):
            out.twist.covariance[i * 6 + i] = vv
        self.pub.publish(out)
        self.n_out += 1


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = VoAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""Camera relay: sim camera driver stream (TCP, drone network) -> /maverick/camera/* topics.

Receives CameraFrames (maverick_bridge.camera_protocol), attaches the PX4 pose (latest
vehicle_local_position_v1 + vehicle_attitude from the PX4 DDS domain) and the fixed camera mount
(sim/models/maverick/camera_mount.json), and publishes in the host-facing domain, where
zenoh-bridge-ros2dds forwards them best-effort over the wifi link:

  /maverick/camera/rgb/compressed  sensor_msgs/CompressedImage, format "jpeg", 320x240
  /maverick/camera/depth/png       sensor_msgs/CompressedImage, format "16UC1; png" (lossless,
                                   mm, 0 = invalid), 320x240, registered with the RGB image
  /maverick/camera/semantic/png    sensor_msgs/CompressedImage, format "mono8; png", 160x120
                                   (simulation only)
  /maverick/camera/info            sensor_msgs/CameraInfo (320x240 intrinsics)
  /maverick/camera/pose            geometry_msgs/PoseStamped, camera_optical in map (ENU)

The images are forwarded exactly as the driver encoded them (no pixel work on the bridge); the
host's camera_unpack node turns them into raw images. All five share one header.stamp (the frame's capture time on the
shared wall clock) and the pose is the PX4 pose interpolated at that time. A frame is dropped (and counted) when
there is no fresh PX4 pose with valid xy and z, so the host never maps against a stale pose.
Env: CAMERA_LISTEN (default 0.0.0.0), CAMERA_PORT (default 5600), CAMERA_MOUNT (path).
"""

from __future__ import annotations

import os
import socket
import sys
import threading
import time
from collections import deque

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage

from maverick_bridge import camera_protocol as proto
from maverick_bridge.camera_pose import DEFAULT_MOUNT_PATH, CameraMount, camera_pose_enu

POSE_MAX_AGE_S = 0.5


class PoseCache:
    """Recent PX4 position and attitude samples, looked up by capture time (wall clock).

    The image was rendered `latency` before it reached the bridge (render, encode, TCP): using the
    newest pose would shift every depth point by speed x latency (about 0.07 m at 1 m/s) and by
    yaw rate x latency x range when turning. The pose at the frame's capture time is interpolated
    instead (linear position, normalised linear quaternion).
    """

    def __init__(self, history_s: float = 2.0) -> None:
        self._lock = threading.Lock()
        self._hist_s = history_s
        self._pos: deque = deque()  # (t, (x, y, z)) valid positions only
        self._q: deque = deque()  # (t, (w, x, y, z))
        self._pos_invalid_t = 0.0

    def set_position(self, x: float, y: float, z: float, valid: bool, now: float) -> None:
        with self._lock:
            if valid:
                self._pos.append((now, (x, y, z)))
                while self._pos and now - self._pos[0][0] > self._hist_s:
                    self._pos.popleft()
            else:
                self._pos.clear()

    def set_attitude(self, q: tuple[float, float, float, float], now: float) -> None:
        with self._lock:
            self._q.append((now, q))
            while self._q and now - self._q[0][0] > self._hist_s:
                self._q.popleft()

    @staticmethod
    def _interp(hist: deque, t: float, hold_s: float):
        if not hist or t < hist[0][0] - 1e-6:
            return None
        if t >= hist[-1][0]:
            return hist[-1][1] if t - hist[-1][0] <= hold_s else None
        prev = hist[0]
        for cur in hist:
            if cur[0] >= t:
                span = cur[0] - prev[0]
                w = 0.0 if span <= 0 else (t - prev[0]) / span
                return tuple(a + w * (b - a) for a, b in zip(prev[1], cur[1], strict=True))
            prev = cur
        return None

    def at(self, t: float, hold_s: float = POSE_MAX_AGE_S):
        """(position NED, quaternion wxyz) at wall time t, or None without fresh enough data."""
        with self._lock:
            p = self._interp(self._pos, t, hold_s)
            q = self._interp(self._q, t, hold_s)
        if p is None or q is None:
            return None
        n = sum(c * c for c in q) ** 0.5
        return p, tuple(c / n for c in q)


class Px4SideNode(Node):
    """Lives in the PX4 DDS domain."""

    def __init__(self, context: Context, cache: PoseCache) -> None:
        super().__init__("maverick_camera_px4_side", context=context)
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            lambda m: cache.set_position(
                float(m.x),
                float(m.y),
                float(m.z),
                bool(m.xy_valid and m.z_valid),
                time.time(),
            ),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            lambda m: cache.set_attitude(tuple(float(v) for v in m.q), time.time()),
            qos_profile_sensor_data,
        )


class CameraRelayNode(Node):
    def __init__(self, mount: CameraMount, cache: PoseCache) -> None:
        super().__init__("maverick_camera_relay")
        self.mount, self.cache = mount, cache
        self.pub_rgb = self.create_publisher(
            CompressedImage, "/maverick/camera/rgb/compressed", qos_profile_sensor_data
        )
        self.pub_depth = self.create_publisher(
            CompressedImage, "/maverick/camera/depth/png", qos_profile_sensor_data
        )
        self.pub_sem = self.create_publisher(
            CompressedImage, "/maverick/camera/semantic/png", qos_profile_sensor_data
        )
        self.pub_info = self.create_publisher(
            CameraInfo, "/maverick/camera/info", qos_profile_sensor_data
        )
        self.pub_pose = self.create_publisher(
            PoseStamped, "/maverick/camera/pose", qos_profile_sensor_data
        )
        self.frames_ok = self.frames_no_pose = 0
        self.create_timer(5.0, self._log_counts)

    def _log_counts(self) -> None:
        self.get_logger().info(
            f"camera frames published {self.frames_ok}, no pose {self.frames_no_pose}"
        )

    def info_msg(self, stamp, width: int, height: int) -> CameraInfo:
        fx, fy, cx, cy = self.mount.intrinsics(width, height)
        m = CameraInfo()
        m.header.stamp = stamp
        m.header.frame_id = self.mount.frame_id
        m.width, m.height = width, height
        m.distortion_model = "plumb_bob"
        m.d = [0.0] * 5
        m.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        m.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return m

    def on_frame(self, frame: proto.CameraFrame) -> bool:
        pose = self.cache.at(frame.t_wall)
        if pose is None:
            self.frames_no_pose += 1
            return False
        (pos_ned, q) = pose
        stamp = Time(sec=int(frame.t_wall), nanosec=int((frame.t_wall % 1.0) * 1e9))  # capture time
        frame_id = self.mount.frame_id
        dw, dh = frame.depth_size

        rgb = CompressedImage()
        rgb.header.stamp, rgb.header.frame_id = stamp, frame_id
        rgb.format = "jpeg"
        rgb.data = frame.jpeg

        depth = CompressedImage()
        depth.header.stamp, depth.header.frame_id = stamp, frame_id
        depth.format = "16UC1; png"
        depth.data = frame.depth_png

        sem = CompressedImage()
        sem.header.stamp, sem.header.frame_id = stamp, frame_id
        sem.format = "mono8; png"
        sem.data = frame.semantic_png

        (px, py, pz), (qw, qx, qy, qz) = camera_pose_enu(pos_ned, q, self.mount)
        ps = PoseStamped()
        ps.header.stamp, ps.header.frame_id = stamp, "map"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = px, py, pz
        ps.pose.orientation.w, ps.pose.orientation.x = qw, qx
        ps.pose.orientation.y, ps.pose.orientation.z = qy, qz

        self.pub_rgb.publish(rgb)
        self.pub_depth.publish(depth)
        self.pub_sem.publish(sem)
        self.pub_info.publish(self.info_msg(stamp, dw, dh))
        self.pub_pose.publish(ps)
        self.frames_ok += 1
        return True


def serve(node: CameraRelayNode, host: str, port: int) -> None:
    """Accept one driver connection at a time (a new one replaces the old), forever."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    node.get_logger().info(f"camera stream listening on {host}:{port}")
    while True:
        conn, addr = srv.accept()
        node.get_logger().info(f"camera driver connected from {addr[0]}")
        conn.settimeout(10.0)
        reader = proto.FrameReader()
        try:
            while True:
                data = conn.recv(65536)
                if not data:
                    break
                for frame in reader.feed(data):
                    node.on_frame(frame)
        except (OSError, proto.ProtocolError) as e:
            node.get_logger().warning(f"camera stream dropped: {e}")
        finally:
            conn.close()


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    mount = CameraMount.load(os.environ.get("CAMERA_MOUNT", DEFAULT_MOUNT_PATH))
    cache = PoseCache()
    node = CameraRelayNode(mount, cache)
    px4_context = Context()
    px4_context.init(domain_id=int(os.environ.get("PX4_DDS_DOMAIN_ID", "1")))
    px4_node = Px4SideNode(px4_context, cache)
    px4_executor = SingleThreadedExecutor(context=px4_context)
    px4_executor.add_node(px4_node)
    threading.Thread(target=px4_executor.spin, daemon=True).start()
    threading.Thread(
        target=serve,
        args=(
            node,
            os.environ.get("CAMERA_LISTEN", "0.0.0.0"),
            int(os.environ.get("CAMERA_PORT", proto.DEFAULT_PORT)),
        ),
        daemon=True,
    ).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        px4_executor.shutdown()
        px4_node.destroy_node()
        node.destroy_node()
        px4_context.try_shutdown()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

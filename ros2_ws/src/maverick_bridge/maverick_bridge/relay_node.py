"""Thin rclpy node around RelayCore.

Subscribes to the host's FlightSetpoint/FlightCommand (arriving through Zenoh) and publishes
validated PX4 messages on /fmu/in/*. It publishes nothing on its own except BridgeStatus (1 Hz):
no setpoint repetition and no heartbeat, so a dead link lets PX4's offboard-loss failsafe fire.
"""

from __future__ import annotations

import math
import os
import socket
import sys
import threading
import time

import rclpy
from maverick_interfaces.msg import BridgeStatus, FlightCommand, FlightSetpoint
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleOdometry,
    VehicleStatus,
)
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from maverick_bridge import mocap_protocol
from maverick_bridge.core import RelayCore
from maverick_bridge.vision import VisionRelay

# PX4 telemetry that crosses the link (docs/s1-plan.md): read in the PX4 domain, republished
# under the same names in the host-facing domain, where zenoh-bridge-ros2dds picks it up.
TELEMETRY = (
    ("/fmu/out/vehicle_local_position_v1", VehicleLocalPosition),
    ("/fmu/out/vehicle_status_v1", VehicleStatus),
    ("/fmu/out/vehicle_land_detected", VehicleLandDetected),
    ("/fmu/out/vehicle_command_ack", VehicleCommandAck),
)


# External vision into PX4's EKF (uORB vehicle_visual_odometry). Topic name as in v1.17.0's
# dds_topics.yaml; no _v1 suffix (checked on the live uXRCE topic list).
VISION_TOPIC = os.environ.get("VISION_TOPIC", "/fmu/in/vehicle_visual_odometry")


MOCAP_POS_VAR = (4e-6, 4e-6, 4e-6)  # (2 mm)^2
MOCAP_ORI_VAR = (7.6e-5, 7.6e-5, 7.6e-5)  # (0.5 deg)^2


def mocap_listener(node: RelayNode, host: str, port: int) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    while True:
        data, _ = sock.recvfrom(256)
        try:
            node.on_mocap(mocap_protocol.decode(data))
        except mocap_protocol.MocapProtocolError:
            pass


def stamp_to_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class TelemetryNode(Node):
    """Lives in the PX4 DDS domain; hands PX4 telemetry to the host-facing node."""

    def __init__(self, context: Context, sink: RelayNode) -> None:
        super().__init__("maverick_bridge_px4_side", context=context)
        for topic, msg_type in TELEMETRY:
            self.create_subscription(
                msg_type, topic, sink.telemetry_callback(topic), qos_profile_sensor_data
            )
        self.mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )
        self.sp_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.vision_pub = self.create_publisher(VehicleOdometry, VISION_TOPIC, 10)


class RelayNode(Node):
    def __init__(self) -> None:
        super().__init__("maverick_bridge_relay")
        self.px4: TelemetryNode | None = None
        self.core = RelayCore()
        self.vision = VisionRelay()
        self.create_subscription(
            FlightSetpoint, "/maverick/flight/setpoint", self._on_setpoint, qos_profile_sensor_data
        )
        self.create_subscription(
            FlightCommand,
            "/maverick/flight/command",
            self._on_command,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.create_subscription(
            Odometry, "/maverick/vision/odometry", self._on_vision, qos_profile_sensor_data
        )
        self._telemetry_pubs = {
            topic: self.create_publisher(msg_type, topic, qos_profile_sensor_data)
            for topic, msg_type in TELEMETRY
        }
        self._status_pub = self.create_publisher(
            BridgeStatus, "/maverick/bridge/status", qos_profile_sensor_data
        )
        self.create_timer(1.0, self._publish_status)

    def telemetry_callback(self, topic: str):
        pub = self._telemetry_pubs[topic]
        return pub.publish

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _now_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def _on_setpoint(self, msg: FlightSetpoint) -> None:
        out = self.core.on_setpoint(
            stamp_to_s(msg.stamp),
            self._now_s(),
            (float(msg.position_ned[0]), float(msg.position_ned[1]), float(msg.position_ned[2])),
            float(msg.yaw),
        )
        if out is None:
            return
        mode = OffboardControlMode()
        mode.timestamp = self._now_us()
        mode.position = True
        mode.velocity = False
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        self.px4.mode_pub.publish(mode)
        sp = TrajectorySetpoint()
        sp.timestamp = self._now_us()
        sp.position = [float(v) for v in out.position_ned]
        sp.yaw = float(out.yaw) if math.isfinite(out.yaw) else math.nan
        self.px4.sp_pub.publish(sp)

    def _on_vision(self, msg: Odometry) -> None:
        pc, tc = msg.pose.covariance, msg.twist.covariance
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
        out = self.vision.on_odometry(
            stamp_to_s(msg.header.stamp),
            self._now_s(),
            (float(p.x), float(p.y), float(p.z)),
            (float(q.x), float(q.y), float(q.z), float(q.w)),
            (float(v.x), float(v.y), float(v.z)),
            (float(pc[0]), float(pc[7]), float(pc[14])),
            (float(pc[21]), float(pc[28]), float(pc[35])),
            (float(tc[0]), float(tc[7]), float(tc[14])),
        )
        if out is not None:
            self._publish_vision(out)

    def _publish_vision(self, out) -> None:
        odo = VehicleOdometry()
        odo.timestamp = self._now_us()
        odo.timestamp_sample = int(out.sample_time_s * 1e6)
        odo.pose_frame = VehicleOdometry.POSE_FRAME_NED
        odo.position = [float(c) for c in out.position_ned]
        odo.q = [float(c) for c in out.q_frd_ned]
        odo.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD
        odo.velocity = [float(c) for c in out.velocity_frd]
        odo.angular_velocity = [math.nan] * 3
        odo.position_variance = [float(c) for c in out.position_var]
        odo.orientation_variance = [float(c) for c in out.orientation_var]
        odo.velocity_variance = [float(c) for c in out.velocity_var]
        odo.reset_counter = 0
        odo.quality = 0
        self.px4.vision_pub.publish(odo)

    def on_mocap(self, sample: mocap_protocol.MocapSample) -> None:
        """Simulated mocap pose: same conversion, staleness rule and counters as the VO path."""
        out = self.vision.on_odometry(
            sample.t_s,
            time.time(),  # wall clock: the driver stamps with the shared wall clock
            sample.position_enu,
            sample.quat_xyzw,
            (math.nan, math.nan, math.nan),
            MOCAP_POS_VAR,
            MOCAP_ORI_VAR,
            (0.01, 0.01, 0.01),
        )
        if out is not None:
            self._publish_vision(out)

    def _on_command(self, msg: FlightCommand) -> None:
        spec = self.core.on_command(stamp_to_s(msg.stamp), self._now_s(), int(msg.command))
        if spec is None:
            return
        out = VehicleCommand()
        out.timestamp = self._now_us()
        out.command = spec.command
        out.param1 = spec.param1
        out.param2 = spec.param2
        out.param3 = spec.param3
        out.target_system = 1
        out.target_component = 1
        out.source_system = 1
        out.source_component = 1
        out.from_external = True
        self.px4.cmd_pub.publish(out)

    def _publish_status(self) -> None:
        k = self.core.counters()
        msg = BridgeStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.setpoints_forwarded = k.setpoints_forwarded
        msg.setpoints_dropped_stale = k.setpoints_dropped_stale
        msg.setpoints_dropped_invalid = k.setpoints_dropped_invalid
        msg.commands_forwarded = k.commands_forwarded
        msg.commands_dropped_stale = k.commands_dropped_stale
        msg.last_setpoint_age_ms = float(k.last_setpoint_age_ms)
        msg.vision_forwarded = self.vision.counters.forwarded
        msg.vision_dropped_stale = self.vision.counters.dropped_stale
        msg.vision_dropped_invalid = self.vision.counters.dropped_invalid
        self._status_pub.publish(msg)


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = RelayNode()  # default context: host-facing domain (ROS_DOMAIN_ID)
    px4_context = Context()
    px4_context.init(domain_id=int(os.environ.get("PX4_DDS_DOMAIN_ID", "1")))
    node.px4 = TelemetryNode(px4_context, node)
    px4_executor = SingleThreadedExecutor(context=px4_context)
    px4_executor.add_node(node.px4)
    threading.Thread(target=px4_executor.spin, daemon=True).start()
    if os.environ.get("MOCAP_LISTEN"):  # drone-network address only; unset = no mocap input
        threading.Thread(
            target=mocap_listener,
            args=(
                node,
                os.environ["MOCAP_LISTEN"],
                int(os.environ.get("MOCAP_PORT", mocap_protocol.DEFAULT_PORT)),
            ),
            daemon=True,
        ).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        px4_executor.shutdown()
        node.px4.destroy_node()
        node.destroy_node()
        px4_context.try_shutdown()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

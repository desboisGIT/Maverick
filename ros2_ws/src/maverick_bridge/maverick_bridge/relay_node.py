"""Thin rclpy node around RelayCore.

Subscribes to the host's FlightSetpoint/FlightCommand (arriving through Zenoh) and publishes
validated PX4 messages on /fmu/in/*. It publishes nothing on its own except BridgeStatus (1 Hz):
no setpoint repetition and no heartbeat, so a dead link lets PX4's offboard-loss failsafe fire.
"""

from __future__ import annotations

import math
import os
import sys
import threading

import rclpy
from maverick_interfaces.msg import BridgeStatus, FlightCommand, FlightSetpoint
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from maverick_bridge.core import RelayCore

# PX4 telemetry that crosses the link (docs/s1-plan.md): read in the PX4 domain, republished
# under the same names in the host-facing domain, where zenoh-bridge-ros2dds picks it up.
TELEMETRY = (
    ("/fmu/out/vehicle_local_position_v1", VehicleLocalPosition),
    ("/fmu/out/vehicle_status_v1", VehicleStatus),
    ("/fmu/out/vehicle_land_detected", VehicleLandDetected),
    ("/fmu/out/vehicle_command_ack", VehicleCommandAck),
)


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


class RelayNode(Node):
    def __init__(self) -> None:
        super().__init__("maverick_bridge_relay")
        self.px4: TelemetryNode | None = None
        self.core = RelayCore()
        self.create_subscription(
            FlightSetpoint, "/maverick/flight/setpoint", self._on_setpoint, qos_profile_sensor_data
        )
        self.create_subscription(
            FlightCommand,
            "/maverick/flight/command",
            self._on_command,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
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

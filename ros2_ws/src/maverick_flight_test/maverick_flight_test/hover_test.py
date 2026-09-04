"""ROS 2 node wrapping HoverMission: offboard take off, hover, land on PX4 SITL.

Message fields and QoS follow PX4 v1.17 (msg/versioned/*.msg) and the px4_ros_com
offboard_control example. Topic names are the ones v1.17.0 SITL actually publishes,
checked in the uxrce_dds_client log: versioned messages get a _vN suffix.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args

from maverick_flight_test.hover_mission import (
    CMD_ARM,
    CMD_LAND,
    CMD_OFFBOARD,
    Action,
    Config,
    HoverMission,
    Observation,
)

TICK_S = 0.1  # 10 Hz


class HoverTestNode(Node):
    def __init__(self, mission: HoverMission) -> None:
        super().__init__("hover_test")
        self.mission = mission
        # Defaults: topic names published by PX4 v1.17.0 SITL (see module docstring).
        topic = self.declare_parameter
        pos_topic = topic("local_position_topic", "/fmu/out/vehicle_local_position_v1").value
        status_topic = topic("status_topic", "/fmu/out/vehicle_status_v1").value
        land_topic = topic("land_detected_topic", "/fmu/out/vehicle_land_detected").value
        mode_topic = topic("offboard_mode_topic", "/fmu/in/offboard_control_mode").value
        sp_topic = topic("setpoint_topic", "/fmu/in/trajectory_setpoint").value
        cmd_topic = topic("command_topic", "/fmu/in/vehicle_command").value

        self._pos: VehicleLocalPosition | None = None
        self._status: VehicleStatus | None = None
        self._land: VehicleLandDetected | None = None

        # PX4 /fmu/out publishers are best-effort: subscribe with sensor-data QoS.
        self.create_subscription(
            VehicleLocalPosition, pos_topic, self._on_pos, qos_profile_sensor_data
        )
        self.create_subscription(
            VehicleStatus, status_topic, self._on_status, qos_profile_sensor_data
        )
        self.create_subscription(
            VehicleLandDetected, land_topic, self._on_land, qos_profile_sensor_data
        )
        self._mode_pub = self.create_publisher(OffboardControlMode, mode_topic, 10)
        self._sp_pub = self.create_publisher(TrajectorySetpoint, sp_topic, 10)
        self._cmd_pub = self.create_publisher(VehicleCommand, cmd_topic, 10)
        self.create_timer(TICK_S, self._tick)

    def _on_pos(self, msg: VehicleLocalPosition) -> None:
        self._pos = msg

    def _on_status(self, msg: VehicleStatus) -> None:
        self._status = msg

    def _on_land(self, msg: VehicleLandDetected) -> None:
        self._land = msg

    def _now_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def _observe(self) -> Observation:
        p, s, ld = self._pos, self._status, self._land
        valid = p is not None and p.xy_valid and p.z_valid
        return Observation(
            t=time.monotonic(),
            position=(float(p.x), float(p.y), float(p.z)) if valid and p is not None else None,
            arming_state=int(s.arming_state) if s is not None else None,
            nav_state=int(s.nav_state) if s is not None else None,
            landed=bool(ld.landed) if ld is not None else None,
        )

    def _tick(self) -> None:
        action = self.mission.step(self._observe())
        self._execute(action)

    def _execute(self, action: Action) -> None:
        if action.setpoint is not None:
            mode = OffboardControlMode()
            mode.timestamp = self._now_us()
            mode.position = True
            mode.velocity = False
            mode.acceleration = False
            mode.attitude = False
            mode.body_rate = False
            self._mode_pub.publish(mode)
            sp = TrajectorySetpoint()
            sp.timestamp = self._now_us()
            sp.position = [float(v) for v in action.setpoint]
            sp.yaw = 0.0
            self._sp_pub.publish(sp)
        for name in action.commands:
            if name == CMD_OFFBOARD:
                self._command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            elif name == CMD_ARM:
                self._command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            elif name == CMD_LAND:
                self._command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def _command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp = self._now_us()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._cmd_pub.publish(msg)


def main(argv: list[str] | None = None) -> int:
    args_in = remove_ros_args(argv if argv is not None else sys.argv)[1:]
    parser = argparse.ArgumentParser(description="PX4 offboard take off / hover / land test")
    parser.add_argument("--altitude", type=float, default=2.0)
    parser.add_argument("--hover-seconds", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args(args_in)

    mission = HoverMission(Config(args.altitude, args.hover_seconds, args.timeout))
    rclpy.init(args=argv)
    node = HoverTestNode(mission)
    try:
        while rclpy.ok() and not mission.done:
            rclpy.spin_once(node, timeout_sec=TICK_S)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    v = mission.verdict
    if v is None:
        print(json.dumps({"result": "fail", "reason": "interrupted", "max_alt_error_m": 0.0,
                          "max_xy_drift_m": 0.0, "duration_s": 0.0}))
        return 1
    print(json.dumps({
        "result": "pass" if v.passed else "fail",
        "reason": v.reason,
        "max_alt_error_m": round(v.max_alt_error_m, 3),
        "max_xy_drift_m": round(v.max_xy_drift_m, 3),
        "duration_s": round(v.duration_s, 1),
    }))
    return 0 if v.passed else 1


if __name__ == "__main__":
    sys.exit(main())

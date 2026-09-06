"""Referee-side observers, run with `docker exec` in the bridge container. Print JSON lines.

  px4_monitor px4      PX4 domain (ROS_DOMAIN_ID=1): nav_state / arming / landed / failsafe, local
                       position, and every setpoint the relay hands to PX4. Independent of the
                       relay and of the host, so it still reports when either is killed.
  px4_monitor bridge   host-facing domain: BridgeStatus counters (needs the relay alive).

Timestamps are wall-clock arrival times, comparable with the other containers' clocks.
"""

from __future__ import annotations

import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args


def emit(obj: dict) -> None:
    print(json.dumps({"t": round(time.time(), 3), **obj}), flush=True)


class Px4Monitor(Node):
    def __init__(self) -> None:
        super().__init__("referee_px4_monitor")
        from px4_msgs.msg import (
            TrajectorySetpoint,
            VehicleLandDetected,
            VehicleLocalPosition,
            VehicleStatus,
        )

        q = qos_profile_sensor_data
        self._last_status: tuple | None = None
        self._last_land: bool | None = None
        self._last_pos_t = 0.0
        self._last_sp_t = 0.0
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self._on_status, q)
        self.create_subscription(
            VehicleLandDetected, "/fmu/out/vehicle_land_detected", self._on_land, q
        )
        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._on_pos, q
        )
        self.create_subscription(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", self._on_sp, 10)

    def _on_status(self, msg) -> None:
        key = (int(msg.nav_state), int(msg.arming_state), bool(msg.failsafe))
        if key != self._last_status:
            self._last_status = key
            emit({"k": "status", "nav": key[0], "arm": key[1], "failsafe": key[2]})

    def _on_land(self, msg) -> None:
        if bool(msg.landed) != self._last_land:
            self._last_land = bool(msg.landed)
            emit({"k": "landed", "landed": self._last_land})

    def _on_pos(self, msg) -> None:
        now = time.time()
        if now - self._last_pos_t >= 0.1:
            self._last_pos_t = now
            emit(
                {
                    "k": "pos",
                    "x": round(float(msg.x), 3),
                    "y": round(float(msg.y), 3),
                    "z": round(float(msg.z), 3),
                    "valid": bool(msg.xy_valid and msg.z_valid),
                }
            )

    def _on_sp(self, msg) -> None:
        # Every setpoint, not throttled: F6 asserts that nothing unexpected reaches PX4.
        emit({"k": "sp", "p": [round(float(v), 3) for v in msg.position]})


class BridgeMonitor(Node):
    def __init__(self) -> None:
        super().__init__("referee_bridge_monitor")
        from maverick_interfaces.msg import BridgeStatus, FlightSetpoint

        self.create_subscription(
            BridgeStatus, "/maverick/bridge/status", self._on_msg, qos_profile_sensor_data
        )
        self.create_subscription(
            FlightSetpoint, "/maverick/flight/setpoint", self._on_arrival, qos_profile_sensor_data
        )

    def _on_arrival(self, msg) -> None:
        # Age of every setpoint when it reaches the bridge, measured here independently of the
        # relay's own counters.
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        emit({"k": "arrive", "age_ms": round((now - stamp) * 1000.0, 1)})

    def _on_msg(self, msg) -> None:
        emit(
            {
                "k": "bridge",
                "forwarded": int(msg.setpoints_forwarded),
                "dropped_stale": int(msg.setpoints_dropped_stale),
                "dropped_invalid": int(msg.setpoints_dropped_invalid),
                "age_ms": float(msg.last_setpoint_age_ms),
            }
        )


def main(argv: list[str] | None = None) -> int:
    args = remove_ros_args(argv if argv is not None else sys.argv)[1:]
    kind = args[0] if args else "px4"
    rclpy.init(args=argv)
    node = Px4Monitor() if kind == "px4" else BridgeMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

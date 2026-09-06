"""ROS 2 node `flight_client`: the host side of the flight link. Thin shell around FlightMission.

Topics
  out /maverick/flight/setpoint  maverick_interfaces/FlightSetpoint, 20 Hz, through the SafetyGate
  out /maverick/flight/command   maverick_interfaces/FlightCommand (ARM, OFFBOARD, LAND)
  out /maverick/flight/status    std_msgs/String, JSON, 1 Hz and on every event
  in  /maverick/flight/goal      std_msgs/String, JSON, one goal per message:
         {"mode": "goto", "x": N, "y": E, "z": D, "speed": m/s (optional)}   walk there, <= limit
         {"mode": "setpoint", "x": .., "y": .., "z": ..}                      use it directly
      (PX4 local NED metres.) The gate rejects NaN/inf, outside the geofence, a "setpoint" that
      jumps more than 1 m from the current one, and a speed above the limit; each is logged.
  in  /maverick/flight/mission   std_msgs/String: "land" ends the hover and lands
  in  PX4 telemetry from the bridge (local position, status, land detected) and BridgeStatus

Every state change is also printed to stdout as one JSON line (flushed), so it can be read
with `docker logs` / `docker exec`.

Parameters: altitude (2.0), hover_seconds (20.0; <= 0 means until "land"), geofence_min /
geofence_max (NED boxes), max_speed (1.0), max_jump (1.0), gap_abort_s (5.0).
"""

from __future__ import annotations

import json
import sys
import time

import rclpy
from maverick_interfaces.msg import BridgeStatus, FlightCommand, FlightSetpoint
from px4_msgs.msg import VehicleLandDetected, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String

from maverick_flight.mission import (
    CMD_ARM,
    CMD_LAND,
    CMD_OFFBOARD,
    FlightMission,
    MissionConfig,
    Observation,
)
from maverick_flight.safety_gate import GateConfig, SafetyGate

TICK_S = 0.05
COMMANDS = {
    CMD_ARM: FlightCommand.ARM,
    CMD_OFFBOARD: FlightCommand.OFFBOARD,
    CMD_LAND: FlightCommand.LAND,
}


def emit(obj: dict) -> None:
    print(json.dumps({"t": round(time.time(), 3), **obj}), flush=True)


class FlightClient(Node):
    def __init__(self) -> None:
        super().__init__("flight_client")
        p = self.declare_parameter
        altitude = p("altitude", 2.0).value
        hover_s = p("hover_seconds", 20.0).value
        gate = SafetyGate(
            GateConfig(
                tuple(p("geofence_min", [-5.0, -5.0, -4.0]).value),
                tuple(p("geofence_max", [5.0, 5.0, 0.3]).value),
                p("max_jump", 1.0).value,
                p("max_speed", 1.0).value,
            )
        )
        self.mission = FlightMission(
            MissionConfig(
                altitude=altitude,
                hover_seconds=hover_s if hover_s > 0 else None,
                gap_abort_s=p("gap_abort_s", 5.0).value,
            ),
            gate,
        )
        self._pos: VehicleLocalPosition | None = None
        self._status: VehicleStatus | None = None
        self._land: VehicleLandDetected | None = None
        self._last_rx = -1e9
        self._bridge: dict | None = None
        self._last_phase_line = ""

        q = qos_profile_sensor_data
        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._on_pos, q
        )
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self._on_status, q)
        self.create_subscription(
            VehicleLandDetected, "/fmu/out/vehicle_land_detected", self._on_land, q
        )
        self.create_subscription(BridgeStatus, "/maverick/bridge/status", self._on_bridge, q)
        self.create_subscription(String, "/maverick/flight/goal", self._on_goal, 10)
        self.create_subscription(String, "/maverick/flight/mission", self._on_mission, 10)
        self._sp_pub = self.create_publisher(FlightSetpoint, "/maverick/flight/setpoint", q)
        self._cmd_pub = self.create_publisher(
            FlightCommand,
            "/maverick/flight/command",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self._status_pub = self.create_publisher(String, "/maverick/flight/status", 10)
        self.create_timer(TICK_S, self._tick)
        self.create_timer(1.0, self._publish_status)
        emit({"event": "start", "phase": self.mission.phase.value})

    # -- callbacks ---------------------------------------------------------------------------
    def _rx(self) -> None:
        self._last_rx = time.monotonic()

    def _on_pos(self, msg) -> None:
        self._pos = msg
        self._rx()

    def _on_status(self, msg) -> None:
        self._status = msg
        self._rx()

    def _on_land(self, msg) -> None:
        self._land = msg
        self._rx()

    def _on_bridge(self, msg: BridgeStatus) -> None:
        self._bridge = {
            "forwarded": int(msg.setpoints_forwarded),
            "dropped_stale": int(msg.setpoints_dropped_stale),
            "dropped_invalid": int(msg.setpoints_dropped_invalid),
        }

    def _on_goal(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
            pos = (float(d["x"]), float(d["y"]), float(d["z"]))
            speed = float(d["speed"]) if d.get("speed") is not None else None
            mode = str(d.get("mode", "goto"))
            if mode not in ("goto", "setpoint"):
                raise ValueError(f"unknown mode {mode!r}")
        except (ValueError, KeyError, TypeError) as exc:
            emit({"event": "goal_malformed", "error": str(exc), "data": msg.data[:200]})
            return
        self.mission.on_goal(mode, pos, speed)
        self._flush_events()

    def _on_mission(self, msg: String) -> None:
        if msg.data.strip().lower() == "land":
            self.mission.request_land()

    # -- loop --------------------------------------------------------------------------------
    def _observe(self) -> Observation:
        p, s, ld = self._pos, self._status, self._land
        valid = p is not None and p.xy_valid and p.z_valid
        return Observation(
            t=time.monotonic(),
            position=(float(p.x), float(p.y), float(p.z)) if valid and p is not None else None,
            arming_state=int(s.arming_state) if s is not None else None,
            nav_state=int(s.nav_state) if s is not None else None,
            landed=bool(ld.landed) if ld is not None else None,
            telemetry_age_s=time.monotonic() - self._last_rx,
        )

    def _tick(self) -> None:
        action = self.mission.step(self._observe())
        if action.setpoint is not None:
            sp = FlightSetpoint()
            sp.stamp = self.get_clock().now().to_msg()
            sp.position_ned = [float(v) for v in action.setpoint]
            sp.yaw = 0.0
            self._sp_pub.publish(sp)
        for name in action.commands:
            cmd = FlightCommand()
            cmd.stamp = self.get_clock().now().to_msg()
            cmd.command = COMMANDS[name]
            self._cmd_pub.publish(cmd)
            emit({"event": "command", "command": name})
        self._flush_events()

    def _flush_events(self) -> None:
        for ev in self.mission.drain_events():
            emit(ev)

    def _status_dict(self) -> dict:
        m = self.mission
        g = m.gate
        return {
            "phase": m.phase.value,
            "result": m.result,
            "setpoint": m.setpoint,
            "hold": m.hold,
            "rejections": dict(g.rejections),
            "last_rejection": (
                {"reason": g.last_rejection.reason, "detail": g.last_rejection.detail}
                if g.last_rejection
                else None
            ),
            "telemetry_age_s": round(time.monotonic() - self._last_rx, 2),
            "bridge": self._bridge,
        }

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(self._status_dict())
        self._status_pub.publish(msg)


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = FlightClient()
    try:
        while rclpy.ok() and not node.mission.done:
            rclpy.spin_once(node, timeout_sec=TICK_S)
        end = time.monotonic() + 1.0  # let the final status out
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    result = node.mission.result
    emit({"event": "exit", "result": result})
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if result == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())

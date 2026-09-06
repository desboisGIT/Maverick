"""Pure-Python relay rules (no ROS imports). See docs/s1-plan.md, "Bridge relay rules".

The relay never invents traffic: it only decides whether a message that arrived from the
host may be forwarded to PX4, and counts what it dropped.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

SETPOINT_MAX_AGE_S = 0.200
SETPOINT_MAX_FUTURE_S = 0.050
COMMAND_MAX_AGE_S = 0.500

# maverick_interfaces/FlightCommand constants.
CMD_ARM = 1
CMD_DISARM = 2
CMD_OFFBOARD = 3
CMD_LAND = 4
CMD_HOLD = 5

# MAVLink command ids (px4_msgs VehicleCommand.VEHICLE_CMD_*).
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_NAV_LAND = 21
MAV_CMD_COMPONENT_ARM_DISARM = 400

# PX4 custom modes (src/modules/commander/px4_custom_mode.h, v1.17.0).
PX4_CUSTOM_MAIN_MODE_AUTO = 4
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1.0


@dataclass(frozen=True)
class VehicleCommandSpec:
    command: int
    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0


_COMMANDS: dict[int, VehicleCommandSpec] = {
    CMD_ARM: VehicleCommandSpec(MAV_CMD_COMPONENT_ARM_DISARM, 1.0),
    CMD_DISARM: VehicleCommandSpec(MAV_CMD_COMPONENT_ARM_DISARM, 0.0),
    CMD_OFFBOARD: VehicleCommandSpec(
        MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, float(PX4_CUSTOM_MAIN_MODE_OFFBOARD)
    ),
    CMD_LAND: VehicleCommandSpec(MAV_CMD_NAV_LAND),
    CMD_HOLD: VehicleCommandSpec(
        MAV_CMD_DO_SET_MODE,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        float(PX4_CUSTOM_MAIN_MODE_AUTO),
        float(PX4_CUSTOM_SUB_MODE_AUTO_LOITER),
    ),
}


@dataclass(frozen=True)
class Setpoint:
    position_ned: tuple[float, float, float]
    yaw: float


@dataclass(frozen=True)
class Counters:
    setpoints_forwarded: int
    setpoints_dropped_stale: int
    setpoints_dropped_invalid: int
    commands_forwarded: int
    commands_dropped_stale: int
    last_setpoint_age_ms: float  # NaN until the first setpoint arrives


class RelayCore:
    def __init__(
        self,
        setpoint_max_age_s: float = SETPOINT_MAX_AGE_S,
        setpoint_max_future_s: float = SETPOINT_MAX_FUTURE_S,
        command_max_age_s: float = COMMAND_MAX_AGE_S,
    ) -> None:
        self.setpoint_max_age_s = setpoint_max_age_s
        self.setpoint_max_future_s = setpoint_max_future_s
        self.command_max_age_s = command_max_age_s
        self.setpoints_forwarded = 0
        self.setpoints_dropped_stale = 0
        self.setpoints_dropped_invalid = 0
        self.commands_forwarded = 0
        self.commands_dropped_stale = 0
        self.commands_dropped_unknown = 0  # not in BridgeStatus
        self.last_setpoint_age_ms = math.nan

    def on_setpoint(
        self, stamp_s: float, now_s: float, position_ned: tuple[float, float, float], yaw: float
    ) -> Setpoint | None:
        """Return the setpoint to forward to PX4, or None if it must be dropped."""
        age = now_s - stamp_s
        self.last_setpoint_age_ms = age * 1000.0
        if not all(math.isfinite(v) for v in position_ned) or math.isinf(yaw):
            self.setpoints_dropped_invalid += 1
            return None
        if age > self.setpoint_max_age_s or age < -self.setpoint_max_future_s:
            self.setpoints_dropped_stale += 1
            return None
        self.setpoints_forwarded += 1
        return Setpoint(position_ned, yaw)

    def on_command(self, stamp_s: float, now_s: float, command: int) -> VehicleCommandSpec | None:
        """Return the VehicleCommand to send to PX4, or None if it must be dropped."""
        age = now_s - stamp_s
        if age > self.command_max_age_s or age < -self.setpoint_max_future_s:
            self.commands_dropped_stale += 1
            return None
        spec = _COMMANDS.get(command)
        if spec is None:
            self.commands_dropped_unknown += 1
            return None
        self.commands_forwarded += 1
        return spec

    def counters(self) -> Counters:
        return Counters(
            self.setpoints_forwarded,
            self.setpoints_dropped_stale,
            self.setpoints_dropped_invalid,
            self.commands_forwarded,
            self.commands_dropped_stale,
            self.last_setpoint_age_ms,
        )

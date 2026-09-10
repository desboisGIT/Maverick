"""PX4 local NED <-> map ENU (ROS REP 103). The only place the conversion lives.

Map frame: x east, y north, z up, metres, right-handed, yaw counter-clockwise from +x (east).
PX4 local frame: x north, y east, z down, yaw clockwise from north.
"""

from __future__ import annotations

import math


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def ned_to_enu(x_ned: float, y_ned: float, z_ned: float) -> tuple[float, float, float]:
    return y_ned, x_ned, -z_ned


def enu_to_ned(x_enu: float, y_enu: float, z_enu: float) -> tuple[float, float, float]:
    return y_enu, x_enu, -z_enu


def yaw_ned_to_enu(yaw_ned: float) -> float:
    return wrap_pi(math.pi / 2 - yaw_ned)


def yaw_enu_to_ned(yaw_enu: float) -> float:
    return wrap_pi(math.pi / 2 - yaw_enu)

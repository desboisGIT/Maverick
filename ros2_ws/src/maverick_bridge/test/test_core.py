import math

from maverick_bridge.core import (
    CMD_ARM,
    CMD_DISARM,
    CMD_HOLD,
    CMD_LAND,
    CMD_OFFBOARD,
    RelayCore,
)

POS = (1.0, 2.0, -2.0)
NOW = 1000.0


def test_fresh_setpoint_forwarded():
    c = RelayCore()
    sp = c.on_setpoint(NOW - 0.05, NOW, POS, 0.5)
    assert sp is not None and sp.position_ned == POS and sp.yaw == 0.5
    assert c.counters().setpoints_forwarded == 1
    assert abs(c.counters().last_setpoint_age_ms - 50.0) < 1e-6


def test_stale_setpoint_dropped():
    c = RelayCore()
    assert c.on_setpoint(NOW - 0.201, NOW, POS, 0.0) is None
    assert c.on_setpoint(NOW - 0.199, NOW, POS, 0.0) is not None
    k = c.counters()
    assert (k.setpoints_dropped_stale, k.setpoints_forwarded) == (1, 1)


def test_future_setpoint_dropped():
    c = RelayCore()
    assert c.on_setpoint(NOW + 0.051, NOW, POS, 0.0) is None
    assert c.on_setpoint(NOW + 0.049, NOW, POS, 0.0) is not None
    assert c.counters().setpoints_dropped_stale == 1


def test_nan_or_inf_position_dropped_but_nan_yaw_ok():
    c = RelayCore()
    assert c.on_setpoint(NOW, NOW, (math.nan, 0.0, -1.0), 0.0) is None
    assert c.on_setpoint(NOW, NOW, (0.0, math.inf, -1.0), 0.0) is None
    assert c.on_setpoint(NOW, NOW, POS, math.nan) is not None
    k = c.counters()
    assert (k.setpoints_dropped_invalid, k.setpoints_forwarded) == (2, 1)


def test_age_nan_until_first_setpoint():
    assert math.isnan(RelayCore().counters().last_setpoint_age_ms)


def test_command_staleness_500ms():
    c = RelayCore()
    assert c.on_command(NOW - 0.6, NOW, CMD_ARM) is None
    assert c.on_command(NOW - 0.4, NOW, CMD_ARM) is not None
    k = c.counters()
    assert (k.commands_dropped_stale, k.commands_forwarded) == (1, 1)


def test_unknown_command_dropped():
    c = RelayCore()
    assert c.on_command(NOW, NOW, 99) is None
    assert c.counters().commands_forwarded == 0


def test_command_translation():
    c = RelayCore()
    arm = c.on_command(NOW, NOW, CMD_ARM)
    disarm = c.on_command(NOW, NOW, CMD_DISARM)
    offboard = c.on_command(NOW, NOW, CMD_OFFBOARD)
    land = c.on_command(NOW, NOW, CMD_LAND)
    hold = c.on_command(NOW, NOW, CMD_HOLD)
    assert (arm.command, arm.param1) == (400, 1.0)
    assert (disarm.command, disarm.param1) == (400, 0.0)
    assert (offboard.command, offboard.param1, offboard.param2) == (176, 1.0, 6.0)
    assert land.command == 21
    assert (hold.command, hold.param1, hold.param2, hold.param3) == (176, 1.0, 4.0, 3.0)

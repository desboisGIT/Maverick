import math

from maverick_mapping.frames import enu_to_ned, ned_to_enu, yaw_enu_to_ned, yaw_ned_to_enu


def test_position_axes_and_roundtrip():
    assert ned_to_enu(1.0, 0.0, 0.0) == (0.0, 1.0, -0.0)  # north -> +y
    assert ned_to_enu(0.0, 1.0, 0.0) == (1.0, 0.0, -0.0)  # east -> +x
    assert ned_to_enu(0.0, 0.0, -1.2)[2] == 1.2  # up
    p = (1.5, -2.0, -1.2)
    assert enu_to_ned(*ned_to_enu(*p)) == p


def test_yaw():
    assert math.isclose(yaw_ned_to_enu(0.0), math.pi / 2)  # facing north
    assert math.isclose(yaw_ned_to_enu(math.pi / 2), 0.0)  # facing east
    for y in (-3.0, -1.0, 0.0, 0.7, 3.0):
        assert math.isclose(yaw_enu_to_ned(yaw_ned_to_enu(y)), y, abs_tol=1e-9)

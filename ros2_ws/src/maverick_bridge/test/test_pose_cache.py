import pytest

pytest.importorskip("rclpy")
pytest.importorskip("px4_msgs")
from maverick_bridge.camera_node import PoseCache  # noqa: E402


def test_interpolates_at_capture_time():
    c = PoseCache()
    for k in range(5):  # 10 Hz, moving 1 m/s north
        c.set_position(float(k) * 0.1, 0.0, -1.0, True, 100.0 + 0.1 * k)
        c.set_attitude((1.0, 0.0, 0.0, 0.0), 100.0 + 0.1 * k)
    pos, q = c.at(100.25)
    assert pos[0] == pytest.approx(0.25)
    assert q == pytest.approx((1.0, 0.0, 0.0, 0.0))


def test_none_when_too_old_or_missing():
    c = PoseCache()
    assert c.at(100.0) is None
    c.set_position(0.0, 0.0, -1.0, True, 100.0)
    c.set_attitude((1.0, 0.0, 0.0, 0.0), 100.0)
    assert c.at(100.3) is not None  # hold within 0.5 s of the last sample
    assert c.at(100.7) is None
    assert c.at(99.0) is None  # before the history
    c.set_position(0.0, 0.0, -1.0, False, 100.8)  # invalid position clears the history
    assert c.at(100.8) is None

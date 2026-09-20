import math

import pytest
from maverick_bridge import vision as v
from maverick_bridge.camera_pose import matrix_to_quat, quat_to_matrix, rpy_to_matrix


def call(relay, stamp=100.0, now=100.05, pos=(1.0, 2.0, 3.0), q=(0.0, 0.0, 0.0, 1.0), **kw):
    args = dict(
        velocity_flu=(math.nan,) * 3,
        pos_var_enu=(0.01, 0.02, 0.03),
        ori_var_flu=(0.001, 0.002, 0.003),
        vel_var_flu=(0.1, 0.1, 0.1),
    )
    args.update(kw)
    return relay.on_odometry(stamp, now, pos, q, **args)


def test_position_and_variance_enu_to_ned():
    out = call(v.VisionRelay())
    assert out.position_ned == (2.0, 1.0, -3.0)
    assert out.position_var == (0.02, 0.01, 0.03)
    assert out.orientation_var == (0.001, 0.002, 0.003)


def test_level_body_facing_east_is_ned_yaw_90():
    # ENU/FLU identity: body x (forward) points east, up is up -> NED/FRD yaw = +90 deg
    out = call(v.VisionRelay())
    w, x, y, z = out.q_frd_ned
    assert (w, x, y) == pytest.approx((math.cos(math.pi / 4), 0.0, 0.0), abs=1e-9)
    assert z == pytest.approx(math.sin(math.pi / 4))


def test_facing_north_is_ned_yaw_zero_and_roll_pitch_map():
    # ENU yaw +90 deg (facing north) -> NED yaw 0
    q = matrix_to_quat(rpy_to_matrix(0, 0, math.pi / 2))
    w, x, y, z = q
    out = call(v.VisionRelay(), q=(x, y, z, w))
    assert out.q_frd_ned == pytest.approx((1.0, 0.0, 0.0, 0.0), abs=1e-9)
    # +0.2 rad pitch about FLU y (left) is nose DOWN (ROS/SDF convention)
    q = matrix_to_quat(rpy_to_matrix(0, 0.2, math.pi / 2))
    out = call(v.VisionRelay(), q=(q[1], q[2], q[3], q[0]))
    m = quat_to_matrix(*out.q_frd_ned)  # R_ned_frd for a body facing north, nose down
    # FRD x axis (forward) in NED: north with a downward (positive) component
    assert (m[0][0], m[2][0]) == pytest.approx((math.cos(0.2), math.sin(0.2)), abs=1e-9)


def test_velocity_flu_to_frd():
    out = call(v.VisionRelay(), velocity_flu=(1.0, 2.0, 3.0))
    assert out.velocity_frd == (1.0, -2.0, -3.0)


def test_staleness_and_future():
    r = v.VisionRelay()
    assert call(r, stamp=100.0, now=100.199) is not None
    assert call(r, stamp=100.0, now=100.201) is None
    assert call(r, stamp=100.0, now=99.94) is None  # stamped 60 ms in the future
    assert call(r, stamp=100.0, now=99.96) is not None
    k = r.counters
    assert (k.forwarded, k.dropped_stale, k.dropped_invalid) == (2, 2, 0)


@pytest.mark.parametrize(
    "kw",
    [
        {"pos": (math.nan, 0.0, 0.0)},
        {"pos": (math.inf, 0.0, 0.0)},
        {"q": (0.0, 0.0, 0.0, 0.0)},
        {"q": (math.nan, 0.0, 0.0, 1.0)},
        {"pos_var_enu": (0.01, -1.0, 0.01)},
        {"ori_var_flu": (math.nan, 0.0, 0.0)},
    ],
)
def test_invalid_is_dropped(kw):
    r = v.VisionRelay()
    assert call(r, **kw) is None
    assert r.counters.dropped_invalid == 1 and r.counters.forwarded == 0

import math
from pathlib import Path

import numpy as np
import pytest
from maverick_bridge.camera_pose import CameraMount, mount_in_body_flu
from maverick_vo import core


def rz(a):
    c, s = math.cos(a), math.sin(a)
    T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]
    return T


def trans(x, y, z):
    T = np.eye(4)
    T[:3, 3] = (x, y, z)
    return T


def test_quat_matrix_roundtrip():
    T = rz(0.7)
    q = core.matrix_to_quat(T[:3, :3])
    assert np.allclose(core.quat_to_matrix(*q), T[:3, :3])
    assert q[3] >= 0


def test_invert():
    T = trans(1, 2, 3) @ rz(0.4)
    assert np.allclose(T @ core.invert_T(T), np.eye(4))


def test_anchor_maps_odometry_into_map_frame():
    a = core.OdomAnchor()
    T0 = trans(2, 1, 0.5) @ rz(0.3)  # PX4-based camera pose at the first odometry frame
    a.push_pose(10.0, T0)
    assert a.to_map(10.0, np.eye(4)) is not None
    # camera moves 1 m forward (along its own z) after the start
    T_odom = trans(0, 0, 1)
    expected = T0 @ T_odom
    assert np.allclose(a.to_map(10.1, T_odom), expected)


def test_anchor_waits_for_matching_pose():
    a = core.OdomAnchor(pose_tol_s=0.02)
    a.push_pose(10.0, np.eye(4))
    assert a.to_map(10.5, np.eye(4)) is None  # no pose near this stamp
    assert a.T_map_odom is None
    assert a.to_map(10.01, np.eye(4)) is not None


def test_anchor_is_fixed_after_first_frame():
    a = core.OdomAnchor()
    a.push_pose(1.0, trans(1, 0, 0))
    a.to_map(1.0, np.eye(4))
    a.push_pose(2.0, trans(50, 50, 0))  # later PX4 poses (drifting) must not move the anchor
    assert np.allclose(a.to_map(2.0, trans(0, 0, 1))[:3, 3], (1, 0, 1))


def test_body_from_camera_with_real_mount():
    m = CameraMount.load(
        str(Path(__file__).resolve().parents[4] / "sim/models/maverick/camera_mount.json")
    )
    T_body_cam = core.body_to_camera(*mount_in_body_flu(m))
    # optical z (forward) is body x (forward); optical origin is 0.12 m ahead, 0.10 m above body
    assert np.allclose(T_body_cam[:3, 2], (1, 0, 0))
    assert np.allclose(T_body_cam[:3, 3], (0.12, 0, 0.10))


def test_validity_and_jump():
    assert core.odom_is_valid((1, 2, 3), 0.01)
    assert not core.odom_is_valid((1, 2, 3), 9999.0)
    assert not core.odom_is_valid((math.nan, 0, 0), 0.01)
    assert not core.jump_too_large(None, np.eye(4), 0.1)
    assert core.jump_too_large(np.eye(4), trans(2, 0, 0), 0.1, vmax=4.0)
    assert not core.jump_too_large(np.eye(4), trans(0.3, 0, 0), 0.1, vmax=4.0)
    assert pytest.approx(0.0) == 0.0

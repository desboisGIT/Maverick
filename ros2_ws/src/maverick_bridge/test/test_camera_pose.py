import json
import math
import re
import sys
from pathlib import Path

import pytest
from maverick_bridge.camera_pose import (
    CameraMount,
    camera_pose_enu,
    matrix_to_quat,
    quat_to_matrix,
    rpy_to_matrix,
)

ROOT = Path(__file__).resolve().parents[4]
MODELS = ["maverick", "maverick_small"]


def mount(xyz=(0.12, 0.0, 0.10), rpy=(0.0, 0.0, 0.0)) -> CameraMount:
    return CameraMount(
        "camera_optical", xyz, rpy, 1.274, (320, 240), (320, 240), (160, 120), (0.2, 19.1), 10.0
    )


def ned_attitude(roll=0.0, pitch=0.0, yaw=0.0):
    """PX4 q (w, x, y, z), FRD -> NED, from yaw (about down), pitch, roll."""
    return matrix_to_quat(rpy_to_matrix(roll, pitch, yaw))


def axes(q):
    """Columns of R(q): where the optical x, y, z axes point in ENU."""
    r = quat_to_matrix(*q)
    return [tuple(r[i][j] for i in range(3)) for j in range(3)]


def close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b, strict=True))


def test_facing_north_level():
    # yaw 0 = north. Optical z (forward) = north = +y ENU, x (right) = east = +x, y (down) = -z.
    (px, py, pz), q = camera_pose_enu((0.0, 0.0, -2.0), ned_attitude(), mount())
    x, y, z = axes(q)
    assert close(z, (0, 1, 0)) and close(x, (1, 0, 0)) and close(y, (0, 0, -1))
    # 0.12 m forward (north) and 0.10 m up of the body at 2 m height
    assert close((px, py, pz), (0.0, 0.12, 2.10))


def test_facing_east():
    (px, py, pz), q = camera_pose_enu((1.0, 2.0, -1.0), ned_attitude(yaw=math.pi / 2), mount())
    x, y, z = axes(q)
    assert close(z, (1, 0, 0)) and close(x, (0, -1, 0)) and close(y, (0, 0, -1))
    # body at east 2, north 1; camera 0.12 further east
    assert close((px, py, pz), (2.12, 1.0, 1.10))


def test_pitch_up_looks_up():
    # nose-up pitch in FRD/NED is positive pitch about the right wing
    _, q = camera_pose_enu((0, 0, 0), ned_attitude(pitch=math.radians(30)), mount())
    fwd = axes(q)[2]
    assert fwd[2] > 0.49 and close(fwd, (0, math.cos(math.radians(30)), 0.5))


def test_roll_right_wing_down():
    # roll right: the right wing (optical +x) goes down
    _, q = camera_pose_enu((0, 0, 0), ned_attitude(roll=math.radians(20)), mount())
    assert axes(q)[0][2] < 0


def test_mount_pitch_down_and_lever_arm_rotates_with_yaw():
    m = mount(xyz=(0.0, 0.1, 0.0), rpy=(0.0, math.radians(30), 0.0))  # sensor pitched down 30 deg
    (px, py, _), q = camera_pose_enu((0, 0, 0), ned_attitude(yaw=math.pi / 2), m)
    fwd = axes(q)[2]
    assert close(fwd, (math.cos(math.radians(30)), 0, -0.5))  # facing east, tilted down
    # body-left offset 0.1 m while facing east is north
    assert close((px, py), (0.0, 0.1))


@pytest.mark.parametrize("seed", range(5))
def test_matches_mapping_frames_yaw_rule(seed):
    """Camera heading in ENU equals maverick_mapping.frames.yaw_ned_to_enu(PX4 yaw)."""
    sys.path.insert(0, str(ROOT / "ros2_ws/src/maverick_mapping"))
    try:
        from maverick_mapping.frames import ned_to_enu, yaw_ned_to_enu
    finally:
        sys.path.pop(0)
    import random

    rng = random.Random(seed)
    yaw = rng.uniform(-math.pi, math.pi)
    pos = (rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-3, 0))
    m = mount(xyz=(0.0, 0.0, 0.0))
    p, q = camera_pose_enu(pos, ned_attitude(yaw=yaw), m)
    assert close(p, ned_to_enu(*pos))
    fwd = axes(q)[2]
    assert math.isclose(math.atan2(fwd[1], fwd[0]), yaw_ned_to_enu(yaw), abs_tol=1e-9)


def test_quaternion_roundtrip_is_unit_and_w_positive():
    for yaw in (-3.0, -1.0, 0.3, 2.9):
        for pitch in (-0.5, 0.0, 0.5):
            q = ned_attitude(roll=0.2, pitch=pitch, yaw=yaw)
            assert math.isclose(sum(v * v for v in q), 1.0) and q[0] >= 0
            r1, r2 = quat_to_matrix(*q), rpy_to_matrix(0.2, pitch, yaw)
            assert all(abs(r1[i][j] - r2[i][j]) < 1e-9 for i in range(3) for j in range(3))


def test_intrinsics_scale_by_two():
    m = mount()
    fx, fy, cx, cy = m.intrinsics(160, 120)
    Fx, Fy, Cx, Cy = m.intrinsics(320, 240)
    assert math.isclose(Fx, 2 * fx) and math.isclose(Cx, 2 * cx) and math.isclose(Cy, 2 * cy)
    assert math.isclose(Fx, 216.248, abs_tol=1e-3) and Fx == Fy


@pytest.mark.parametrize("model", MODELS)
def test_mount_json_matches_model_sdf(model):
    MOUNT_JSON = ROOT / f"sim/models/{model}/camera_mount.json"
    j = json.loads(MOUNT_JSON.read_text())
    sdf = (ROOT / f"sim/models/{model}/model.sdf").read_text()
    sdf = sdf[sdf.index("<link name='camera_link'>") :]  # the camera link only
    pose = re.search(r"<pose relative_to='base_link'>([^<]+)</pose>", sdf).group(1).split()
    assert [float(v) for v in pose] == pytest.approx(j["xyz_flu"] + j["rpy_flu"])
    fovs = {float(v) for v in re.findall(r"<horizontal_fov>([^<]+)</horizontal_fov>", sdf)}
    assert fovs == {j["horizontal_fov"]}
    sizes = {
        (int(w), int(h)) for w, h in re.findall(r"<width>(\d+)</width><height>(\d+)</height>", sdf)
    }
    assert sizes == {tuple(j["rgb_size"])}
    rates = {float(v) for v in re.findall(r"<update_rate>([^<]+)</update_rate>", sdf)}
    assert rates == {j["rate_hz"]}
    # depth is registered with the RGB image; semantic is rendered at that size and published
    # at an integer fraction of it
    assert j["depth_size"] == j["rgb_size"]
    assert j["rgb_size"][0] % j["semantic_size"][0] == 0
    assert j["rgb_size"][0] // j["semantic_size"][0] == j["rgb_size"][1] // j["semantic_size"][1]
    m = CameraMount.load(MOUNT_JSON)
    assert m.frame_id == "camera_optical"

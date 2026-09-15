"""Pure parts of the mapping node: message conversions, stamp sync, voxel delta encoding."""

from types import SimpleNamespace as NS

import numpy as np
from maverick_mapping import CameraIntrinsics, VoxelMap, optical_rotation
from maverick_mapping.ros_conv import (
    RateLimiter,
    SyncBuffer,
    camera_euler,
    depth_to_metres,
    grid_to_fields,
    hfov_deg,
    image_to_array,
    info_to_intrinsics,
    intrinsics_for,
    matrix_to_quat,
    occupancy_to_grid,
    pose_to_tr,
    quat_to_matrix,
    stamp_key,
)
from maverick_mapping.voxel_delta import DeltaEncoder, OccupiedSet, decode, encode


def image(arr: np.ndarray, encoding: str, step_pad: int = 0) -> NS:
    h, w = arr.shape
    row = np.zeros((h, w + step_pad), arr.dtype)
    row[:, :w] = arr
    return NS(
        encoding=encoding,
        height=h,
        width=w,
        step=row.strides[0],
        is_bigendian=0,
        data=row.tobytes(),
    )


def test_depth_16uc1_mm_to_metres_with_invalid():
    d = np.array([[0, 1000, 2500], [65535, 300, 0]], np.uint16)
    out = depth_to_metres(image(d, "16UC1"))
    assert out.dtype == np.float32
    assert np.isnan(out[0, 0]) and np.isnan(out[1, 2])
    assert np.allclose(out[0, 1:], [1.0, 2.5]) and np.isclose(out[1, 1], 0.3)


def test_image_with_row_padding_and_mono8():
    a = np.arange(12, dtype=np.uint8).reshape(3, 4)
    assert np.array_equal(image_to_array(image(a, "mono8", step_pad=4)), a)


def test_intrinsics_and_scaling():
    k = [100.0, 0, 79.5, 0, 100.0, 59.5, 0, 0, 1]
    intr = info_to_intrinsics(NS(width=160, height=120, k=k))
    assert (intr.fx, intr.cx, intr.cy) == (100.0, 79.5, 59.5)
    half = intrinsics_for(intr, 80, 60)
    assert half.width == 80 and abs(half.fx - 50.0) < 1e-9
    assert abs(hfov_deg(CameraIntrinsics.from_hfov(160, 120, 73.0)) - 73.0) < 1e-6


def test_quaternion_roundtrip_and_pose():
    for yaw in (-2.5, 0.0, 1.0, 3.0):
        r = optical_rotation(yaw, 0.1)
        q = matrix_to_quat(r)
        assert np.allclose(quat_to_matrix(*q), r, atol=1e-9)
    r = optical_rotation(0.7)
    q = matrix_to_quat(r)
    msg = NS(
        pose=NS(position=NS(x=1.0, y=2.0, z=3.0), orientation=NS(x=q[0], y=q[1], z=q[2], w=q[3]))
    )
    t, rr = pose_to_tr(msg)
    assert np.allclose(t, [1, 2, 3]) and np.allclose(rr, r, atol=1e-9)
    yaw, pitch, roll = camera_euler(rr)
    assert abs(yaw - 0.7) < 1e-9 and abs(pitch) < 1e-9 and abs(roll) < 1e-9


def test_camera_euler_pitch_up_and_roll_signs():
    assert camera_euler(optical_rotation(0.0, 0.3))[1] > 0.29  # nose up positive
    # optical x (right) pointing downwards = right side down = positive roll
    r = optical_rotation(0.0)
    c, s = np.cos(0.2), np.sin(0.2)
    roll_about_fwd = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])  # rotate about optical z
    r2 = r @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    assert abs(camera_euler(r2)[2]) > 0.19
    assert roll_about_fwd.shape == (3, 3)


def test_grid_roundtrip_through_occupancy_fields():
    g = np.full((5, 3), -1, np.int8)
    g[1, 2], g[4, 0], g[2, 1] = 100, 0, 0
    f = grid_to_fields(g, 0.1, (-1.0, 2.0))
    assert (f["width"], f["height"]) == (5, 3)
    msg = NS(
        info=NS(
            width=f["width"],
            height=f["height"],
            resolution=f["resolution"],
            origin=NS(position=NS(x=f["origin_x"], y=f["origin_y"])),
        ),
        data=f["data"],
    )
    g2, res, org = occupancy_to_grid(msg)
    assert np.array_equal(g2, g) and res == 0.1 and org == (-1.0, 2.0)
    # ROS layout: cell (ix=1, iy=2) is data[iy * width + ix]
    assert f["data"][2 * 5 + 1] == 100


def test_sync_buffer_matches_by_stamp_in_any_order():
    s = SyncBuffer(["depth", "pose"], tol=0.01)
    assert s.push("depth", 1.0, "d1") == []
    assert s.push("depth", 1.2, "d2") == []
    out = s.push("pose", 1.2, "p2")
    assert out == [{"depth": "d2", "pose": "p2"}]
    assert s.push("pose", 1.0, "p1") == []  # older than a completed set: dropped
    assert s.push("pose", 1.4, "p3") == []
    assert s.push("depth", 1.405, "d3") == [{"depth": "d3", "pose": "p3"}]
    assert s.push("depth", 1.6, "d4") == [] and s.push("pose", 1.7, "p4") == []  # tol exceeded


def test_sync_buffer_three_topics_and_stamp_key():
    s = SyncBuffer(["a", "b", "c"], tol=0.0)
    assert s.push("a", 5.0, 1) == [] and s.push("b", 5.0, 2) == []
    assert s.push("c", 5.0, 3) == [{"a": 1, "b": 2, "c": 3}]
    assert stamp_key(NS(sec=2, nanosec=5)) == 2_000_000_005


def test_rate_limiter():
    now = [0.0]
    r = RateLimiter(5.0, lambda: now[0])
    assert r.ready() and not r.ready()
    now[0] = 0.19
    assert not r.ready()
    now[0] = 0.21
    assert r.ready()


def test_voxel_code_roundtrip_negative_indices():
    idx = np.array([[-5, 0, 3], [10, -2000, 2000], [0, 0, 0]])
    assert np.array_equal(np.sort(decode(encode(idx)), axis=0), np.sort(idx, axis=0))


def wall_scan(vmap: VoxelMap, x_wall: float) -> None:
    intr = CameraIntrinsics.from_hfov(40, 30, 70.0)
    vmap.integrate_depth(
        np.full((30, 40), x_wall), intr, np.array([0.0, 0.0, 1.0]), optical_rotation(0.0)
    )


def test_delta_stream_reconstructs_the_map_across_growth():
    vm = VoxelMap(0.1)
    enc, cons = DeltaEncoder(), OccupiedSet()
    wall_scan(vm, 2.0)
    d0 = enc.delta(vm)
    assert d0.reset and len(d0.add) > 20
    cons.apply(d0.to_json(vm.res, vm._lattice, 0.0, 1))
    assert enc.delta(vm).empty()
    wall_scan(vm, 3.0)  # grows the array, adds a second wall
    d1 = enc.delta(vm)
    assert not d1.reset and len(d1.add) > 20
    cons.apply(d1.to_json(vm.res, vm._lattice, 0.0, 2))
    truth = np.sort(
        np.stack(np.nonzero(vm.occupied_mask()), 1)
        + np.round((vm.origin - vm._lattice) / vm.res).astype(int),
        axis=0,
    )
    got = np.sort(decode(cons.codes), axis=0)
    assert np.array_equal(got, truth)
    # voxel centre convention of the gateway doc: origin + (index + 0.5) * resolution
    c = cons.origin[0] + (got[:, 0] + 0.5) * cons.resolution
    assert np.all(c > 1.9) and np.all(c < 3.2)
    snap = cons.snapshot()
    assert snap["reset"] and len(snap["add"]) == 3 * len(got)


def test_delta_reports_removed_voxels():
    vm = VoxelMap(0.1)
    enc = DeltaEncoder()
    wall_scan(vm, 2.0)
    enc.delta(vm)
    vm.logodds[:] = 0.0  # everything forgotten
    d = enc.delta(vm)
    assert len(d.add) == 0 and len(d.remove) > 20

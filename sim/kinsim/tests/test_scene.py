"""Ray caster and contact model against the offline harness' brute force and the referee."""

import math

import kinsim  # noqa: F401  (sys.path of the drone stack)
import numpy as np
import pytest
from kinsim.camera import DepthCamera, Mount, camera_pose
from kinsim.scene import KScene


@pytest.fixture(scope="module")
def scene():
    return KScene.from_seed(30)


@pytest.fixture(scope="module")
def cam(scene):
    return DepthCamera(scene, Mount.load(), map_stride=2)


def test_raycast_matches_brute_force(scene, cam):
    """Depth of the mapping pixels equals the legacy exact ray cast (to the mm quantisation)."""
    rng = np.random.default_rng(1)
    for _ in range(4):
        body = np.array([rng.uniform(-1, 3), rng.uniform(-1, 3), 1.2])
        yaw = rng.uniform(-math.pi, math.pi)
        ct, cr = camera_pose(body, yaw, cam.mount)
        d = cam.depth_for_mapping(ct, cr)[::2, ::2].ravel()
        z = scene.ref_scene.raycast(ct, cam.map_rays.dirs.astype(float) @ cr.T, 19.1)
        ok = np.isfinite(z) & np.isfinite(d)
        assert np.array_equal(np.isfinite(z), np.isfinite(d))
        # visual-only boxes (rugs, small objects) may be nearer by a few cm; walls agree to 1 mm
        assert np.median(np.abs(z[ok] - d[ok])) < 1e-3
        assert np.mean(np.abs(z[ok] - d[ok]) < 2e-3) > 0.97


def test_semantic_image_labels_walls_and_floor(scene, cam):
    ct, cr = camera_pose(np.array([0.0, 0.0, 1.2]), scene.spawn[2], cam.mount)
    sem, depth = cam.semantic_and_depth(ct, cr)
    assert sem.shape == depth.shape == (120, 160)
    assert 2 in set(np.unique(sem))  # walls are class 2 in worldgen
    assert np.isfinite(depth).mean() > 0.9


def test_contact_clearance_equals_referee(scene):
    rng = np.random.default_rng(2)
    for _ in range(25):
        p = np.array([rng.uniform(-1, 4), rng.uniform(-1, 4), 1.2])
        c, n, g = scene.contact(p, 0.12)
        ref = scene.ref_scene.horizontal_clearance(p, 0.12)
        if math.isfinite(c):
            assert c == pytest.approx(ref, abs=1e-6)
            assert np.hypot(*n) == pytest.approx(1.0, abs=1e-6)
        else:
            assert ref > 1.0


def test_contact_normal_points_away_from_the_wall(scene):
    # step towards the nearest solid: the normal at the contact points back to the free point
    p = np.array([0.0, 0.0, 1.2])
    c, n, g = scene.contact(p, 0.12)
    assert g >= 0
    q = p.copy()
    q[:2] -= n * (c - 0.05)  # move toward the obstacle until 5 cm away
    c2, n2, _ = scene.contact(q, 0.12)
    assert c2 == pytest.approx(0.05, abs=1e-3)
    assert float(n @ n2) > 0.5


def test_scene_is_in_the_map_frame(scene):
    # the spawn point is the origin and is free
    c, _, _ = scene.contact(np.array([0.0, 0.0, 1.2]), 0.12)
    assert c > 0.5

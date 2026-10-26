import math

import numpy as np
from maverick_mapping.frontiers import detect_frontiers, frontier_mask
from maverick_mapping.planner import (
    ExplorationPlanner,
    PlannerConfig,
    PlanningGrid,
    astar,
    shorten,
)
from maverick_mapping.synthetic import CameraSim, Scene
from maverick_mapping.voxel_map import FREE, OCCUPIED, UNKNOWN, VoxelMap, optical_rotation


def _box_scene() -> Scene:
    # 6 x 6 room: floor slab and four 0.1 m walls, 2.5 m high
    c = [[3, 3, -0.5], [3, 0, 1.25], [3, 6, 1.25], [0, 3, 1.25], [6, 3, 1.25]]
    h = [
        [4, 4, 0.5],
        [3.05, 0.05, 1.25],
        [3.05, 0.05, 1.25],
        [0.05, 3.05, 1.25],
        [0.05, 3.05, 1.25],
    ]
    manifest = {
        "footprint": [[0, 0], [6, 6]],
        "ceiling_height": 2.5,
        "spawn": {"pose": [3, 3, 0, 0]},
    }
    return Scene(c, [np.eye(3)] * 5, h, manifest)


def test_raycast_hits_wall():
    sc = _box_scene()
    t = sc.raycast(np.array([3.0, 3.0, 1.2]), np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]), 10)
    assert math.isclose(float(t[0]), 2.95, abs_tol=1e-3)
    assert np.isinf(t[1])  # no ceiling in this scene


def test_integration_marks_wall_and_free_space():
    sc = _box_scene()
    cam = CameraSim(noise_base=0.0, noise_quad=0.0)
    vm = VoxelMap(0.1)
    t = np.array([3.0, 3.0, 1.2])
    r = optical_rotation(0.0)
    vm.integrate_depth(cam.render(sc, t, r), cam.intr, t, r)
    assert vm.state(np.array([3.5, 3.0, 1.2])) == FREE
    assert vm.state(np.array([5.95, 3.0, 1.2])) == OCCUPIED
    assert vm.state(np.array([3.0, 5.0, 1.2])) == UNKNOWN  # behind the camera's FOV
    grid, res, org = vm.project_2d(0.3, 2.0)
    assert res == 0.1 and (grid == OCCUPIED).any() and (grid == FREE).any()
    assert vm.stats()["observed_volume_m3"] > 0.5


def test_invalid_depth_ignored_and_map_grows():
    vm = VoxelMap(0.1)
    cam = CameraSim()
    depth = np.full((cam.height, cam.width), np.nan)
    vm.integrate_depth(depth, cam.intr, np.zeros(3), optical_rotation(0.0))
    assert vm.shape == (0, 0, 0)
    depth[:] = 2.0
    vm.integrate_depth(depth, cam.intr, np.array([10.0, 10.0, 1.0]), optical_rotation(0.0))
    n = vm.shape
    vm.integrate_depth(depth, cam.intr, np.array([-5.0, 10.0, 1.0]), optical_rotation(0.0))
    assert vm.shape[0] > n[0]
    assert vm.state(np.array([12.0, 10.0, 1.0])) == OCCUPIED  # survived the growth


def test_frontier_detection_and_planner():
    g = np.full((40, 40), UNKNOWN, np.int8)
    g[5:30, 5:30] = FREE
    g[4, 4:31] = g[30, 4:31] = g[4:31, 4] = OCCUPIED
    g[4:31, 30] = OCCUPIED
    g[30, 15:20] = UNKNOWN  # an opening
    fm = frontier_mask(g)
    assert fm[29, 15:20].all() and not fm[10, 10]
    fr = detect_frontiers(g, 0.1, (0.0, 0.0), min_size=3)
    assert len(fr) == 1 and fr[0].size >= 3
    pl = ExplorationPlanner(PlannerConfig())
    res = pl.plan(g, 0.1, (0.0, 0.0), (1.0, 1.0))
    assert res.status == "goal" and res.waypoints
    g2 = g.copy()
    g2[30, 15:20] = OCCUPIED
    assert ExplorationPlanner().plan(g2, 0.1, (0.0, 0.0), (1.0, 1.0)).status == "done"


def test_astar_avoids_obstacle_and_shortens():
    g = np.zeros((60, 60), np.int8)
    g[30, 10:50] = OCCUPIED
    pg = PlanningGrid.build(g, 0.1, (0.0, 0.0), PlannerConfig())
    path = astar(pg, (10, 10), (50, 50))
    assert path is not None and path[0] == (10, 10) and path[-1] == (50, 50)
    wp = shorten(pg, path, (1.05, 1.05))
    assert 2 <= len(wp) <= 6
    assert astar(pg, (10, 10), (30, 30)) is None  # goal inside the inflated obstacle


def test_scene_from_worldgen_and_gt_volume():
    sc = Scene.from_seed(7)
    assert sc.n_boxes > 20
    from maverick_mapping.metrics import GroundTruth

    gt = GroundTruth(sc)
    assert gt.volume_m3 > 50
    sx, sy, _ = sc.spawn
    assert sc.min_clearance(np.array([sx, sy, 1.2])) > 0.5


def test_c6_area_recall_weights_by_area_and_ignores_regions_the_drone_cannot_fit_in():
    from maverick_mapping.metrics import c6_area_recall

    # one snapshot: 400 cells found, a 30-cell sliver missed -> 0.93 by area (the count rule: 0.5)
    snap = [(400, 6, True), (30, 5, False)]
    assert abs(c6_area_recall([snap]) - 400 / 430) < 1e-9
    # the same sliver with clearance level 2 (narrower than the drone) is not scored at all
    assert c6_area_recall([[(400, 6, True), (30, 2, False)]]) == 1.0
    # a snapshot without any scored region does not count; no snapshot at all -> 1.0
    assert c6_area_recall([[(30, 2, False)], [(100, 5, True)]]) == 1.0
    assert c6_area_recall([]) == 1.0
    # snapshots average with equal weight
    assert abs(c6_area_recall([[(100, 5, True)], [(100, 5, False)]]) - 0.5) < 1e-9

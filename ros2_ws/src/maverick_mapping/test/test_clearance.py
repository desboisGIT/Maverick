"""Sector clearance around the robot from the voxel map."""

import math

import numpy as np
from maverick_mapping.clearance import occupied_window, sector_clearance
from maverick_mapping.voxel_map import VoxelMap


def occ_grid(cells, shape=(30, 30)):
    g = np.zeros(shape, bool)
    for i, j in cells:
        g[i, j] = True
    return g


def test_no_obstacle_is_rmax():
    out = sector_clearance(occ_grid([]), 0.1, (0.0, 0.0), (1.5, 1.5), rmax=0.8, n_sectors=24)
    assert out == [0.8] * 24


def test_obstacle_ahead_lands_in_the_right_sector():
    # cell (19, 15) has its centre at (1.95, 1.55); robot at (1.5, 1.5) -> 0.45 m, bearing ~0 rad
    out = sector_clearance(occ_grid([(19, 15)]), 0.1, (0.0, 0.0), (1.5, 1.5), 0.8, 24)
    k = int(np.argmin(out))
    w = 2 * math.pi / 24
    b = -math.pi + (k + 0.5) * w
    assert abs(b) < w
    assert abs(out[k] - math.hypot(0.45, 0.05)) < 1e-3
    assert sum(1 for d in out if d < 0.8) == 1


def test_nearest_cell_wins_and_far_cells_are_ignored():
    cells = [(19, 15), (17, 15), (28, 15)]  # 0.45, 0.25 and 1.35 m away
    out = sector_clearance(occ_grid(cells), 0.1, (0.0, 0.0), (1.5, 1.5), 0.8, 24)
    assert min(out) < 0.26
    assert all(d <= 0.8 for d in out)


def test_window_from_voxel_map():
    vm = VoxelMap(0.1)
    vm.ensure(np.array([0.0, 0.0, 0.0]), np.array([3.0, 3.0, 2.0]))
    # a wall voxel column at x = 1.95 (band 0.35-1.6)
    i = vm.index(np.array([[1.95, 1.55, 1.0]]))[0]
    vm.logodds[i[0], i[1], :] = 2.0
    occ, org = occupied_window(vm, (1.5, 1.5), 0.8, (0.35, 1.6))
    assert occ.any()
    out = sector_clearance(occ, vm.res, org, (1.5, 1.5), 0.8, 24)
    assert abs(min(out) - math.hypot(0.45, 0.05)) < 0.06


def test_obstacle_outside_the_band_is_ignored():
    vm = VoxelMap(0.1)
    vm.ensure(np.array([0.0, 0.0, 0.0]), np.array([3.0, 3.0, 2.0]))
    i = vm.index(np.array([[1.95, 1.55, 0.1]]))[0]  # a floor-level box below the 0.35 m band
    vm.logodds[i[0], i[1], i[2]] = 2.0
    occ, org = occupied_window(vm, (1.5, 1.5), 0.8, (0.35, 1.6))
    assert not occ.any()


def test_window_outside_the_map_is_empty():
    vm = VoxelMap(0.1)
    occ, _ = occupied_window(vm, (1.5, 1.5), 0.8, (0.35, 1.6))
    assert occ.size == 0
    assert sector_clearance(occ, 0.1, (0.0, 0.0), (1.5, 1.5), 0.8, 24) == [0.8] * 24

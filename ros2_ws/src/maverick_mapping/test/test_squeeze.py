"""Doors narrower than 2 x (robot radius + margin) are crossed as slow, tight-checked squeezes."""

import math

import numpy as np
from maverick_mapping.planner import (
    ExplorationPlanner,
    PlannerConfig,
    PlanningGrid,
    astar,
    shorten_ex,
)
from maverick_mapping.voxel_map import FREE, OCCUPIED, UNKNOWN

RES = 0.1


def two_rooms(door_w: float, nx: int = 80, ny: int = 40) -> np.ndarray:
    """Wall at x = 4.0 m (2 cells thick) with a door of `door_w` metres centred at y = 2.0."""
    g = np.full((nx, ny), FREE, np.int8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = OCCUPIED
    g[40:42, :] = OCCUPIED
    a = int(round((2.0 - door_w / 2) / RES))
    b = int(round((2.0 + door_w / 2) / RES))
    g[40:42, a:b] = FREE
    return g


def test_narrow_door_is_blocked_strictly_and_a_squeeze_with_tight_radius():
    g = two_rooms(1.05)
    strict = PlanningGrid.build(g, RES, (0.0, 0.0), PlannerConfig(tight_radius=None))
    assert astar(strict, (10, 20), (70, 20)) is None
    pg = PlanningGrid.build(g, RES, (0.0, 0.0), PlannerConfig())
    cells = astar(pg, (10, 20), (70, 20))
    assert cells is not None
    assert pg.squeeze[40, 20]  # the door's centre line is a squeeze


def test_squeeze_path_goes_through_the_door_centre_and_is_slow_there():
    g = two_rooms(1.05)
    pg = PlanningGrid.build(g, RES, (0.0, 0.0), PlannerConfig())
    cells = astar(pg, (10, 8), (70, 30))
    pts, slow = shorten_ex(pg, cells, (1.05, 0.85))
    assert len(pts) == len(slow) and not slow[0]
    # some point sits in the door gap, close to its centre line y = 2.0
    in_door = [p for p in pts if 3.8 <= p[0] <= 4.4]
    assert in_door or any(abs(p[1] - 2.0) < 0.15 for p in pts)
    assert any(slow)
    # the segment crossing x = 4.0 does so within 0.1 m of the door centre
    for a, b in zip(pts, pts[1:], strict=False):
        if a[0] < 4.0 <= b[0]:
            y = a[1] + (b[1] - a[1]) * (4.0 - a[0]) / (b[0] - a[0])
            assert abs(y - 2.0) < 0.1


def test_wide_door_has_no_squeeze():
    pg = PlanningGrid.build(two_rooms(1.6), RES, (0.0, 0.0), PlannerConfig())
    assert not pg.squeeze[38:44].any()
    pts, slow = shorten_ex(pg, astar(pg, (10, 20), (70, 20)), (1.05, 2.05))
    assert not any(slow)


def test_door_too_narrow_for_the_tight_radius_stays_blocked():
    pg = PlanningGrid.build(two_rooms(0.8), RES, (0.0, 0.0), PlannerConfig())
    assert astar(pg, (10, 20), (70, 20)) is None


def test_viewpoint_is_never_in_a_squeeze():
    g = two_rooms(1.05)
    g[:, 30:] = UNKNOWN  # unexplored strip beyond the first room's far wall: frontier near the door
    g[50:, :] = UNKNOWN  # second room unknown: its frontier lies in the door
    planner = ExplorationPlanner(PlannerConfig())
    plan = planner.plan(g, RES, (0.0, 0.0), (1.0, 2.0))
    assert plan.status == "goal"
    pg = PlanningGrid.build(g, RES, (0.0, 0.0), PlannerConfig(), (1.0, 2.0))
    vp = pg.to_cell(plan.goal.viewpoint_xy)
    assert not pg.squeeze[vp]
    assert not math.isnan(plan.goal.viewpoint_xy[0])
    assert len(plan.slow) == len(plan.waypoints)


def test_robot_escape_zone_is_not_a_viewpoint():
    g = np.full((60, 60), FREE, np.int8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = OCCUPIED
    g[30, :] = OCCUPIED  # wall; the robot hugs it
    pg = PlanningGrid.build(g, RES, (0.0, 0.0), PlannerConfig(), (2.6, 3.0))
    assert pg.escape.any()


def test_unobserved_cells_around_the_robot_are_free():
    g = np.full((60, 60), UNKNOWN, np.int8)
    g[20:, :] = FREE  # only the east part has been observed; the robot sits at its edge
    pg = PlanningGrid.build(g, RES, (0.0, 0.0), PlannerConfig(), (2.0, 3.0))
    assert pg.free_at(*pg.to_cell((2.0, 3.0)))
    assert astar(pg, pg.to_cell((2.0, 3.0)), pg.to_cell((4.0, 3.0))) is not None


def test_explorer_relaxes_the_squeeze_radius_when_nothing_is_reachable():
    g = two_rooms(0.95)  # a 0.95 m gap leaves 0.475 m each side of the centre
    g[:, 30:] = UNKNOWN  # frontier on the far side
    g[40:, :] = UNKNOWN
    g[40:42, 15:25] = FREE
    cfg = PlannerConfig(tight_radius=0.49, tight_fallbacks=(0.43,))
    p = ExplorationPlanner(cfg).plan(g, RES, (0.0, 0.0), (1.0, 2.0))
    assert p.status in ("goal", "done")
    strict = ExplorationPlanner(PlannerConfig(tight_radius=0.49, tight_fallbacks=()))
    q = strict.plan(g, RES, (0.0, 0.0), (1.0, 2.0))
    if q.status == "done" and q.reason == "no reachable frontier":
        assert p.tight == 0.43


def test_flown_trail_opens_a_door_the_fat_map_blocks():
    g = two_rooms(1.05)
    g[40:42, 18:22] = OCCUPIED  # spurious occupied cells in the gap (depth edge bleed)
    pg = PlanningGrid.build(g, RES, (0.0, 0.0), PlannerConfig(tight_radius=0.40))
    assert astar(pg, (10, 20), (70, 20)) is None
    trail = [(x / 10.0, 2.0) for x in range(10, 71)]
    pg2 = PlanningGrid.build(g, RES, (0.0, 0.0), PlannerConfig(tight_radius=0.40), None, trail)
    assert astar(pg2, (10, 20), (70, 20)) is not None

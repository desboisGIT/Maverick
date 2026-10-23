"""Careful flying: slow segments near obstacles and at sharp turns; viewpoints keep clearance."""

import numpy as np
from maverick_mapping.planner import (
    ExplorationPlanner,
    PlannerConfig,
    PlanningGrid,
    mark_careful,
)
from maverick_mapping.voxel_map import FREE, OCCUPIED, UNKNOWN

RES = 0.1


def room(nx=80, ny=60) -> np.ndarray:
    g = np.full((nx, ny), FREE, np.int8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = OCCUPIED
    return g


def test_segment_near_a_wall_is_slow_and_one_in_open_space_is_not():
    cfg = PlannerConfig(robot_radius=0.15, margin=0.2)
    pg = PlanningGrid.build(room(), RES, (0.0, 0.0), cfg)
    # 0.5 m from the south wall: within hard (0.35) + careful (0.25) of it
    assert mark_careful(pg, (1.0, 0.5), [(6.0, 0.5)], [False], 0.9) == [True]
    assert mark_careful(pg, (1.5, 3.0), [(6.0, 3.0)], [False], 0.9) == [False]


def test_sharp_turn_slows_both_segments_around_it():
    cfg = PlannerConfig(robot_radius=0.15, margin=0.2)
    pg = PlanningGrid.build(room(), RES, (0.0, 0.0), cfg)
    pts = [(4.0, 3.0), (4.0, 1.5), (4.0, 2.5)]  # 180 degree reversal at the 2nd waypoint
    slow = mark_careful(pg, (2.0, 3.0), pts[:2], [False, False], 0.9)
    assert slow[0] and slow[1]
    slow = mark_careful(pg, (2.0, 3.0), [(4.0, 3.0), (6.0, 4.0)], [False, False], 0.9)
    assert not any(slow)  # 27 degrees


def test_unobserved_cells_count_as_careful():
    g = room()
    g[40:, :] = UNKNOWN
    pg = PlanningGrid.build(g, RES, (0.0, 0.0), PlannerConfig(robot_radius=0.15, margin=0.2))
    assert mark_careful(pg, (2.0, 3.0), [(3.5, 3.0)], [False], 0.9) == [True]


def test_viewpoint_prefers_clearance_among_cells_near_a_frontier():
    g = room(80, 60)
    g[:, 40:] = UNKNOWN  # unexplored top half, frontier along y = 3.9
    g[30:33, 20:40] = OCCUPIED  # a pillar by the frontier
    cfg = PlannerConfig(robot_radius=0.15, margin=0.2, vp_clear_weight=1.5)
    plan = ExplorationPlanner(cfg).plan(g, RES, (0.0, 0.0), (1.0, 1.0))
    assert plan.status == "goal"
    pg = PlanningGrid.build(g, RES, (0.0, 0.0), cfg, (1.0, 1.0))
    vp = pg.to_cell(plan.goal.viewpoint_xy)
    free_far = PlannerConfig(robot_radius=0.15, margin=0.2, vp_clear_weight=0.0)
    plan0 = ExplorationPlanner(free_far).plan(g, RES, (0.0, 0.0), (1.0, 1.0))
    vp0 = pg.to_cell(plan0.goal.viewpoint_xy)
    assert pg.vp_pen[vp] <= pg.vp_pen[vp0]
    assert len(plan.slow) == len(plan.waypoints) and plan.slow[-1]

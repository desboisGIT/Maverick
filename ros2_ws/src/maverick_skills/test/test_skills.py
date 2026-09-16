"""Skill state machines on a hand-made 2D grid: waypoint following, explore, goto_room."""

import math

import numpy as np
from maverick_mapping.frames import ned_to_enu, yaw_ned_to_enu
from maverick_skills.skills import (
    Ctx,
    ExploreSkill,
    GotoRoomSkill,
    SkillConfig,
    WaypointFollower,
    goal_message,
    make_skill,
    snap_free,
)

RES = 0.1


def open_room_grid() -> tuple[np.ndarray, float, tuple[float, float]]:
    """6 x 6 m of known free floor, walls on three sides, the east side open onto unknown."""
    g = np.full((90, 70), -1, np.int8)
    g[5:65, 5:65] = 0
    g[5:65, 5:7] = 100  # south wall
    g[5:65, 63:65] = 100  # north wall
    g[5:7, 5:65] = 100  # west wall
    return g, RES, (0.0, 0.0)


def ctx(now, robot, grid=None, grid_time=None):
    g = grid if grid is not None else open_room_grid()
    return Ctx(now, robot, g, now if grid_time is None else grid_time)


def test_goal_message_is_ned_and_roundtrips():
    m = goal_message(3.0, 1.0, 1.2, math.radians(30), 0.8)
    assert (m["mode"], m["speed"]) == ("goto", 0.8)
    assert (m["x"], m["y"], m["z"]) == (1.0, 3.0, -1.2)  # north = ENU y, east = ENU x, down = -z
    assert ned_to_enu(m["x"], m["y"], m["z"]) == (3.0, 1.0, 1.2)
    assert abs(yaw_ned_to_enu(m["yaw"]) - math.radians(30)) < 1e-3


def test_follower_advances_and_reports_arrival():
    f = WaypointFollower(arrive_radius=0.2)
    f.set([(1.0, 0.0), (1.0, 2.0)], 0.0)
    assert f.take_new_target() == (1.0, 0.0) and f.take_new_target() is None
    assert f.update((0.0, 0.0), 0.1) == "moving"
    assert f.update((0.95, 0.05), 0.2) == "moving"  # first reached, now heading to the second
    assert f.take_new_target() == (1.0, 2.0)
    assert f.update((1.0, 1.95), 0.3) == "arrived"
    assert f.target is None


def test_follower_detects_a_stall_but_not_slow_progress():
    f = WaypointFollower(arrive_radius=0.2, stuck_s=5.0, min_progress=0.1)
    f.set([(5.0, 0.0)], 0.0)
    assert f.update((0.0, 0.0), 0.0) == "moving"
    assert f.update((0.3, 0.0), 4.0) == "moving"  # progress resets the clock
    assert f.update((0.31, 0.0), 8.0) == "moving"
    assert f.update((0.31, 0.0), 9.5) == "stuck"


def test_follower_same_first_target_is_not_resent():
    f = WaypointFollower()
    f.set([(1.0, 1.0), (2.0, 2.0)], 0.0)
    f.take_new_target()
    f.set([(1.0, 1.01), (3.0, 3.0)], 1.0)
    assert f.take_new_target() is None
    f.set([(0.0, 1.0)], 2.0)
    assert f.take_new_target() == (0.0, 1.0)


def test_make_skill_validates_requests():
    cfg = SkillConfig()
    assert make_skill({"skill": "cancel"}, cfg, 0.0) is None
    s = make_skill({"skill": "explore", "budget_s": 120}, cfg, 0.0)
    assert isinstance(s, ExploreSkill) and s.budget == 120
    assert make_skill({"skill": "explore"}, cfg, 0.0).budget == cfg.default_budget_s
    g = make_skill({"skill": "goto_room", "room_type": "Kitchen"}, cfg, 0.0)
    assert isinstance(g, GotoRoomSkill) and g.room_type == "kitchen"
    for bad in ({"skill": "goto_room"}, {"skill": "dance"}, {}):
        try:
            make_skill(bad, cfg, 0.0)
        except ValueError:
            continue
        raise AssertionError(bad)


def test_explore_plans_a_goal_towards_the_open_side_and_reports_progress():
    cfg = SkillConfig()
    s = ExploreSkill(cfg, 0.0, budget_s=100.0)
    out = s.step(ctx(0.0, (2.0, 3.0)))
    assert out.goal is not None and out.path["status"] == "executing"
    x, y, _ = out.goal
    assert x > 2.0  # the only frontier is east
    assert out.path["waypoints"][0][:2] == [2.0, 3.0]
    assert out.progress["state"] == "running" and 0 <= out.progress["progress"] < 0.1
    assert out.progress["eta_s"] == 100.0


def test_explore_waits_for_a_map_and_pose():
    s = ExploreSkill(SkillConfig(), 0.0)
    out = s.step(Ctx(0.0, None, None))
    assert out.goal is None and "waiting" in out.progress["message"] and not out.finished


def drive(skill, robot, steps=400, dt=0.5, grid=None):
    """Teleport the robot to every goal the skill sends; returns (final out, goals sent)."""
    goals, now = [], 0.0
    out = None
    for _ in range(steps):
        now += dt
        out = skill.step(ctx(now, robot, grid))
        if out.goal is not None:
            goals.append(out.goal)
            robot = (out.goal[0], out.goal[1])
        if out.finished:
            break
    return out, goals


def test_explore_terminates_when_frontiers_never_resolve():
    s = ExploreSkill(SkillConfig(look_s=0.5), 0.0, budget_s=1e6)
    out, goals = drive(s, (2.0, 3.0))
    assert out.finished and out.progress["state"] == "succeeded"
    assert "explored" in out.progress["message"] and goals
    assert out.path["waypoints"] == []  # path cleared on success


def test_explore_stuck_blacklists_the_viewpoint_and_replans():
    cfg = SkillConfig(stuck_s=3.0)
    s = ExploreSkill(cfg, 0.0, budget_s=1e6)
    first = s.step(ctx(0.0, (2.0, 3.0))).goal
    sent = []
    for k in range(1, 40):  # the robot never moves
        out = s.step(ctx(k * 0.5, (2.0, 3.0)))
        if out.goal is not None:
            sent.append(out.goal)
    assert first is not None
    assert s.planner.failed_vp  # the stalled viewpoint was given up on
    assert all(math.dist(g[:2], first[:2]) > 1e-6 for g in sent[:1]) or sent == []


def test_explore_budget_and_cancel_hold_the_drone():
    s = ExploreSkill(SkillConfig(), 0.0, budget_s=10.0)
    s.step(ctx(1.0, (2.0, 3.0)))
    out = s.step(ctx(11.0, (2.5, 3.0)))
    assert out.finished and out.hold and out.progress["state"] == "succeeded"
    s2 = ExploreSkill(SkillConfig(), 0.0)
    s2.step(ctx(1.0, (2.0, 3.0)))
    c = s2.cancel(ctx(2.0, (2.2, 3.0)))
    assert c.finished and c.hold and c.progress["state"] == "cancelled"


def test_goto_room_query_answer_path_and_arrival():
    cfg = SkillConfig(goal_tolerance=0.4)
    s = GotoRoomSkill(cfg, 0.0, "kitchen")
    out = s.step(ctx(0.0, (1.0, 1.0)))
    q = out.query
    assert (
        q["query"] == "goal_for_type" and q["room_type"] == "kitchen" and q["start"] == [1.0, 1.0]
    )
    assert s.step(ctx(0.5, (1.0, 1.0))).goal is None  # still waiting for memory
    s.on_answer({"id": "other", "ok": True, "goal": [9, 9]})  # not ours: ignored
    s.on_answer({"id": q["id"], "ok": True, "goal": [4.0, 4.0]})
    out = s.step(ctx(1.0, (1.0, 1.0)))
    assert out.goal is not None and out.path["status"] == "executing"
    assert out.path["goal"][:2] == [4.0, 4.0]
    robot, now = tuple(out.goal[:2]), 1.0  # the robot flew to the first goal
    for _ in range(40):
        now += 0.5
        out = s.step(ctx(now, robot))
        if out.goal:
            robot = out.goal[:2]
        if out.finished:
            break
    assert out.finished and out.progress["state"] == "succeeded"
    assert math.dist(robot, (4.0, 4.0)) < 0.6


def test_goto_room_fails_cleanly_when_the_room_is_unknown_or_memory_silent():
    s = GotoRoomSkill(SkillConfig(), 0.0, "bathroom")
    q = s.step(ctx(0.0, (1.0, 1.0))).query
    s.on_answer({"id": q["id"], "ok": False, "reason": "no bathroom known yet"})
    out = s.step(ctx(0.5, (1.0, 1.0)))
    assert out.finished and out.progress["state"] == "failed"
    assert "bathroom" in out.progress["message"] and out.hold
    s2 = GotoRoomSkill(SkillConfig(query_timeout_s=2.0), 0.0, "kitchen")
    s2.step(ctx(0.0, (1.0, 1.0)))
    assert s2.step(ctx(1.0, (1.0, 1.0))).finished is False
    assert s2.step(ctx(2.5, (1.0, 1.0))).progress["state"] == "failed"


def test_goto_room_no_path_gives_up_after_the_timeout():
    s = GotoRoomSkill(SkillConfig(path_fail_s=3.0), 0.0, "kitchen")
    q = s.step(ctx(0.0, (1.0, 1.0))).query
    s.on_answer({"id": q["id"], "ok": True, "goal": [8.0, 3.0]})  # in unknown space
    last = None
    for k in range(1, 12):
        last = s.step(ctx(k * 0.5, (1.0, 1.0)))
        if last.finished:
            break
    assert last.finished and "no path" in last.progress["message"]


def test_snap_free_finds_nearest_unblocked_cell():
    from maverick_mapping.planner import PlannerConfig, PlanningGrid

    g, res, org = open_room_grid()
    pg = PlanningGrid.build(g, res, org, PlannerConfig())
    assert snap_free(pg, (3.0, 3.0), 0.5) is not None
    assert snap_free(pg, (0.6, 0.6), 0.1) is None  # in the wall


def test_explore_scans_first_with_yaw_feedback_then_plans():
    s = ExploreSkill(SkillConfig(), 0.0, budget_s=1e6)
    robot, yaw, now = (2.0, 3.0), 0.0, 0.0
    goals = []
    for _ in range(60):
        now += 0.5
        out = s.step(Ctx(now, robot, open_room_grid(), now, yaw))
        if out.goal is None:
            continue
        goals.append(out.goal)
        if s.state == "scan":
            assert out.goal[:2] == robot  # spins in place
            yaw = out.goal[2]  # the vehicle reaches the commanded yaw
        else:
            break
    spin = [g[2] for g in goals[:-1]]
    assert len(spin) == 6 and abs(spin[-1] - 2 * math.pi) < 1e-6  # full turn in 60 degree steps
    assert s.state == "fly" and goals[-1][:2] != robot  # then the first frontier goal


def test_explore_scan_step_times_out_without_yaw_control():
    s = ExploreSkill(SkillConfig(scan_step_timeout_s=1.0), 0.0, budget_s=1e6)
    now, st = 0.0, []
    for _ in range(40):
        now += 0.5
        s.step(Ctx(now, (2.0, 3.0), open_room_grid(), now, 0.0))  # yaw never changes
        st.append(s.state)
    assert "scan" in st and st[-1] != "scan"  # gave up on the scan, moved on to planning

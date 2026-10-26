"""Reactive clearance guard decisions, velocity estimate and the skills' reaction."""

import math

import numpy as np
from maverick_skills.guard import ClearanceGuard, GuardConfig, VelocityEstimate, bearing_of
from maverick_skills.skills import Ctx, ExploreSkill, SkillConfig, WaypointFollower

N = 24


def sectors(**at):
    """All sectors clear except {bearing_deg: distance}."""
    s = [0.8] * N
    for deg, d in at.items():
        b = math.radians(int(deg.lstrip("b").replace("m", "-")))
        k = min(
            range(N),
            key=lambda k: abs(
                math.atan2(math.sin(bearing_of(k, N) - b), math.cos(bearing_of(k, N) - b))
            ),
        )
        s[k] = d
    return s


g = ClearanceGuard(GuardConfig())


def test_clear_is_ok():
    assert g.evaluate([0.8] * N, 0.0).action == "ok"


def test_obstacle_ahead_slows_then_stops():
    assert g.evaluate(sectors(b0=0.45), 0.0, 0.3).action == "slow"
    assert g.evaluate(sectors(b0=0.45), 0.0, 0.3).speed_cap == g.cfg.slow_speed
    d = g.evaluate(sectors(b0=0.28), 0.0, 0.3)  # closing in at 0.3 m/s
    assert d.action == "stop"
    assert abs(d.bearing) < 0.2  # the obstacle's bearing, for the keepout


def test_stop_distance_grows_with_speed_and_needs_closing_in():
    assert g.evaluate(sectors(b0=0.5), 0.0, 0.3).action == "slow"
    assert g.evaluate(sectors(b0=0.5), 0.0, 0.8).action == "stop"  # 0.30 + 0.35 * 0.8 = 0.58
    # at rest beside an obstacle 0.25 m ahead the drone may move (slowly); only inside the hard
    # stop distance is it held whatever its speed
    assert g.evaluate(sectors(b0=0.25), 0.0, 0.0).action == "slow"
    assert g.evaluate(sectors(b0=0.19), 0.0, 0.0).action == "stop"


def test_obstacle_beside_does_not_stop():
    d = g.evaluate(sectors(b90=0.22), 0.0)  # a wall alongside the direction of travel
    assert d.action == "ok"
    assert d.d_fwd > 0.5


def test_obstacle_behind_does_not_stop_moving_away():
    assert g.evaluate(sectors(b180=0.15), 0.0).action == "nudge"  # too close: step away, forward
    # moving forward away from it is not a stop


def test_nudge_points_away_from_the_obstacle_and_is_capped():
    d = g.evaluate(sectors(b90=0.12), None)  # hovering, obstacle to the left (+y)
    assert d.action == "nudge"
    assert d.away[1] < -0.9
    assert 0 < d.step <= g.cfg.nudge_max_m


def test_nudge_between_two_walls_cancels():
    d = g.evaluate(sectors(b90=0.15, bm90=0.15), None)
    assert d.action in ("ok", "nudge")
    if d.action == "nudge":
        assert abs(d.away[1]) < 0.5 or d.step <= g.cfg.nudge_max_m


def test_no_heading_never_stops():
    assert g.evaluate(sectors(b0=0.28), None, 0.3).action == "ok"


def test_heading_from_velocity_else_goal():
    assert abs(g.heading((0.5, 0.0), (0.0, 1.0))) < 1e-9
    assert abs(g.heading((0.01, 0.0), (0.0, 1.0)) - math.pi / 2) < 1e-9
    assert g.heading(None, (0.05, 0.0)) is None
    assert g.heading(None, None) is None


def test_velocity_estimate():
    v = VelocityEstimate(0.6)
    assert v.velocity() is None
    for i in range(10):
        v.add(i * 0.1, (0.5 * i * 0.1, 0.0))
    vx, vy = v.velocity()
    assert abs(vx - 0.5) < 1e-6 and abs(vy) < 1e-6


def test_follower_resend_and_force_stuck():
    f = WaypointFollower(0.1, 12.0, 0.15)
    f.set([(1.0, 0.0)], 0.0)
    assert f.take_new_target() == (1.0, 0.0)
    assert f.take_new_target() is None
    f.resend()
    assert f.take_new_target() == (1.0, 0.0)
    f.force_stuck()
    assert f.update((0.0, 0.0), 1.0) == "stuck"
    assert f.update((0.0, 0.0), 1.1) == "moving"


def explore_ctx():
    grid = np.full((90, 70), -1, np.int8)
    grid[5:65, 5:65] = 0
    return Ctx(1.0, (3.0, 3.0), (grid, 0.1, (0.0, 0.0)), 0.5, robot_yaw=0.0)


def test_second_guard_stop_at_the_same_waypoint_is_a_stall():
    s = ExploreSkill(SkillConfig(), 0.0)
    s.follow.set([(4.0, 3.0)], 0.0)
    s.follow.take_new_target()
    ctx = explore_ctx()
    s.on_guard(ctx, {"d_fwd": 0.19})
    assert s.guard_n == 1
    assert s.follow.take_new_target() == (4.0, 3.0)  # sent again after the hold
    s.on_guard(ctx, {"d_fwd": 0.19})
    assert s.guard_n == 2
    assert s.follow.update((3.0, 3.0), 2.0) == "stuck"
    assert any(e["ev"] == "guard_stop" for e in s.events)
    s.follow.set([(1.0, 1.0)], 3.0)  # a different waypoint starts the count again
    s.on_guard(ctx, {"d_fwd": 0.19})
    assert s.guard_n == 1


def test_door_frame_passed_at_a_quarter_metre_does_not_stop():
    # a frame corner 0.25 m to the side and 0.36 m ahead (bearing 35 deg, 0.44 m away)
    d = g.evaluate(sectors(b45=0.44, bm45=0.44), 0.0)
    assert d.action in ("ok", "slow")
    # but an obstacle dead ahead at 0.28 m stops
    assert g.evaluate(sectors(b0=0.28), 0.0, 0.3).action == "stop"

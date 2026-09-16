import math
from argparse import Namespace

import pytest
from referee import s3
from referee.s3_frames import map_to_world, model_pose_env, spawn_origin, world_to_map
from referee.s3_verdicts import (
    c2_required_clearance,
    exit_result_warning,
    goto_metrics,
    hover_metrics,
    inscribed_radius,
    seed_set_of,
    summarise,
    verdict_explore,
    verdict_goto,
    verdict_hover,
)

MAN = {
    "spawn": {"pose": [3.0, 4.0, 0.0, 1.2]},
    "rooms": [
        {"type": "kitchen", "polygon": [[0, 0], [4, 0], [4, 3], [0, 3]]},
        {"type": "bathroom", "polygon": [[4, 0], [5.5, 0], [5.5, 1.5], [4, 1.5]]},
    ],
}


def test_frames_translation_only():
    o = spawn_origin(MAN, ground_z=0.05)
    assert world_to_map((3.0, 4.0, 0.05), o) == (0.0, 0.0, 0.0)
    # world +y is PX4 north: a drone 2 m north of the spawn is at map y = +2, whatever the yaw
    assert world_to_map((3.0, 6.0, 1.25), o) == pytest.approx((0.0, 2.0, 1.2))
    p = (1.5, -2.0, 0.7)
    assert map_to_world(world_to_map(p, o), o) == pytest.approx(p)


def test_model_pose_env():
    assert model_pose_env(MAN) == "3.0000,4.0000,0.0000,0,0,1.2000"


def test_polygon_helpers():
    sq = [(0, 0), (4, 0), (4, 3), (0, 3)]
    assert inscribed_radius(sq) == pytest.approx(1.5, abs=0.05)
    assert c2_required_clearance(1.5) == 1.0
    assert c2_required_clearance(0.8) == pytest.approx(0.75)


def test_goto_verdicts():
    o = spawn_origin(MAN)
    # kitchen in the map frame: x -3..1, y -4..-1; centre (-1, -2.5)
    m = goto_metrics(MAN, "kitchen", (-1.0, -2.5), o)
    assert verdict_goto(m).passed
    assert not verdict_goto(goto_metrics(MAN, "kitchen", (0.8, -2.5), o)).passed  # 0.2 m in
    assert not verdict_goto(goto_metrics(MAN, "kitchen", (5.0, 5.0), o)).passed  # outside
    # small bathroom: inscribed 0.75, so need 0.7
    b = goto_metrics(MAN, "bathroom", (4.75 - 3.0, 0.75 - 4.0), o)
    assert b["required_m"] == pytest.approx(0.7)
    assert verdict_goto(b).passed
    assert not verdict_goto(goto_metrics(MAN, "office", (0, 0), o)).passed
    assert not verdict_goto({**m, "skill_succeeded": False, "skill_state": "failed"}).passed


GOOD = {"coverage": 0.93, "collisions": 0, "surface_err_mean_m": 0.04, "c6_recall": 0.97}


def test_explore_verdict():
    assert verdict_explore(GOOD).passed
    assert not verdict_explore({**GOOD, "coverage": 0.85}).passed
    assert not verdict_explore({**GOOD, "collisions": 1}).passed
    assert not verdict_explore({**GOOD, "surface_err_mean_m": 0.12}).passed
    assert not verdict_explore({**GOOD, "surface_err_mean_m": math.nan}).passed
    v = verdict_explore({**GOOD, "c6_recall": 0.9, "coverage": 0.5})
    assert "C6" in v.reason and "coverage" in v.reason


def _hover_inputs(drift=0.05, landed=True, arm=1, z_end=0.02):
    flight = [
        {"t": 10.0, "event": "phase", "phase": "hover"},
        {"t": 20.0, "event": "phase", "phase": "land"},
    ]
    gt = [{"t": 10.0 + k * 0.1, "x": drift * (k % 2), "y": 0.0, "z": 1.2} for k in range(100)]
    gt.append({"t": 30.0, "x": 0.0, "y": 0.0, "z": z_end})
    px4 = [{"t": 29.0, "k": "status", "arm": arm}, {"t": 29.0, "k": "landed", "landed": landed}]
    return gt, flight, px4


def test_hover_verdict():
    assert verdict_hover(hover_metrics(*_hover_inputs(), ground_z=0.0)).passed
    assert not verdict_hover(hover_metrics(*_hover_inputs(drift=0.4), ground_z=0.0)).passed
    assert not verdict_hover(hover_metrics(*_hover_inputs(arm=2), ground_z=0.0)).passed
    assert not verdict_hover(hover_metrics(*_hover_inputs(landed=False), ground_z=0.0)).passed
    assert not verdict_hover(hover_metrics([], [], [], 0.0)).passed


def test_seed_sets_and_warning():
    assert seed_set_of(7) == "tuning" and seed_set_of(1_000_005) == "held_out"
    assert seed_set_of(500_000) == "other"
    assert exit_result_warning("explore", {"tuning"}) is not None
    assert exit_result_warning("explore", {"held_out"}) is None
    assert exit_result_warning("hover", {"tuning"}) is None
    rows = [{"verdict": "pass", "seed_set": "tuning"}, {"verdict": "error", "seed_set": "tuning"}]
    s = summarise(rows, "explore")
    assert s["exit_result"] is False and "warning" in s and s["error"] == 1
    assert summarise([{"verdict": "pass", "seed_set": "held_out"}], "explore")["exit_result"]


def test_seed_selection():
    assert s3.parse_seeds("1,3-5") == [1, 3, 4, 5]
    ns = Namespace(seeds=None, seed_set="held_out", runs=3)
    assert s3.pick_seeds(ns) == [1_000_000, 1_000_001, 1_000_002]
    ns = Namespace(seeds="7,8", seed_set=None, runs=3)
    assert s3.pick_seeds(ns) == [7, 8, 7]
    with pytest.raises(SystemExit):
        s3.pick_seeds(Namespace(seeds="7", seed_set="held_out", runs=None))

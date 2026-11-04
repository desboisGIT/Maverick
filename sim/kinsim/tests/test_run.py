"""End-to-end: determinism, result format, the agreement report can be regenerated."""

import json

import kinsim  # noqa: F401
import pytest
from kinsim import agreement
from kinsim.scenarios import run_explore
from kinsim.vehicle import KinParams

FAST = KinParams()
FAST.map_stride = 4


@pytest.fixture(scope="module")
def two_runs():
    a = run_explore(30, 0, FAST, budget=40.0)
    b = run_explore(30, 0, FAST, budget=40.0)
    return a, b


def test_deterministic_per_seed_and_run(two_runs):
    a, b = two_runs
    for k in ("duration_s", "pid"):
        a.pop(k, None)
        b.pop(k, None)
    for r in (a, b):
        r["kin"].pop("profile_s", None)
    assert json.dumps(a, sort_keys=True, default=str) == json.dumps(b, sort_keys=True, default=str)


def test_run_index_changes_the_noise():
    a = run_explore(30, 0, FAST, budget=25.0)
    b = run_explore(30, 1, FAST, budget=25.0)
    assert a["metrics"]["est_err_horiz_mean_m"] != b["metrics"]["est_err_horiz_mean_m"]


def test_row_format_is_the_referees(two_runs):
    r = two_runs[0]
    assert r["sim"] == "kinsim" and r["scenario"] == "explore" and r["seed_set"] == "tuning"
    assert r["verdict"] in ("pass", "fail")
    for k in ("coverage", "collisions", "c6_recall", "surface_err_mean_m", "explore_skill"):
        assert k in r["metrics"]


def test_agreement_report_can_be_regenerated(two_runs):
    """The calibration table follows from the committed Gazebo reference and fresh kinsim runs."""
    ref = agreement.load_reference()
    seeds = sorted({r["seed"] for r in ref})[:3]
    rows = [run_explore(s, 0, FAST, budget=60.0) for s in seeds]
    rep = agreement.report(ref, rows)
    assert set(rep) == {"fit", "validation", "all"}
    assert rep["all"]["n"] == 3
    for k in ("pass_fail_agreement", "coverage_mae", "duration_ratio_mean", "features"):
        assert k in rep["all"]
    assert "pass_fail_agreement" in agreement.format_table(rep)


def test_agreement_statistics_on_synthetic_rows():
    g = [
        {"seed": 20, "verdict": "pass", "coverage": 0.95, "collisions": 0, "c6_recall": 1.0,
         "skill_s": 100.0, "features": {}},
        {"seed": 21, "verdict": "fail", "coverage": 0.60, "collisions": 2, "c6_recall": 1.0,
         "skill_s": 50.0, "features": {}},
        {"seed": 22, "verdict": "pass", "coverage": 0.97, "collisions": 0, "c6_recall": 1.0,
         "skill_s": 80.0, "features": {}},
    ]  # fmt: skip
    k = [dict(r, skill_s=r["skill_s"] * 1.2) for r in g]
    k[2] = dict(k[2], verdict="fail", coverage=0.5)
    a = agreement.agreement(g, k)
    assert a["pass_fail_agreement"] == pytest.approx(2 / 3, abs=1e-3)
    assert a["duration_ratio_mean"] == pytest.approx(1.2, abs=1e-3)
    assert a["collision_run_agreement"] == 1.0


def test_goto_room_and_search_run_end_to_end():
    """Both memory-based scenarios produce a result row (short budgets: failures are fine)."""
    from kinsim.search import run_search

    g = run_explore(21, 0, FAST, budget=45.0, goto_room="kitchen")
    assert g["scenario"] == "goto_room" and "goto" in g and g["goto"]["room_type"] == "kitchen"
    s = run_search(21, 0, FAST, budget=45.0, obj="banana", search_budget=30.0)
    assert s["scenario"] == "search" and "banana" in s["search"]
    assert s["search"]["banana"]["verdict"] in ("pass", "fail")

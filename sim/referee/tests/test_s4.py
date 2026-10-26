from referee.s4_verdicts import (
    min_distance,
    summarise_s4,
    verdict_come_to,
    verdict_return,
    verdict_search,
)

MAN = {
    "small_objects": [{"id": "banana_0", "class": "banana", "position": [5.0, 4.0, 0.95]}],
    "presence": {"banana": True, "bread": False},
}
SPAWN = (2.0, 1.0)  # map frame = world - spawn


def found(x, y):
    return {
        "found": True,
        "position": [x - SPAWN[0], y - SPAWN[1], 0.95],
        "rooms_searched": ["kitchen"],
    }


def test_search_present_and_found_within_tolerance():
    v = verdict_search(MAN, "banana", found(5.1, 4.1), SPAWN)
    assert v.passed and v.metrics["position_error_m"] < 0.3


def test_search_present_but_reported_far_away_is_a_false_positive():
    v = verdict_search(MAN, "banana", found(6.0, 4.0), SPAWN)
    assert not v.passed and v.metrics["false_positive"]


def test_search_present_but_missed():
    v = verdict_search(MAN, "banana", {"found": False, "rooms_searched": ["kitchen"]}, SPAWN)
    assert not v.passed and "missed" in v.reason


def test_search_absent_is_correct_only_when_not_found():
    assert verdict_search(MAN, "bread", {"found": False, "rooms_searched": []}, SPAWN).passed
    v = verdict_search(MAN, "bread", found(5.0, 4.0), SPAWN)
    assert not v.passed and v.metrics["false_positive"]


def test_search_without_a_result_fails():
    assert not verdict_search(MAN, "banana", None, SPAWN).passed


def test_come_to_window():
    p = (4.0, 4.0)
    assert verdict_come_to(p, (4.0, 5.4), 1.3, "succeeded").passed
    assert not verdict_come_to(p, (4.0, 5.4), 0.8, "succeeded").passed  # got too close on the way
    assert not verdict_come_to(p, (4.0, 4.7), 1.2, "succeeded").passed  # ended too close
    assert not verdict_come_to(p, (4.0, 6.2), 2.0, "succeeded").passed  # ended too far
    assert not verdict_come_to(p, (4.0, 5.4), 1.3, "failed").passed


def test_return_and_min_distance():
    assert verdict_return((0, 0), (0.2, 0.1), "succeeded").passed
    assert not verdict_return((0, 0), (1.0, 0.1), "succeeded").passed
    track = [{"t": 1, "x": 0, "y": 0}, {"t": 2, "x": 3, "y": 0}, {"t": 9, "x": 3, "y": 4}]
    assert min_distance(track, (3, 2), 0, 5) == 2.0 and min_distance(track, (0, 0), 20, 30) is None


def test_summary_counts():
    ok = {"verdict": "pass"}
    rows = [
        {
            "verdict": "pass",
            "search": {"banana": {**ok, "present": True}, "bread": {**ok, "present": False}},
            "come_to": ok,
            "return": ok,
        },
        {
            "verdict": "fail",
            "search": {
                "banana": {"verdict": "fail", "present": True, "false_positive": True},
                "bread": {**ok, "present": True},
            },
        },
    ]
    s = summarise_s4(rows)
    assert (s["search_total"], s["search_correct"]) == (4, 3)
    assert (s["search_present_total"], s["search_present_correct"]) == (3, 2)
    assert (s["search_absent_total"], s["search_absent_correct"]) == (1, 1)
    assert (
        s["false_positives"] == 1 and s["come_to_pass"] == 1 and s["seeds_search_all_correct"] == 1
    )

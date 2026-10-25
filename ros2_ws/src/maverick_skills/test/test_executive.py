"""The executive's validation: every unsafe or invalid request is rejected with a reason."""

import pytest
from maverick_skills.executive import Executive, WorldView

KNOWN = WorldView(
    room_types=frozenset({"kitchen", "living"}),
    people=[(3.0, 2.0)],
    robot_xy=(0.0, 0.0),
    home_xy=(0.0, 0.0),
)
EMPTY = WorldView(robot_xy=(0.0, 0.0))

# 12+ hand-written unsafe or invalid calls: (name, request, view, expected code)
UNSAFE = [
    ("not an object", "explore", KNOWN, "malformed"),
    ("a list", ["explore"], KNOWN, "malformed"),
    ("no skill", {"budget_s": 60}, KNOWN, "unknown_skill"),
    ("unknown skill", {"skill": "set_position", "x": 1, "y": 2}, KNOWN, "unknown_skill"),
    (
        "setpoint smuggled in",
        {"skill": "goto_room", "room_type": "kitchen", "x": 1.0},
        KNOWN,
        "malformed",
    ),
    ("negative budget", {"skill": "explore", "budget_s": -5}, KNOWN, "bad_argument"),
    ("huge budget", {"skill": "explore", "budget_s": 1e9}, KNOWN, "bad_argument"),
    ("nan budget", {"skill": "explore", "budget_s": float("nan")}, KNOWN, "bad_argument"),
    ("string budget", {"skill": "explore", "budget_s": "60"}, KNOWN, "bad_argument"),
    (
        "bool budget",
        {"skill": "search", "object": "banana", "budget_s": True},
        KNOWN,
        "bad_argument",
    ),
    (
        "room not in the vocabulary",
        {"skill": "goto_room", "room_type": "garage"},
        KNOWN,
        "bad_argument",
    ),
    ("room not known yet", {"skill": "goto_room", "room_type": "bedroom"}, KNOWN, "unknown_room"),
    ("no room known at all", {"skill": "goto_room", "room_type": "kitchen"}, EMPTY, "unknown_room"),
    (
        "search in a room not known",
        {"skill": "search", "object": "banana", "region": "office", "budget_s": 60},
        KNOWN,
        "unknown_room",
    ),
    (
        "search nothing known",
        {"skill": "search", "object": "banana", "budget_s": 60},
        EMPTY,
        "unknown_room",
    ),
    (
        "search object with a path",
        {"skill": "search", "object": "../../etc", "budget_s": 60},
        KNOWN,
        "bad_argument",
    ),
    (
        "search empty object",
        {"skill": "search", "object": "  ", "budget_s": 60},
        KNOWN,
        "bad_argument",
    ),
    (
        "come to a person nobody has seen",
        {"skill": "come_to", "person": "operator"},
        EMPTY,
        "unknown_person",
    ),
    (
        "come to within 0.5 m",
        {"skill": "come_to", "person": "operator", "standoff_m": 0.5},
        KNOWN,
        "unsafe",
    ),
    (
        "come to within 0.99 m",
        {"skill": "come_to", "person": "operator", "standoff_m": 0.99},
        KNOWN,
        "unsafe",
    ),
    ("come to without a name", {"skill": "come_to"}, KNOWN, "bad_argument"),
    ("return with an unknown take-off point", {"skill": "return"}, EMPTY, "no_home"),
    ("return, land is a string", {"skill": "return", "land": "yes"}, KNOWN, "bad_argument"),
    (
        "bad request id",
        {"skill": "explore", "budget_s": 60, "id": "a b;rm -rf"},
        KNOWN,
        "malformed",
    ),
]


@pytest.mark.parametrize("name,req,view,code", UNSAFE, ids=[u[0] for u in UNSAFE])
def test_unsafe_or_invalid_calls_are_rejected(name, req, view, code):
    d = Executive().submit(req, view)
    assert not d.accepted, name
    assert d.code == code, (name, d.code, d.reason)
    assert d.reason and d.request_id
    p = d.progress()
    assert p["state"] == "failed" and p["rejected"] is True and p["request_id"] == d.request_id


def test_there_are_at_least_twelve_hand_written_unsafe_calls():
    assert len(UNSAFE) >= 12


def test_valid_requests_are_accepted_and_normalised():
    ex = Executive()
    d = ex.submit({"skill": "explore", "budget_s": 120, "id": "mine-1"}, KNOWN)
    assert (
        d.accepted
        and d.request_id == "mine-1"
        and d.request == {"skill": "explore", "budget_s": 120.0}
    )
    ex.finished()
    d = ex.submit({"skill": "goto_room", "room_type": " Kitchen "}, KNOWN)
    assert d.accepted and d.request["room_type"] == "kitchen"
    ex.finished()
    d = ex.submit(
        {"skill": "search", "object": "Banana", "region": "kitchen", "budget_s": 90}, KNOWN
    )
    assert d.accepted and d.request == {
        "skill": "search", "object": "banana", "region": "kitchen", "budget_s": 90.0
    }  # fmt: skip
    ex.finished()
    d = ex.submit({"skill": "search", "object": "bread"}, KNOWN)  # budget defaults
    assert d.accepted and d.request["budget_s"] == ex.limits.default_search_budget_s
    ex.finished()
    d = ex.submit({"skill": "come_to", "person": "operator"}, KNOWN)
    assert d.accepted and d.request["standoff_m"] == 1.5
    ex.finished()
    d = ex.submit({"skill": "come_to", "person": "operator", "standoff_m": 1.0}, KNOWN)
    assert d.accepted  # exactly 1.0 m is the limit
    ex.finished()
    d = ex.submit({"skill": "return", "land": True}, KNOWN)
    assert d.accepted and d.request == {"skill": "return", "land": True}


def test_request_ids_are_unique_and_echoed():
    ex = Executive()
    a = ex.submit({"skill": "explore", "budget_s": 60}, KNOWN)
    ex.finished()
    b = ex.submit({"skill": "explore", "budget_s": 60}, KNOWN)
    assert a.request_id != b.request_id
    assert ex.submit({"skill": "cancel", "id": "stop-1"}, KNOWN).request_id == "stop-1"


def test_one_motion_skill_at_a_time_and_cancel_always_works():
    ex = Executive()
    d = ex.submit({"skill": "explore", "budget_s": 60}, KNOWN)
    ex.started(d)
    busy = ex.submit({"skill": "goto_room", "room_type": "kitchen"}, KNOWN)
    assert not busy.accepted and busy.code == "busy" and "explore" in busy.reason
    c = ex.submit({"skill": "cancel"}, KNOWN)
    assert c.accepted  # cancel is never refused
    ex.finished()
    assert ex.submit({"skill": "goto_room", "room_type": "kitchen"}, KNOWN).accepted


def test_goal_guard_keeps_one_metre_from_people():
    ex = Executive()
    assert ex.goal_violation((3.0, 2.9), KNOWN) is not None  # 0.9 m from the person at (3, 2)
    assert ex.goal_violation((3.0, 2.0), KNOWN) is not None
    assert ex.goal_violation((3.0, 3.05), KNOWN) is None  # 1.05 m
    assert ex.goal_violation((3.0, 2.5), EMPTY) is None  # nobody known

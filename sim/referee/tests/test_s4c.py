from referee.s4c_verdicts import (
    answer_polarity,
    photo_shows_banana,
    spoken_answers,
    summarise_h2,
    verdict_h2,
)

SPAWN = (2.0, 1.0)
PRESENT = {"small_objects": [{"class": "banana", "position": [5.0, 4.0, 0.9]}]}
ABSENT = {"small_objects": [{"class": "bread", "position": [5.0, 4.0, 0.9]}]}
OBJECTS = [{"id": "obj_1", "label": "banana", "position": [3.05, 3.0, 0.9]}]  # map = world - spawn


def ev(t, role, text, att=None):
    d = {"id": f"m{t}", "role": role, "text": text}
    if att:
        d["attachments"] = att
    return {"type": "transcript", "t": t, "data": d}


PHOTO = [{"kind": "photo", "object_id": "obj_1", "label": "banana"}]
END = {"ended": "end_turn", "calls": 4}


def good_events():
    return [
        ev(100, "user", "go check"),
        ev(101, "maverick", "On it."),
        ev(150, "maverick", "Yes, there are bananas in the kitchen."),
        ev(151, "maverick", "found it", PHOTO),
    ]


def run(man=PRESENT, events=None, objects=OBJECTS, end=1.3, mn=1.2, coll=0, t_end=170.0):
    return verdict_h2(
        man, events or good_events(), 100.0, objects, SPAWN, end, mn, coll, END, t_end
    )


def test_answer_polarity():
    yes = [
        "Yes, there are bananas.",
        "I found some bananas on the counter.",
        "yeah there are three",
    ]
    no = ["No, there are no bananas.", "I did not find any bananas.", "There aren't any.",
          "Sorry, nothing in the kitchen."]  # fmt: skip
    assert all(answer_polarity(t) == "yes" for t in yes)
    assert all(answer_polarity(t) == "no" for t in no)
    assert answer_polarity("I checked the kitchen.") is None and answer_polarity("") is None


def test_spoken_answers_skip_ack_and_photo_captions_and_old_events():
    texts = spoken_answers([ev(50, "maverick", "old"), *good_events()], 100.0)
    assert texts == ["Yes, there are bananas in the kitchen."]


def test_pass_when_present():
    v = run()
    assert v.passed and all(v.checks.values()), v.reason


def test_pass_when_absent_no_photo():
    ev_abs = [
        ev(100, "user", "go"),
        ev(101, "maverick", "On it."),
        ev(150, "maverick", "No, none."),
    ]
    assert run(ABSENT, ev_abs).passed


def test_absent_with_photo_fails_as_perception():
    ev_abs = [ev(150, "maverick", "No, none."), ev(151, "maverick", "x", PHOTO)]
    steps = {"type": "agent_step", "t": 152, "data": {"steps": [
        {"id": "a", "label": "Search for banana", "status": "done",
         "detail": "found the banana in the kitchen (object_id obj_1)"}]}}  # fmt: skip
    v = run(ABSENT, [*ev_abs, steps])
    assert not v.passed and not v.checks["photo"] and "perception" in v.layers


def test_wrong_answer_with_correct_skills_is_an_agent_failure():
    events = good_events()
    events[2] = ev(150, "maverick", "No, there are none.")
    steps = {"type": "agent_step", "t": 152, "data": {"steps": [
        {"id": "a", "label": "Search for banana", "status": "done",
         "detail": "found the banana in the kitchen (object_id obj_1)"}]}}  # fmt: skip
    v = run(events=[*events, steps])
    assert not v.checks["answer"] and v.layers == ["agent decision"]


def test_miss_is_perception_and_executive_rejection_is_attributed():
    miss = {"type": "agent_step", "t": 152, "data": {"steps": [
        {"id": "a", "label": "Search for banana", "status": "done",
         "detail": "not found, searched kitchen"}]}}  # fmt: skip
    events = good_events()
    events[2] = ev(150, "maverick", "No, there are none.")
    v = run(events=[*events[:3], miss])
    assert not v.passed and "perception" in v.layers
    rej = {"type": "agent_step", "t": 152, "data": {"steps": [
        {"id": "a", "label": "Search for banana", "status": "failed",
         "detail": "failed: rejected (unknown_room): no kitchen known"}]}}  # fmt: skip
    v = run(events=[*events[:3], rej])
    assert v.layers[0] == "executive rejection"


def test_distance_collision_and_time_checks():
    assert not run(end=1.8).checks["distance"]
    assert not run(mn=0.8).checks["distance"]
    v = run(coll=2)
    assert not v.checks["collisions"] and "flight/collision" in v.layers
    assert not run(t_end=100.0 + 241).checks["time"]
    assert not verdict_h2(
        PRESENT, good_events(), 100.0, OBJECTS, SPAWN, 1.3, 1.2, 0, None, None
    ).passed


def test_photo_must_be_a_banana_near_the_real_one():
    ok, _ = photo_shows_banana(PHOTO[0], {"obj_1": OBJECTS[0]}, PRESENT["small_objects"][0], SPAWN)
    assert ok
    bread = {"obj_1": {"label": "bread", "position": [3.05, 3.0, 1.0]}}
    assert not photo_shows_banana(PHOTO[0], bread, PRESENT["small_objects"][0], SPAWN)[0]
    far = {"obj_1": {"label": "banana", "position": [4.0, 3.0, 1.0]}}
    assert not photo_shows_banana(PHOTO[0], far, PRESENT["small_objects"][0], SPAWN)[0]
    assert not photo_shows_banana({"object_id": "zzz"}, {}, PRESENT["small_objects"][0], SPAWN)[0]


def test_summary_counts_layers_and_cost():
    rows = [
        {"seed": 1, "verdict": "pass", "layers": [], "cost_eur": 0.01, "model_calls": 4,
         "model_latencies_s": [1.0, 2.0], "mission_s": 100},
        {"seed": 2, "verdict": "fail", "layers": ["perception"], "cost_eur": 0.02,
         "model_calls": 3, "model_latencies_s": [1.5], "mission_s": 120},
        {"seed": 3, "verdict": "skipped", "layers": []},
        {"seed": 4, "verdict": "error", "layers": []},
    ]  # fmt: skip
    s = summarise_h2(rows)
    assert s["scored"] == 2 and s["passed"] == 1 and s["skipped"] == [3] and s["errors"] == [4]
    assert s["failures_by_layer"] == {"perception": 1} and s["cost_eur"] == 0.03
    assert s["model_calls"] == 7

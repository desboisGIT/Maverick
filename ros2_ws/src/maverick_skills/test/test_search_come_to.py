"""search, come_to and return on a synthetic 2D map: the robot teleports to each goal."""

import math

import numpy as np
from maverick_skills.come_to import ComeToSkill, ReturnSkill, standoff_points
from maverick_skills.extra_skills import make_extra_skill
from maverick_skills.search import (
    SearchSkill,
    order_nearest,
    order_rooms,
    surface_viewpoints,
)
from maverick_skills.skills import Ctx, SkillConfig, make_skill

RES = 0.1


def room_grid():
    """8 x 8 m free floor with walls all round (grid[ix, iy], origin 0, 0)."""
    g = np.full((90, 90), -1, np.int8)
    g[5:85, 5:85] = 0
    g[5:85, 5:7] = 100
    g[5:85, 83:85] = 100
    g[5:7, 5:85] = 100
    g[83:85, 5:85] = 100
    return g


ROOM = {
    "id": "room_0",
    "type": "kitchen",
    "goal": [2.0, 4.0],
    "surfaces": [
        {"id": "obj_1", "label": "kitchen_counter", "lo": [5.0, 3.0, 0.0], "hi": [5.6, 5.0, 0.9]}
    ],
    "known": False,
}


class World:
    """Drives a skill: the robot reaches each goal at once, memory answers, the detector sees."""

    def __init__(self, items=(), view_range=2.6, view_half_angle=0.6, fp=None, start=(1.0, 1.0)):
        self.xy, self.yaw, self.t = start, 0.0, 0.0
        self.items = list(items)  # (label, x, y, z)
        self.range, self.half = view_range, view_half_angle
        self.fp = fp or []  # extra frames (dicts) injected at given step numbers
        self.goals: list = []
        self.grid = room_grid()
        self.people: list = []

    def frame(self):
        boxes = []
        for label, x, y, z in self.items:
            d = math.dist(self.xy, (x, y))
            ang = abs(
                math.atan2(
                    math.sin(math.atan2(y - self.xy[1], x - self.xy[0]) - self.yaw),
                    math.cos(math.atan2(y - self.xy[1], x - self.xy[0]) - self.yaw),
                )
            )
            if d <= self.range and ang <= self.half:
                boxes.append(
                    {
                        "label": label,
                        "confidence": 0.8,
                        "box": [0.4, 0.4, 0.1, 0.1],
                        "position": [x, y, z],
                    }
                )
        return {"stamp": self.t, "boxes": boxes}

    def answer(self, q):
        if q["query"] == "search_plan":
            return {"id": q["id"], "ok": True, "rooms": [ROOM]}
        if q["query"] == "find" and q.get("label") == "person":
            return {"id": q["id"], "ok": True, "objects": self.people}
        if q["query"] == "find":
            objs = [
                {"id": "obj_9", "position": [x, y, z], "confidence": 0.9, "photo": "/tmp/x.jpg"}
                for label, x, y, z in self.items
                if label == q["label"]
            ]
            return {"id": q["id"], "ok": bool(objs), "objects": objs}
        return {"id": q["id"], "ok": False}

    def run(self, skill, max_steps=600, dt=0.5):
        last_out = None
        for k in range(max_steps):
            self.t += dt
            ctx = Ctx(
                self.t,
                self.xy,
                (self.grid, RES, (0.0, 0.0)),
                self.t,
                robot_yaw=self.yaw,
                frames=[self.frame()] + [f for s, f in self.fp if s == k],
                people=self.people_xy(),
            )
            out = skill.step(ctx)
            last_out = out
            if out.query:
                skill.on_answer(self.answer(out.query))
            if out.goal:
                self.goals.append(out.goal)
                self.xy, self.yaw = (out.goal[0], out.goal[1]), out.goal[2]
            if skill.finished:
                return last_out
        raise AssertionError("did not finish: " + str(last_out.progress))

    def people_xy(self):
        return [tuple(p["position"][:2]) for p in self.people]


def result(out):
    return out.progress["result"]


def test_search_finds_a_banana_seen_from_the_sweep():
    w = World(items=[("banana", 4.0, 4.0, 0.95)])
    s = SearchSkill(SkillConfig(), 0.0, "banana", "kitchen", 120)
    out = w.run(s)
    assert s.result == "succeeded" and result(out)["found"] is True
    x, y, _ = result(out)["position"]
    assert math.hypot(x - 4.0, y - 4.0) < 0.05
    assert result(out)["photo"] == "/tmp/x.jpg" and result(out)["object_id"] == "obj_9"
    assert "found the banana" in out.progress["message"]


def test_search_visits_a_surface_viewpoint_when_the_sweep_is_too_far():
    # 3.4 m from the room goal (out of range): only a viewpoint at the counter can see it
    w = World(items=[("banana", 5.3, 4.0, 0.95)], view_range=1.9)
    s = SearchSkill(SkillConfig(), 0.0, "banana", "kitchen", 300)
    out = w.run(s)
    assert result(out)["found"] is True
    assert any(e["ev"] == "views" and e["n"] >= 1 for e in s.events)


def test_search_reports_not_found_and_the_rooms_searched():
    w = World(items=[("bread", 5.3, 4.0, 0.95)])  # wrong object
    s = SearchSkill(SkillConfig(), 0.0, "banana", "kitchen", 300)
    out = w.run(s)
    assert s.result == "succeeded" and result(out)["found"] is False
    assert out.progress["message"] == "not found, searched kitchen"
    assert result(out)["rooms_searched"] == ["kitchen"]


def test_search_stops_when_the_budget_runs_out():
    w = World(items=[])
    s = SearchSkill(SkillConfig(), 0.0, "banana", "kitchen", 10)
    out = w.run(s)
    assert result(out)["found"] is False and result(out)["reason"] == "time budget used up"


def test_one_sighting_or_disagreeing_sightings_are_not_a_find():
    base = {"label": "banana", "confidence": 0.9, "box": [0, 0, 0.1, 0.1]}
    one = [(3, {"boxes": [{**base, "position": [4.0, 4.0, 0.9]}]})]
    far = [
        (3, {"boxes": [{**base, "position": [4.0, 4.0, 0.9]}]}),
        (5, {"boxes": [{**base, "position": [1.0, 7.0, 0.9]}]}),
    ]
    weak = [
        (3, {"boxes": [{**base, "confidence": 0.4, "position": [4.0, 4.0, 0.9]}]}),
        (5, {"boxes": [{**base, "confidence": 0.4, "position": [4.0, 4.0, 0.9]}]}),
    ]
    for fps in (one, far, weak):
        w = World(items=[], fp=fps)
        out = w.run(SearchSkill(SkillConfig(), 0.0, "banana", "kitchen", 60))
        assert result(out)["found"] is False, fps


def test_two_sightings_in_different_frames_are_a_find():
    base = {"label": "banana", "confidence": 0.7, "box": [0, 0, 0.1, 0.1]}
    fps = [
        (2, {"boxes": [{**base, "position": [4.0, 4.0, 0.9]}]}),
        (4, {"boxes": [{**base, "position": [4.1, 4.05, 0.9]}]}),
    ]
    out = World(items=[], fp=fps).run(SearchSkill(SkillConfig(), 0.0, "banana", None, 60))
    assert result(out)["found"] is True


def test_search_fails_when_memory_knows_no_room():
    class Empty(World):
        def answer(self, q):
            return {"id": q["id"], "ok": False, "reason": "no room known yet"}

    out = Empty().run(SearchSkill(SkillConfig(), 0.0, "banana", None, 60))
    assert out.progress["state"] == "failed" and "cannot search" in out.progress["message"]


def test_viewpoints_face_the_surface_and_skip_blocked_sides():
    free = lambda p: p[0] < 4.0  # only the west side is free  # noqa: E731
    pts = surface_viewpoints((5.0, 3.0), (5.6, 5.0), free, 1.3)
    assert pts and all(p[0] < 4.0 for p, _ in pts)
    (px, py), yaw = pts[0]
    assert abs(math.sin(yaw)) < 0.5 and math.cos(yaw) > 0  # looks east, at the counter
    # a 2 m long counter side gets two viewpoints, a short one a single viewpoint
    assert len(surface_viewpoints((5.0, 3.0), (5.6, 5.0), lambda p: True, 1.3)) == 1 + 1 + 2 + 2


def test_viewpoint_order_drops_points_near_people():
    pts = [((1.0, 1.0), 0.0), ((3.0, 3.0), 0.0), ((2.0, 2.0), 0.0)]
    tour = order_nearest(pts, (0.0, 0.0), avoid=[(3.0, 3.2)], min_dist=1.2)
    assert [p for p, _ in tour] == [(1.0, 1.0), (2.0, 2.0)]


def test_rooms_are_ordered_by_evidence_then_prior_then_distance():
    rooms = [
        {"id": "a", "type": "bedroom", "goal": [1, 1], "known": False},
        {"id": "b", "type": "kitchen", "goal": [9, 9], "known": False},
        {"id": "c", "type": "living", "goal": [2, 2], "known": False},
        {"id": "d", "type": "office", "goal": [8, 8], "known": True},
    ]
    ids = [r["id"] for r in order_rooms(rooms, "banana", (0.0, 0.0), None)]
    assert ids == ["d", "b", "c", "a"]


# -- come_to / return ----------------------------------------------------------------------------
PERSON = {"id": "obj_2", "position": [6.0, 6.0, 0.9], "confidence": 0.9, "last_seen": 3.0}


def test_come_to_stops_at_the_standoff_and_faces_the_person():
    w = World(start=(1.5, 1.5))
    w.people = [PERSON]
    s = ComeToSkill(SkillConfig(), 0.0, "operator", 1.5)
    out = w.run(s)
    assert s.result == "succeeded"
    d = math.dist(w.xy, (6.0, 6.0))
    assert 1.3 <= d <= 1.8  # the standoff, less the arrival tolerance
    want = math.atan2(6.0 - w.xy[1], 6.0 - w.xy[0])
    assert abs(math.atan2(math.sin(w.yaw - want), math.cos(w.yaw - want))) < 0.05
    assert result(out)["distance_m"] >= 1.0


def test_come_to_never_plans_a_point_closer_than_one_metre():
    pts = standoff_points((5.0, 5.0), (1.0, 1.0), lambda p: True, 0.5, min_dist=1.0)
    assert pts and all(math.dist(p, (5.0, 5.0)) >= 1.0 - 1e-9 for p in pts)
    near = standoff_points((5.0, 5.0), (1.0, 1.0), lambda p: True, 1.5, 1.0)
    assert math.dist(near[0], (1.0, 1.0)) < math.dist(near[-1], (1.0, 1.0))  # robot side first


def test_come_to_fails_when_the_person_is_unknown():
    w = World()
    out = w.run(ComeToSkill(SkillConfig(), 0.0, "operator"))
    assert out.progress["state"] == "failed" and "person" in out.progress["message"]


def test_return_flies_home_and_optionally_lands():
    w = World(start=(6.0, 6.0))
    out = w.run(ReturnSkill(SkillConfig(), 0.0, (1.0, 1.0), land=True))
    assert math.dist(w.xy, (1.0, 1.0)) < 0.5 and out.mission == "land"
    w2 = World(start=(6.0, 6.0))
    out2 = w2.run(ReturnSkill(SkillConfig(), 0.0, (1.0, 1.0)))
    assert out2.mission is None


def test_make_skill_builds_the_new_skills():
    cfg = SkillConfig()
    assert isinstance(make_skill({"skill": "search", "object": "banana"}, cfg, 0.0), SearchSkill)
    assert isinstance(make_skill({"skill": "come_to", "person": "me"}, cfg, 0.0), ComeToSkill)
    r = make_skill({"skill": "return", "home_xy": [1, 2]}, cfg, 0.0)
    assert isinstance(r, ReturnSkill) and r.home == (1.0, 2.0)
    for bad in ({"skill": "return"}, {"skill": "search"}, {"skill": "come_to"}):
        try:
            make_extra_skill(bad, cfg, 0.0)
        except ValueError:
            continue
        raise AssertionError(bad)

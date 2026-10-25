"""The executive's pure core (no ROS imports): validates every skill request.

The skill node hands each request here before anything flies. A request is accepted only when it
is well formed, within budget limits, refers to a room / person the drone knows, no other motion
skill is running, and it cannot bring the drone closer than `min_person_m` to a person. The
decision carries a request id; the node echoes it in every progress message of that request, so
the agent can match results to its calls.

  accepted:  Decision(accepted=True,  request=<normalised dict>, request_id="ex-3")
  rejected:  Decision(accepted=False, code="busy", reason="explore is running; cancel it first")

Request grammar (JSON on /maverick/skill/request; unknown keys are rejected):
  {"skill": "explore", "budget_s": 10..900}
  {"skill": "goto_room", "room_type": "kitchen"}                      room must be known
  {"skill": "search", "object": "banana", "region"?: "kitchen", "budget_s"?: 10..900}
  {"skill": "come_to", "person": "operator", "standoff_m"?: 1.0..6.0} a person must be known
  {"skill": "return", "land"?: true}
  {"skill": "cancel"}
Every request may carry "id" (string, at most 40 characters of [A-Za-z0-9_.:-]).
"""

from __future__ import annotations

import itertools
import math
import re
from dataclasses import dataclass, field

XY = tuple[float, float]

MOTION_SKILLS = ("explore", "goto_room", "search", "come_to", "return")
ROOM_TYPES = ("living", "kitchen", "bedroom", "bathroom", "hallway", "office")
_ID_RE = re.compile(r"^[A-Za-z0-9_.:-]{1,40}$")
_NAME_RE = re.compile(r"^[a-z][a-z0-9 _'-]{0,63}$")
PROGRESS_NAME = {"goto_room": "goto"}  # the name progress messages use for a skill
ALLOWED_KEYS = {
    "explore": {"skill", "id", "budget_s"},
    "goto_room": {"skill", "id", "room_type"},
    "search": {"skill", "id", "object", "region", "budget_s"},
    "come_to": {"skill", "id", "person", "standoff_m"},
    "return": {"skill", "id", "land"},
    "cancel": {"skill", "id"},
}


@dataclass(frozen=True)
class Limits:
    min_budget_s: float = 10.0
    max_budget_s: float = 900.0
    default_search_budget_s: float = 120.0
    default_explore_budget_s: float = 360.0
    min_person_m: float = 1.0  # never closer than this to a person (hard)
    default_standoff_m: float = 1.5
    max_standoff_m: float = 6.0


@dataclass
class WorldView:
    """What the executive knows when a request arrives (the node fills it from memory)."""

    room_types: frozenset[str] = frozenset()
    people: list[XY] = field(default_factory=list)  # last known positions
    robot_xy: XY | None = None
    home_xy: XY | None = None  # take-off point


@dataclass(frozen=True)
class Decision:
    accepted: bool
    request: dict
    request_id: str
    code: str = "ok"  # ok | malformed | unknown_skill | bad_argument | unknown_room |
    #                    unknown_person | busy | unsafe | no_home
    reason: str = ""

    def progress(self) -> dict:
        """Progress/result message of a rejected request (the node publishes it)."""
        skill = self.request.get("skill") if isinstance(self.request, dict) else None
        return {
            "skill": PROGRESS_NAME.get(skill, skill if isinstance(skill, str) else "?")[:40],
            "state": "failed",
            "progress": -1,
            "message": f"rejected ({self.code}): {self.reason}",
            "request_id": self.request_id,
            "rejected": True,
            "code": self.code,
        }


class Executive:
    def __init__(self, limits: Limits | None = None) -> None:
        self.limits = limits or Limits()
        self._ids = itertools.count(1)
        self.active_id: str | None = None
        self.active_skill: str | None = None

    # -- bookkeeping -------------------------------------------------------------------------
    def started(self, d: Decision) -> None:
        if d.request.get("skill") in MOTION_SKILLS:
            self.active_id, self.active_skill = d.request_id, d.request["skill"]

    def finished(self) -> None:
        self.active_id = self.active_skill = None

    # -- validation --------------------------------------------------------------------------
    def submit(self, req: object, view: WorldView) -> Decision:
        rid = self._request_id(req)
        if not isinstance(req, dict):
            return self._no("malformed", "a request must be a JSON object", {}, rid)
        kind = req.get("skill")
        if not isinstance(kind, str) or kind not in ALLOWED_KEYS:
            return self._no("unknown_skill", f"unknown skill {str(kind)[:40]!r}", req, rid)
        extra = sorted(set(req) - ALLOWED_KEYS[kind])
        if extra:
            return self._no(
                "malformed", f"unexpected key(s): {', '.join(map(str, extra))}", req, rid
            )
        if "id" in req and not (isinstance(req["id"], str) and _ID_RE.match(req["id"])):
            return self._no(
                "malformed", "id must be 1-40 characters of A-Z a-z 0-9 _ . : -", req, rid
            )
        if kind == "cancel":
            return Decision(True, {"skill": "cancel"}, rid)
        if self.active_skill is not None:
            return self._no("busy", f"{self.active_skill} is running; cancel it first", req, rid)
        check = getattr(self, f"_check_{kind}")
        out = check(req, view)
        if isinstance(out, str):
            code, _, reason = out.partition("|")
            return self._no(code, reason, req, rid)
        return Decision(True, {**out, "skill": kind}, rid)

    def _request_id(self, req: object) -> str:
        if isinstance(req, dict) and isinstance(req.get("id"), str) and _ID_RE.match(req["id"]):
            return req["id"]
        return f"ex-{next(self._ids)}"

    @staticmethod
    def _no(code: str, reason: str, req: object, rid: str) -> Decision:
        return Decision(False, req if isinstance(req, dict) else {}, rid, code, reason)

    def _budget(self, req: dict, default: float | None) -> float | str:
        lim = self.limits
        b = req.get("budget_s", default)
        if b is None:
            return "bad_argument|budget_s is required"
        if isinstance(b, bool) or not isinstance(b, (int, float)) or not math.isfinite(b):
            return "bad_argument|budget_s must be a finite number of seconds"
        if not lim.min_budget_s <= b <= lim.max_budget_s:
            lo, hi = lim.min_budget_s, lim.max_budget_s
            return f"bad_argument|budget_s must be between {lo:g} and {hi:g}"
        return float(b)

    def _room(self, name: object, view: WorldView, label: str) -> str:
        if not isinstance(name, str) or name.strip().lower() not in ROOM_TYPES:
            return f"bad_argument|{label} must be one of {', '.join(ROOM_TYPES)}"
        rt = name.strip().lower()
        if rt not in view.room_types:
            return f"unknown_room|I do not know where the {rt} is yet; explore first"
        return rt

    def _check_explore(self, req: dict, view: WorldView):
        b = self._budget(req, self.limits.default_explore_budget_s)
        return b if isinstance(b, str) else {"budget_s": b}

    def _check_goto_room(self, req: dict, view: WorldView):
        rt = self._room(req.get("room_type"), view, "room_type")
        if "|" in rt:  # an error: "code|reason"
            return rt
        # a goal is chosen by memory later; the node's goal guard enforces the person distance
        return {"room_type": rt}

    def _check_search(self, req: dict, view: WorldView):
        obj = req.get("object")
        if not isinstance(obj, str) or not _NAME_RE.match(obj.strip().lower()):
            return "bad_argument|object must be a short lower-case class name such as 'banana'"
        out: dict = {"object": obj.strip().lower().replace(" ", "_")}
        if req.get("region") is not None:
            rt = self._room(req["region"], view, "region")
            if "|" in rt:
                return rt
            out["region"] = rt
        elif not view.room_types:
            return "unknown_room|no room is known yet; explore first"
        b = self._budget(req, self.limits.default_search_budget_s)
        if isinstance(b, str):
            return b
        out["budget_s"] = b
        return out

    def _check_come_to(self, req: dict, view: WorldView):
        lim = self.limits
        who = req.get("person")
        if not isinstance(who, str) or not _NAME_RE.match(who.strip().lower()):
            return "bad_argument|person must be a short name such as 'operator'"
        st = req.get("standoff_m", lim.default_standoff_m)
        if isinstance(st, bool) or not isinstance(st, (int, float)) or not math.isfinite(st):
            return "bad_argument|standoff_m must be a number of metres"
        if st < lim.min_person_m:
            return f"unsafe|I never go closer than {lim.min_person_m:g} m to a person"
        if st > lim.max_standoff_m:
            return f"bad_argument|standoff_m must be at most {lim.max_standoff_m:g}"
        if not view.people:
            return "unknown_person|I have not seen anyone yet"
        return {"person": who.strip().lower(), "standoff_m": float(st)}

    def _check_return(self, req: dict, view: WorldView):
        if "land" in req and not isinstance(req["land"], bool):
            return "bad_argument|land must be true or false"
        if view.home_xy is None:
            return "no_home|I do not know my take-off point yet"
        return {"land": bool(req.get("land", False))}

    # -- run-time guard ----------------------------------------------------------------------
    def goal_violation(self, goal: XY, view: WorldView) -> str | None:
        """Reason a flight goal must not be sent (closer than min_person_m to a person)."""
        for p in view.people:
            if math.dist(goal, p) < self.limits.min_person_m:
                return (
                    f"goal is {math.dist(goal, p):.2f} m from a person; "
                    f"never closer than {self.limits.min_person_m:g} m"
                )
        return None

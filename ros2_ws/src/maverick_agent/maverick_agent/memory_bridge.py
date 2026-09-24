"""memory_query(question) <-> the memory node's query/answer JSON (no ROS imports).

The memory node answers three typed queries (goal_for_type, find, room_of; see
maverick_memory.queries). Free-text questions are mapped to them by keyword.
"""

from __future__ import annotations

import re

from .tools import ROOM_TYPES

_FIND_RE = re.compile(r"\b(?:where(?:'s| is| are)|find|seen|see|any|is there an?|have you seen)\b")
_STOP = {"the", "a", "an", "any", "some", "of", "my", "our", "you", "i", "have", "seen", "is"}


def question_to_query(question: str, qid: str) -> dict | None:
    q = question.lower().strip(" ?.!")
    for rt in ROOM_TYPES:
        if re.search(rf"\b{rt}\b", q) and re.search(r"\b(know|where|which|is there|room)\b", q):
            if not _FIND_RE.search(q) or "room" in q or "know" in q:
                return {"query": "goal_for_type", "room_type": rt, "id": qid}
    if _FIND_RE.search(q):
        words = [
            w for w in re.findall(r"[a-z_]+", q) if w not in _STOP and not _FIND_RE.fullmatch(w)
        ]
        if words:
            return {
                "query": "find",
                "label": words[-1].rstrip("s") if len(words[-1]) > 3 else words[-1],
                "id": qid,
            }
    return None


def format_answer(ans: dict) -> str:
    if not ans.get("ok"):
        return ans.get("reason") or "nothing known about that yet"
    kind = ans.get("query")
    if kind == "goal_for_type":
        return f"yes, a {ans.get('room_type')} is known (room {ans.get('room_id')})"
    if kind == "find":
        objs = ans.get("objects", [])
        return f"{len(objs)} seen: " + "; ".join(
            f"{o['id']} in room {o.get('room_id') or 'unknown'}" for o in objs
        )
    if kind == "room_of":
        return f"in room {ans.get('room_id')} ({ans.get('room_type')})"
    return "ok"

"""Gateway events the agent produces (docs/gateway-events.md): `transcript` and `agent_step`.

Each helper returns the `data` dict; the node wraps it in the envelope. No ROS imports.
"""

from __future__ import annotations

import itertools

_ids = itertools.count(1)

_LABELS = {
    "explore": "Explore",
    "goto_room": "Go to {room}",
    "search": "Search for {object}",
    "come_to": "Come to {person}",
    "return_home": "Return home",
    "memory_query": "Check memory",
    "say": "Say something",
    "send_photo": "Send photo",
}


def step_label(tool: str, args: dict) -> str:
    try:
        return _LABELS.get(tool, tool).format(**{k: str(v) for k, v in args.items()})
    except (KeyError, IndexError):
        return _LABELS.get(tool, tool).split(" {")[0]


def transcript_event(role: str, text: str, source: str | None = None, msg_id: str | None = None):
    d = {"id": msg_id or f"agent-{next(_ids)}", "role": role, "text": text}
    if source:
        d["source"] = source
    return d


def agent_step_event(plan_id: str, goal: str, steps: list[dict]) -> dict:
    """`steps`: dicts with id, label, status (pending|running|done|failed|skipped), detail?."""
    return {"plan_id": plan_id, "goal": goal, "steps": [dict(s) for s in steps]}

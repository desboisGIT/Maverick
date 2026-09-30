"""Typed tool specs for the agent: JSON schemas (strict) and pure-Python argument validation.

No tool takes coordinates or setpoints (CLAUDE.md decision): places are room types, objects are
class names, people are descriptions. No ROS imports.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

try:  # the room-type vocabulary lives in maverick_memory; fall back when numpy is missing
    from maverick_memory.room_typing import TYPES as ROOM_TYPES
except ImportError:  # pragma: no cover
    ROOM_TYPES = ("living", "kitchen", "bedroom", "bathroom", "hallway", "office")

PHOTO_CURRENT_VIEW = "current_view"
MAX_BUDGET_S = 900.0


class ToolError(ValueError):
    """Invalid tool call; the message goes back to the model as an is_error tool_result."""


@dataclass(frozen=True)
class Prop:
    type: str  # string | number | integer
    description: str
    enum: tuple[str, ...] | None = None
    minimum: float | None = None  # validated here only: strict schemas do not accept bounds
    maximum: float | None = None
    max_length: int = 200
    required: bool = True

    def schema(self) -> dict:
        d: dict[str, Any] = {"type": self.type, "description": self.description}
        if self.enum:
            d["enum"] = list(self.enum)
        return d

    def check(self, name: str, v: Any) -> Any:
        if self.type == "string":
            if not isinstance(v, str):
                raise ToolError(f"{name} must be a string")
            v = v.strip()
            if not v:
                raise ToolError(f"{name} must not be empty")
            if len(v) > self.max_length:
                raise ToolError(f"{name} is longer than {self.max_length} characters")
            if self.enum is not None:
                v = v.lower()
                if v not in self.enum:
                    raise ToolError(f"{name} must be one of {', '.join(self.enum)}")
            return v
        want = (int,) if self.type == "integer" else (int, float)
        if isinstance(v, bool) or not isinstance(v, want):
            raise ToolError(f"{name} must be a {self.type}")
        if self.minimum is not None and v < self.minimum:
            raise ToolError(f"{name} must be at least {self.minimum:g}")
        if self.maximum is not None and v > self.maximum:
            raise ToolError(f"{name} must be at most {self.maximum:g}")
        return v


@dataclass(frozen=True)
class ToolSpec:
    name: str
    description: str
    props: dict[str, Prop] = field(default_factory=dict)
    motion: bool = False

    def to_api(self) -> dict:
        return {
            "name": self.name,
            "description": self.description,
            "strict": True,
            "input_schema": {
                "type": "object",
                "properties": {k: p.schema() for k, p in self.props.items()},
                "required": [k for k, p in self.props.items() if p.required],
                "additionalProperties": False,
            },
        }

    def validate(self, args: Any) -> dict:
        if not isinstance(args, dict):
            raise ToolError("arguments must be an object")
        extra = sorted(set(args) - set(self.props))
        if extra:
            raise ToolError(f"unexpected argument(s): {', '.join(extra)}")
        out: dict = {}
        for k, p in self.props.items():
            if k not in args or args[k] is None:
                if p.required:
                    raise ToolError(f"missing required argument {k}")
                continue
            out[k] = p.check(k, args[k])
        return out


_ROOM = Prop("string", "Room type.", enum=tuple(ROOM_TYPES))
_BUDGET = Prop(
    "number", f"Time budget in seconds (10 to {MAX_BUDGET_S:g}).", minimum=10, maximum=MAX_BUDGET_S
)

TOOLS: dict[str, ToolSpec] = {
    t.name: t
    for t in (
        ToolSpec(
            "explore",
            "Fly around to map unexplored space and discover rooms. Returns when the budget runs "
            "out or nothing reachable is left unexplored.",
            {"budget_s": _BUDGET},
            motion=True,
        ),
        ToolSpec(
            "goto_room",
            "Fly to a room of the given type that is already known. Fails if no such room has "
            "been found yet; explore first.",
            {"room": _ROOM},
            motion=True,
        ),
        ToolSpec(
            "search",
            "Look for an object (a class name such as 'banana') by flying through known space, "
            "optionally restricted to one room type. Returns what was found and where.",
            {
                "object": Prop("string", "Object class to look for, lower case.", max_length=64),
                "region": Prop(
                    "string",
                    "Room type to restrict the search to.",
                    enum=tuple(ROOM_TYPES),
                    required=False,
                ),
                "budget_s": _BUDGET,
            },
            motion=True,
        ),
        ToolSpec(
            "come_to",
            "Fly to a person and hover near them.",
            {"person": Prop("string", "Who, e.g. 'operator'.", max_length=40)},
            motion=True,
        ),
        ToolSpec("return_home", "Fly back to the take-off point and land.", motion=True),
        ToolSpec(
            "memory_query",
            "Ask what the drone already knows: rooms, objects seen and where. Does not move.",
            {"question": Prop("string", "A short question.", max_length=200)},
        ),
        ToolSpec(
            "say",
            "Speak to the operator. Keep it to one or two short sentences.",
            {"text": Prop("string", "What to say.", max_length=400)},
        ),
        ToolSpec(
            "send_photo",
            "Send a photo to the operator's phone.",
            {
                "target": Prop(
                    "string",
                    f"An object id from memory, or '{PHOTO_CURRENT_VIEW}'.",
                    max_length=64,
                ),
                "caption": Prop("string", "Caption.", max_length=200),
            },
        ),
    )
}


def tool_schemas() -> list[dict]:
    return [t.to_api() for t in TOOLS.values()]


def validate_call(name: str, args: Any) -> dict:
    spec = TOOLS.get(name)
    if spec is None:
        raise ToolError(f"unknown tool {name!r}; available: {', '.join(TOOLS)}")
    out = spec.validate(args)
    if name == "send_photo":
        t = out["target"]
        if t != PHOTO_CURRENT_VIEW and not all(c.isalnum() or c in "_:.-" for c in t):
            raise ToolError("target must be 'current_view' or an object id")
    return out


def skill_request(name: str, args: dict) -> dict:
    """Validated motion-tool call -> the JSON for /maverick/skill/request."""
    if name == "explore":
        return {"skill": "explore", "budget_s": float(args["budget_s"])}
    if name == "goto_room":
        return {"skill": "goto_room", "room_type": args["room"]}
    if name == "search":
        d = {"skill": "search", "object": args["object"], "budget_s": float(args["budget_s"])}
        if "region" in args:
            d["region"] = args["region"]
        return d
    if name == "come_to":
        return {"skill": "come_to", "person": args["person"]}
    if name == "return_home":
        return {"skill": "return"}
    raise ToolError(f"{name} is not a motion tool")

"""Validator for gateway -> console frames, mirroring console/src/gateway/types.ts (protocol v1).

Used by the unit tests and by the end-to-end smoke test on live frames. `validate(frame)` returns
a list of problems (empty = valid). Unknown event types are valid (the protocol ignores them).
"""

from __future__ import annotations

import numbers

ROOM_TYPES = {"unknown", "living", "kitchen", "bedroom", "bathroom", "hallway", "office", "other"}
SKILLS = {"explore", "goto", "search", "inspect", "follow", "monitor", "return", "report"}


def _num(v) -> bool:
    return isinstance(v, numbers.Real) and not isinstance(v, bool)


def _vec(v, n: int) -> bool:
    return isinstance(v, list) and len(v) == n and all(_num(x) for x in v)


def _check(cond: bool, msg: str, errs: list[str]) -> None:
    if not cond:
        errs.append(msg)


def _pose(d, e):
    _check(_vec(d.get("position"), 3) and _vec(d.get("velocity"), 3), "position/velocity Vec3", e)
    for k in ("yaw", "pitch", "roll"):
        _check(_num(d.get(k)), f"{k} number", e)


def _camera_frame(d, e):
    _check(isinstance(d.get("seq"), int), "seq int", e)
    _check(isinstance(d.get("width"), int) and isinstance(d.get("height"), int), "width/height", e)
    _check(d.get("mime") in ("image/jpeg", "image/png", "image/svg+xml"), "mime", e)
    _check(isinstance(d.get("data"), str) and not d["data"].startswith("data:"), "data base64", e)
    _check(_num(d.get("hfov_deg")), "hfov_deg", e)


def _detections(d, e):
    _check(isinstance(d.get("frame_seq"), int), "frame_seq int", e)
    for b in d.get("boxes", []):
        _check(isinstance(b.get("label"), str), "box label", e)
        _check(_num(b.get("confidence")) and 0 <= b["confidence"] <= 1, "box confidence", e)
        _check(
            _vec(b.get("box"), 4) and all(-0.01 <= x <= 1.01 for x in b["box"]), "box [x,y,w,h]", e
        )
    _check(isinstance(d.get("boxes"), list), "boxes list", e)


def _map_update(d, e):
    _check(_num(d.get("resolution")) and d["resolution"] > 0, "resolution", e)
    _check(_vec(d.get("origin"), 3), "origin Vec3", e)
    _check(isinstance(d.get("reset"), bool), "reset bool", e)
    for k in ("add", "remove"):
        v = d.get(k)
        _check(
            isinstance(v, list) and len(v) % 3 == 0 and all(isinstance(x, int) for x in v),
            f"{k} flat int triples",
            e,
        )


def _frontiers(d, e):
    for f in d.get("frontiers", []):
        _check(
            isinstance(f.get("id"), str) and _vec(f.get("position"), 3), "frontier id/position", e
        )
        _check(isinstance(f.get("size"), int), "frontier size", e)
    _check(isinstance(d.get("frontiers"), list), "frontiers list", e)


def _path(d, e):
    _check(isinstance(d.get("path_id"), str), "path_id", e)
    _check(d.get("status") in ("planned", "executing", "done", "failed"), "path status", e)
    _check(
        isinstance(d.get("waypoints"), list) and all(_vec(w, 3) for w in d["waypoints"]),
        "waypoints Vec3[]",
        e,
    )
    _check("goal" not in d or _vec(d["goal"], 3), "goal Vec3", e)


def _rooms(d, e):
    for r in d.get("rooms", []):
        _check(isinstance(r.get("id"), str) and isinstance(r.get("label"), str), "room id/label", e)
        _check(r.get("type") in ROOM_TYPES, f"room type {r.get('type')!r}", e)
        _check(
            isinstance(r.get("polygon"), list) and all(_vec(p, 2) for p in r["polygon"]),
            "polygon Vec2[]",
            e,
        )
        _check(
            _num(r.get("confidence")) and isinstance(r.get("visited"), bool), "room conf/visited", e
        )
    _check(isinstance(d.get("rooms"), list), "rooms list", e)


def _objects(d, e):
    for o in d.get("objects", []):
        _check(
            isinstance(o.get("id"), str) and isinstance(o.get("label"), str), "object id/label", e
        )
        _check(_vec(o.get("position"), 3) and _vec(o.get("size"), 3), "object position/size", e)
        _check(_num(o.get("confidence")) and _num(o.get("last_seen")), "object conf/last_seen", e)
        _check(o.get("status") in ("present", "gone", "unverified"), "object status", e)
    _check(isinstance(d.get("objects"), list), "objects list", e)


def _transcript(d, e):
    _check(isinstance(d.get("id"), str) and isinstance(d.get("text"), str), "transcript id/text", e)
    _check(d.get("role") in ("user", "maverick", "system"), "transcript role", e)


def _agent_step(d, e):
    _check(isinstance(d.get("plan_id"), str) and isinstance(d.get("goal"), str), "plan_id/goal", e)
    for s in d.get("steps", []):
        _check(isinstance(s.get("id"), str) and isinstance(s.get("label"), str), "step id/label", e)
        _check(
            s.get("status") in ("pending", "running", "done", "failed", "skipped"), "step status", e
        )
    _check(isinstance(d.get("steps"), list), "steps list", e)


def _skill_progress(d, e):
    _check(d.get("skill") in SKILLS, f"skill {d.get('skill')!r}", e)
    _check(d.get("state") in ("running", "succeeded", "failed", "cancelled"), "skill state", e)
    _check(_num(d.get("progress")) and isinstance(d.get("message"), str), "progress/message", e)


def _status(d, e):
    _check(_num(d.get("battery_pct")), "battery_pct", e)
    link = d.get("link", {})
    _check(link.get("quality") in ("good", "degraded", "lost"), "link.quality", e)
    _check(isinstance(d.get("mode"), str) and isinstance(d.get("armed"), bool), "mode/armed", e)
    _check(_num(d.get("vram_used_gb")) and _num(d.get("vram_total_gb")), "vram", e)


CHECKS = {
    "pose": _pose,
    "camera_frame": _camera_frame,
    "detections": _detections,
    "map_update": _map_update,
    "frontiers": _frontiers,
    "path": _path,
    "rooms": _rooms,
    "objects": _objects,
    "transcript": _transcript,
    "agent_step": _agent_step,
    "skill_progress": _skill_progress,
    "status": _status,
}


def validate(frame: dict) -> list[str]:
    """Problems with one parsed gateway frame (envelope and data), empty if valid."""
    errs: list[str] = []
    if not isinstance(frame, dict) or set(frame) != {"type", "t", "data"}:
        return ["envelope must be exactly {type, t, data}"]
    _check(isinstance(frame["type"], str) and _num(frame["t"]), "type str / t number", errs)
    if not isinstance(frame["data"], dict):
        return errs + ["data must be an object"]
    check = CHECKS.get(frame["type"])
    if check is not None:
        sub: list[str] = []
        check(frame["data"], sub)
        errs += [f"{frame['type']}: {m}" for m in sub]
    return errs

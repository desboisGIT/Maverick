"""Pure helpers behind memory_node: query handling and the detections JSON (no ROS imports)."""

from __future__ import annotations

from .lifting import Detection
from .scene_graph import SceneGraph


def detections_json(dets: list[Detection], width: int, height: int, stamp: float) -> dict:
    """Boxes normalised to the image ([x, y, w, h], top-left origin) as the gateway wants them."""
    boxes = []
    for d in dets:
        if d.box is None:
            continue
        u0, v0, u1, v1 = d.box
        boxes.append(
            {
                "label": d.label,
                "confidence": round(float(d.confidence), 3),
                "box": [
                    round(u0 / width, 4),
                    round(v0 / height, 4),
                    round((u1 - u0 + 1) / width, 4),
                    round((v1 - v0 + 1) / height, 4),
                ],
            }
        )
    return {"stamp": stamp, "width": width, "height": height, "boxes": boxes}


def answer_query(sg: SceneGraph, q: dict, robot_xy: tuple[float, float] | None) -> dict:
    """Answer a memory query. Every answer echoes `id` and has `ok`.

    goal_for_type {room_type}: nearest room of the type and a navigation goal in it
        -> {ok, room_id, room_type, confidence, goal: [x, y]} or {ok: false, reason}
    find {label}: confirmed objects of the class -> {ok, objects: [{id, position, room_id}]}
    room_of {xy}: the room containing a point
    """
    out: dict = {"id": q.get("id"), "ok": False, "query": q.get("query")}
    kind = q.get("query")
    start = tuple(q["start"]) if q.get("start") else robot_xy
    if kind == "goal_for_type":
        rtype = str(q.get("room_type", ""))
        out["room_type"] = rtype
        res = sg.goal_for_type(rtype, start)
        if res is None and sg.rooms_of_type(rtype):
            # not connected to the robot through the (fat-walled) map: still name a goal in the room
            # and let the skill's planner decide whether it can get there
            res = sg.goal_for_type(rtype, None)
        if res is None:
            out["reason"] = f"no {rtype} known yet" if not sg.rooms_of_type(rtype) else "no goal"
            return out
        room, goal = res
        out.update(
            ok=True,
            room_id=room.id,
            confidence=round(room.confidence, 3),
            goal=[round(goal[0], 3), round(goal[1], 3)],
        )
    elif kind == "find":
        objs = sg.find(str(q.get("label", "")))
        out.update(
            ok=bool(objs),
            objects=[
                {
                    "id": o.id,
                    "position": [round(float(v), 3) for v in o.position],
                    "room_id": o.room_id,
                }
                for o in objs
            ],
        )
    elif kind == "room_of":
        r = sg.room_of(tuple(q.get("xy", ())) or (0.0, 0.0))
        out.update(ok=r is not None, room_id=r.id if r else None, room_type=r.type if r else None)
    else:
        out["reason"] = f"unknown query {kind!r}"
    return out

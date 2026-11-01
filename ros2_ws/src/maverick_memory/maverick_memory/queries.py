"""Pure helpers behind memory_node: query handling and the detections JSON (no ROS imports)."""

from __future__ import annotations

from .lifting import Detection
from .scene_graph import SceneGraph
from .store import canonical_label


def detections_json(
    dets: list[Detection],
    width: int,
    height: int,
    stamp: float,
    positions: list | None = None,
) -> dict:
    """Boxes normalised to the image ([x, y, w, h], top-left origin) as the gateway wants them.

    `positions` (optional, aligned with `dets`): the lifted map-frame position of each detection
    or None; it is added as "position" for the skills (search), the gateway ignores it.
    """
    boxes = []
    for k, d in enumerate(dets):
        if d.box is None:
            continue
        u0, v0, u1, v1 = d.box
        pos = positions[k] if positions is not None else None
        boxes.append(
            {
                **({"position": [round(float(v), 3) for v in pos]} if pos is not None else {}),
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


# Classes with a surface an object can rest on: the search skill looks at each of them.
SEARCH_SURFACES = (
    "kitchen_counter",
    "dining_table",
    "desk",
    "coffee_table",
    "nightstand",
    "tv_unit",
    "shelves",
    "bed",
)


def search_plan(sg: SceneGraph, label: str, region, start) -> dict:
    """Rooms to search for `label` (the rooms of type `region`, else every known room): a goal
    in each, the surfaces memory holds there, and whether the class was already seen there."""
    if region:
        rooms = sg.rooms_of_type(str(region)) or [
            r for r in sg.room_list() if r.secondary == region
        ]
    else:
        rooms = sg.room_list()
    if not rooms:
        return {"ok": False, "reason": f"no {region} known yet" if region else "no room known yet"}
    seen_in = {o.room_id for o in sg.objects(confirmed_only=False) if o.label == label}
    out = []
    for r in rooms:
        goal = sg.goal_in_room(r.id, start=tuple(start) if start else None) or sg.goal_in_room(r.id)
        if goal is None:
            goal = (float(r.centroid[0]), float(r.centroid[1]))
        surfaces = [
            {
                "id": o.id,
                "label": o.label,
                "lo": [round(float(v), 3) for v in o.lo],
                "hi": [round(float(v), 3) for v in o.hi],
            }
            for o in sg.objects_in(r.id)
            if o.label in SEARCH_SURFACES
        ]
        cands = [
            {
                "id": o.id,
                "position": [round(float(v), 3) for v in o.position[:3]],
                "confidence": round(float(o.confidence), 3),
            }
            for o in sg.objects_in(r.id, confirmed_only=False)
            if o.label == label
        ]
        out.append(
            {
                "id": r.id,
                "type": r.type,
                "goal": [round(goal[0], 3), round(goal[1], 3)],
                "surfaces": surfaces,
                "candidates": cands,  # earlier, unconfirmed sightings of the class: look first
                "known": r.id in seen_in,
            }
        )
    return {"ok": True, "rooms": out}


def answer_query(sg: SceneGraph, q: dict, robot_xy: tuple[float, float] | None) -> dict:
    """Answer a memory query. Every answer echoes `id` and has `ok`.

    goal_for_type {room_type}: nearest room of the type and a navigation goal in it
        -> {ok, room_id, room_type, confidence, goal: [x, y]} or {ok: false, reason}
    find {label, confirmed?}: objects of the class (confirmed ones unless "confirmed": false)
        -> {ok, objects: [{id, position, room_id, confidence, n_obs, photo?}]}
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
        label = str(q.get("label", ""))
        if q.get("confirmed", True):
            objs = sg.find(label)
        else:
            lab = canonical_label(label)
            objs = sorted(
                (o for o in sg.objects(confirmed_only=False) if o.label == lab),
                key=lambda o: -o.confidence,
            )
        out.update(
            ok=bool(objs),
            objects=[
                {
                    "id": o.id,
                    "position": [round(float(v), 3) for v in o.position],
                    "room_id": o.room_id,
                    "confidence": round(o.confidence, 3),
                    "n_obs": o.n_obs,
                    "last_seen": round(float(o.last_seen), 2),
                    **({"photo": o.extra["photo"]} if o.extra.get("photo") else {}),
                }
                for o in objs
            ],
        )
    elif kind == "search_plan":
        out.update(
            search_plan(sg, canonical_label(str(q.get("label", ""))), q.get("region"), start)
        )
    elif kind == "room_of":
        r = sg.room_of(tuple(q.get("xy", ())) or (0.0, 0.0))
        out.update(ok=r is not None, room_id=r.id if r else None, room_type=r.type if r else None)
    else:
        out["reason"] = f"unknown query {kind!r}"
    return out

import numpy as np
from maverick_mapping.synthetic import CameraSim, Scene
from maverick_mapping.voxel_map import optical_rotation
from maverick_memory.lifting import Detection, lift_detection
from maverick_memory.room_typing import score_room
from maverick_memory.rooms import RoomTracker, distance_cells, segment_rooms
from maverick_memory.scene_graph import SceneGraph
from maverick_memory.store import ObjectStore


def _room_scene(extra_box=None) -> Scene:
    c = [[3, 3, -0.5], [3, 0, 1.25], [3, 6, 1.25], [0, 3, 1.25], [6, 3, 1.25]]
    h = [
        [4, 4, 0.5],
        [3.05, 0.05, 1.25],
        [3.05, 0.05, 1.25],
        [0.05, 3.05, 1.25],
        [0.05, 3.05, 1.25],
    ]
    if extra_box:
        c.append(extra_box[0])
        h.append(extra_box[1])
    man = {"footprint": [[0, 0], [6, 6]], "ceiling_height": 2.5, "spawn": {"pose": [3, 3, 0, 0]}}
    return Scene(c, [np.eye(3)] * len(c), h, man)


def test_lift_box_detection_recovers_object_centre():
    # a 0.6 x 0.6 x 1.0 box centred at (5, 3), camera at (2.5, 3, 0.6) looking +x
    sc = _room_scene(([5.0, 3.0, 0.5], [0.3, 0.3, 0.5]))
    cam = CameraSim(80, 60, noise_base=0.0, noise_quad=0.0)
    t, r = np.array([2.5, 3.0, 0.6]), optical_rotation(0.0)
    depth = cam.render(sc, t, r)
    # the box spans u in about [32, 47] in this 80x60 image; give a loose box that includes wall
    det = Detection("fridge", 0.9, box=(28, 12, 52, 42))
    obs = lift_detection(det, depth, cam.intr, t, r)
    assert obs is not None
    # visible front face at x = 4.7, y in [2.7, 3.3]; z up to 1.0
    assert abs(obs.centroid[0] - 4.7) < 0.1
    assert abs(obs.center[1] - 3.0) < 0.15
    assert obs.hi[2] < 1.1
    # the wall behind the box (x = 6) must not have been lifted
    assert obs.hi[0] < 5.0


def test_lift_rejects_empty_depth():
    cam = CameraSim(80, 60)
    d = np.full((60, 80), np.nan)
    assert lift_detection(Detection("sofa", 0.9, box=(10, 10, 30, 30)), d, cam.intr,
                          np.zeros(3), np.eye(3)) is None  # fmt: skip


def _obs(label, x, y, conf=0.8, t=0.0, size=0.5):
    from maverick_memory.lifting import ObjectObservation

    lo = np.array([x - size / 2, y - size / 2, 0.0])
    hi = np.array([x + size / 2, y + size / 2, 0.8])
    return ObjectObservation(label, conf, lo, hi, (lo + hi) / 2, 50, 3.0, False, t)


def test_store_associates_merges_and_prunes():
    st = ObjectStore()
    for k in range(4):
        st.add_frame([_obs("sofa", 1.0 + 0.02 * k, 2.0), _obs("sofa", 4.0, 2.0)], now=k * 0.3)
    assert len(st.confirmed()) == 2  # two sofas, each with 4 observations, stable ids
    ids = {o.id for o in st.confirmed()}
    st.add_frame([_obs("sofa", 1.05, 2.0, t=2.0)], now=2.0)
    assert {o.id for o in st.confirmed()} == ids
    # a single false positive is never confirmed and is pruned after a while
    st.add_frame([_obs("toilet", 9.0, 9.0, conf=0.4, t=2.0)], now=2.0)
    assert len(st.objects) == 3 and len(st.confirmed()) == 2
    st.prune(now=20.0)
    assert len(st.objects) == 2
    # a different class at the same place is a different object
    st.add_frame([_obs("chair", 1.0, 2.0, t=3.0)], now=3.0)
    assert len(st.objects) == 3
    # a partial view (thin slab on one face) of the same object merges into it
    from maverick_memory.lifting import ObjectObservation

    lo, hi = np.array([3.75, 1.75, 0.0]), np.array([3.8, 2.25, 0.8])
    slab = ObjectObservation("sofa", 0.8, lo, hi, (lo + hi) / 2, 20, 2.0, False, 3.0)
    st.add_frame([slab], now=3.0)
    assert len([o for o in st.objects.values() if o.label == "sofa"]) == 2


def test_distance_cells_exact():
    m = np.zeros((20, 20), bool)
    m[2:19, 2:19] = True
    d = distance_cells(m)
    assert abs(d[10, 10] - 9.0) < 1e-6 or abs(d[10, 10] - 8.0) < 1e-6
    assert d[2, 2] == 1.0 and d[0, 0] == 0.0
    assert abs(d[3, 3] - 2.0) < 1e-6


def test_typing_rules():
    assert score_room([("fridge", 0.9), ("kitchen_counter", 0.8)]).type == "kitchen"
    assert score_room([("bed", 0.9), ("desk", 0.8), ("office_chair", 0.8)]).type == "bedroom"
    assert score_room([("desk", 0.9), ("office_chair", 0.8)]).type == "office"
    assert score_room([("toilet", 0.9), ("sink", 0.7)]).type == "bathroom"
    assert score_room([("plant", 0.9)]).type == "unknown"
    assert score_room([], elongation=3.0, width=1.4).type == "hallway"
    assert score_room([("sofa", 0.9), ("tv_unit", 0.9)], elongation=3.0, width=1.4).type == "living"
    open_plan = score_room([("fridge", 0.9), ("oven", 0.9), ("sofa", 0.9), ("tv_unit", 0.9)])
    assert open_plan.secondary in ("kitchen", "living") and open_plan.type != open_plan.secondary
    assert open_plan.confidence < 0.7


def _ideal_grid(seed: int, res: float = 0.1, drone: str = "x500"):
    sc = Scene.from_seed(seed, drone)
    lo, hi = sc.bounds
    org = (lo[0] - 0.45, lo[1] - 0.45)
    nx, ny = int((hi[0] - lo[0] + 0.9) / res), int((hi[1] - lo[1] + 0.9) / res)
    xs = org[0] + (np.arange(nx) + 0.5) * res
    ys = org[1] + (np.arange(ny) + 0.5) * res
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    grid = np.zeros((nx, ny), np.int8)
    pts = np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, 1.25)], axis=1)
    occ = np.concatenate(
        [sc.occupied_points(pts[i : i + 3000]) for i in range(0, len(pts), 3000)]
    ).reshape(nx, ny)
    grid[occ] = 100
    # outside the footprint stays unknown
    inside = (gx > lo[0]) & (gx < hi[0]) & (gy > lo[1]) & (gy < hi[1])
    grid[~inside & ~occ] = -1
    return sc, grid, res, org


def test_rooms_from_ideal_map_and_stable_ids():
    sc, grid, res, org = _ideal_grid(1000003)
    regs = segment_rooms(grid, res, org)
    assert len(regs) == len(sc.manifest["rooms"])
    tr = RoomTracker()
    ids = tr.assign(regs)
    # the map "grows": hide the right 40 % as unknown, then reveal it. Ids must persist.
    part = grid.copy()
    part[int(grid.shape[0] * 0.6) :, :] = -1
    ids_part = tr.assign(segment_rooms(part, res, org))
    assert set(ids_part) <= set(ids)
    ids_full = tr.assign(segment_rooms(grid, res, org))
    assert ids_full == ids


def test_scene_graph_goal_in_kitchen():
    from maverick_memory.store import StoredObject

    sc, grid, res, org = _ideal_grid(1000003)
    sg = SceneGraph()
    for k, f in enumerate(sc.manifest["furniture"]):
        lo, hi = np.array(f["bbox"][0]), np.array(f["bbox"][1])
        sg.store.objects[f"obj_{k}"] = StoredObject(f"obj_{k}", f["class"], lo, hi, 0.05, 9, 0, 1)
    sg.update_rooms(grid, res, org)
    types = sorted(r.type for r in sg.room_list())
    assert types == sorted(r["type"] for r in sc.manifest["rooms"])
    kit = next(r for r in sc.manifest["rooms"] if r["type"] == "kitchen")
    room, goal = sg.goal_for_type("kitchen", start=sc.spawn[:2])
    xs, ys = zip(*kit["polygon"], strict=True)
    assert min(xs) + 1.0 <= goal[0] <= max(xs) - 1.0 and min(ys) + 1.0 <= goal[1] <= max(ys) - 1.0
    assert sg.room_of(goal).id == room.id
    # objects know their room; queries
    fridge = sg.find("fridge")[0]
    assert fridge.room_id == room.id and fridge in sg.objects_in(room.id)
    assert sg.nearest_room_of_type("bathroom", goal).type == "bathroom"
    assert sg.room_of((-20.0, -20.0)) is None
    gw = sg.gateway_rooms()
    assert {"id", "type", "label", "polygon", "confidence", "visited"} <= set(gw[0])
    assert {
        "id",
        "label",
        "position",
        "size",
        "room_id",
        "confidence",
        "last_seen",
        "status",
    } <= set(sg.gateway_objects()[0])

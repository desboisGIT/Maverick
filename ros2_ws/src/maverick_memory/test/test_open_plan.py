"""Open-plan kitchens (kitchen and living room in one space) and the deepest-point goal."""

import numpy as np
from maverick_memory.rooms import FREE, OCC
from maverick_memory.rooms_split import split_by_clusters
from maverick_memory.scene_graph import SceneGraph, SceneGraphConfig
from maverick_memory.store import StoredObject

RES = 0.1
ORG = (0.0, 0.0)


def _box_grid(w_m: float, h_m: float) -> np.ndarray:
    """One rectangular room, walls on the border of the array."""
    g = np.full((int(w_m / RES), int(h_m / RES)), FREE, np.int8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = OCC
    return g


def _obj(sg: SceneGraph, k: int, label: str, x: float, y: float, sx=0.6, sy=0.6) -> None:
    lo = np.array([x - sx / 2, y - sy / 2, 0.0])
    hi = np.array([x + sx / 2, y + sy / 2, 0.9])
    sg.store.objects[f"o{k}"] = StoredObject(f"o{k}", label, lo, hi, 0.05, 9, 0, 1)


def _open_plan(sg: SceneGraph, w=8.0, h=4.0):
    """Kitchen furniture along the west wall, sofa group in the east half."""
    _obj(sg, 0, "kitchen_counter", 0.7, 1.0, 0.6, 1.2)
    _obj(sg, 1, "oven", 0.7, 2.0)
    _obj(sg, 2, "fridge", 0.7, 3.0)
    _obj(sg, 3, "sofa", 6.5, 2.0, 0.9, 1.9)
    _obj(sg, 4, "tv_unit", 7.6, 2.0, 0.4, 1.4)
    _obj(sg, 5, "coffee_table", 5.2, 2.0, 0.6, 1.1)
    sg.update_rooms(_box_grid(w, h), RES, ORG)


def test_open_plan_region_splits_into_kitchen_and_living():
    sg = SceneGraph()
    _open_plan(sg)
    types = {r.type: r for r in sg.room_list()}
    assert set(types) == {"kitchen", "living"}
    kit, liv = types["kitchen"], types["living"]
    assert kit.centroid[0] < liv.centroid[0]
    # the parts tile the region without overlap, and the kitchen part is the west side
    assert not (kit.region.mask & liv.region.mask).any()
    assert kit.region.area + liv.region.area >= 0.95 * 7.8 * 3.8
    assert 2.0 < kit.region.area < 20.0 and liv.region.area > 10.0
    assert sg.room_of((0.9, 2.0)).type == "kitchen" and sg.room_of((7.0, 2.0)).type == "living"
    # object membership follows the parts
    assert sg.find("fridge")[0].room_id == kit.id and sg.find("sofa")[0].room_id == liv.id


def test_goal_is_deepest_point_of_the_kitchen_part_not_its_edge():
    sg = SceneGraph(SceneGraphConfig(wall_clear=3.0, robot_clear=0.3))
    _open_plan(sg)
    room, goal = sg.goal_for_type("kitchen", start=(4.0, 2.0))
    assert room.type == "kitchen"
    kit = room.region.mask
    # clearance of the goal to the kitchen part's boundary, mapped on the part's own distance map
    from maverick_memory.rooms import distance_cells

    d = (distance_cells(kit) - 1.0) * RES
    i, j = int(goal[0] / RES), int(goal[1] / RES)
    assert d[i, j] >= d.max() - 0.15, (goal, d[i, j], d.max())
    assert d[i, j] > 1.0  # not at the near edge (the old goal sat 0.3-0.4 m inside)


def test_no_split_without_two_clusters():
    sg = SceneGraph()
    _obj(sg, 0, "kitchen_counter", 0.7, 1.0, 0.6, 1.2)
    _obj(sg, 1, "oven", 0.7, 2.0)
    _obj(sg, 2, "fridge", 0.7, 3.0)
    sg.update_rooms(_box_grid(8.0, 4.0), RES, ORG)
    assert [r.type for r in sg.room_list()] == ["kitchen"]


def test_ambiguous_sink_and_dining_table_do_not_seed():
    sg = SceneGraph()
    _obj(sg, 0, "kitchen_counter", 0.7, 1.0, 0.6, 1.2)
    _obj(sg, 1, "oven", 0.7, 2.0)
    _obj(sg, 2, "sink", 6.5, 2.0)  # weak for kitchen and bathroom: no cluster of its own
    _obj(sg, 3, "dining_table", 6.0, 1.0, 1.4, 0.9)
    sg.update_rooms(_box_grid(8.0, 4.0), RES, ORG)
    assert len(sg.room_list()) == 1


def test_small_region_is_not_split():
    sg = SceneGraph()
    _obj(sg, 0, "oven", 0.7, 1.0)
    _obj(sg, 1, "fridge", 0.7, 2.0)
    _obj(sg, 2, "sofa", 2.5, 1.5, 0.9, 1.9)
    _obj(sg, 3, "tv_unit", 3.0, 1.5, 0.4, 1.4)
    sg.update_rooms(_box_grid(3.2, 3.0), RES, ORG)  # under 10 m^2
    assert len(sg.room_list()) == 1


def test_split_parts_partition_the_mask_and_drop_tiny_parts():
    g = _box_grid(8.0, 4.0)
    mask = g == FREE
    seeds = {"kitchen": [(0.5, 1.0, 1.1, 3.0)], "living": [(6.0, 1.0, 7.0, 3.0)]}
    parts = split_by_clusters(mask, mask, seeds, RES, ORG)
    assert parts is not None and set(parts) == {"kitchen", "living"}
    assert (parts["kitchen"] | parts["living"] == mask).all()
    assert not (parts["kitchen"] & parts["living"]).any()
    # the bisector sits between the clusters, roughly midway
    xs = np.nonzero(parts["kitchen"].any(axis=1))[0]
    assert 3.0 < (xs.max() + 1) * RES < 4.6
    # a part below the minimum area makes the split void
    tiny = {"kitchen": [(0.2, 0.2, 0.4, 0.4)], "living": [(5.0, 2.0, 7.0, 3.0)]}
    small = np.zeros_like(mask)
    small[1:20, 1:15] = True  # 2.7 m^2 region: below min_region_area
    assert split_by_clusters(small, small, tiny, RES, ORG) is None


def test_lone_counter_does_not_split_and_best_kitchen_wins():
    sg = SceneGraph()
    _obj(sg, 0, "kitchen_counter", 4.0, 2.0, 0.6, 1.2)  # one stray counter in the living part
    _obj(sg, 1, "sofa", 6.5, 2.0, 0.9, 1.9)
    _obj(sg, 2, "tv_unit", 7.6, 2.0, 0.4, 1.4)
    _obj(sg, 3, "coffee_table", 5.2, 2.0, 0.6, 1.1)
    sg.update_rooms(_box_grid(8.0, 4.0), RES, ORG)
    assert [r.type for r in sg.room_list()] == ["living"]  # not split: one counter only
    # two kitchens in the scene graph: the one with the evidence is chosen, not the nearest
    g = np.full((160, 40), FREE, np.int8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = OCC
    g[80, :] = OCC  # wall between two rooms
    g[80, 15:25] = FREE
    sg2 = SceneGraph()
    _obj(sg2, 0, "kitchen_counter", 3.0, 2.0, 0.6, 1.2)  # west room: a counter
    _obj(sg2, 1, "fridge", 12.0, 1.0)  # east room: fridge, oven, counter
    _obj(sg2, 2, "oven", 12.0, 2.0)
    _obj(sg2, 3, "kitchen_counter", 12.0, 3.0, 0.6, 1.2)
    sg2.update_rooms(g, RES, ORG)
    room, _ = sg2.goal_for_type("kitchen", start=(2.0, 2.0))
    assert room.centroid[0] > 8.0


def _two_zones(gap_cells: int) -> np.ndarray:
    """3 m wide room, 7 m long, wall stubs on both sides at y = 4 m leaving `gap_cells` cells."""
    g = _box_grid(3.0, 7.0)
    stub = (30 - gap_cells) // 2
    for x in list(range(0, stub)) + list(range(30 - stub, 30)):
        g[x, 40] = OCC
    return g


def test_evidence_free_region_behind_a_wide_opening_joins_its_neighbour():
    # a wall stub (or a plant against the wall) fakes the end of a wall: the 2.2 m opening is
    # no door, the top part has no furniture of its own and belongs to the kitchen
    sg = SceneGraph(SceneGraphConfig(wall_clear=3.0, robot_clear=0.3))
    for k, (lab, y) in enumerate((("kitchen_counter", 1.0), ("oven", 2.0), ("fridge", 3.0))):
        _obj(sg, k, lab, 1.0, y, 0.6, 0.9)
    sg.update_rooms(_two_zones(22), RES, ORG)
    rooms = sg.room_list()
    assert len(rooms) == 1 and rooms[0].type == "kitchen"
    assert sg.room_of((1.5, 6.0)).id == rooms[0].id
    _, goal = sg.goal_for_type("kitchen", start=(1.5, 2.0))
    assert sg.room_of(goal).id == rooms[0].id and 1.2 < goal[0] < 1.8  # mid-width of the room


def test_door_width_opening_keeps_rooms_apart():
    sg = SceneGraph()
    for k, (lab, y) in enumerate((("kitchen_counter", 1.0), ("oven", 2.0), ("fridge", 3.0))):
        _obj(sg, k, lab, 1.0, y, 0.6, 0.9)
    sg.update_rooms(_two_zones(10), RES, ORG)  # 1.0 m door
    assert len(sg.room_list()) == 2

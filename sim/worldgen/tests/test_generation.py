import json
import time
import xml.etree.ElementTree as ET

import pytest
from worldgen import generate, seed_set, world_name
from worldgen.cli import main
from worldgen.manifest import HELD_OUT_SEEDS, TUNING_SEEDS
from worldgen.profile import DEFAULT_PROFILE, PROFILES
from worldgen.programme import FLAT_TYPES
from worldgen.validate import validate
from worldgen.walls import build_walls

TUNING = range(TUNING_SEEDS[0], TUNING_SEEDS[0] + 100)
HELD_OUT = range(HELD_OUT_SEEDS[0], HELD_OUT_SEEDS[0] + 100)


@pytest.mark.parametrize("drone", sorted(PROFILES))
def test_100_tuning_and_100_held_out_seeds_are_valid_and_fast(drone):
    started = time.perf_counter()
    slowest = 0.0
    flat_types = set()
    for seed in [*TUNING, *HELD_OUT]:
        t = time.perf_counter()
        world = generate(seed, drone=drone)
        slowest = max(slowest, time.perf_counter() - t)
        problems = validate(world.manifest, world.sdf)
        assert world.manifest["drone_profile"] == drone
        assert not problems, (seed, problems)
        assert len(world.sdf) < 2_000_000
        flat_types.add(world.manifest["flat_type"])
    assert slowest < 1.0, f"slowest flat took {slowest:.2f} s"
    assert flat_types == set(FLAT_TYPES)
    print(f"{drone}: 200 flats in {time.perf_counter() - started:.1f} s, slowest {slowest:.3f} s")


def test_generation_is_deterministic():
    for seed in (0, 7, 99_999, 1_000_000):
        a, b = generate(seed), generate(seed)
        assert a.sdf == b.sdf
        assert a.manifest_json() == b.manifest_json()
    assert generate(1).sdf != generate(2).sdf


def test_output_options_do_not_change_the_layout():
    base = generate(5).manifest
    for kwargs in ({"ceiling": False}, {"low_texture": True}, {"lighting": "evening"}):
        other = generate(5, **kwargs).manifest
        for key in ("rooms", "doors", "furniture", "spawn", "footprint"):
            assert other[key] == base[key]


def test_ceiling_option_and_low_texture_option():
    with_ceiling = ET.fromstring(generate(3).sdf).find("world")
    without = ET.fromstring(generate(3, ceiling=False).sdf).find("world")
    assert with_ceiling.find("model[@name='ceiling']") is not None
    assert without.find("model[@name='ceiling']") is None
    flat = generate(3, low_texture=True)
    assert len(flat.sdf) < len(generate(3).sdf) * 0.7
    assert not validate(flat.manifest, flat.sdf)


def test_world_name_and_structure():
    world = generate(11)
    assert world.name == world_name(11) == "apt_11"
    root = ET.fromstring(world.sdf)
    assert root.tag == "sdf" and root.find("world").get("name") == "apt_11"
    assert root.find("world/spherical_coordinates") is not None
    assert root.find("world/model[@name='ground_plane']") is not None
    assert root.findall("world/plugin") == []  # systems come from PX4's server config


def test_manifest_schema():
    m = generate(7).manifest
    assert m["seed"] == 7 and m["seed_set"] == "tuning" and m["generator_version"]
    assert {
        "footprint",
        "ceiling_height",
        "rooms",
        "doors",
        "furniture",
        "spawn",
        "lighting",
    } <= m.keys()
    room = m["rooms"][0]
    assert {"id", "type", "polygon", "area_m2"} <= room.keys()
    assert {"id", "rooms", "center", "width"} <= m["doors"][0].keys()
    f = m["furniture"][0]
    assert {"id", "class", "room", "pose", "bbox", "surfaces"} <= f.keys()
    assert len(f["pose"]) == 4 and len(f["bbox"]) == 2
    assert {"room", "pose"} == m["spawn"].keys() and len(m["spawn"]["pose"]) == 4
    json.loads(generate(7).manifest_json())


def test_seed_sets():
    assert seed_set(0) == seed_set(99_999) == "tuning"
    assert seed_set(1_000_000) == seed_set(1_099_999) == "held_out"
    assert seed_set(500_000) == "other"


def test_surfaces_exist_for_scenario_placement():
    classes = {}
    for seed in range(30):
        for f in generate(seed).manifest["furniture"]:
            if f["surfaces"]:
                classes.setdefault(f["class"], 0)
                classes[f["class"]] += 1
    assert {"kitchen_counter", "dining_table", "bed"} <= classes.keys()


def test_walls_leave_openings_free():
    for seed in range(20):
        m = generate(seed).manifest
        from worldgen.floorplan import Room
        from worldgen.geometry import Rect
        from worldgen.validate import _openings

        rooms = [Room(r["id"], r["type"], Rect.from_points(r["polygon"])) for r in m["rooms"]]
        ops = _openings(m, {r.id: r for r in rooms})
        for s in build_walls(rooms, ops):
            for o in ops:
                if s.orient == o.orient and abs(s.pos - o.pos) < 1e-6:
                    assert s.end <= o.start + 1e-6 or s.start >= o.end - 1e-6
        for o in ops:
            if o.kind == "door":
                assert round(o.width * 10) in PROFILES[m["drone_profile"]].door_widths_dm


def test_validator_catches_defects():
    world = generate(7)
    m = json.loads(world.manifest_json())
    m["furniture"][0]["bbox"][0][0] -= 5  # pokes through a wall
    assert any("wall" in e or "overlap" in e for e in validate(m))
    m = json.loads(world.manifest_json())
    m["doors"] = []
    assert any("unreachable" in e for e in validate(m))
    m = json.loads(world.manifest_json())
    m["spawn"]["pose"][0] += 0.6
    assert validate(m)
    assert validate(world.manifest, world.sdf.replace('name="apt_7"', 'name="x"'))


def test_cli_writes_files_named_after_the_world(tmp_path, capsys):
    assert main(["--seed", "3", "--count", "2", "--out", str(tmp_path)]) == 0
    assert sorted(p.name for p in tmp_path.iterdir()) == [
        "apt_3.json",
        "apt_3.sdf",
        "apt_4.json",
        "apt_4.sdf",
    ]
    assert "apt_3" in capsys.readouterr().out


@pytest.mark.parametrize("drone", sorted(PROFILES))
def test_door_openings_wider_than_drone(drone):
    lo, hi = PROFILES[drone].door_range
    for seed in range(10):
        for d in generate(seed, drone=drone).manifest["doors"]:
            if d["kind"] == "door":
                assert lo - 1e-6 <= d["width"] <= hi + 1e-6
            else:
                assert d["width"] >= PROFILES[drone].open_min_dm / 10 - 1e-6


def test_small_is_default_and_profiles_differ():
    # default follows MAVERICK_DRONE (small unless set)
    assert generate(4).manifest["drone_profile"] == DEFAULT_PROFILE
    assert generate(4, drone="x500").sdf != generate(4, drone="small").sdf
    assert generate(4, drone="small").sdf == generate(4, drone="small").sdf


def test_every_model_has_a_label_matching_the_class_table():
    from worldgen.assets import ASSETS
    from worldgen.classes import CLASSES, FURNITURE

    assert set(FURNITURE) == set(ASSETS)
    assert CLASSES["floor"] == 1 and len(set(CLASSES.values())) == len(CLASSES)
    assert 0 not in CLASSES.values()
    for seed in range(15):
        world = generate(seed)
        assert world.manifest["classes"] == CLASSES
        by_id = {
            f["id"]: f["class"]
            for f in world.manifest["furniture"]
            + world.manifest["small_objects"]
            + world.manifest["people"]
        }
        structural = {
            "ground_plane": "floor",
            "floor": "floor",
            "walls": "wall",
            "doors": "door",
            "ceiling": "ceiling",
            "windows": "window",
            "decor": "decor",
        }
        for m in ET.fromstring(world.sdf).find("world").findall("model"):
            plugins = m.findall("plugin")
            assert len(plugins) == 1, m.get("name")
            assert plugins[0].get("filename") == "gz-sim-label-system"
            assert plugins[0].get("name") == "gz::sim::systems::Label"
            expected = structural.get(m.get("name")) or by_id[m.get("name")]
            assert int(plugins[0].findtext("label")) == CLASSES[expected]


def test_new_classes_are_appended_without_renumbering():
    from worldgen.classes import CLASSES, FURNITURE, SMALL, STRUCTURAL

    n = len(STRUCTURAL) + len(FURNITURE)
    assert [CLASSES[c] for c in SMALL] == [n + 1, n + 2, n + 3]
    assert CLASSES["floor"] == 1 and CLASSES["rug"] == n


def test_extras_do_not_change_the_layout_and_are_deterministic():
    plain = generate(5, small_objects=False, people=False)
    full = generate(5)
    for key in ("rooms", "doors", "furniture", "spawn", "footprint"):
        assert plain.manifest[key] == full.manifest[key]
    assert plain.manifest["small_objects"] == [] and plain.manifest["people"] == []
    assert full.sdf == generate(5).sdf and full.manifest_json() == generate(5).manifest_json()


def test_small_objects_and_people_statistics():
    present = {"banana": 0, "bread": 0}
    people = kitchen_people = 0
    n = 100
    for seed in range(n):
        world = generate(seed)
        m = world.manifest
        assert not validate(m, world.sdf), seed
        for cls, there in m["presence"].items():
            present[cls] += there
            assert there == any(o["class"] == cls for o in m["small_objects"])
        for o in m["small_objects"]:
            room = next(r for r in m["rooms"] if r["id"] == o["room"])
            assert room["type"] == "kitchen" and o["position"][2] > 0.4
        for p in m["people"]:
            room = next(r for r in m["rooms"] if r["id"] == p["room"])
            assert room["type"] != "bathroom"
            kitchen_people += room["type"] == "kitchen"
        people += len(m["people"])
    for cls, k in present.items():
        assert 0.5 * n <= k <= 0.9 * n, (cls, k)  # about 70 % present
    assert people == n  # every apartment has an operator
    assert kitchen_people <= 0.1 * n  # the kitchen is the last resort

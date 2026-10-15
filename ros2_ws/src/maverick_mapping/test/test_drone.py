import json
from pathlib import Path

import pytest
from maverick_mapping.drone import DRONES, drone_config

ROOT = Path(__file__).resolve().parents[4]


def test_default_is_small(monkeypatch):
    monkeypatch.delenv("MAVERICK_DRONE", raising=False)
    assert drone_config().name == "small"
    monkeypatch.setenv("MAVERICK_DRONE", "x500")
    assert drone_config().name == "x500"
    with pytest.raises(ValueError):
        drone_config("nope")


def test_x500_planner_keeps_the_tuned_values():
    p = DRONES["x500"].planner()
    assert (p.robot_radius, p.margin, p.tight_radius, p.tight_fallbacks) == (
        0.39,
        0.22,
        0.46,
        (0.43, 0.40),
    )


def test_small_planner_and_door_fit():
    d = DRONES["small"]
    p = d.planner()
    assert p.robot_radius == 0.15 and p.tight_radius == pytest.approx(0.22)
    # the narrowest small-profile door (0.8 m) is passable as a squeeze, not only at the hard radius
    assert 0.8 / 2 > p.tight_radius and 0.8 / 2 > p.robot_radius + p.margin - 0.06


@pytest.mark.parametrize("name", DRONES)
def test_camera_forward_matches_mount_json(name):
    d = DRONES[name]
    j = json.loads((ROOT / f"sim/models/{d.mount_dir}/camera_mount.json").read_text())
    assert j["xyz_flu"][0] == d.camera_forward_m

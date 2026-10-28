"""Collisions of one h2_text run (numpy; runs in the tools container like referee.s3_metrics).

  python3 -m referee.s4c_tools <raw.json>   -> prints one JSON line {"collisions": n, ...}

Input: the apartment SDF and manifest, the ground-truth track (world frame) and the time window
of the mission. Same body model and obstacle test as the S3 exploration check.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

from referee.s3_frames import spawn_origin, world_to_map
from referee.s3_metrics import collisions, load_scene_map_frame


def h2_collisions(raw: dict) -> dict:
    scene = load_scene_map_frame(raw["sdf"], raw["manifest"], raw.get("ground_z", 0.0))
    origin = spawn_origin({"spawn": {"pose": raw["spawn_pose"]}}, raw.get("ground_z", 0.0))
    track = np.array(raw["track"], float).reshape(-1, 4)
    sel = track[(track[:, 0] >= raw["t0"]) & (track[:, 0] <= raw["t1"])]
    in_map = np.array([(t, *world_to_map((x, y, z), origin)) for t, x, y, z in sel]).reshape(-1, 4)
    return collisions(scene, in_map)


def main(argv: list[str] | None = None) -> int:
    raw = json.loads(Path((argv or sys.argv[1:])[0]).read_text())
    print(json.dumps(h2_collisions(raw)))
    return 0


if __name__ == "__main__":
    sys.exit(main())

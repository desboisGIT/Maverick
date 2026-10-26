"""Ground-truth metrics of an S3 exploration (numpy; runs in the tools container, not on the host).

  python3 -m referee.s3_metrics <run dir>/raw-<n>.json     -> prints one JSON line

Input: the raw data the runner collected (ground-truth track in the world frame, map dumps of the
drone's own VoxelMap in the map frame, the worldgen SDF and manifest). The scene is shifted into
the map frame (referee.s3_frames) so the metrics of maverick_mapping.metrics apply unchanged:
C1 coverage and surface error, C6 frontier recall on the snapshots taken during the run.
"""

from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
for _p in (ROOT / "ros2_ws/src/maverick_mapping", ROOT / "sim/worldgen", ROOT / "sim/referee"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import numpy as np  # noqa: E402
from maverick_mapping.map_io import load_map  # noqa: E402
from maverick_mapping.metrics import GroundTruth, c6_area_recall  # noqa: E402
from maverick_mapping.synthetic import Scene  # noqa: E402

from referee.s3_frames import Vec3, spawn_origin, world_to_map  # noqa: E402
from referee.s3_verdicts import BODY_RADIUS_M  # noqa: E402

# Obstacle band of the 2D grid (mapping_node default). The height estimate follows the
# rangefinder, so anything taller than 0.35 m under the drone lifts it: it is an obstacle.
BAND = (0.35, 1.6)
BODY_HALF_HEIGHT = 0.12
MIN_FLIGHT_Z = 0.4  # track samples below this are take-off / landing, not flight


def shift_scene(scene: Scene, origin: Vec3) -> Scene:
    """The scene in the map frame (every coordinate minus the spawn origin)."""
    off = np.asarray(origin, float)
    s = copy.copy(scene)
    s.aabb_lo = scene.aabb_lo - off
    s.aabb_hi = scene.aabb_hi - off
    s.obb_c = scene.obb_c - off
    m = copy.deepcopy(scene.manifest)
    m["footprint"] = [[x - off[0], y - off[1]] for x, y in m["footprint"]]
    s.manifest = m
    s.spawn = (scene.spawn[0] - off[0], scene.spawn[1] - off[1], scene.spawn[2])
    return s


def load_scene_map_frame(sdf_path: str, manifest_path: str, ground_z: float = 0.0) -> Scene:
    manifest = json.loads(Path(manifest_path).read_text())
    scene = Scene.from_sdf(Path(sdf_path).read_text(), manifest)
    return shift_scene(scene, spawn_origin(manifest, ground_z))


def collisions(scene_map: Scene, track_map: np.ndarray, radius: float = BODY_RADIUS_M) -> dict:
    """Count entries of the drone body (vertical cylinder) into obstacles, from the GT track.

    `track_map`: rows (t, x, y, z) in the map frame. Samples below MIN_FLIGHT_Z are ignored (on
    the ground the floor slab is within reach). A collision is one entry below `radius`.
    """
    count, inside, min_clear, events = 0, False, float("inf"), []
    for t, x, y, z in track_map:
        if z < MIN_FLIGHT_Z:
            inside = False
            continue
        c = scene_map.horizontal_clearance(np.array([x, y, z]), BODY_HALF_HEIGHT)
        min_clear = min(min_clear, c)
        if c < radius:
            if not inside:
                count += 1
                events.append(
                    {"t": round(float(t), 2), "x": round(float(x), 2), "y": round(float(y), 2)}
                )
            inside = True
        else:
            inside = False
    return {
        "collisions": count,
        "min_clearance_m": None if min_clear == float("inf") else round(float(min_clear), 3),
        "collision_at": events[:10],
    }


def explore_metrics(raw: dict) -> dict:
    scene = load_scene_map_frame(raw["sdf"], raw["manifest"], raw.get("ground_z", 0.0))
    gt = GroundTruth(scene, 0.1, (0.3, 2.0), BAND)
    track = np.array(raw["track"], float).reshape(-1, 4)  # t, x, y, z in the world frame
    origin = spawn_origin({"spawn": {"pose": raw["spawn_pose"]}}, raw.get("ground_z", 0.0))
    t0, t1 = raw["t_explore_start"], raw["t_explore_end"]
    sel = track[(track[:, 0] >= t0) & (track[:, 0] <= t1)]
    in_map = np.array([(t, *world_to_map((x, y, z), origin)) for t, x, y, z in sel]).reshape(-1, 4)
    out = collisions(scene, in_map)

    final = load_map(raw["final_map"])
    out["coverage"] = round(gt.coverage(final), 4)
    surf = gt.surface_error(final)
    out["surface_err_mean_m"] = round(surf["mean"], 4)
    out["surface_err_p95_m"] = round(surf["p95"], 4)
    out["reachable_volume_m3"] = round(gt.volume_m3, 1)
    out["map_frames"] = final.frames

    regions = detected = 0
    details = []
    cov_t = []
    for snap in raw.get("snapshots", []):
        vm = load_map(snap["path"])
        grid, res, org = vm.project_2d(*BAND)
        rec = gt.frontier_recall(grid, res, org)
        regions += rec["regions"]
        detected += rec["detected"]
        details.append(rec["detail"])
        cov_t.append((round(snap["t"] - t0, 1), round(gt.coverage(vm), 3)))
    out["c6_regions"], out["c6_detected"] = regions, detected
    # C6 rule of 2026-11-12: area-weighted, drone-sized regions (maverick_mapping.metrics); the
    # original count rule stays next to it as c6_recall_v1 (docs/s3-plan.md, decisions)
    out["c6_recall"] = round(c6_area_recall(details), 4)
    out["c6_recall_v1"] = round(detected / regions, 4) if regions else 1.0
    out["coverage_over_time"] = cov_t
    out["n_snapshots"] = len(raw.get("snapshots", []))
    return out


def main(argv: list[str] | None = None) -> int:
    raw = json.loads(Path((argv or sys.argv[1:])[0]).read_text())
    print(json.dumps(explore_metrics(raw), allow_nan=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())

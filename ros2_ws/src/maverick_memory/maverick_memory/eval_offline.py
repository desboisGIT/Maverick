"""Offline scene-graph evaluation: explore a generated apartment, detect, segment, type, query.

    python3 -m maverick_memory.eval_offline --seed 1000000 --batch 10

Runs maverick_mapping's offline exploration with a simulated detector (oracle instance labels from
the manifest furniture boxes, plus misses / false positives / box jitter), lifts the detections
with the noisy depth into the object store, segments the final 2D map into rooms, types them, and
scores everything against the worldgen manifest. Oracle-quality perception: not a detector score.
Prints one JSON line per seed and a summary line (--batch).
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass

import numpy as np
from maverick_mapping import explore_offline as eo
from maverick_mapping.drone import drone_config
from maverick_mapping.synthetic import CameraSim, Scene

from .scene_graph import SceneGraph, SceneGraphConfig
from .sim_detector import DetectorNoise, SimDetector

FRAME_DT = 0.3


@dataclass
class EvalConfig:
    budget: float = 360.0
    band: tuple[float, float] = (1.0, 1.5)
    box_only: bool = False
    clean_detector: bool = False  # no misses / false positives / jitter
    obj_tol: float = 0.5  # m, object position error for a match
    room_iou: float = 0.5
    c2_clear: float = 1.0  # m, required distance of the goal from the GT room edge


def points_in_polygon(px: np.ndarray, py: np.ndarray, poly: list) -> np.ndarray:
    inside = np.zeros(px.shape, bool)
    n = len(poly)
    for k in range(n):
        x0, y0 = poly[k]
        x1, y1 = poly[(k + 1) % n]
        if y0 == y1:
            continue
        cond = (y0 > py) != (y1 > py)
        xi = x0 + (py - y0) * (x1 - x0) / (y1 - y0)
        inside ^= cond & (px < xi)
    return inside


def polygon_edge_distance(p: tuple[float, float], poly: list) -> float:
    best = np.inf
    n = len(poly)
    for k in range(n):
        a, b = np.array(poly[k], float), np.array(poly[(k + 1) % n], float)
        ab = b - a
        t = np.clip(np.dot(np.array(p) - a, ab) / max(float(ab @ ab), 1e-12), 0.0, 1.0)
        best = min(best, float(np.linalg.norm(np.array(p) - (a + t * ab))))
    return best


def run_seed(seed: int, ec: EvalConfig | None = None) -> dict:
    ec = ec or EvalConfig()
    t0 = time.perf_counter()
    drone = drone_config(None)  # MAVERICK_DRONE, default small
    scene = Scene.from_seed(seed, drone=drone.name)
    man = scene.manifest
    sg = SceneGraph(
        SceneGraphConfig(
            robot_clear=drone.goal_clear,
            connect_clear=drone.connect_clear,
            wall_clear=drone.wall_clear,
        )
    )
    state = {"det": None, "n": 0, "det_s": 0.0, "n_det": 0, "n_fp": 0}

    class RecordingCam(CameraSim):
        def render(self, scene_, t, r):
            z = super().render(scene_, t, r)
            if state["det"] is None:
                noise = DetectorNoise(use_mask=not ec.box_only)
                if ec.clean_detector:
                    noise.miss_rate = noise.fp_rate = noise.box_jitter = 0.0
                    noise.mask_dropout = 0.0
                state["det"] = SimDetector(man, self.intr, noise, seed=seed + 7)
            c0 = time.perf_counter()
            dets, truth = state["det"].detect(z, t, r)
            sg.add_detections(dets, z, self.intr, t, r, now=state["n"] * FRAME_DT)
            state["n"] += 1
            state["n_det"] += len(dets)
            state["n_fp"] += sum(1 for x in truth if x == "fp")
            state["det_s"] += time.perf_counter() - c0
            return z

    orig = eo.CameraSim
    eo.CameraSim = RecordingCam
    try:
        cfg = eo.ExploreConfig(budget=ec.budget, plan_band=ec.band, seed=seed)
        res = eo.run_exploration(scene, cfg)
    finally:
        eo.CameraSim = orig

    r0 = time.perf_counter()
    grid, g_res, g_org = res.vmap.project_2d(ec.band[0], ec.band[1])
    sg.update_rooms(grid, g_res, g_org)
    seg_s = time.perf_counter() - r0
    m = score(man, sg, grid, g_res, g_org, ec)
    m.update(
        seed=seed,
        flat=man.get("flat_type"),
        coverage=res.metrics["coverage"],
        collisions=res.metrics["collisions"],
        frames=state["n"],
        detections=state["n_det"],
        fp_detections=state["n_fp"],
        explore_s=round(res.metrics["wall_s"], 1),
        memory_s=round(state["det_s"], 1),
        segment_s=round(seg_s, 3),
        wall_s=round(time.perf_counter() - t0, 1),
    )
    return m


def score(man: dict, sg: SceneGraph, grid, res: float, org, ec: EvalConfig) -> dict:
    nx, ny = grid.shape
    xs = org[0] + (np.arange(nx) + 0.5) * res
    ys = org[1] + (np.arange(ny) + 0.5) * res
    px, py = np.meshgrid(xs, ys, indexing="ij")
    rooms = sg.room_list()
    # --- rooms: each GT room matched by IoU >= threshold ---
    matched, typed_ok = 0, 0
    detail = []
    for gr in man["rooms"]:
        g = points_in_polygon(px, py, gr["polygon"])
        best, best_i = 0.0, None
        for r in rooms:
            inter = int((g & r.region.mask).sum())
            iou = inter / max(int((g | r.region.mask).sum()), 1)
            if iou > best:
                best, best_i = iou, r
        ok = best >= ec.room_iou
        matched += ok
        t_ok = ok and best_i.type == gr["type"]
        typed_ok += t_ok
        detail.append([gr["id"], gr["type"], best_i.type if ok else None, round(best, 2)])
    n_gt = len(man["rooms"])
    # --- objects: furniture recall / precision at position error < tol ---
    objs = sg.objects()
    gt = [(f["class"], 0.5 * (np.array(f["bbox"][0]) + np.array(f["bbox"][1]))) for f in
          man["furniture"]]  # fmt: skip
    pairs = []
    for gi, (lab, c) in enumerate(gt):
        for oi, o in enumerate(objs):
            if o.label == lab:
                d = float(np.linalg.norm(o.position - c))
                if d < ec.obj_tol:
                    pairs.append((d, gi, oi))
    used_g, used_o = set(), set()
    errs = []
    for d, gi, oi in sorted(pairs):
        if gi not in used_g and oi not in used_o:
            used_g.add(gi)
            used_o.add(oi)
            errs.append(d)
    non_rug = [i for i, (lab, _) in enumerate(gt) if lab != "rug"]
    # --- C2: "go to the kitchen" ---
    c2 = {"c2_inside": False, "c2_clear": False, "c2_pass": False, "c2_goal": None}
    kit = next((r for r in man["rooms"] if r["type"] == "kitchen"), None)
    sx, sy = man["spawn"]["pose"][0], man["spawn"]["pose"][1]
    if kit is not None:
        g = points_in_polygon(px, py, kit["polygon"])
        edge = np.array([polygon_edge_distance((x, y), kit["polygon"]) for x, y in
                         zip(px[g], py[g], strict=True)])  # fmt: skip
        gt_max = float(edge.max()) if len(edge) else 0.0
        need = min(ec.c2_clear, 0.6 * gt_max)  # docs/s3-plan.md C2 rule
        out = sg.goal_for_type("kitchen", start=(sx, sy))
        c2["c2_gt_inscribed_m"] = round(gt_max, 2)
        if out is not None:
            room, goal = out
            inside = bool(points_in_polygon(np.array([goal[0]]), np.array([goal[1]]),
                                            kit["polygon"])[0])  # fmt: skip
            clr = polygon_edge_distance(goal, kit["polygon"]) if inside else 0.0
            c2.update(
                c2_goal=[round(goal[0], 2), round(goal[1], 2)],
                c2_inside=inside,
                c2_clear=inside and clr >= ec.c2_clear,
                c2_clearance_m=round(clr, 2),
                c2_pass=inside and clr >= need,
            )
    return {
        "rooms_gt": n_gt,
        "rooms_pred": len(rooms),
        "rooms_matched": matched,
        "typing_ok": typed_ok,
        "typing_acc_matched": round(typed_ok / matched, 3) if matched else 0.0,
        "typing_acc_all": round(typed_ok / n_gt, 3),
        "objects_gt": len(gt),
        "objects_pred": len(objs),
        "obj_recall": round(len(used_g) / max(len(gt), 1), 3),
        "obj_precision": round(len(used_o) / max(len(objs), 1), 3),
        "obj_recall_non_rug": round(len(used_g & set(non_rug)) / max(len(non_rug), 1), 3),
        "obj_pos_err_mean_m": round(float(np.mean(errs)), 3) if errs else None,
        "rooms": detail,
        **c2,
    }


def summarise(rs: list[dict]) -> dict:
    def tot(k):
        return sum(r[k] for r in rs)

    return {
        "summary": True,
        "n": len(rs),
        "rooms_matched": f"{tot('rooms_matched')}/{tot('rooms_gt')}",
        "rooms_pred": tot("rooms_pred"),
        "typing_acc_matched": round(tot("typing_ok") / max(tot("rooms_matched"), 1), 3),
        "typing_acc_all": round(tot("typing_ok") / max(tot("rooms_gt"), 1), 3),
        "obj_recall": round(
            sum(r["obj_recall"] * r["objects_gt"] for r in rs) / max(tot("objects_gt"), 1), 3
        ),
        "obj_precision": round(
            sum(r["obj_precision"] * r["objects_pred"] for r in rs) / max(tot("objects_pred"), 1), 3
        ),
        "c2_inside": sum(r["c2_inside"] for r in rs),
        "c2_clear_1m": sum(r["c2_clear"] for r in rs),
        "c2_pass": sum(r["c2_pass"] for r in rs),
        "collisions_total": tot("collisions"),
        "coverage_mean": round(float(np.mean([r["coverage"] for r in rs])), 3),
        "wall_s_mean": round(float(np.mean([r["wall_s"] for r in rs])), 1),
        "memory_s_mean": round(float(np.mean([r["memory_s"] for r in rs])), 1),
        "segment_s_mean": round(float(np.mean([r["segment_s"] for r in rs])), 3),
    }


def _one(args) -> dict:
    seed, ec = args
    return run_seed(seed, ec)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seed", type=int, default=1000000)
    ap.add_argument("--batch", type=int, default=0, help="run N consecutive seeds from --seed")
    ap.add_argument("--budget", type=float, default=360.0)
    ap.add_argument("--box-only", action="store_true", help="detector gives boxes, no masks")
    ap.add_argument("--clean-detector", action="store_true", help="no misses / FPs / jitter")
    ap.add_argument("--jobs", type=int, default=1)
    a = ap.parse_args(argv)
    ec = EvalConfig(budget=a.budget, box_only=a.box_only, clean_detector=a.clean_detector)
    if a.batch <= 0:
        print(json.dumps(run_seed(a.seed, ec)))
        return 0
    jobs = [(a.seed + i, ec) for i in range(a.batch)]
    if a.jobs > 1:
        from multiprocessing import Pool

        with Pool(a.jobs) as p:
            results = p.map(_one, jobs)
        for r in results:
            print(json.dumps(r))
    else:
        results = []
        for j in jobs:
            r = _one(j)
            print(json.dumps(r), flush=True)
            results.append(r)
    print(json.dumps(summarise(results)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

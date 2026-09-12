"""Explore a generated apartment offline (no Gazebo) and score it.

PYTHONPATH=sim/worldgen python3 -m maverick_mapping.explore_offline --seed 1000003 --budget 360
PYTHONPATH=sim/worldgen python3 -m maverick_mapping.explore_offline --seed 1000000 --batch 10
"""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass, field

import numpy as np

from .frontiers import detect_frontiers
from .metrics import GroundTruth
from .planner import ExplorationPlanner, PlannerConfig
from .pngwrite import write_png
from .synthetic import CameraSim, KinematicDrone, Scene
from .voxel_map import VoxelMap


@dataclass
class ExploreConfig:
    budget: float = 360.0
    res: float = 0.1
    altitude: float = 1.2
    max_range: float = 5.0
    plan_band: tuple[float, float] | None = None  # obstacle band; default altitude +- 0.4 m
    band_half: float = 0.4  # vertical clearance: furniture below altitude - 0.4 is overflown
    free_z: float | None = None  # if set, free cells need an observation near this height
    frame_dt: float = 0.3  # depth frame period (simulated s)
    sim_dt: float = 0.1
    replan_period: float = 1.5
    scan_every_m: float = 4.0  # 360 degree yaw scan when this far from the last one
    cam_width: int = 80
    cam_height: int = 60
    body_radius: float | None = None  # default from manifest drone_profile (x500 0.39, small 0.15)
    near_miss_margin: float = 0.05  # also count entries inside body_radius + this
    body_half_height: float = 0.12
    arrive_radius: float = 0.1  # waypoint reached when the vehicle is this close
    track_margin: float = 0.05  # extra inflation on top of body radius + res / 2
    drift_sigma: float = 0.0  # m / sqrt(s) random walk of the position estimate (optional)
    depth_noise: bool = True
    snapshot_period: float = 20.0
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    seed: int = 0
    verbose: bool = False


@dataclass
class ExploreResult:
    metrics: dict
    vmap: VoxelMap
    trail: list
    frontiers: list
    goals: list
    scene: Scene


def _spin_to(d: KinematicDrone, target: float, tick) -> None:
    pass


def run_exploration(scene: Scene, cfg: ExploreConfig | None = None) -> ExploreResult:
    cfg = cfg or ExploreConfig()
    t0 = time.perf_counter()
    cfg.planner.altitude = cfg.altitude
    prof = scene.manifest.get("drone_profile", "x500")
    if cfg.body_radius is None:
        cfg.body_radius = {"x500": 0.39, "small": 0.15}.get(prof, 0.39)
    if cfg.plan_band is None:
        cfg.plan_band = (cfg.altitude - cfg.band_half, cfg.altitude + cfg.band_half)
    cfg.planner.band = cfg.plan_band
    cfg.planner.robot_radius = cfg.body_radius
    cfg.planner.margin = cfg.res / 2 + cfg.track_margin  # voxel quantisation + tracking residual
    sx, sy, syaw = scene.spawn
    drone = KinematicDrone(
        (sx, sy, cfg.altitude), syaw, drift_sigma=cfg.drift_sigma, seed=cfg.seed + 1
    )
    cam = CameraSim(cfg.cam_width, cfg.cam_height, seed=cfg.seed + 2)
    if not cfg.depth_noise:
        cam.noise_base = cam.noise_quad = 0.0
    lo, hi = scene.bounds
    vmap = VoxelMap(cfg.res, max_range=cfg.max_range)
    gt = GroundTruth(scene, cfg.res, (0.3, 2.0), cfg.plan_band)
    planner = ExplorationPlanner(cfg.planner)
    band = cfg.plan_band

    st = {
        "t": 0.0,
        "next_frame": 0.0,
        "collisions": 0,
        "in_col": False,
        "near_misses": 0,
        "in_nm": False,
        "render_s": 0.0,
    }
    trail: list[tuple[float, float]] = []
    goals: list[tuple[float, float]] = []
    recall = {"regions": 0, "detected": 0}
    cov_t: list[tuple[float, float]] = []
    next_snap = [0.0]
    min_clear = [9.0]
    col_at: list = []

    def log(*a) -> None:
        if cfg.verbose:
            print(f"[{st['t']:6.1f}s]", *a, flush=True)

    def sense() -> None:
        if st["t"] + 1e-9 >= st["next_frame"]:
            st["next_frame"] += cfg.frame_dt
            r0 = time.perf_counter()
            t_true, r_true = drone.camera_pose(est=False)
            depth = cam.render(scene, t_true, r_true)
            t_est, r_est = drone.camera_pose(est=True)
            vmap.integrate_depth(depth, cam.intr, t_est, r_est)
            st["render_s"] += time.perf_counter() - r0

    def snapshot() -> None:
        if st["t"] + 1e-9 < next_snap[0]:
            return
        next_snap[0] += cfg.snapshot_period
        grid, res, org = vmap.project_2d(band[0], band[1], free_z=cfg.free_z)
        rec = gt.frontier_recall(grid, res, org, cfg.planner.min_cluster)
        recall["regions"] += rec["regions"]
        recall["detected"] += rec["detected"]
        cov_t.append((st["t"], gt.coverage(vmap)))

    def tick(move_fn) -> None:
        move_fn()
        st["t"] += cfg.sim_dt
        c = scene.horizontal_clearance(drone.pos, cfg.body_half_height)
        min_clear[0] = min(min_clear[0], c)
        if c < cfg.body_radius:
            if not st["in_col"]:
                st["collisions"] += 1
                col_at.append(
                    (
                        round(st["t"], 1),
                        round(float(drone.pos[0]), 2),
                        round(float(drone.pos[1]), 2),
                        round(float(drone.vel[0]), 2),
                        round(float(drone.vel[1]), 2),
                        scene.nearest_box(drone.pos),
                    )
                )
            st["in_col"] = True
        else:
            st["in_col"] = False
        if c < cfg.body_radius + cfg.near_miss_margin:
            if not st["in_nm"]:
                st["near_misses"] += 1
            st["in_nm"] = True
        else:
            st["in_nm"] = False
        if int(st["t"] / cfg.sim_dt) % 5 == 0:
            trail.append((float(drone.pos[0]), float(drone.pos[1])))
        sense()
        snapshot()

    def spin(angle: float) -> None:
        total = 0.0
        start = drone.yaw
        sign = 1.0 if angle >= 0 else -1.0
        while abs(total) < abs(angle) - 1e-6 and st["t"] < cfg.budget:
            step = min(drone.yaw_rate * cfg.sim_dt, abs(angle) - abs(total))
            total += sign * step

            def rot(y=start + total):
                drone.yaw = y
                drone.hold(cfg.sim_dt)

            tick(rot)

    def turn_to(yaw: float) -> None:
        while st["t"] < cfg.budget:
            err = []

            def rot(err=err):
                err.append(drone.turn(yaw, cfg.sim_dt))
                drone.hold(cfg.sim_dt)

            tick(rot)
            if abs(err[0]) < 0.05:
                break

    def do_plan():
        grid, res, org = vmap.project_2d(band[0], band[1], free_z=cfg.free_z)
        return planner.plan(grid, res, org, (drone.est[0], drone.est[1])), (grid, res, org)

    def fly(wps: list, last_plan_t: float):
        """Fly the waypoint list like the flight client: straight setpoint segments at <= 1 m/s,
        one waypoint at a time (the next segment starts once the vehicle is at the waypoint).
        Returns (arrived, new_plan or None)."""
        queue = list(wps)
        drone.sp = drone.est[:2].copy()
        while queue and st["t"] < cfg.budget:
            tgt = np.array(queue[0][:2])
            delta = tgt - drone.est[:2]
            if float(np.linalg.norm(delta)) < cfg.arrive_radius:
                queue.pop(0)
                continue
            dsp = tgt - drone.sp
            heading = math.atan2(dsp[1], dsp[0]) if np.linalg.norm(dsp) > 1e-6 else drone.yaw
            e = (heading - drone.yaw + math.pi) % (2 * math.pi) - math.pi
            cap = drone.vmax * max(0.0, 1.0 - abs(e) / math.radians(50.0))

            def mv(h=heading, c=cap, tg=tgt):
                drone.turn(h, cfg.sim_dt)
                drone.track(tg, c, cfg.sim_dt)

            tick(mv)
            if st["t"] - last_plan_t >= cfg.replan_period:
                res, _ = do_plan()
                if res.status == "done":
                    return False, res
                if res.goal is not None and (
                    planner.goal_centroid is None or math.dist(res.goal.centroid, goal_c[0]) > 1.2
                ):
                    return False, res
                queue = list(res.waypoints)
                drone.sp = drone.est[:2].copy()
                last_plan_t = st["t"]
        return not queue, None

    goal_c = [(sx, sy)]
    finish = "budget"
    last_scan_odo = -1e9
    stuck = 0
    while st["t"] < cfg.budget:
        if drone.odometer - last_scan_odo >= cfg.scan_every_m:
            spin(2 * math.pi)
            last_scan_odo = drone.odometer
            if st["t"] >= cfg.budget:
                break
        res, (grid, g_res, g_org) = do_plan()
        if res.status == "done":
            finish = res.reason
            break
        if math.dist(res.goal.viewpoint_xy, (drone.est[0], drone.est[1])) < 0.35:
            # already standing at the chosen viewpoint and the frontier is still there
            log(
                "viewpoint failed",
                res.goal.viewpoint_xy,
                "frontier",
                res.goal.centroid,
                res.goal.size,
            )
            planner.mark_viewpoint_failed(res.goal.viewpoint_xy)
            continue
        goal_c[0] = res.goal.centroid
        goals.append(res.goal.viewpoint_xy)
        log(
            "goal vp",
            res.goal.viewpoint_xy,
            "cl",
            res.goal.centroid,
            "n",
            res.goal.size,
            "wps",
            len(res.waypoints),
            "from",
            drone.est[:2].round(2),
        )
        arrived, newplan = fly(res.waypoints, st["t"])
        while newplan is not None and newplan.status == "goal" and st["t"] < cfg.budget:
            goal_c[0] = newplan.goal.centroid
            goals.append(newplan.goal.viewpoint_xy)
            arrived, newplan = fly(newplan.waypoints, st["t"])
        if newplan is not None and newplan.status == "done":
            finish = newplan.reason
            break
        if st["t"] >= cfg.budget:
            break
        # arrived at a viewpoint: look at the frontier (or scan), then check it was resolved
        cx, cy = goal_c[0]
        face = math.atan2(cy - drone.est[1], cx - drone.est[0])
        turn_to(face)
        if drone.odometer - last_scan_odo >= cfg.scan_every_m:
            spin(2 * math.pi)
            last_scan_odo = drone.odometer
        grid, g_res, g_org = vmap.project_2d(band[0], band[1], free_z=cfg.free_z)
        near = [
            f
            for f in detect_frontiers(grid, g_res, g_org, cfg.planner.min_cluster)
            if np.min(
                np.hypot(
                    g_org[0] + (f.cells[:, 0] + 0.5) * g_res - drone.est[0],
                    g_org[1] + (f.cells[:, 1] + 0.5) * g_res - drone.est[1],
                )
            )
            < 0.9
        ]
        log("arrived; frontiers near:", len(near))
        if near:
            planner.mark_failed((float(drone.est[0]), float(drone.est[1])))
            stuck += 1
    snapshot_force = next_snap
    snapshot_force[0] = 0.0
    snapshot()

    surf = gt.surface_error(vmap)
    cov = gt.coverage(vmap)
    t90 = next((t for t, c in cov_t if c >= 0.9), None)
    grid, g_res, g_org = vmap.project_2d(band[0], band[1], free_z=cfg.free_z)
    final_fronts = detect_frontiers(grid, g_res, g_org, cfg.planner.min_cluster)
    metrics = {
        "seed": scene.manifest.get("seed"),
        "flat": scene.manifest.get("flat_type"),
        "coverage": round(cov, 4),
        "time_s": round(st["t"], 1),
        "time_to_90_s": None if t90 is None else round(t90, 1),
        "finish": finish,
        "collisions": st["collisions"],
        "min_clearance_m": round(min_clear[0], 3),
        "surface_err_mean_m": round(surf["mean"], 4),
        "surface_err_p95_m": round(surf["p95"], 4),
        "c6_regions": recall["regions"],
        "c6_detected": recall["detected"],
        "c6_recall": round(recall["detected"] / recall["regions"], 4) if recall["regions"] else 1.0,
        "near_misses": st["near_misses"],
        "collision_at": col_at,
        "distance_m": round(drone.odometer, 1),
        "reachable_volume_m3": round(gt.volume_m3, 1),
        "wall_s": round(time.perf_counter() - t0, 1),
        "render_integrate_s": round(st["render_s"], 1),
    }
    return ExploreResult(metrics, vmap, trail, final_fronts, goals, scene)


def render_png(res: ExploreResult, path: str, cfg: ExploreConfig | None = None, ppm: int = 6):
    """Top-down PNG: map (grey unknown, white free, black occupied), GT walls, path, frontiers."""
    cfg = cfg or ExploreConfig()
    vm = res.vmap
    grid, r, org = vm.project_2d(*cfg.plan_band, free_z=cfg.free_z)
    scale = max(1, int(ppm))
    img = np.full(grid.shape + (3,), 128, np.uint8)
    img[grid == 0] = (255, 255, 255)
    img[grid == 100] = (20, 20, 20)
    for f in res.frontiers:
        img[f.cells[:, 0], f.cells[:, 1]] = (230, 40, 40)
    img = img.repeat(scale, axis=0).repeat(scale, axis=1)
    px = scale / r

    def put(x, y, col, rad=1):
        i = int((x - org[0]) * px)
        j = int((y - org[1]) * px)
        img[max(i - rad, 0) : i + rad + 1, max(j - rad, 0) : j + rad + 1] = col

    for x, y in res.trail:
        put(x, y, (40, 90, 230), 1)
    for x, y in res.goals:
        put(x, y, (30, 170, 60), 3)
    sx, sy, _ = res.scene.spawn
    put(sx, sy, (255, 140, 0), 4)
    # x is the first array axis (rows); flip so +x points up and +y right
    write_png(path, img[::-1])


def _one(args) -> dict:
    seed, cfg = args
    scene = Scene.from_seed(seed)
    cfg.seed = seed
    return run_exploration(scene, cfg).metrics


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seed", type=int, default=1000000)
    ap.add_argument("--batch", type=int, default=0, help="run N consecutive seeds from --seed")
    ap.add_argument("--budget", type=float, default=360.0)
    ap.add_argument("--plot", help="write a top-down PNG (single seed only)")
    ap.add_argument("--res", type=float, default=0.1)
    ap.add_argument("--drift", type=float, default=0.0, help="position drift, m per sqrt(s)")
    ap.add_argument("--no-noise", action="store_true")
    ap.add_argument(
        "--band",
        type=float,
        nargs=2,
        default=(1.0, 1.5),
        help="obstacle band z0 z1 (spec flight band: 0.3 2.0)",
    )
    ap.add_argument("--jobs", type=int, default=1)
    a = ap.parse_args(argv)

    def make_cfg() -> ExploreConfig:
        return ExploreConfig(
            budget=a.budget, res=a.res, drift_sigma=0.0 if a.no_noise else a.drift,
            depth_noise=not a.no_noise, plan_band=tuple(a.band) if a.band else None,
        )  # fmt: skip

    if a.batch <= 0:
        cfg = make_cfg()
        cfg.seed = a.seed
        res = run_exploration(Scene.from_seed(a.seed), cfg)
        print(json.dumps(res.metrics))
        if a.plot:
            render_png(res, a.plot, cfg)
        return 0
    seeds = [a.seed + i for i in range(a.batch)]
    jobs = [(s, make_cfg()) for s in seeds]
    if a.jobs > 1:
        from multiprocessing import Pool

        with Pool(a.jobs) as p:
            results = p.map(_one, jobs)
        for m in results:
            print(json.dumps(m))
    else:
        results = []
        for j in jobs:
            m = _one(j)
            print(json.dumps(m), flush=True)
            results.append(m)
    ok = [r for r in results if r["coverage"] >= 0.9 and r["collisions"] == 0]
    summary = {
        "summary": True,
        "n": len(results),
        "pass_c1": len(ok),
        "coverage_mean": round(float(np.mean([r["coverage"] for r in results])), 4),
        "coverage_min": min(r["coverage"] for r in results),
        "collisions_total": sum(r["collisions"] for r in results),
        "c6_recall": round(
            sum(r["c6_detected"] for r in results) / max(1, sum(r["c6_regions"] for r in results)),
            4,
        ),
        "surface_err_mean_m": round(float(np.mean([r["surface_err_mean_m"] for r in results])), 4),
        "wall_s_mean": round(float(np.mean([r["wall_s"] for r in results])), 1),
    }
    print(json.dumps(summary))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

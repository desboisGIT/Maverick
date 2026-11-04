"""Scenarios (explore, goto_room, search) and their scoring with the referee's own code.

A result row has the format of `referee.s3` / `referee.s4` (`results.jsonl`) plus `sim: "kinsim"`.
Metrics come from `maverick_mapping.metrics` (coverage, surface error, C6) and
`referee.s3_metrics.collisions` / `referee.s3_verdicts` (verdicts, goto geometry): kinsim only
supplies the map, the ground-truth track and the skill outcome.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np
from maverick_mapping.metrics import GroundTruth, c6_area_recall
from referee.s3_metrics import BAND, collisions
from referee.s3_verdicts import (
    estimate_error,
    explore_pass_c6_v1,
    goto_metrics,
    label_reason,
    old_rule_pass,
    seed_set_of,
    verdict_explore,
    verdict_goto,
)

from .features import track_features
from .reference import event_counts
from .scene import KScene
from .stack import KinStack
from .vehicle import KinParams

WALL_OVERHEAD_S = (
    4.6  # referee: request + poll latency between the skill's clock and its wall clock
)
HOVER_BEFORE_S = 10.0  # hover (map building from 2 m) before the request arrives
POST_EXPLORE_S = 10.0  # referee: poll + map dump between explore and the goto request
GOTO_TIMEOUT_S = 200.0
SETTLE_S = 2.0


@dataclass
class Scored:
    """Ground-truth scoring state of one explore (the referee's snapshots every ~30 s)."""

    gt: GroundTruth
    snaps: list[dict] = field(default_factory=list)
    next_snap: float = 0.0

    def snapshot(self, vmap, t_rel: float) -> None:
        grid, res, org = vmap.project_2d(*BAND)
        rec = self.gt.frontier_recall(grid, res, org)
        self.snaps.append({"t": t_rel, "rec": rec, "cov": self.gt.coverage(vmap)})


def _explore_metrics(st: KinStack, sc: Scored, t0: float, t1: float) -> dict:
    scene = st.scene.ref_scene
    gt = sc.gt
    track = np.array(st.track, float).reshape(-1, 4)
    sel = track[(track[:, 0] >= t0) & (track[:, 0] <= t1)]
    out = collisions(scene, sel)
    final = st.vmap
    out["coverage"] = round(gt.coverage(final), 4)
    surf = gt.surface_error(final)
    out["surface_err_mean_m"] = round(surf["mean"], 4)
    out["surface_err_p95_m"] = round(surf["p95"], 4)
    out["reachable_volume_m3"] = round(gt.volume_m3, 1)
    out["map_frames"] = final.frames
    regions = sum(s["rec"]["regions"] for s in sc.snaps)
    detected = sum(s["rec"]["detected"] for s in sc.snaps)
    out["c6_regions"], out["c6_detected"] = regions, detected
    out["c6_recall"] = round(c6_area_recall([s["rec"]["detail"] for s in sc.snaps]), 4)
    out["c6_recall_v1"] = round(detected / regions, 4) if regions else 1.0
    out["coverage_over_time"] = [(round(s["t"], 1), round(s["cov"], 3)) for s in sc.snaps]
    out["n_snapshots"] = len(sc.snaps)
    return out


def _estimate_rows(st: KinStack, t0: float, t1: float) -> dict:
    """The referee's estimate-vs-truth statistics, on kinsim's own truth and estimate streams."""
    gt = [{"t": t, "x": x, "y": y, "z": z} for t, x, y, z in st.track]
    # estimate_error works in PX4 NED rows; build them from the estimate stream (ENU -> NED)
    px4 = [
        {"k": "pos", "t": t, "x": y, "y": x, "z": -z, "valid": True} for t, x, y, z in st.est_track
    ]
    return estimate_error(gt, px4, (0.0, 0.0), t0, t1, 0.0)


def run_explore(
    seed: int,
    run: int = 0,
    params: KinParams | None = None,
    budget: float = 360.0,
    goto_room: str | None = None,
    scene: KScene | None = None,
    keep: bool = False,
    verbose: bool = False,
) -> dict:
    """Explore (and optionally goto_room) one apartment; returns the result row."""
    t_wall, t_cpu = time.perf_counter(), time.process_time()
    scene = scene or KScene.from_seed(seed)
    st = KinStack(scene, params, seed, run, with_memory=goto_room is not None)
    sc = Scored(GroundTruth(scene.ref_scene, 0.1, (0.3, 2.0), BAND))
    st.run_for(HOVER_BEFORE_S)
    st.runner.on_first_hover()
    t_req = st.t
    st.runner.request(st.t, {"skill": "explore", "budget_s": budget})
    period = st.params.snapshot_period_s
    next_snap = period
    while not st.skill_done() and st.t - t_req < budget + 90:
        st.run_for(0.5, until=st.skill_done)
        if st.t - t_req >= next_snap:
            sc.snapshot(st.vmap, st.t - t_req)
            next_snap += period
    prog = _last_progress(st, "explore")
    t_done = st.t
    # let the final state settle (the referee dumps the final map a few seconds later)
    st.run_for(WALL_OVERHEAD_S)
    sc.snapshot(st.vmap, st.t - t_req)
    explore_skill_s = _finish_time(st, "explore", t_req)
    t_end = st.t
    result: dict = {
        "explore_skill": {k: prog.get(k) for k in ("skill", "state", "progress", "message")},
        "explore_wall_s": round(explore_skill_s + WALL_OVERHEAD_S, 1),
    }
    goto_m = None
    if goto_room:
        st.run_for(POST_EXPLORE_S)
        t_g = st.t
        rej = st.runner.request(st.t, {"skill": "goto_room", "room_type": goto_room})
        if rej is None:
            while not st.skill_done() and st.t - t_g < GOTO_TIMEOUT_S:
                st.run_for(0.5, until=st.skill_done)
            gprog = _last_progress(st, "goto")
        else:
            gprog = rej
        st.run_for(SETTLE_S)
        gprog_state = gprog.get("state") if st.skill_done() else "timeout"
        t_end = st.t
        goto_m = {
            "skill_state": gprog_state,
            "skill_message": gprog.get("message"),
            "skill_succeeded": gprog_state == "succeeded",
            "world_xy": [float(st.vehicle.pos[0]), float(st.vehicle.pos[1])],
        }
    metrics = _explore_metrics(st, sc, t_req, t_end)
    metrics.update(result)
    metrics.update(_estimate_rows(st, t_req, t_end))
    v = verdict_explore(metrics)
    rec: dict = {"verdict": "pass" if v.passed else "fail", "reason": v.reason, "metrics": metrics}
    rec["c6_old_rule_pass"] = explore_pass_c6_v1(metrics)
    scenario = "goto_room" if goto_room else "explore"
    if goto_m is not None:
        gm = goto_metrics(
            scene.manifest,
            goto_room,
            tuple(goto_m["world_xy"]),
            (scene.origin[0], scene.origin[1], 0.0),
        )
        gm.update(goto_m)
        gv = verdict_goto(gm)
        rec["goto"] = {"verdict": "pass" if gv.passed else "fail", "reason": gv.reason, **gm}
        rec["goto"]["old_rule_pass"] = old_rule_pass(gm)
        rec["verdict"], rec["reason"] = rec["goto"]["verdict"], gv.reason
        rec["old_rule_pass"] = rec["goto"]["old_rule_pass"]
    rec = {
        "scenario": scenario,
        "run": run,
        "seed": seed,
        "seed_set": seed_set_of(seed),
        "localisation": "mocap",
        "sim": "kinsim",
        "duration_s": round(time.perf_counter() - t_wall, 2),
        "cpu_s": round(time.process_time() - t_cpu, 2),
        "sim_s": round(st.t, 1),
        "rtf": None,
        **rec,
    }
    rec["reason"] = label_reason(rec["reason"], "mocap")
    rec["kin"] = {
        "guard": dict(st.runner.guard_stats),
        "contacts": st.vehicle.contacts,
        "crashes": st.vehicle.crashes,
        "integrations": st.integrations,
        "profile_s": {k: round(v, 2) for k, v in st.stats.items()},
        "explore_skill_s": round(explore_skill_s, 1),
        "t_done": round(t_done - t_req, 1),
        "features": track_features(
            np.array(st.track, float).reshape(-1, 4),
            [e for e in st.events if e.get("skill") == "explore"],
            t_req,
            t_req + explore_skill_s,
        ),
        "event_counts": event_counts([e for e in st.events if e.get("skill") == "explore"]),
        "n_stops": sum(
            1 for e in st.events if e.get("skill") == "explore" and e.get("ev") == "guard_stop"
        ),
        "collision_t": [
            round(c["t"] - t_req, 1) for c in metrics.get("collision_at", []) if "t" in c
        ],
    }
    if keep:
        rec["_stack"] = st
    if verbose:
        print(rec["verdict"], rec["reason"], rec["kin"], flush=True)
    return rec


def _last_progress(st: KinStack, skill: str) -> dict:
    ps = [p for p in st.runner.progress if p.get("skill") == skill]
    return ps[-1] if ps else {}


def _finish_time(st: KinStack, skill: str, t_req: float) -> float:
    for e in st.events:
        if e.get("skill") == skill and e.get("ev") == "finish":
            return float(e["t"])
    return st.t - t_req

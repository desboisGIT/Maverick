"""Scenario `search`: explore, then search for an object (S4b), scored like `referee.s4`.

Not calibrated: there is no Gazebo reference run of the search scenario on the current code, so
only the pieces it shares with explore / goto_room carry measured fidelity (docs/kinsim.md).
The world has the worldgen extras (bananas, bread, a mannequin) and the detector sees their
visual boxes, which have no collision (small objects are not obstacles).
"""

from __future__ import annotations

import time

import numpy as np
from referee.s3_verdicts import seed_set_of
from referee.s4_verdicts import verdict_search

from .scenarios import HOVER_BEFORE_S, POST_EXPLORE_S, WALL_OVERHEAD_S, _last_progress
from .scene import KScene
from .stack import KinStack
from .vehicle import KinParams

SEARCH_TIMEOUT_S = 240.0


def run_search(
    seed: int,
    run: int = 0,
    params: KinParams | None = None,
    budget: float = 300.0,
    obj: str = "banana",
    search_budget: float = 150.0,
    keep: bool = False,
) -> dict:
    t_wall, t_cpu = time.perf_counter(), time.process_time()
    scene = KScene.from_seed(seed, extras=True)
    st = KinStack(scene, params, seed, run, with_memory=True)
    st.run_for(HOVER_BEFORE_S)
    st.runner.on_first_hover()
    t_req = st.t
    st.runner.request(st.t, {"skill": "explore", "budget_s": budget})
    while not st.skill_done() and st.t - t_req < budget + 90:
        st.run_for(0.5, until=st.skill_done)
    st.run_for(WALL_OVERHEAD_S + POST_EXPLORE_S)
    t_s = st.t
    rej = st.runner.request(st.t, {"skill": "search", "object": obj, "budget_s": search_budget})
    if rej is None:
        while not st.skill_done() and st.t - t_s < SEARCH_TIMEOUT_S:
            st.run_for(0.5, until=st.skill_done)
        prog = _last_progress(st, "search")
    else:
        prog = rej
    res = prog.get("result")
    if prog.get("state") != "succeeded" and not res:
        res = {"found": False, "reason": f"skill {prog.get('state')}: {prog.get('message')}"}
    v = verdict_search(scene.manifest, obj, res, scene.spawn_world)
    d = v.as_dict()
    d.update(skill_state=prog.get("state"), skill_message=prog.get("message"))
    track = np.array(st.track, float).reshape(-1, 4)
    from referee.s3_metrics import collisions

    col = collisions(scene.ref_scene, track[track[:, 0] >= t_req])
    row = {
        "scenario": "search",
        "run": run,
        "seed": seed,
        "seed_set": seed_set_of(seed),
        "localisation": "mocap",
        "sim": "kinsim",
        "duration_s": round(time.perf_counter() - t_wall, 2),
        "cpu_s": round(time.process_time() - t_cpu, 2),
        "sim_s": round(st.t, 1),
        "verdict": d["verdict"],
        "reason": f"[with mocap] {d['reason']}",
        "metrics": {"collisions": col["collisions"], "min_clearance_m": col["min_clearance_m"]},
        "search": {obj: d},
        "kin": {"guard": dict(st.runner.guard_stats), "contacts": st.vehicle.contacts},
    }
    if keep:
        row["_stack"] = st
    return row

"""Extract the compact Gazebo reference the calibration compares against (committed as JSON).

    python3 -m kinsim.reference --runs runs/g4-explore runs/g4b-explore \
        --out sim/kinsim/data/gazebo_explore.json

Each row is one Gazebo run on a tuning seed: verdict and metrics from `results.jsonl`, the skill
log's finish time, and the track features of `kinsim.features` computed from `raw.json`.
"""

from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import numpy as np

from .features import align_offset, track_features

KEYS = (
    "coverage", "collisions", "min_clearance_m", "c6_recall", "c6_recall_v1", "surface_err_mean_m",
    "surface_err_p95_m", "est_err_horiz_mean_m", "est_err_horiz_p95_m", "est_err_horiz_max_m",
    "n_snapshots", "explore_wall_s", "reachable_volume_m3", "map_frames",
)  # fmt: skip


def event_counts(events: list[dict]) -> dict:
    out: dict = {}
    for e in events:
        out[e.get("ev")] = out.get(e.get("ev"), 0) + 1
    return out


def extract_run(run_dir: str, row: dict) -> dict:
    d = Path(run_dir)
    raw = json.loads((d / "raw.json").read_text())
    events = [json.loads(line) for line in (d / "skill_events.jsonl").read_text().splitlines()]
    sp = raw["spawn_pose"]
    tr = np.array(raw["track"], float).reshape(-1, 4)
    tr[:, 1] -= sp[0]
    tr[:, 2] -= sp[1]
    ev = [e for e in events if e.get("skill") == "explore"]
    t_start = align_offset(tr[tr[:, 0] >= raw["t_explore_start"] - 5], ev, 0.0, 10.0)
    fin = next((e for e in ev if e.get("ev") == "finish"), None)
    skill_s = float(fin["t"]) if fin else None
    t_end = t_start + (skill_s if skill_s is not None else 360.0)
    m = row["metrics"]
    out = {
        "seed": row["seed"],
        "scenario": row["scenario"],
        "verdict": row["verdict"],
        "reason": row["reason"],
        "c6_old_rule_pass": row.get("c6_old_rule_pass"),
        "skill_s": skill_s,
        "skill_state": (m.get("explore_skill") or {}).get("state"),
        "skill_message": (m.get("explore_skill") or {}).get("message"),
        "rtf_mean": (row.get("rtf") or {}).get("mean"),
        **{k: m.get(k) for k in KEYS},
        "features": track_features(tr, ev, t_start, t_end),
        "t_offset_s": round(t_start - raw["t_explore_start"], 2),
        "event_counts": event_counts(ev),
        "n_stops": sum(1 for e in ev if e.get("ev") == "guard_stop"),
        "n_nudges": sum(1 for e in ev if e.get("ev") == "guard_nudge"),
        "collision_t": [c["t"] - t_start for c in m.get("collision_at", [])],
    }
    if "goto" in row:
        g = row["goto"]
        out["goto"] = {k: g.get(k) for k in (
            "verdict", "reason", "skill_state", "skill_message", "inside", "edge_distance_m",
            "required_m", "inscribed_m", "old_rule_pass")}  # fmt: skip
    return out


def extract(run_roots: list[str]) -> list[dict]:
    rows = []
    for root in run_roots:
        for line in Path(root, "results.jsonl").read_text().splitlines():
            r = json.loads(line)
            if r["verdict"] == "error" or r["seed"] >= 1_000_000:
                continue  # never held-out seeds (docs/seed-ledger.md)
            hits = glob.glob(f"{root}/run-{r['run']:03d}-*")
            if not hits or not Path(hits[0], "raw.json").exists():
                continue
            e = extract_run(hits[0], r)
            e["source"] = Path(root).name
            rows.append(e)
    return rows


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", nargs="+", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    rows = extract(a.runs)
    Path(a.out).write_text(json.dumps(rows, indent=1) + "\n")
    print(f"{len(rows)} runs -> {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

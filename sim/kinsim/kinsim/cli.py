"""kinsim command line: many apartments in minutes, results in the referee's format.

    python3 -m kinsim --scenario explore --seeds 20-79 --jobs 12 --out runs/kinsim-explore
    (or: just kinsim --scenario goto_room --seeds 20-39 --jobs 12)

Writes `results.jsonl` (one row per run, `sim: "kinsim"`) and `summary.json` into --out. The
numbers are tuning and screening numbers: see docs/kinsim.md, "When to trust kinsim".
"""

from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os
import sys
import time
from pathlib import Path

from . import paths as _paths  # noqa: F401
from .params_io import load_params

SCENARIOS = ("explore", "goto_room", "search")


def parse_seeds(text: str) -> list[int]:
    seeds: list[int] = []
    for part in text.split(","):
        part = part.strip()
        if "-" in part:
            a, b = part.split("-", 1)
            seeds += list(range(int(a), int(b) + 1))
        elif part:
            seeds.append(int(part))
    return seeds


def run_one(job: tuple) -> dict:
    """One (scenario, seed, run) in a worker process."""
    scenario, seed, run, params_path, kw = job
    from .scenarios import run_explore

    params = load_params(params_path, kw.get("map_stride"))
    try:
        if scenario == "explore":
            row = run_explore(seed, run, params, budget=kw["budget"])
        elif scenario == "goto_room":
            row = run_explore(seed, run, params, budget=kw["budget"], goto_room=kw["room"])
        else:
            from .search import run_search

            row = run_search(seed, run, params, budget=kw["budget"], obj=kw["object"])
    except Exception as exc:  # noqa: BLE001 - a broken run must not end the batch
        import traceback

        row = {
            "scenario": scenario, "run": run, "seed": seed, "sim": "kinsim", "verdict": "error",
            "reason": f"{type(exc).__name__}: {exc}", "metrics": {},
            "trace": traceback.format_exc()[-1500:],
        }  # fmt: skip
    row["pid"] = os.getpid()
    return row


def summarise(rows: list[dict], scenario: str, wall_s: float, jobs: int) -> dict:
    from referee.s3_verdicts import summarise as s3_summarise

    ok = [r for r in rows if "seed_set" in r]
    out = s3_summarise(ok, scenario, "mocap") if ok else {"scenario": scenario, "runs": 0}
    out["sim"] = "kinsim"
    out["label"] = (
        "kinsim (kinematic simulator): tuning / screening only, NOT an S3 / S4 result "
        "(docs/kinsim.md)"
    )
    out["exit_result"] = False
    out["errors"] = len(rows) - len(ok)
    out["wall_s"] = round(wall_s, 1)
    out["jobs"] = jobs
    sim_s = sum(r.get("sim_s", 0.0) for r in ok)
    cpu_s = sum(r.get("cpu_s", r.get("duration_s", 0.0)) for r in ok)
    out["sim_s_total"] = round(sim_s, 1)
    out["x_real_time_per_core"] = round(sim_s / cpu_s, 2) if cpu_s else None  # on CPU time
    out["runs_per_minute"] = round(60.0 * len(rows) / wall_s, 1) if wall_s else None
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--scenario", choices=SCENARIOS, required=True)
    ap.add_argument("--seeds", required=True, help="comma list / ranges, e.g. 20-79 or 1,5,9-12")
    ap.add_argument("--runs-per-seed", type=int, default=1)
    ap.add_argument("--first-run", type=int, default=0, help="run index of the first repeat")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--budget", type=float, default=360.0, help="explore / search budget, s")
    ap.add_argument("--room", default="kitchen", help="goto_room target type")
    ap.add_argument("--object", default="banana", help="search target")
    ap.add_argument("--params", default=None, help="KinParams JSON (default: calibrated)")
    ap.add_argument(
        "--map-stride", type=int, default=None,
        help="depth sampling of the mapping node: 2 = real launch file (exact, ~2.5x real time), "
        "4 = fast (about 7x); default from the params file",
    )  # fmt: skip
    ap.add_argument(
        "--held-out-ok", action="store_true",
        help="allow held-out seeds (>= 1000000); kinsim is for tuning seeds",
    )  # fmt: skip
    a = ap.parse_args(argv)

    seeds = parse_seeds(a.seeds)
    if any(s >= 1_000_000 for s in seeds) and not a.held_out_ok:
        ap.error("held-out seeds (>= 1000000) are for Gazebo; kinsim is for tuning seeds "
                 "(docs/seed-ledger.md). Pass --held-out-ok to override")  # fmt: skip
    out = a.out or Path("runs") / f"kinsim-{a.scenario}-{time.strftime('%Y%m%d-%H%M%S')}"
    out.mkdir(parents=True, exist_ok=True)
    kw = {"budget": a.budget, "room": a.room, "object": a.object, "map_stride": a.map_stride}
    runs = range(a.first_run, a.first_run + a.runs_per_seed)
    jobs = [(a.scenario, s, r, a.params, kw) for r in runs for s in seeds]
    t0 = time.time()
    rows: list[dict] = []
    res_path = out / "results.jsonl"
    res_path.write_text("")
    with mp.get_context("fork").Pool(min(a.jobs, len(jobs))) as pool:
        for row in pool.imap_unordered(run_one, jobs):
            rows.append(row)
            with res_path.open("a") as f:
                f.write(json.dumps(row, allow_nan=True, default=str) + "\n")
            print(
                f"[kinsim] {row['scenario']} seed {row['seed']}: "
                f"{str(row['verdict']).upper()} {row['reason']}  "
                f"({row.get('duration_s', '?')} s wall, {row.get('sim_s', '?')} s sim)",
                flush=True,
            )
    rows.sort(key=lambda r: (r.get("run", 0), r["seed"]))
    summary = summarise(rows, a.scenario, time.time() - t0, a.jobs)
    (out / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    print(f"Results in {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

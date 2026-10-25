"""S4b scenarios: search (bananas, bread), come_to a person, return, in generated apartments.

  python3 -m referee.s4 --seeds 10-14 --stacks 3 --localisation mocap
  (or: just test-s4 --seeds 10-14 --stacks 3)

Per seed, on one fresh stack: the world has small objects (present 70 % each) and a static
person (worldgen extras); the drone takes off, explores (`--budget`), then `search` banana,
`search` bread, `come_to` the person and `return`. Verdicts (referee.s4_verdicts):
  search : found when present within 0.3 m of the real object, "not found" when absent
           (a report of an object that is not there, or 0.3 m off, is a false positive)
  come_to: ends 1.0 - 1.5 m from the person, never closer than 1.0 m during the skill
  return : ends within 0.5 m of the take-off point
Only tuning seeds (0 - 99 999) are accepted: S4b results are not exit results.
Runs on the host (stdlib only) and reuses the S3 referee's stack handling.
Output: runs/s4-<timestamp>/ (results.jsonl, summary.json, per-run skill events and logs).
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from referee.parallel import run_queue
from referee.s3 import (
    LOCALISATIONS,
    RTF_BY_RUN,
    TAKEOFF_TIMEOUT_S,
    RunError,
    configure_stack,
    echo_json,
    ground_z_of,
    land_and_wait,
    make_world,
    parse_seeds,
    save_logs,
    send_request,
    start_observers,
    stop_observers,
    wait_for,
)
from referee.s3_verdicts import seed_set_of
from referee.s4_verdicts import (
    min_distance,
    summarise_s4,
    verdict_come_to,
    verdict_return,
    verdict_search,
)
from referee.stack import ROOT, Stack, down_all, rtf_summary
from referee.stackcfg import parse_stacks

POLL_S = 4.0
OBJECTS = ("banana", "bread")
TERMINAL = ("succeeded", "failed", "cancelled")


def wait_request(stack: Stack, rid: str, timeout_s: float) -> dict:
    """Poll /maverick/skill/progress until the request `rid` reaches a terminal state."""
    end = time.time() + timeout_s
    last: dict = {}
    while time.time() < end:
        p = echo_json(stack, "/maverick/skill/progress")
        if p and p.get("request_id") == rid:
            last = p
            if p.get("state") in TERMINAL:
                return p
        time.sleep(POLL_S)
    return {**last, "state": "timeout"}


def ground_truth_now(gt) -> dict:
    return [r for r in gt.rows if r["t"] <= time.time()][-1]


def run_skill(stack: Stack, req: dict, timeout_s: float, tag: str) -> tuple[dict, float, float]:
    rid = f"{tag}-{int(time.time())}"
    t0 = time.time()
    send_request(stack, {**req, "id": rid})
    prog = wait_request(stack, rid, timeout_s)
    return prog, t0, time.time()


def run_s4(stack: Stack, world: dict, out: Path, i: int, args) -> dict:
    man = world["manifest"]
    gt, px4 = start_observers(stack, world["name"])
    run_dir = out / f"run-{i:03d}-{world['name']}"
    run_dir.mkdir(parents=True, exist_ok=True)
    rec: dict = {"search": {}}
    try:
        ground_z_of(gt)
        if not wait_for(
            lambda: (
                (echo_json(stack, "/maverick/flight/status", latched=False) or {}).get("phase")
                == "hover"
            ),
            TAKEOFF_TIMEOUT_S,
            2.0,
        ):
            raise RunError("flight client never reached hover")
        spawn = man["spawn"]["pose"][:2]
        prog, _, _ = run_skill(
            stack, {"skill": "explore", "budget_s": args.budget}, args.budget + 90, "explore"
        )
        rec["explore"] = {"state": prog.get("state"), "message": prog.get("message")}
        for obj in OBJECTS:
            prog, t0, t1 = run_skill(
                stack,
                {"skill": "search", "object": obj, "budget_s": args.search_budget},
                args.search_budget + 90,
                f"search-{obj}",
            )
            res = prog.get("result")
            if prog.get("state") not in ("succeeded",) and not res:
                res = {
                    "found": False,
                    "reason": f"skill {prog.get('state')}: {prog.get('message')}",
                }
            v = verdict_search(man, obj, res, spawn)
            d = v.as_dict()
            d.update(
                skill_state=prog.get("state"),
                skill_message=prog.get("message"),
                wall_s=round(t1 - t0, 1),
                photo=(res or {}).get("photo"),
            )
            rec["search"][obj] = d
            print(f"  search {obj}: {d['verdict']} {d['reason']}", flush=True)
        people = man.get("people") or []
        if people:
            px, py = people[0]["pose"][:2]
            prog, t0, t1 = run_skill(
                stack, {"skill": "come_to", "person": "operator"}, args.come_to_budget, "come"
            )
            time.sleep(2.0)
            end = ground_truth_now(gt)
            md = min_distance(list(gt.rows), (px, py), t0, t1 + 2.0)
            v = verdict_come_to((px, py), (end["x"], end["y"]), md, str(prog.get("state")))
            rec["come_to"] = {
                **v.as_dict(),
                "skill_message": prog.get("message"),
                "wall_s": round(t1 - t0, 1),
            }
            print(f"  come_to: {rec['come_to']['verdict']} {v.reason}", flush=True)
            prog, t0, t1 = run_skill(stack, {"skill": "return"}, args.come_to_budget, "return")
            time.sleep(2.0)
            end = ground_truth_now(gt)
            v = verdict_return(spawn, (end["x"], end["y"]), str(prog.get("state")))
            rec["return"] = {**v.as_dict(), "skill_message": prog.get("message")}
        try:
            ev = stack.exec("host", "cat /tmp/skill_events.jsonl", timeout=30)
            (run_dir / "skill_events.jsonl").write_text(ev.stdout)
            ph = stack.exec("host", "ls /tmp/maverick_photos 2>/dev/null", timeout=30)
            rec["photos"] = ph.stdout.split()
        except Exception:  # noqa: BLE001
            pass
        land_and_wait(stack, px4)
    finally:
        stop_observers(stack)
        for s in (gt, px4):
            s.stop()
    checks = [v["verdict"] == "pass" for v in rec["search"].values()]
    checks += [rec[k]["verdict"] == "pass" for k in ("come_to", "return") if k in rec]
    rec["verdict"] = "pass" if checks and all(checks) else "fail"
    rec["reason"] = "; ".join(
        f"{k}: {v['reason']}" for k, v in rec["search"].items() if v["verdict"] != "pass"
    ) or ("all verdicts pass" if rec["verdict"] == "pass" else "see come_to / return")
    for k in ("come_to", "return"):
        if k in rec and rec[k]["verdict"] != "pass":
            rec["reason"] += f"; {k}: {rec[k]['reason']}"
    rec["manifest_extras"] = {
        "presence": man.get("presence"),
        "small_objects": man.get("small_objects"),
        "people": man.get("people"),
    }
    return rec


def run_one(stack: Stack, seed: int, out: Path, i: int, args) -> dict:
    world = make_world(seed, extras=True)
    configure_stack(stack, world, "s3", args.localisation)
    stack.restart()
    rtf_stream = stack.start_rtf(world["name"])
    try:
        return run_s4(stack, world, out, i, args)
    finally:
        stack.exec("referee", "pkill -f '[r]eferee.rtf'; true", timeout=30)
        rtf_stream.stop()
        rtf = rtf_summary(rtf_stream.rows)
        RTF_BY_RUN[(threading.get_ident(), i)] = rtf
        save_logs(stack, out / f"run-{i:03d}-{world['name']}")
        stack.down()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--seeds", required=True, help="tuning seeds: comma list / ranges, e.g. 10-14")
    ap.add_argument("--budget", type=float, default=300.0, help="explore budget, s")
    ap.add_argument("--search-budget", type=float, default=150.0, help="per search, s")
    ap.add_argument("--come-to-budget", type=float, default=150.0, help="come_to / return, s")
    ap.add_argument("--localisation", choices=LOCALISATIONS, default="mocap")
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--stacks", help="stack indices, e.g. 3 (never use stacks that are not yours)")
    ap.add_argument("--no-build", action="store_true", help="skip colcon build")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args(argv)
    try:
        stack_ids = parse_stacks(args.jobs, args.stacks)
    except ValueError as exc:
        ap.error(str(exc))
    seeds = parse_seeds(args.seeds)
    if any(seed_set_of(s) != "tuning" for s in seeds):
        raise SystemExit("S4b runs on tuning seeds (0-99999) only")
    out = args.out or ROOT / "runs" / f"s4-{datetime.now():%Y%m%d-%H%M%S}"
    out.mkdir(parents=True, exist_ok=True)
    stacks = {i: Stack(index=i) for i in stack_ids}
    rows: list[dict] = []
    t_suite = time.time()

    def make_worker(sid: int):
        def work(i: int, seed: int) -> dict:
            t_run = time.time()
            try:
                rec = run_one(stacks[sid], seed, out, i, args)
            except Exception as exc:  # noqa: BLE001 - a broken run must not end the suite
                rec = {"verdict": "error", "reason": f"{type(exc).__name__}: {exc}"}
            return {
                "scenario": "s4_search_come_to",
                "run": i,
                "seed": seed,
                "localisation": args.localisation,
                "exit_result": False,
                "duration_s": round(time.time() - t_run, 1),
                "rtf": RTF_BY_RUN.pop((threading.get_ident(), i), None),
                **rec,
            }

        return work

    def on_result(_i: int, _sid: int, rec: dict) -> None:
        rows.append(rec)
        with (out / "results.jsonl").open("a") as f:
            f.write(json.dumps(rec, allow_nan=True) + "\n")
        print(
            f"[stack {rec['stack']}] seed {rec['seed']}: {rec['verdict'].upper()} {rec['reason']}",
            flush=True,
        )

    try:
        if not args.no_build:
            print("colcon build ...", flush=True)
            stacks[stack_ids[0]].build_workspace()
        run_queue(seeds, stack_ids, make_worker, on_result)
    finally:
        rows.sort(key=lambda r: r["run"])
        summary = summarise_s4(rows)
        summary.update(
            wall_s=round(time.time() - t_suite), stacks=stack_ids, localisation=args.localisation
        )
        (out / "summary.json").write_text(json.dumps(summary, indent=2))
        down_all(list(stacks.values()))
    print(json.dumps(summary, indent=2))
    print(f"Results in {out}")
    return 0 if all(r["verdict"] == "pass" for r in rows) else 1


if __name__ == "__main__":
    sys.exit(main())

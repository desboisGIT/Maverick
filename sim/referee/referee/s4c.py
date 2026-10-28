"""S4c scenario `h2_text`: the first end-to-end mission by text (H2 without voice).

  python3 -m referee.s4c --seeds 10-19 --stacks 3 --backend scripted
  python3 -m referee.s4c --seeds 10-19 --stacks 3 --backend deepseek --cap-eur 0.30
  (or: just test-s4c --seeds 10-19 --stacks 3 --backend scripted)

Per seed, on one fresh stack (host launch `s4`: skills, memory, mapping, gateway and agent_node):
  setup   the world has a mannequin (the operator) and bananas present or absent (worldgen
          extras); the drone takes off, explores (so the kitchen is known, as H2 assumes) and
          comes to the operator, who is static.
  t0      the referee publishes the operator text on /maverick/operator/text:
          "Maverick, we're sitting down. Can you go check in the kitchen if there are bananas
          for us, and come back and tell us?"
  verdict referee.s4c_verdicts: spoken answer vs truth, photo shows the banana (only when
          present), ends within 1.5 m of the operator and never under 1.0 m, 0 collisions,
          mission over within 240 s of the command.
Recorded per run: the full transcript, the agent's plan steps, model calls, latency, tokens and
estimated cost (agent_node's /tmp/agent_calls.jsonl). Seeds without a mannequin are skipped.
A shared spend ledger (`--ledger`) stops real-model runs at `--cap-eur`.
Only tuning seeds (0 - 99 999). Output: runs/s4c-<timestamp>/ (results.jsonl, summary.json, logs).
"""

from __future__ import annotations

import argparse
import json
import shlex
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
    in_repo,
    land_and_wait,
    make_world,
    parse_seeds,
    save_logs,
    start_observers,
    stop_observers,
    wait_for,
)
from referee.s3_verdicts import seed_set_of
from referee.s4 import ground_truth_now, run_skill
from referee.s4_verdicts import min_distance
from referee.s4c_verdicts import TIME_LIMIT_S, summarise_h2, verdict_h2
from referee.stack import ROOT, Stack, down_all, rtf_summary
from referee.stackcfg import parse_stacks

OPERATOR_TEXT = (
    "Maverick, we're sitting down. Can you go check in the kitchen if there are bananas for us, "
    "and come back and tell us?"
)
POLL_S = 4.0
PER_RUN_CAP_EUR = 0.05


# -- spend ledger (real-model runs) ------------------------------------------------------------
_ledger_lock = threading.Lock()


def ledger_total(path: Path) -> float:
    try:
        return float(json.loads(path.read_text()).get("total_eur", 0.0))
    except (OSError, ValueError):
        return 0.0


def ledger_add(path: Path, eur: float, note: str) -> float:
    with _ledger_lock:
        try:
            d = json.loads(path.read_text())
        except (OSError, ValueError):
            d = {"total_eur": 0.0, "entries": []}
        d["total_eur"] = round(d["total_eur"] + eur, 6)
        d["entries"].append({"t": round(time.time()), "eur": round(eur, 6), "note": note})
        path.write_text(json.dumps(d, indent=1))
        return d["total_eur"]


# -- host helpers ------------------------------------------------------------------------------
def read_jsonl(stack: Stack, path: str) -> list[dict]:
    r = stack.exec("host", f"cat {path} 2>/dev/null", timeout=30)
    rows = []
    for line in r.stdout.splitlines():
        try:
            rows.append(json.loads(line))
        except ValueError:
            pass
    return rows


def echo_list(stack: Stack, topic: str) -> list[dict]:
    r = stack.exec(
        "host",
        "timeout 8 ros2 topic echo --once --field data --qos-durability transient_local "
        f"--qos-reliability reliable {topic} 2>/dev/null",
        timeout=40,
    )
    txt = r.stdout.replace("\n---", "").strip()
    try:
        v = json.loads(txt.strip("'\""))
    except ValueError:
        return []
    return v if isinstance(v, list) else []


def publish_operator_text(stack: Stack, text: str) -> None:
    inner = json.dumps({"text": text, "speaker": "operator"})
    payload = shlex.quote("{data: " + json.dumps(inner) + "}")  # YAML double-quoted string
    r = stack.exec(
        "host",
        f"ros2 topic pub --once -w 1 /maverick/operator/text std_msgs/msg/String {payload}",
        timeout=60,
    )
    if r.returncode != 0:
        raise RunError(f"operator text not published: {r.stderr[-300:]}")


def collisions_in_tools(stack: Stack, raw_path: Path) -> int:
    cmd = (
        f"cd /maverick && PYTHONPATH=sim/referee python3 -m referee.s4c_tools "
        f"{shlex.quote(in_repo(raw_path))}"
    )
    r = stack.compose(
        "--profile", "tools", "run", "--rm", "-T", "tools", "bash", "-lc", cmd, timeout=900
    )
    line = next((ln for ln in r.stdout.splitlines() if ln.startswith("{")), None)
    if r.returncode != 0 or line is None:
        raise RunError(f"collision check failed: {r.stdout[-300:]} {r.stderr[-300:]}")
    return int(json.loads(line)["collisions"])


# -- one run -----------------------------------------------------------------------------------
def run_h2(stack: Stack, world: dict, out: Path, i: int, args) -> dict:
    man = world["manifest"]
    gt, px4 = start_observers(stack, world["name"])
    run_dir = out / f"run-{i:03d}-{world['name']}"
    run_dir.mkdir(parents=True, exist_ok=True)
    rec: dict = {}
    try:
        ground_z = ground_z_of(gt)
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
        px, py = man["people"][0]["pose"][:2]
        # setup: explore, then stand near the operator (H2 assumes the flat is known)
        prog, _, _ = run_skill(
            stack, {"skill": "explore", "budget_s": args.budget}, args.budget + 90, "explore"
        )
        rec["setup_explore"] = prog.get("state")
        prog, _, _ = run_skill(
            stack, {"skill": "come_to", "person": "operator"}, args.setup_budget, "come"
        )
        rec["setup_come_to"] = prog.get("state")
        if prog.get("state") != "succeeded":
            try:
                ev = stack.exec("host", "cat /tmp/skill_events.jsonl", timeout=30)
                (run_dir / "skill_events.jsonl").write_text(ev.stdout)
            except Exception:  # noqa: BLE001
                pass
            raise RunError(f"setup failed: come_to {prog.get('state')}: {prog.get('message')}")
        if not wait_for(
            lambda: "agent_node" in stack.exec("host", "ros2 node list", timeout=30).stdout, 30, 2.0
        ):
            raise RunError("agent_node is not running")
        time.sleep(2.0)
        # the command
        t0 = time.time()
        publish_operator_text(stack, OPERATOR_TEXT)
        mission_end, events = None, []
        deadline = t0 + TIME_LIMIT_S + 60
        while time.time() < deadline:
            time.sleep(POLL_S)
            events = read_jsonl(stack, "/tmp/agent_events.jsonl")
            mission_end = next(
                (e for e in events if e["type"] == "mission_end" and e["t"] >= t0), None
            )
            if mission_end:
                break
        t_end = mission_end["t"] if mission_end else time.time()
        time.sleep(3.0)  # let the drone settle after the last skill
        end = ground_truth_now(gt)
        t_cmd = next(
            (
                e["t"]
                for e in events
                if e["type"] == "transcript" and e["data"].get("role") == "user"
            ),
            t0,
        )
        md = min_distance(list(gt.rows), (px, py), t_cmd, time.time())
        end_dist = ((end["x"] - px) ** 2 + (end["y"] - py) ** 2) ** 0.5
        raw = {
            "sdf": in_repo(world["sdf"]),
            "manifest": in_repo(world["manifest_path"]),
            "spawn_pose": man["spawn"]["pose"],
            "ground_z": ground_z,
            "track": [[r["t"], r["x"], r["y"], r["z"]] for r in gt.rows],
            "t0": t_cmd,
            "t1": time.time(),
        }
        raw_path = run_dir / "raw.json"
        raw_path.write_text(json.dumps(raw))
        n_coll = collisions_in_tools(stack, raw_path)
        objects = echo_list(stack, "/maverick/memory/objects")
        calls = read_jsonl(stack, "/tmp/agent_calls.jsonl")
        (run_dir / "agent_events.jsonl").write_text("".join(json.dumps(e) + "\n" for e in events))
        (run_dir / "agent_calls.jsonl").write_text("".join(json.dumps(c) + "\n" for c in calls))
        (run_dir / "memory_objects.json").write_text(json.dumps(objects))
        try:
            ev = stack.exec("host", "cat /tmp/skill_events.jsonl", timeout=30)
            (run_dir / "skill_events.jsonl").write_text(ev.stdout)
        except Exception:  # noqa: BLE001
            pass
        v = verdict_h2(
            man,
            events,
            t_cmd,
            objects,
            spawn,
            end_dist,
            md,
            n_coll,
            mission_end["data"] if mission_end else None,
            t_end if mission_end else None,
        )
        rec.update(v.as_dict())
        lat = [c["latency_s"] for c in calls if "latency_s" in c]
        rec["model_calls"] = len(lat)
        rec["model_latencies_s"] = lat
        rec["cost_eur"] = round(max((c.get("cum_cost_eur", 0.0) for c in calls), default=0.0), 6)
        rec["tokens"] = {
            k: sum(c.get("usage", {}).get(k, 0) for c in calls)
            for k in ("input_tokens", "output_tokens", "cache_read_input_tokens")
        }
        rec["transcript"] = [
            {
                "t": round(e["t"] - t_cmd, 1),
                "role": e["data"].get("role"),
                "text": e["data"].get("text"),
            }
            for e in events
            if e["type"] == "transcript"
        ]
        print(f"  {rec['verdict']}: {rec['reason']} (layers {rec['layers']})", flush=True)
        land_and_wait(stack, px4)
    finally:
        stop_observers(stack)
        for s in (gt, px4):
            s.stop()
    rec["manifest_extras"] = {
        "presence": man.get("presence"),
        "small_objects": man.get("small_objects"),
        "people": man.get("people"),
    }
    return rec


def run_one(stack: Stack, seed: int, out: Path, i: int, args) -> dict:
    world = make_world(seed, extras=True)
    if not world["manifest"].get("people"):
        return {"verdict": "skipped", "reason": "no person in this apartment", "layers": []}
    launch = f"agent_backend:={args.backend} agent_cap_eur:={args.run_cap:.4f}"
    configure_stack(stack, world, "s4", args.localisation)
    stack.env["MAVERICK_LAUNCH_ARGS"] = launch
    stack.restart()
    rtf_stream = stack.start_rtf(world["name"])
    try:
        return run_h2(stack, world, out, i, args)
    finally:
        stack.exec("referee", "pkill -f '[r]eferee.rtf'; true", timeout=30)
        rtf_stream.stop()
        RTF_BY_RUN[(threading.get_ident(), i)] = rtf_summary(rtf_stream.rows)
        save_logs(stack, out / f"run-{i:03d}-{world['name']}")
        stack.down()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--seeds", required=True, help="tuning seeds: comma list / ranges, e.g. 10-19")
    ap.add_argument("--backend", choices=("scripted", "deepseek", "claude"), default="scripted")
    ap.add_argument("--cap-eur", type=float, default=0.30, help="total spend cap (real models)")
    ap.add_argument("--ledger", type=Path, default=ROOT / "runs" / "s4c-spend.json")
    ap.add_argument("--budget", type=float, default=300.0, help="setup explore budget, s")
    ap.add_argument("--setup-budget", type=float, default=150.0, help="setup come_to, s")
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
        raise SystemExit("S4c runs on tuning seeds (0-99999) only")
    out = args.out or ROOT / "runs" / f"s4c-{datetime.now():%Y%m%d-%H%M%S}"
    out.mkdir(parents=True, exist_ok=True)
    args.ledger.parent.mkdir(parents=True, exist_ok=True)
    stacks = {i: Stack(index=i) for i in stack_ids}
    rows: list[dict] = []
    t_suite = time.time()

    def make_worker(sid: int):
        def work(i: int, seed: int) -> dict:
            t_run = time.time()
            spent = ledger_total(args.ledger) if args.backend != "scripted" else 0.0
            args.run_cap = min(PER_RUN_CAP_EUR, max(0.0, args.cap_eur - spent))
            if args.backend != "scripted" and args.run_cap <= 0.001:
                return {"scenario": "h2_text", "run": i, "seed": seed, "verdict": "skipped",
                        "reason": "spend cap reached", "layers": []}  # fmt: skip
            try:
                rec = run_one(stacks[sid], seed, out, i, args)
            except Exception as exc:  # noqa: BLE001 - a broken run must not end the suite
                why = f"{type(exc).__name__}: {exc}"
                layer = "setup skill failure" if "setup failed" in why else "infrastructure"
                rec = {"verdict": "error", "reason": why, "layers": [layer]}
            if args.backend != "scripted":
                total = ledger_add(args.ledger, rec.get("cost_eur", 0.0), f"seed {seed}")
                print(f"  cumulative spend EUR {total:.4f} (cap {args.cap_eur})", flush=True)
            return {
                "scenario": "h2_text",
                "run": i,
                "seed": seed,
                "backend": args.backend,
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
        summary = summarise_h2(rows)
        summary.update(
            wall_s=round(time.time() - t_suite),
            stacks=stack_ids,
            backend=args.backend,
            localisation=args.localisation,
        )
        (out / "summary.json").write_text(json.dumps(summary, indent=2))
        down_all(list(stacks.values()))
    print(json.dumps(summary, indent=2))
    print(f"Results in {out}")
    return 0 if all(r["verdict"] in ("pass", "skipped") for r in rows) else 1


if __name__ == "__main__":
    sys.exit(main())

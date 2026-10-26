"""S3 scenarios in generated apartments: hover, explore (C1/C6), goto_room (C2).

  python3 -m referee.s3 --scenario hover --seed-set held_out --runs 10
  python3 -m referee.s3 --scenario explore --seeds 1000000,1000001 --budget 360
  python3 -m referee.s3 --scenario goto_room --seed-set held_out --runs 10 --room kitchen
  (or: just test-s3 --scenario explore --seeds 1000000)

Runs on the HOST machine (stdlib only) and drives `docker compose` like referee.failsafe. Per run:
  1. generate `apt_<seed>` with worldgen into sim/worlds/generated/ (manifest beside it);
  2. fresh stack with PX4_GZ_WORLD / PX4_GZ_WORLDS / PX4_GZ_MODEL_POSE set (drone at the spawn);
  3. hover : flight client (MAVERICK_HOST_LAUNCH=none) takes off, hovers 10 s, lands;
     explore: MAVERICK_HOST_LAUNCH=s3, `{"skill":"explore"}` sent to the skill node, the drone's own
              map dumped every 30 s (service /maverick/map/dump), scored after the skill ends;
     goto_room: as explore, then `{"skill":"goto_room","room_type":"kitchen"}`;
  4. ground truth (Gazebo pose, referee container) and PX4 state (px4_monitor) are recorded the
     whole time; metrics are computed in the tools container (numpy, referee.s3_metrics).
Parallel: `--jobs K` / `--stacks 1,2,3` spread the runs over K stacks (referee.stackcfg); each
results.jsonl line gets `stack` and `rtf` (Gazebo real-time factor during the run).
Output: runs/s3-<scenario>-<timestamp>/ (results.jsonl, summary.json, raw-*.json, map dumps).
Capability scenarios (explore, goto_room) on tuning seeds print a warning and are marked
`exit_result: false`: exit results come from held-out seeds only.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import shlex
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from referee.parallel import run_queue
from referee.s3_frames import model_pose_env
from referee.s3_verdicts import (
    estimate_error,
    exit_result_warning,
    explore_pass_c6_v1,
    goto_metrics,
    hover_metrics,
    label_reason,
    old_rule_pass,
    seed_set_of,
    summarise,
    verdict_explore,
    verdict_goto,
    verdict_hover,
)
from referee.stack import ROOT, Stack, down_all, rtf_summary
from referee.stackcfg import parse_stacks

sys.path.insert(0, str(ROOT / "sim" / "worldgen"))

SCENARIOS = ("hover", "explore", "goto_room")
LOCALISATIONS = ("flow", "mocap", "vo")
WORLDS_DIR = ROOT / "sim" / "worlds" / "generated"
WORLDS_DIR_IN_SIM = "/maverick/sim/worlds/generated"
FLIGHT_BIN = "/maverick/ros2_ws/install/maverick_flight/lib/maverick_flight"
ALTITUDE_M = 1.2
HOVER_S = 10.0
SNAPSHOT_PERIOD_S = 30.0
POLL_S = 4.0
TAKEOFF_TIMEOUT_S = 90.0
GOTO_TIMEOUT_S = 200.0
LAND_TIMEOUT_S = 90.0
SEED_SET_START = {"tuning": 0, "held_out": 1_000_000}


RTF_BY_RUN: dict[tuple[int, int], dict | None] = {}  # (thread, run index) -> RTF summary


class RunError(Exception):
    """Infrastructure problem (not a verdict on the system)."""


def wait_for(pred, timeout: float, poll: float = 0.5):
    end = time.time() + timeout
    while time.time() < end:
        v = pred()
        if v:
            return v
        time.sleep(poll)
    return None


# -- world -------------------------------------------------------------------------------------
def make_world(seed: int, extras: bool = False) -> dict:
    """Generate and validate apt_<seed>; return its manifest and file paths."""
    from worldgen import generate
    from worldgen.validate import validate

    world = generate(seed, small_objects=extras, people=extras)
    problems = validate(world.manifest, world.sdf)
    if problems:
        raise RunError(f"worldgen seed {seed} invalid: {problems}")
    WORLDS_DIR.mkdir(parents=True, exist_ok=True)
    sdf = WORLDS_DIR / f"{world.name}.sdf"
    man = WORLDS_DIR / f"{world.name}.json"
    for path, text in ((sdf, world.sdf), (man, world.manifest_json())):
        tmp = path.with_name(f".{path.name}.{os.getpid()}.{threading.get_ident()}.tmp")
        tmp.write_text(text)
        tmp.replace(path)  # atomic: parallel runs of one seed write identical files
    return {"name": world.name, "manifest": world.manifest, "sdf": sdf, "manifest_path": man}


def configure_stack(
    stack: Stack, world: dict, host_launch: str, localisation: str = "flow"
) -> None:
    stack.env["MAVERICK_LOCALISATION"] = localisation
    stack.env["PX4_GZ_WORLD"] = world["name"]
    stack.env["PX4_GZ_WORLDS"] = WORLDS_DIR_IN_SIM
    stack.env["PX4_GZ_MODEL_POSE"] = model_pose_env(world["manifest"])
    stack.env["MAVERICK_HOST_LAUNCH"] = host_launch


def in_repo(path: Path) -> str:
    return "/maverick/" + str(path.resolve().relative_to(ROOT))


# -- host helpers ------------------------------------------------------------------------------
def echo_json(stack: Stack, topic: str, timeout: float = 6.0, latched: bool = True) -> dict | None:
    """Latest message of a latched std_msgs/String JSON topic, or None."""
    qos = "--qos-durability transient_local --qos-reliability reliable " if latched else ""
    r = stack.exec(
        "host",
        f"timeout {timeout:.0f} ros2 topic echo --once --field data {qos}{topic} 2>/dev/null",
        timeout=timeout + 20,
    )
    m = re.search(r"\{.*\}", r.stdout, re.S)
    if not m:
        return None
    try:
        return json.loads(m.group(0))
    except ValueError:
        return None


def send_request(stack: Stack, req: dict) -> None:
    payload = shlex.quote("{data: " + shlex.quote(json.dumps(req)) + "}")
    r = stack.exec(
        "host",
        f"ros2 topic pub --once -w 1 /maverick/skill/request std_msgs/msg/String {payload}",
        timeout=60,
    )
    if r.returncode != 0:
        raise RunError(f"skill request failed: {r.stderr[-300:]}")


def dump_map(stack: Stack, dest: Path) -> bool:
    r = stack.exec(
        "host",
        "ros2 service call /maverick/map/dump std_srvs/srv/Trigger 2>&1 | grep -q 'success=True' "
        f"&& cp /tmp/map_dump.npz {shlex.quote(in_repo(dest))}",
        timeout=60,
    )
    return r.returncode == 0 and dest.exists()


def wait_skill(stack: Stack, name: str, timeout_s: float, on_poll=None) -> dict:
    """Poll /maverick/skill/progress until skill `name` is succeeded / failed / cancelled."""
    end = time.time() + timeout_s
    last: dict = {}
    while time.time() < end:
        p = echo_json(stack, "/maverick/skill/progress")
        if p and p.get("skill") == name:
            last = p
            if p.get("state") in ("succeeded", "failed", "cancelled"):
                return p
        if on_poll:
            on_poll()
        time.sleep(POLL_S)
    return {**last, "state": "timeout"}


def vision_counters(stack: Stack) -> dict | None:
    """Bridge relay's visual-odometry counters (forwarded / dropped), read through the link."""
    r = stack.exec(
        "host",
        "timeout 8 ros2 topic echo --once --qos-reliability best_effort /maverick/bridge/status "
        "2>/dev/null | grep -E '^vision_'",
        timeout=40,
    )
    out = {}
    for line in r.stdout.splitlines():
        k, _, v = line.partition(":")
        if v.strip().isdigit():
            out[k.strip()] = int(v)
    return out or None


def start_observers(stack: Stack, world: str):
    env_gz = {"GZ_PARTITION": stack.gz_partition or "", "PYTHONPATH": "/maverick/sim/referee"}
    gt = stack.stream("gt", "referee", f"exec python3 -m referee.gt --world {world}", env_gz)
    px4 = stack.stream(
        "px4", "bridge", "ROS_DOMAIN_ID=1 exec ros2 run maverick_flight px4_monitor px4"
    )
    return gt, px4


def stop_observers(stack: Stack) -> None:
    stack.exec("referee", "pkill -f '[r]eferee.gt'; true", timeout=30)
    stack.exec("bridge", "pkill -f '[p]x4_monitor'; true", timeout=30)
    stack.exec("host", "pkill -9 -f '[m]averick_flight/flight_client'; true", timeout=30)


def ground_z_of(gt) -> float:
    if not wait_for(lambda: len(gt.rows) >= 5, 30):
        raise RunError("no ground truth samples")
    return sorted(r["z"] for r in gt.rows[:5])[2]


def land_and_wait(stack: Stack, px4) -> None:
    stack.exec(
        "host",
        "ros2 topic pub --once -w 1 /maverick/flight/mission std_msgs/msg/String '{data: land}'",
        timeout=60,
    )

    def disarmed() -> bool:
        st = [r for r in px4.rows if r.get("k") == "status"]
        return bool(st) and st[-1]["arm"] == 1

    wait_for(disarmed, LAND_TIMEOUT_S, 1.0)
    time.sleep(2.0)


def score_in_tools(stack: Stack, raw_path: Path) -> dict:
    cmd = (
        f"cd /maverick && PYTHONPATH=sim/referee python3 -m referee.s3_metrics "
        f"{shlex.quote(in_repo(raw_path))}"
    )
    r = stack.compose("--profile", "tools", "run", "--rm", "-T", "tools", "bash", "-lc", cmd,
                      timeout=900)  # fmt: skip
    line = next((ln for ln in r.stdout.splitlines() if ln.startswith("{")), None)
    if r.returncode != 0 or line is None:
        raise RunError(f"metrics failed: {r.stdout[-500:]} {r.stderr[-500:]}")
    return json.loads(line)


# -- scenarios ---------------------------------------------------------------------------------
def run_hover(stack: Stack, world: dict, out: Path, i: int, args) -> dict:
    gt, px4 = start_observers(stack, world["name"])
    stack.exec("host", ": > /tmp/flight.log", timeout=30)
    flight = stack.stream("flight", "host", "exec tail -n +1 -F /tmp/flight.log")
    try:
        ground_z = ground_z_of(gt)
        stack.exec(
            "host",
            f"cd /tmp && nohup {FLIGHT_BIN}/flight_client --ros-args -p altitude:={ALTITUDE_M} "
            f"-p hover_seconds:={HOVER_S} > /tmp/flight.log 2>&1 &",
            timeout=30,
        )
        done = wait_for(
            lambda: any(e.get("event") == "exit" for e in list(flight.rows)),
            TAKEOFF_TIMEOUT_S + HOVER_S + LAND_TIMEOUT_S,
            1.0,
        )
        if not done:
            raise RunError("flight client did not finish")
        time.sleep(3.0)
        m = hover_metrics(list(gt.rows), list(flight.rows), list(px4.rows), ground_z)
        sp = world["manifest"]["spawn"]["pose"]
        g0 = gt.rows[0]
        m["spawn_err_m"] = round(math.hypot(g0["x"] - sp[0], g0["y"] - sp[1]), 3)
        m["ground_z"] = round(ground_z, 3)
    finally:
        stop_observers(stack)
        for s in (gt, px4, flight):
            s.stop()
    v = verdict_hover(m)
    return {"verdict": "pass" if v.passed else "fail", "reason": v.reason, "metrics": v.metrics}


def run_explore(stack: Stack, world: dict, out: Path, i: int, args, goto: bool) -> dict:
    gt, px4 = start_observers(stack, world["name"])
    run_dir = out / f"run-{i:03d}-{world['name']}"
    run_dir.mkdir(parents=True, exist_ok=True)
    snaps: list[dict] = []
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
        t_start = time.time()
        send_request(stack, {"skill": "explore", "budget_s": args.budget})
        last_snap = [time.time()]

        def snapshot() -> None:
            if time.time() - last_snap[0] >= SNAPSHOT_PERIOD_S:
                last_snap[0] = time.time()
                dest = run_dir / f"map-{len(snaps):03d}.npz"
                if dump_map(stack, dest):
                    snaps.append({"t": time.time(), "path": in_repo(dest)})

        prog = wait_skill(stack, "explore", args.budget + 90, snapshot)
        t_end = time.time()
        final = run_dir / "map-final.npz"
        if not dump_map(stack, final):
            raise RunError("map dump failed")
        snaps.append({"t": t_end, "path": in_repo(final)})
        result: dict = {"explore_skill": prog, "explore_wall_s": round(t_end - t_start, 1)}

        goto_m = None
        if goto:
            kitchen = args.room
            assert any(r["type"] == kitchen for r in world["manifest"]["rooms"]), (
                f"flat {world['name']} has no {kitchen}"
            )
            send_request(stack, {"skill": "goto_room", "room_type": kitchen})
            gprog = wait_skill(stack, "goto", GOTO_TIMEOUT_S)  # progress names it "goto"
            time.sleep(2.0)  # settle
            end_pos = [r for r in gt.rows if r["t"] <= time.time()][-1]
            goto_m = {
                "skill_state": gprog.get("state"),
                "skill_message": gprog.get("message"),
                "skill_succeeded": gprog.get("state") == "succeeded",
                "world_xy": [end_pos["x"], end_pos["y"]],
            }
        t_after = time.time()
        try:  # the skills' decision log (diagnosis)
            ev = stack.exec("host", "cat /tmp/skill_events.jsonl", timeout=30)
            (run_dir / "skill_events.jsonl").write_text(ev.stdout)
        except Exception:  # noqa: BLE001
            pass
        land_and_wait(stack, px4)
    finally:
        stop_observers(stack)
        for s in (gt, px4):
            s.stop()
    raw = {
        "seed": world["manifest"]["seed"],
        "sdf": in_repo(world["sdf"]),
        "manifest": in_repo(world["manifest_path"]),
        "spawn_pose": world["manifest"]["spawn"]["pose"],
        "ground_z": ground_z,
        "track": [[r["t"], r["x"], r["y"], r["z"]] for r in gt.rows],
        "t_explore_start": t_start,
        "t_explore_end": t_after if goto else t_end,
        "snapshots": [s for s in snaps if s["t"] <= t_end],
        "final_map": in_repo(final),
        # diagnosis data (not used by the verdicts): PX4's estimate as the monitor saw it
        "px4_pos": [
            [r["t"], r["x"], r["y"], r["z"], int(bool(r.get("valid")))]
            for r in px4.rows
            if r.get("k") == "pos"
        ],
    }
    raw_path = run_dir / "raw.json"
    raw_path.write_text(json.dumps(raw))
    metrics = score_in_tools(stack, raw_path)
    metrics.update(result)
    spawn = world["manifest"]["spawn"]["pose"]
    metrics.update(
        estimate_error(
            list(gt.rows), list(px4.rows), spawn, t_start, t_after if goto else t_end, ground_z
        )
    )
    metrics["vision"] = vision_counters(stack)
    v = verdict_explore(metrics)
    rec = {"verdict": "pass" if v.passed else "fail", "reason": v.reason, "metrics": metrics}
    if "c6_recall_v1" in metrics:
        rec["c6_old_rule_pass"] = explore_pass_c6_v1(metrics)
    if goto_m is not None:
        origin = (*world["manifest"]["spawn"]["pose"][:2], ground_z)
        gm = goto_metrics(
            world["manifest"],
            args.room,
            (goto_m["world_xy"][0] - origin[0], goto_m["world_xy"][1] - origin[1]),
            origin,
        )
        gm.update(goto_m)
        gv = verdict_goto(gm)
        rec["goto"] = {"verdict": "pass" if gv.passed else "fail", "reason": gv.reason, **gm}
        rec["goto"]["old_rule_pass"] = old_rule_pass(gm)
        if args.scenario == "goto_room":  # C2 is the verdict of this scenario
            rec["verdict"], rec["reason"] = rec["goto"]["verdict"], gv.reason
            rec["old_rule_pass"] = rec["goto"]["old_rule_pass"]
    return rec


def save_logs(stack: Stack, run_dir: Path) -> None:
    """Keep the host (flight client, skills, mapping) and sim logs of the run for debugging."""
    try:
        if run_dir.is_dir():
            for svc in ("host", "sim"):
                r = stack.compose("logs", "--no-color", svc, timeout=120)
                (run_dir / f"stack-{svc}.log").write_text(r.stdout[-3_000_000:])
    except Exception as e:  # never let debugging output fail a run
        print(f"could not save stack logs: {e}", file=sys.stderr)


def save_diag(stack: Stack, run_dir: Path, rtf_rows: list[dict]) -> None:
    """Estimator diagnosis data: wall-clock RTF series and a compact extract of PX4's ulog
    (estimate vs simulator truth, EV innovations, resets), see scripts/ulog_extract.py."""
    try:
        if not run_dir.is_dir():
            return
        (run_dir / "rtf_series.json").write_text(json.dumps([[r["t"], r["rtf"]] for r in rtf_rows]))
        r = stack.exec("sim", "python3 /maverick/scripts/ulog_extract.py", timeout=120)
        if r.returncode == 0 and r.stdout.strip():
            (run_dir / "ekf.json").write_text(r.stdout)
    except Exception as e:  # noqa: BLE001 - diagnostics never fail a run
        print(f"could not save diagnostics: {e}", file=sys.stderr)


def run_one(stack: Stack, scenario: str, seed: int, out: Path, i: int, args) -> dict:
    world = make_world(seed)
    configure_stack(
        stack, world, "none" if scenario == "hover" else "s3", getattr(args, "localisation", "flow")
    )
    stack.restart()
    rtf_stream = stack.start_rtf(world["name"])
    try:
        if scenario == "hover":
            rec = run_hover(stack, world, out, i, args)
        else:
            rec = run_explore(stack, world, out, i, args, goto=scenario == "goto_room")
        return rec
    finally:
        stack.exec("referee", "pkill -f '[r]eferee.rtf'; true", timeout=30)
        rtf_stream.stop()
        rtf = rtf_summary(rtf_stream.rows)
        if rtf:
            print(f"  [stack {stack.index}] seed {seed} real-time factor {rtf}", flush=True)
        RTF_BY_RUN[(threading.get_ident(), i)] = rtf
        run_dir = out / f"run-{i:03d}-{world['name']}"
        save_logs(stack, run_dir)
        save_diag(stack, run_dir, rtf_stream.rows)
        stack.down()


# -- cli ---------------------------------------------------------------------------------------
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


def pick_seeds(args) -> list[int]:
    if args.seeds:
        seeds = parse_seeds(args.seeds)
        if args.seed_set and any(seed_set_of(s) != args.seed_set for s in seeds):
            raise SystemExit(f"--seeds are not all in the {args.seed_set} set")
    else:
        start = SEED_SET_START[args.seed_set or "held_out"]
        seeds = list(range(start, start + (args.runs or 1)))
    runs = args.runs or len(seeds)
    return [seeds[k % len(seeds)] for k in range(runs)]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--scenario", choices=SCENARIOS, required=True)
    ap.add_argument("--seeds", help="comma list / ranges, e.g. 1000000,1000003-1000005")
    ap.add_argument("--seed-set", choices=sorted(SEED_SET_START))
    ap.add_argument("--runs", type=int, help="number of runs (default: one per seed)")
    ap.add_argument("--budget", type=float, default=360.0, help="explore budget, s")
    ap.add_argument("--room", default="kitchen", help="goto_room target type")
    ap.add_argument(
        "--localisation",
        choices=LOCALISATIONS,
        default="flow",
        help="position source: flow (PX4 flow only), mocap (simulated motion capture, NOT an "
        "exit result), vo (host visual odometry)",
    )
    ap.add_argument("--jobs", type=int, default=1, help="parallel stacks (default 1)")
    ap.add_argument(
        "--stacks",
        help="stack indices, e.g. 1,2,3 (default: 0 for one job, else 1..jobs)",
    )
    ap.add_argument("--no-build", action="store_true", help="skip colcon build")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args(argv)

    try:
        stack_ids = parse_stacks(args.jobs, args.stacks)
    except ValueError as exc:
        ap.error(str(exc))
    seeds = pick_seeds(args)
    sets = {seed_set_of(s) for s in seeds}
    warn = exit_result_warning(args.scenario, sets)
    if warn:
        print(warn, flush=True)

    out = args.out or ROOT / "runs" / f"s3-{args.scenario}-{datetime.now():%Y%m%d-%H%M%S}"
    out.mkdir(parents=True, exist_ok=True)
    stacks = {i: Stack(index=i) for i in stack_ids}
    rows: list[dict] = []
    t_suite = time.time()

    def make_worker(sid: int):
        def work(i: int, seed: int) -> dict:
            t_run = time.time()
            try:
                rec = run_one(stacks[sid], args.scenario, seed, out, i, args)
            except Exception as exc:  # noqa: BLE001 - a broken run must not end the suite
                rec = {"verdict": "error", "reason": f"{type(exc).__name__}: {exc}", "metrics": {}}
            rec = {
                "scenario": args.scenario,
                "run": i,
                "seed": seed,
                "seed_set": seed_set_of(seed),
                "localisation": args.localisation,
                "duration_s": round(time.time() - t_run, 1),
                "rtf": RTF_BY_RUN.pop((threading.get_ident(), i), None),
                **rec,
            }
            rec["reason"] = label_reason(rec["reason"], args.localisation)
            return rec

        return work

    def on_result(_i: int, _sid: int, rec: dict) -> None:
        rows.append(rec)
        with (out / "results.jsonl").open("a") as f:
            f.write(json.dumps(rec, allow_nan=True) + "\n")
        print(
            f"[stack {rec['stack']}] {args.scenario} seed {rec['seed']}: "
            f"{rec['verdict'].upper()} {rec['reason']}",
            flush=True,
        )

    try:
        if not args.no_build:
            print("colcon build ...", flush=True)
            stacks[stack_ids[0]].build_workspace()
        run_queue(seeds, stack_ids, make_worker, on_result)
    finally:
        rows.sort(key=lambda r: r["run"])
        summary = summarise(rows, args.scenario, args.localisation)
        summary["wall_s"] = round(time.time() - t_suite)
        summary["stacks"] = stack_ids
        (out / "summary.json").write_text(json.dumps(summary, indent=2))
        down_all(list(stacks.values()))
    print(json.dumps(summary, indent=2))
    print(f"Results in {out}")
    return 0 if summary["pass"] == summary["runs"] else 1


if __name__ == "__main__":
    sys.exit(main())

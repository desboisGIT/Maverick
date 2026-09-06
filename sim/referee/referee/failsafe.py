"""Randomised failsafe suite F1, F2, F3, F4, F6 (docs/s1-plan.md, "exit test").

  python3 -m referee.failsafe --cases F1,F2,F3,F4,F6 --runs-per-case 50 --seed 1
  (or: just test-failsafe --runs-per-case 50 --seed 1)

Runs on the HOST machine and drives the stack with `docker compose`. In the containers:
  referee  Gazebo ground truth (referee.gt)
  bridge   px4_monitor px4 / bridge (PX4 state and bridge counters, independent of the host)
  host     flight_client (the system under test) and inject_goals
Faults: tc netem windows on bridge+host, pkill of the flight client (F3) or of the relay (F4),
bad goals through inject_goals (F6).

The stack is reused between runs when the previous run ended landed and disarmed with nothing
killed and netem cleared; otherwise it is restarted. Output: runs/s1-failsafe-<timestamp>/
results.jsonl (one line per run) and summary.json.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import shlex
import sys
import time
from datetime import datetime
from pathlib import Path

from referee import faults
from referee.scoring import SCORERS, RunData, Verdict, hover_reference
from referee.stack import ROOT, Stack

CASES = ("F1", "F2", "F3", "F4", "F6")
FLIGHT_BIN = "/maverick/ros2_ws/install/maverick_flight/lib/maverick_flight"
HOVER_TIMEOUT_S = 150.0
END_TIMEOUT_S = 150.0
POST_LAND_WATCH_S = 5.0  # watch for a re-arm after the landing
RETRY_ATTEMPTS = 2


# -- parameters ------------------------------------------------------------------------------
def sample_params(case: str, rng: random.Random) -> dict:
    offset = round(rng.uniform(2.0, 6.0), 2)  # s after hover is established
    if case == "F1":
        kind = rng.choice(["short", "medium", "long"])
        lo, hi = {"short": (0.2, 0.4), "medium": (1.0, 4.0), "long": (12.0, 20.0)}[kind]
        d = round(rng.uniform(lo, hi), 2)
        return {
            "kind": kind,
            "outage_s": d,
            "offset_s": offset,
            "hover_s": offset + d + 12.0 if kind != "long" else 120.0,
        }
    if case == "F2":
        d = round(rng.uniform(10.0, 30.0), 1)
        return {"duration_s": d, "offset_s": offset, "hover_s": offset + d + 8.0}
    if case in ("F3", "F4"):
        return {"offset_s": offset, "hover_s": 120.0}
    return {"offset_s": offset, "hover_s": 0.0, "goals": sample_goals(rng)}  # F6: land on request


def sample_goals(rng: random.Random) -> list[dict]:
    """Bad goals around a hold point at the spawn, each with the expected rejection reason."""
    goals = []
    for _ in range(rng.randint(4, 8)):
        kind = rng.choice(["geofence", "nan", "jump", "speed"])
        if kind == "geofence":
            axis = rng.choice("xyz")
            if axis == "z":  # far above, or below the floor
                z = -rng.uniform(6.0, 15.0) if rng.random() < 0.5 else rng.uniform(1.0, 5.0)
                p = {"x": 0.0, "y": 0.0, "z": z}
            else:
                p = {"x": 0.0, "y": 0.0, "z": -2.0}
                p[axis] = rng.choice([-1, 1]) * rng.uniform(6.0, 30.0)
            goal = {"mode": "goto", **p}
        elif kind == "nan":
            p = {"x": 0.5, "y": 0.5, "z": -2.0}
            p[rng.choice("xyz")] = rng.choice([math.nan, math.inf, -math.inf])
            goal = {"mode": "goto", **p}
        elif kind == "jump":
            ang = rng.uniform(0, 2 * math.pi)
            r = rng.uniform(1.3, 3.0)
            goal = {"mode": "setpoint", "x": r * math.cos(ang), "y": r * math.sin(ang), "z": -2.0}
        else:
            goal = {
                "mode": "goto",
                "x": rng.uniform(0.3, 1.0),
                "y": 0.0,
                "z": -2.0,
                "speed": round(rng.uniform(1.5, 5.0), 2),
            }
        goals.append(
            {
                "goal": goal,
                "expect": {
                    "geofence": "geofence",
                    "nan": "non_finite",
                    "jump": "jump",
                    "speed": "speed_limit",
                }[kind],
            }
        )
    return goals


# -- one run ---------------------------------------------------------------------------------
class RunError(Exception):
    """Infrastructure problem before the fault was injected (not a verdict on the system)."""


def wait_for(pred, timeout: float, poll: float = 0.1):
    end = time.time() + timeout
    while time.time() < end:
        v = pred()
        if v:
            return v
        time.sleep(poll)
    return None


def find(rows: list[dict], pred, since: float = 0.0):
    for r in list(rows):
        if r["t"] >= since and pred(r):
            return r
    return None


def stop_observers(stack: Stack) -> None:
    stack.exec("referee", "pkill -f '[r]eferee.gt'; true", timeout=30)
    stack.exec("bridge", "pkill -f '[p]x4_monitor'; true", timeout=30)
    stack.exec(
        "host",
        "pkill -9 -f '[m]averick_flight/flight_client'; pkill -f '[i]nject_goals'; true",
        timeout=30,
    )


def execute(stack: Stack, case: str, params: dict) -> tuple[RunData, dict]:
    """Run one case on a ready stack. Returns the scored data and facts about how it ended."""
    stop_observers(stack)
    env_gz = {"GZ_PARTITION": stack.gz_partition or "", "PYTHONPATH": "/maverick/sim/referee"}
    gt = stack.stream("gt", "referee", "exec python3 -m referee.gt", env_gz)
    px4 = stack.stream(
        "px4", "bridge", "ROS_DOMAIN_ID=1 exec ros2 run maverick_flight px4_monitor px4"
    )
    brm = stack.stream("bridge", "bridge", "exec ros2 run maverick_flight px4_monitor bridge")
    stack.exec("host", ": > /tmp/flight.log", timeout=30)
    flight = stack.stream("flight", "host", "exec tail -n +1 -F /tmp/flight.log")
    streams = [gt, px4, brm, flight]
    facts = {"killed": False, "netem_cleared": True}
    fault: dict = {}
    hover_ref = None
    try:
        # Spawn height on the ground, before take-off.
        if not wait_for(lambda: len(gt.rows) >= 5 and len(px4.rows) >= 5, 30):
            raise RunError("no ground truth / PX4 samples")
        ground_z = sorted(r["z"] for r in gt.rows[-5:])[2]
        hold_s = params["hover_s"]
        stack.exec(
            "host",
            f"cd /tmp && nohup {FLIGHT_BIN}/flight_client --ros-args -p hover_seconds:={hold_s} "
            f"> /tmp/flight.log 2>&1 &",
            timeout=30,
        )
        hover = wait_for(
            lambda: (
                find(flight.rows, lambda e: e.get("event") == "phase" and e.get("phase") == "hover")
                or find(flight.rows, lambda e: e.get("event") == "exit")
            ),
            HOVER_TIMEOUT_S,
        )
        if not hover or hover.get("event") == "exit":
            raise RunError(f"hover not reached: {hover or 'timeout'}")
        t_fault = hover["t"] + params["offset_s"]
        # The settle period (2 s) is already behind us; take the realised hold from the last
        # 2 s of ground truth before the fault, and check it is a sane hover.
        wait_for(lambda: time.time() >= t_fault, params["offset_s"] + 5, 0.02)
        hover_ref = hover_reference(gt.rows, t_fault - 2.0, t_fault)
        if hover_ref is None or not (1.5 <= hover_ref[2] - ground_z <= 2.3):
            raise RunError(f"no hover before the fault (reference {hover_ref}, ground {ground_z})")

        if case == "F1":
            fault = faults.netem_window(stack, "outage", params["outage_s"])
        elif case == "F2":
            fault = faults.netem_window(stack, "bad", params["duration_s"])
        elif case == "F3":
            facts["killed"] = True
            fault = {
                "t_kill": faults.kill_process(stack, "host", "[m]averick_flight/flight_client")
            }
        elif case == "F4":
            facts["killed"] = True
            fault = {"t_kill": faults.kill_process(stack, "bridge", "[m]averick_bridge/relay")}
        else:
            fault = run_goals(stack, params)
            stack.exec(
                "host",
                "ros2 topic pub --once -w 1 /maverick/flight/mission "
                "std_msgs/msg/String '{data: land}'",
                timeout=60,
            )

        # Wait until PX4 is disarmed on the ground and the host client is gone (or give up).
        def disarmed_after() -> bool:
            e = find(px4.rows, lambda r: r.get("k") == "status" and r["arm"] == 1, t_fault)
            return e is not None

        wait_for(disarmed_after, END_TIMEOUT_S, 0.5)
        if case in ("F3", "F4") or facts["killed"]:
            pass
        else:
            wait_for(
                lambda: find(flight.rows, lambda e: e.get("event") == "exit", t_fault), 60, 0.5
            )
        time.sleep(POST_LAND_WATCH_S)
    finally:
        cleared = faults.netem_clear(stack)
        facts["netem_cleared"] = cleared
        stop_observers(stack)
        for s in streams:
            s.stop()

    facts["flight_exited"] = find(flight.rows, lambda e: e.get("event") == "exit") is not None
    status = [r for r in px4.rows if r.get("k") == "status"]
    facts["ended_disarmed"] = bool(status) and status[-1]["arm"] == 1
    landed_rows = [r for r in px4.rows if r.get("k") == "landed"]
    facts["ended_landed"] = bool(landed_rows) and landed_rows[-1]["landed"]
    data = RunData(
        case=case,
        params={k: v for k, v in params.items() if k != "goals"},
        gt=gt.rows,
        px4=px4.rows,
        flight=flight.rows,
        bridge=brm.rows,
        fault=fault,
        hover_ref=hover_ref,
        ground_z=ground_z,
    )
    return data, facts


def run_goals(stack: Stack, params: dict) -> dict:
    goals = [g["goal"] for g in params["goals"]]
    r = stack.exec(
        "host",
        f"ros2 run maverick_flight inject_goals {shlex.quote(json.dumps(goals))} --gap 1.0",
        timeout=120,
    )
    sent = [json.loads(line) for line in r.stdout.splitlines() if line.startswith("{")]
    if len(sent) != len(goals):
        raise RunError(f"injector sent {len(sent)}/{len(goals)} goals: {r.stderr[-300:]}")
    return {
        "goals": [
            {"t": s["t"], "goal": g["goal"], "expect": g["expect"]}
            for s, g in zip(sent, params["goals"], strict=True)
        ]
    }


def run_case(stack: Stack, case: str, params: dict) -> tuple[Verdict, dict]:
    data, facts = execute(stack, case, params)
    verdict = SCORERS[case](data)
    return verdict, facts


# -- suite -----------------------------------------------------------------------------------
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--cases", default=",".join(CASES))
    ap.add_argument("--runs-per-case", type=int, default=3)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--no-build", action="store_true", help="skip colcon build")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args(argv)
    cases = [c.strip().upper() for c in args.cases.split(",") if c.strip()]
    for c in cases:
        if c not in CASES:
            ap.error(f"unknown case {c}")

    out = args.out or ROOT / "runs" / f"s1-failsafe-{datetime.now():%Y%m%d-%H%M%S}"
    out.mkdir(parents=True, exist_ok=True)
    results = out / "results.jsonl"
    stack = Stack()
    if not args.no_build:
        print("colcon build ...", flush=True)
        stack.build_workspace()
    needs_restart = True
    summary: dict[str, dict] = {c: {"runs": 0, "pass": 0, "fail": 0, "error": 0} for c in cases}
    t_suite = time.time()
    try:
        for case in cases:
            for i in range(args.runs_per_case):
                rng = random.Random(f"{args.seed}/{case}/{i}")
                params = sample_params(case, rng)
                record = None
                for attempt in range(1, RETRY_ATTEMPTS + 1):
                    t_run = time.time()
                    restarted = None
                    if needs_restart:
                        restarted = round(stack.restart(), 1)
                    try:
                        verdict, facts = run_case(stack, case, params)
                    except RunError as exc:
                        needs_restart = True
                        record = {
                            "verdict": "error",
                            "reason": f"infrastructure: {exc}",
                            "metrics": {},
                        }
                        print(
                            f"  {case}#{i} attempt {attempt}: infrastructure error: {exc}",
                            flush=True,
                        )
                        continue
                    except Exception as exc:  # noqa: BLE001 - a broken run must not end the suite
                        needs_restart = True
                        record = {
                            "verdict": "error",
                            "reason": f"exception: {exc!r}",
                            "metrics": {},
                        }
                        print(f"  {case}#{i} attempt {attempt}: exception {exc!r}", flush=True)
                        continue
                    needs_restart = not (
                        facts["ended_disarmed"]
                        and facts["ended_landed"]
                        and not facts["killed"]
                        and facts["netem_cleared"]
                        and facts["flight_exited"]
                    )
                    record = {
                        "verdict": "pass" if verdict.passed else "fail",
                        "reason": verdict.reason,
                        "metrics": verdict.metrics,
                        "facts": facts,
                    }
                    break
                assert record is not None
                line = {
                    "case": case,
                    "run": i,
                    "seed": args.seed,
                    "params": {k: v for k, v in params.items() if k != "goals"},
                    "attempts": attempt,
                    "stack_restarted_s": restarted,
                    "duration_s": round(time.time() - t_run, 1),
                    **record,
                }
                if case == "F6":
                    line["params"]["goals"] = params["goals"]
                with results.open("a") as f:
                    f.write(json.dumps(line, allow_nan=True) + "\n")
                s = summary[case]
                s["runs"] += 1
                s[
                    "pass"
                    if record["verdict"] == "pass"
                    else "fail"
                    if record["verdict"] == "fail"
                    else "error"
                ] += 1
                print(
                    f"{case} #{i}: {record['verdict'].upper()} ({line['duration_s']} s) "
                    f"{record['reason']}",
                    flush=True,
                )
    finally:
        (out / "summary.json").write_text(
            json.dumps(
                {"seed": args.seed, "wall_s": round(time.time() - t_suite), "cases": summary},
                indent=2,
            )
        )
        stack.down()
    print("\nSummary (pass/runs):")
    for c, s in summary.items():
        print(f"  {c}: {s['pass']}/{s['runs']}  fail {s['fail']}  error {s['error']}")
    print(f"Results in {out}")
    return 0 if all(s["pass"] == s["runs"] for s in summary.values()) else 1


if __name__ == "__main__":
    sys.exit(main())

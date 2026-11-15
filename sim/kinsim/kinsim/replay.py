"""Vehicle identification by replay: feed a Gazebo run's recorded goals to the vehicle model.

The flight client logs every accepted goal with a wall-clock time (`stack-host.log`), the referee
records the drone's ground-truth track and PX4's own estimate (`raw.json`). Replaying the goals
through the real `FlightMission` / `SafetyGate` and `kinsim.vehicle` and comparing the predicted
track with the recorded one identifies the vehicle parameters without any of the drone stack's
decisions in the loop (those differ from run to run).

    python3 -m kinsim.replay build --runs runs/g4-explore runs/g4b-explore \
        --out sim/kinsim/data/replay.json.gz
    python3 -m kinsim.replay fit --data sim/kinsim/data/replay.json.gz --out vehicle_fit.json
    python3 -m kinsim.replay score --data ... [--params sim/kinsim/kinsim/calibrated.json]
"""

from __future__ import annotations

import argparse
import glob
import gzip
import json
import math
import re
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np
from maverick_flight.mission import (
    ARMING_STATE_ARMED,
    NAV_STATE_OFFBOARD,
    FlightMission,
    MissionConfig,
    Observation,
    Phase,
)
from maverick_flight.safety_gate import GateConfig, SafetyGate

from .vehicle import Vehicle, VehicleParams

DT = 0.025
_GOAL = re.compile(r'(\{"t": [0-9.]+, "event": "goal".*\})')


class _NoScene:
    """Free space: replay identifies the controller, not the contact."""

    def contact(self, p, half_height=0.12):
        return math.inf, np.array([1.0, 0.0]), -1

    def floor_height(self, xy):
        return 0.0


# -- dataset ------------------------------------------------------------------------------------
def build(run_roots: list[str], decimate: float = 0.1) -> list[dict]:
    out = []
    for root in run_roots:
        for line in Path(root, "results.jsonl").read_text().splitlines():
            r = json.loads(line)
            if r["verdict"] == "error" or r["seed"] >= 1_000_000:
                continue
            hits = glob.glob(f"{root}/run-{r['run']:03d}-*")
            if not hits:
                continue
            d = Path(hits[0])
            if not (d / "raw.json").exists() or not (d / "stack-host.log").exists():
                continue
            raw = json.loads((d / "raw.json").read_text())
            sp = raw["spawn_pose"]
            gz = raw["ground_z"]
            tr = np.array(raw["track"], float).reshape(-1, 4)
            tr[:, 1] -= sp[0]
            tr[:, 2] -= sp[1]
            tr[:, 3] -= gz
            keep = np.concatenate([[True], np.diff(tr[:, 0]) > 0])
            tr = tr[keep]
            est = np.array(
                [[t, y, x, -z] for t, x, y, z, ok in raw["px4_pos"] if ok],
                float,
            ).reshape(-1, 4)
            goals = []
            for ln in (d / "stack-host.log").read_text().splitlines():
                m = _GOAL.search(ln)
                if m:
                    g = json.loads(m.group(1))
                    if g.get("phase") == "hover":
                        goals.append([g["t"], *g["goal"], g.get("speed"), g.get("yaw")])
            t0 = raw["t_explore_start"]
            gt0 = [g for g in goals if g[0] >= t0 - 1.0]
            if not gt0:
                continue
            # decimate the track to 10 Hz
            t_arr = tr[:, 0]
            grid = np.arange(t_arr[0], t_arr[-1], decimate)
            tr10 = np.stack([grid] + [np.interp(grid, t_arr, tr[:, k]) for k in (1, 2, 3)], axis=1)
            out.append(
                {
                    "source": Path(root).name,
                    "seed": r["seed"],
                    "t0": t0,
                    "t1": raw["t_explore_end"],
                    "track": np.round(tr10, 3).tolist(),
                    "est": np.round(est, 3).tolist(),
                    "goals": [[round(g[0], 3), *g[1:]] for g in goals],
                    "collisions": [c["t"] for c in r["metrics"].get("collision_at", [])],
                }
            )
    return out


def save(data: list[dict], path: str) -> None:
    with gzip.open(path, "wt") as f:
        json.dump(data, f, separators=(",", ":"))


def load(path: str) -> list[dict]:
    with gzip.open(path, "rt") as f:
        return json.load(f)


# -- replay -------------------------------------------------------------------------------------
def _mission(p_xyz) -> FlightMission:
    gate = SafetyGate(GateConfig((-15.0, -15.0, -2.4), (15.0, 15.0, 0.3), 1.0, 1.0))
    m = FlightMission(MissionConfig(altitude=2.0, hover_seconds=None), gate)
    sp = (float(p_xyz[1]), float(p_xyz[0]), -float(p_xyz[2]))
    m.gate.origin = (0.0, 0.0, 0.0)
    m.setpoint = sp
    m.hold = m.goal = sp
    m.phase = Phase.HOVER
    m._t0 = m._phase_t0 = 0.0
    m.armed_ever = True
    return m


def setpoint_stream(run: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The flight client's setpoint (ENU) and yaw (ENU, NaN = none) at 20 Hz over the whole
    explore: a function of the recorded goals only (the real FlightMission and SafetyGate)."""
    goals = run["goals"]
    t_a = max(run["track"][0][0], goals[0][0] - 5.0)
    t_b = run["track"][-1][0]
    mission = _mission((0.0, 0.0, 2.0))
    gi = 0
    n = int((t_b - t_a) / 0.05)
    tt = t_a + 0.05 * np.arange(1, n + 1)
    sp = np.zeros((n, 3))
    yaw = np.full(n, np.nan)
    for k in range(n):
        while gi < len(goals) and goals[gi][0] <= tt[k]:
            g = goals[gi]
            mission.on_goal("goto", (g[1], g[2], g[3]), g[4], g[5])
            gi += 1
        act = mission.step(
            Observation(
                float(tt[k] - t_a),
                (0.0, 0.0, -2.0),
                ARMING_STATE_ARMED,
                NAV_STATE_OFFBOARD,
                False,
                0.0,
                True,
            )
        )
        if act.setpoint is not None:
            e = act.setpoint
            sp[k] = (e[1], e[0], -e[2])
            if math.isfinite(mission.yaw):
                yaw[k] = math.pi / 2 - mission.yaw
    return tt, sp, yaw


def simulate_window(
    run: dict,
    ts: float,
    warm: float,
    win: float,
    params: VehicleParams,
    seed: int = 0,
    noise: bool = False,
    stream=None,
) -> tuple[np.ndarray, np.ndarray]:
    """Replay [ts - warm, ts + win] with the recorded setpoint stream; returns (truth, predicted)
    positions at the 10 Hz samples in [ts, ts + win] (rows x, y, z)."""
    tr = np.array(run["track"], float)
    t = tr[:, 0]
    a = ts - warm
    i0 = int(np.searchsorted(t, a))
    if i0 + 2 >= len(tr):
        return np.zeros((0, 3)), np.zeros((0, 3))
    p0 = tr[i0, 1:4]
    v0 = (tr[i0 + 2, 1:4] - tr[i0, 1:4]) / (t[i0 + 2] - t[i0])
    tt_s, sp_s, yaw_s = stream if stream is not None else setpoint_stream(run)
    yaw0 = next((y for y in yaw_s if math.isfinite(y)), 0.0)
    rng = np.random.default_rng(seed)
    p = params if noise else replace(params, est_sigma_m=0.0, dist_sigma_acc=0.0)
    veh = Vehicle(_NoScene(), p, tuple(p0), yaw0, rng)  # type: ignore[arg-type]
    veh.vel[:] = v0
    veh.vel[2] = 0.0
    veh.t = 0.0
    veh._hist.clear()
    veh._push_hist()
    veh.est_pos = veh.pos.copy()
    n = int((warm + win) / DT)
    pred_t, pred = [], []
    yaw_sp = None
    for k in range(1, n + 1):
        tt = a + k * DT
        j = min(max(int(np.searchsorted(tt_s, tt, side="right")) - 1, 0), len(tt_s) - 1)
        y = yaw_s[j]
        yaw_sp = float(y) if math.isfinite(y) else yaw_sp
        veh.step(DT, sp_s[j], yaw_sp)
        pred_t.append(tt)
        pred.append(veh.pos.copy())
    pred_t, pred = np.array(pred_t), np.array(pred)
    sel = (t >= ts) & (t <= ts + win)
    truth = tr[sel, 1:4]
    pr = np.stack([np.interp(t[sel], pred_t, pred[:, c]) for c in range(3)], axis=1)
    return truth, pr


def windows(run: dict, win: float, every: float, skip_collision_s: float = 4.0) -> list[float]:
    """Window start times inside the explore, away from collisions (those need a contact model)."""
    t0 = max(run["t0"] + 6.0, run["track"][0][0] + 1)
    t1 = min(run["t1"], run["track"][-1][0]) - win
    out = []
    ts = t0
    while ts < t1:
        if not any(abs(c - ts - win / 2) < skip_collision_s + win / 2 for c in run["collisions"]):
            out.append(ts)
        ts += every
    return out


def run_loss(
    run: dict, params: VehicleParams, win: float = 8.0, every: float = 24.0, warm: float = 3.0
) -> list[float]:
    errs: list[float] = []
    if "_stream" not in run:
        run["_stream"] = setpoint_stream(run)
    stream = run["_stream"]
    for ts in windows(run, win, every):
        truth, pr = simulate_window(run, ts, warm, win, params, stream=stream)
        if len(truth) < 5:
            continue
        e = np.hypot(*(truth[:, :2] - pr[:, :2]).T)
        errs.extend(e.tolist())
    return errs


def huber(x: np.ndarray, d: float = 0.15) -> float:
    a = np.abs(x)
    return float(np.mean(np.where(a <= d, 0.5 * a * a, d * (a - 0.5 * d))))


FIT_NAMES = ("kp_xy", "kv_xy", "a_max", "tau_a", "v_max")
BOUNDS = {
    "kp_xy": (0.3, 3.0), "kv_xy": (0.5, 8.0), "a_max": (0.5, 8.0), "tau_a": (0.02, 0.6),
    "v_max": (0.7, 1.6), "est_delay_s": (0.0, 0.3),
}  # fmt: skip


def objective(x: np.ndarray, runs: list[dict], base: VehicleParams) -> float:
    kw = {}
    for name, v in zip(FIT_NAMES, x, strict=True):
        lo, hi = BOUNDS[name]
        kw[name] = float(min(max(v, lo), hi))
    p = replace(base, **kw)
    errs = []
    for r in runs:
        errs.extend(run_loss(r, p))
    return huber(np.array(errs)) if errs else 1.0


def fit(
    runs: list[dict], base: VehicleParams, iters: int = 160, seed: int = 0
) -> tuple[VehicleParams, float]:
    """(1+1) evolution strategy in log space (no scipy in the image): robust and good enough for
    six parameters with a cheap objective."""
    rng = np.random.default_rng(seed)
    names = FIT_NAMES
    x = np.log(np.array([max(getattr(base, n), 1e-3) for n in names]))
    best = objective(np.exp(x), runs, base)
    sigma = 0.25
    for it in range(iters):
        cand = x + rng.normal(size=len(x)) * sigma * (rng.random(len(x)) < 0.5)
        f = objective(np.exp(cand), runs, base)
        if f < best:
            x, best, sigma = cand, f, min(sigma * 1.3, 0.6)
            print(
                f"  it {it}: {best:.5f} {dict(zip(names, np.round(np.exp(x), 3), strict=True))}",
                flush=True,
            )
        else:
            sigma = max(sigma * 0.93, 0.03)
    kw = {
        n: float(min(max(v, BOUNDS[n][0]), BOUNDS[n][1]))
        for n, v in zip(names, np.exp(x), strict=True)
    }
    return replace(base, **kw), float(best)


def score(runs: list[dict], params: VehicleParams) -> dict:
    errs = np.array([e for r in runs for e in run_loss(r, params)])
    return {
        "windows_samples": int(len(errs)),
        "rmse_m": round(float(np.sqrt(np.mean(errs**2))), 4),
        "median_m": round(float(np.median(errs)), 4),
        "p90_m": round(float(np.percentile(errs, 90)), 4),
        "huber": round(huber(errs), 5),
    }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build")
    b.add_argument("--runs", nargs="+", required=True)
    b.add_argument("--out", required=True)
    f = sub.add_parser("fit")
    f.add_argument("--data", required=True)
    f.add_argument("--out", required=True)
    f.add_argument("--iters", type=int, default=160)
    s = sub.add_parser("score")
    s.add_argument("--data", required=True)
    s.add_argument("--params", default=None)
    a = ap.parse_args(argv)
    if a.cmd == "build":
        data = build(a.runs)
        save(data, a.out)
        print(f"{len(data)} runs -> {a.out}")
        return 0
    data = load(a.data)
    from .agreement import split_of
    from .params_io import load_params

    fit_runs = [r for r in data if split_of(r["seed"]) == "fit"]
    val_runs = [r for r in data if split_of(r["seed"]) == "validation"]
    if a.cmd == "score":
        p = load_params(a.params).vehicle
        print(json.dumps({"fit": score(fit_runs, p), "validation": score(val_runs, p)}, indent=1))
        return 0
    base = load_params().vehicle
    p, val = fit(fit_runs, base, a.iters)
    print("fit loss", val)
    print(json.dumps(asdict(p), indent=1))
    print(json.dumps({"fit": score(fit_runs, p), "validation": score(val_runs, p)}, indent=1))
    Path(a.out).write_text(json.dumps(asdict(p), indent=1) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

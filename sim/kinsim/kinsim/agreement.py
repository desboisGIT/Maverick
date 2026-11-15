"""Agreement between kinsim and Gazebo on the same seeds (the calibration's yardstick).

    python3 -m kinsim.agreement --kin runs/kinsim-explore/results.jsonl --split fit
    (or: `python3 -m kinsim.agreement --regenerate --seeds 20-23`: run kinsim on seeds of the
     reference and print the report; the test of the report uses this)

Statistics, all per seed pair (one Gazebo run vs one kinsim run of the same apartment):
  pass/fail agreement (and Cohen's kappa), coverage correlation, collisions per run and the
  agreement on "had a collision", duration ratio (explore skill time), plus the distribution of
  the track features (speed, tracking error) which do not need matching decisions.
Gazebo against itself (same seed, same code, repeated runs) bounds what any simulator can reach.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

DATA = Path(__file__).resolve().parents[1] / "data"
FEATURES = ("v_p50", "v_p90", "xtrack_p50", "xtrack_p90", "frac_moving", "first_plan_t")


def split_of(seed: int) -> str:
    """Calibration halves: even seeds fit, odd seeds validate (fixed, never re-drawn)."""
    return "fit" if seed % 2 == 0 else "validation"


def flatten(row: dict) -> dict:
    """A kinsim results row in the flat format of the Gazebo reference rows."""
    if "kin" not in row:  # already flat (Gazebo reference)
        return row
    m = row["metrics"]
    k = row["kin"]
    keys = (
        "coverage", "collisions", "min_clearance_m", "c6_recall", "c6_recall_v1",
        "surface_err_mean_m", "est_err_horiz_mean_m", "est_err_horiz_max_m", "n_snapshots",
        "explore_wall_s",
    )  # fmt: skip
    out = {
        "seed": row["seed"],
        "verdict": row["verdict"],
        "reason": row["reason"],
        "skill_s": k.get("explore_skill_s"),
        "n_stops": k.get("n_stops", 0),
        "features": k.get("features", {}),
        **{key: m.get(key) for key in keys},
    }
    if "goto" in row:
        out["goto"] = row["goto"]
    return out


def _corr(a: list[float], b: list[float]) -> float | None:
    if len(a) < 3 or np.std(a) < 1e-9 or np.std(b) < 1e-9:
        return None
    return round(float(np.corrcoef(a, b)[0, 1]), 3)


def _kappa(a: list[bool], b: list[bool]) -> float | None:
    n = len(a)
    if n == 0:
        return None
    po = sum(x == y for x, y in zip(a, b, strict=True)) / n
    pa, pb = sum(a) / n, sum(b) / n
    pe = pa * pb + (1 - pa) * (1 - pb)
    return None if pe >= 1 - 1e-9 else round((po - pe) / (1 - pe), 3)


def agreement(gaz: list[dict], kin: list[dict]) -> dict:
    """Compare matched rows (same seed). Rows are flat (see `flatten`)."""
    g = {r["seed"]: r for r in gaz}
    pairs = [(g[k["seed"]], k) for k in map(flatten, kin) if k["seed"] in g]
    n = len(pairs)
    if n == 0:
        return {"n": 0}
    gp = [r["verdict"] == "pass" for r, _ in pairs]
    kp = [k["verdict"] == "pass" for _, k in pairs]
    out: dict = {
        "n": n,
        "pass_rate_gazebo": round(sum(gp) / n, 3),
        "pass_rate_kinsim": round(sum(kp) / n, 3),
        "pass_fail_agreement": round(sum(a == b for a, b in zip(gp, kp, strict=True)) / n, 3),
        "kappa": _kappa(gp, kp),
    }
    gc, kc = [r["coverage"] for r, _ in pairs], [k["coverage"] for _, k in pairs]
    out["coverage_corr"] = _corr(gc, kc)
    out["coverage_mean_gazebo"] = round(float(np.mean(gc)), 3)
    out["coverage_mean_kinsim"] = round(float(np.mean(kc)), 3)
    out["coverage_mae"] = round(float(np.mean(np.abs(np.array(gc) - np.array(kc)))), 3)
    gn, kn = [r["collisions"] for r, _ in pairs], [k["collisions"] for _, k in pairs]
    out["collisions_per_run_gazebo"] = round(float(np.mean(gn)), 3)
    out["collisions_per_run_kinsim"] = round(float(np.mean(kn)), 3)
    out["collision_runs_gazebo"] = sum(x > 0 for x in gn)
    out["collision_runs_kinsim"] = sum(x > 0 for x in kn)
    out["collision_run_agreement"] = round(
        sum((a > 0) == (b > 0) for a, b in zip(gn, kn, strict=True)) / n, 3
    )
    gd = [(r["skill_s"], k["skill_s"]) for r, k in pairs if r.get("skill_s") and k.get("skill_s")]
    if gd:
        out["duration_ratio_mean"] = round(sum(b for _, b in gd) / sum(a for a, _ in gd), 3)
        out["duration_ratio_median"] = round(float(np.median([b / a for a, b in gd])), 3)
        out["duration_corr"] = _corr([a for a, _ in gd], [b for _, b in gd])
        out["duration_mean_gazebo_s"] = round(float(np.mean([a for a, _ in gd])), 1)
        out["duration_mean_kinsim_s"] = round(float(np.mean([b for _, b in gd])), 1)

    # per failure kind: which verdict components fail (coverage, collision, C6)
    def kinds(r: dict) -> set[str]:
        s = set()
        if r["coverage"] < 0.9:
            s.add("coverage")
        if r["collisions"] > 0:
            s.add("collision")
        if r["c6_recall"] < 0.95:
            s.add("c6")
        return s

    for kind in ("coverage", "collision", "c6"):
        out[f"fail_{kind}_gazebo"] = sum(kind in kinds(r) for r, _ in pairs)
        out[f"fail_{kind}_kinsim"] = sum(kind in kinds(k) for _, k in pairs)
    feats = {}
    for f in FEATURES:
        a = [r["features"][f] for r, _ in pairs if f in r.get("features", {})]
        b = [k["features"][f] for _, k in pairs if f in k.get("features", {})]
        if a and b:
            feats[f] = {
                "gazebo": round(float(np.mean(a)), 3),
                "kinsim": round(float(np.mean(b)), 3),
            }
    out["features"] = feats
    ge = [r["est_err_horiz_mean_m"] for r, _ in pairs if r.get("est_err_horiz_mean_m") is not None]
    ke = [k["est_err_horiz_mean_m"] for _, k in pairs if k.get("est_err_horiz_mean_m") is not None]
    if ge and ke:
        out["est_err_mean_gazebo_m"] = round(float(np.mean(ge)), 3)
        out["est_err_mean_kinsim_m"] = round(float(np.mean(ke)), 3)
    return out


def goto_agreement(gaz: list[dict], kin: list[dict]) -> dict:
    """goto_room: C2 verdict agreement, where the drone ended, how the skill ended."""
    g = {r["seed"]: r for r in gaz if "goto" in r}
    pairs = [(g[k["seed"]], k) for k in map(flatten, kin) if k["seed"] in g and "goto" in k]
    n = len(pairs)
    if n == 0:
        return {"n": 0}
    gp = [r["verdict"] == "pass" for r, _ in pairs]
    kp = [k["verdict"] == "pass" for _, k in pairs]
    ge = [r["goto"].get("edge_distance_m") or 0.0 for r, _ in pairs]
    ke = [k["goto"].get("edge_distance_m") or 0.0 for _, k in pairs]
    return {
        "n": n,
        "pass_rate_gazebo": round(sum(gp) / n, 3),
        "pass_rate_kinsim": round(sum(kp) / n, 3),
        "pass_fail_agreement": round(sum(a == b for a, b in zip(gp, kp, strict=True)) / n, 3),
        "old_rule_pass_gazebo": sum(bool(r["goto"].get("old_rule_pass")) for r, _ in pairs),
        "old_rule_pass_kinsim": sum(bool(k["goto"].get("old_rule_pass")) for _, k in pairs),
        "skill_succeeded_gazebo": sum(
            r["goto"].get("skill_state") == "succeeded" for r, _ in pairs
        ),
        "skill_succeeded_kinsim": sum(
            k["goto"].get("skill_state") == "succeeded" for _, k in pairs
        ),
        "inside_gazebo": sum(bool(r["goto"].get("inside")) for r, _ in pairs),
        "inside_kinsim": sum(bool(k["goto"].get("inside")) for _, k in pairs),
        "edge_distance_mean_gazebo_m": round(float(np.mean(ge)), 3),
        "edge_distance_mean_kinsim_m": round(float(np.mean(ke)), 3),
        "edge_distance_corr": _corr(ge, ke),
        "explore_duration_corr": _corr(
            [r["skill_s"] for r, _ in pairs], [k["skill_s"] for _, k in pairs]
        ),
    }


def gazebo_self_agreement(groups: list[list[dict]]) -> dict:
    """Pass/fail agreement of repeated Gazebo runs of the same seeds (an upper bound).

    `groups`: one list of flat rows per repetition (same seeds in each).
    """
    by_seed: dict[int, list[bool]] = {}
    for grp in groups:
        for r in grp:
            by_seed.setdefault(r["seed"], []).append(r["verdict"] == "pass")
    agree = total = 0
    for v in by_seed.values():
        for i in range(len(v)):
            for j in range(i + 1, len(v)):
                agree += v[i] == v[j]
                total += 1
    return {
        "seeds": len(by_seed),
        "pairs": total,
        "pass_fail_agreement": round(agree / total, 3) if total else None,
    }


def report(gaz: list[dict], kin_rows: list[dict]) -> dict:
    """The agreement table: fit half, validation half, all."""
    out = {}
    for name in ("fit", "validation", "all"):
        gs = [r for r in gaz if name == "all" or split_of(r["seed"]) == name]
        ks = [r for r in kin_rows if name == "all" or split_of(r["seed"]) == name]
        out[name] = agreement(gs, ks)
    return out


def load_reference(path: str | Path | None = None, kind: str = "explore") -> list[dict]:
    return json.loads(Path(path or DATA / f"gazebo_{kind}.json").read_text())


def load_results(path: str | Path) -> list[dict]:
    return [
        json.loads(line)
        for line in Path(path).read_text().splitlines()
        if line.strip() and json.loads(line).get("verdict") in ("pass", "fail")
    ]


def format_table(rep: dict) -> str:
    keys = [
        "n", "pass_rate_gazebo", "pass_rate_kinsim", "pass_fail_agreement", "kappa",
        "coverage_corr", "coverage_mae", "collisions_per_run_gazebo", "collisions_per_run_kinsim",
        "collision_run_agreement", "duration_ratio_mean", "duration_ratio_median", "duration_corr",
    ]  # fmt: skip
    lines = [f"{'':32s}" + "".join(f"{n:>12s}" for n in rep)]
    for k in keys:
        lines.append(f"{k:32s}" + "".join(f"{str(rep[n].get(k, '-')):>12s}" for n in rep))
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--kin", required=True, help="kinsim results.jsonl")
    ap.add_argument("--reference", default=None)
    ap.add_argument(
        "--goto", action="store_true", help="goto_room results (reference: gazebo_goto)"
    )
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    if a.goto:
        gaz, res = load_reference(a.reference, "goto"), load_results(a.kin)
        print(json.dumps(goto_agreement(gaz, res), indent=1))
        return 0
    rep = report(load_reference(a.reference), load_results(a.kin))
    print(json.dumps(rep, indent=1) if a.json else format_table(rep))
    if not a.json:
        for name in ("fit", "validation"):
            print(name, json.dumps(rep[name].get("features", {})))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

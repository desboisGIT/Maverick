"""S4b verdicts (pure Python, stdlib only): search, come_to and return against the manifest.

Ground truth enters here and only here (referee side): the manifest's small objects, presence
flags and person. All positions are compared in the world frame; the drone's map frame is the
world shifted by the spawn point (referee.s3_frames).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

POSITION_TOL_M = 0.3  # found position vs the real object
COME_TO_MAX_M = 1.5  # ends within this of the person ...
COME_TO_MIN_M = 1.0  # ... and never closer than this
RETURN_TOL_M = 0.5


@dataclass
class Verdict:
    passed: bool
    reason: str
    metrics: dict

    def as_dict(self) -> dict:
        return {"verdict": "pass" if self.passed else "fail", "reason": self.reason, **self.metrics}


def search_expected(manifest: dict, label: str) -> dict | None:
    """The placed object of class `label` (manifest entry), or None when it is absent."""
    for o in manifest.get("small_objects", []):
        if o["class"] == label:
            return o
    return None


def verdict_search(manifest: dict, label: str, result: dict | None, spawn_xy) -> Verdict:
    """Correct = found when present (within 0.3 m), not found when absent."""
    truth = search_expected(manifest, label)
    m: dict = {"object": label, "present": truth is not None}
    if not result or "found" not in result:
        return Verdict(False, "the search returned no structured result", m)
    found = bool(result["found"])
    m["found"] = found
    m["rooms_searched"] = result.get("rooms_searched")
    if found:
        px, py = result["position"][:2]
        wx, wy = px + spawn_xy[0], py + spawn_xy[1]  # map -> world
        m["found_world_xy"] = [round(wx, 3), round(wy, 3)]
    if truth is None:
        if found:
            m["false_positive"] = True
            return Verdict(False, f"false positive: reported a {label} that is not there", m)
        return Verdict(True, f"correctly not found ({label} absent)", m)
    tx, ty = truth["position"][:2]
    m["true_xy"] = [tx, ty]
    if not found:
        return Verdict(False, f"missed the {label} (searched {m['rooms_searched']})", m)
    err = math.hypot(wx - tx, wy - ty)
    m["position_error_m"] = round(err, 3)
    if err >= POSITION_TOL_M:
        m["false_positive"] = True
        return Verdict(False, f"{label} reported {err:.2f} m from the real one", m)
    return Verdict(True, f"found the {label}, {err:.2f} m off", m)


def verdict_come_to(person_xy, end_xy, min_dist: float | None, skill_state: str) -> Verdict:
    d = math.hypot(end_xy[0] - person_xy[0], end_xy[1] - person_xy[1])
    m = {
        "end_distance_m": round(d, 3),
        "min_distance_m": round(min_dist, 3) if min_dist is not None else None,
        "skill_state": skill_state,
    }
    if skill_state != "succeeded":
        return Verdict(False, f"skill {skill_state}", m)
    if min_dist is not None and min_dist < COME_TO_MIN_M:
        return Verdict(False, f"came within {min_dist:.2f} m of the person (limit 1.0)", m)
    if d < COME_TO_MIN_M:
        return Verdict(False, f"ended {d:.2f} m from the person (limit 1.0)", m)
    if d > COME_TO_MAX_M:
        return Verdict(False, f"ended {d:.2f} m from the person (needs <= 1.5)", m)
    return Verdict(True, f"ended {d:.2f} m from the person", m)


def verdict_return(spawn_xy, end_xy, skill_state: str) -> Verdict:
    d = math.hypot(end_xy[0] - spawn_xy[0], end_xy[1] - spawn_xy[1])
    m = {"end_distance_m": round(d, 3), "skill_state": skill_state}
    if skill_state != "succeeded":
        return Verdict(False, f"skill {skill_state}", m)
    if d > RETURN_TOL_M:
        return Verdict(False, f"ended {d:.2f} m from the take-off point", m)
    return Verdict(True, f"back within {d:.2f} m of the take-off point", m)


def min_distance(track: list[dict], point, t0: float, t1: float) -> float | None:
    """Closest approach of the ground-truth track (rows with t, x, y) to a point in [t0, t1]."""
    ds = [math.hypot(r["x"] - point[0], r["y"] - point[1]) for r in track if t0 <= r["t"] <= t1]
    return min(ds) if ds else None


def summarise_s4(rows: list[dict]) -> dict:
    """Tally over result rows (each has `search` {label: verdict dict}, `come_to`, `return`)."""
    s = {"runs": len(rows), "errors": sum(r.get("verdict") == "error" for r in rows)}
    srch = [(lab, v) for r in rows for lab, v in (r.get("search") or {}).items()]
    s["search_total"] = len(srch)
    s["search_correct"] = sum(v["verdict"] == "pass" for _, v in srch)
    for key, present in (("present", True), ("absent", False)):
        sub = [v for _, v in srch if v.get("present") is present]
        s[f"search_{key}_total"] = len(sub)
        s[f"search_{key}_correct"] = sum(v["verdict"] == "pass" for v in sub)
    s["false_positives"] = sum(bool(v.get("false_positive")) for _, v in srch)
    ct = [r["come_to"] for r in rows if r.get("come_to")]
    s["come_to_total"] = len(ct)
    s["come_to_pass"] = sum(c["verdict"] == "pass" for c in ct)
    rt = [r["return"] for r in rows if r.get("return")]
    s["return_total"] = len(rt)
    s["return_pass"] = sum(c["verdict"] == "pass" for c in rt)
    # a seed counts as a search pass when every search it ran was correct
    ok = [all(v["verdict"] == "pass" for v in (r.get("search") or {}).values()) for r in rows
          if r.get("search")]  # fmt: skip
    s["seeds_search_all_correct"] = sum(ok)
    s["seeds_with_search"] = len(ok)
    return s

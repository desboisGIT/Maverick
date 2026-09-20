"""Pass/fail logic of the S3 scenarios (stdlib only, unit-tested; metrics come from s3_metrics).

Thresholds are the exit tests in docs/s2-plan.md (hover, item 3) and docs/s3-plan.md (C1, C2, C6).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from referee.scoring import ARM_DISARMED, deviation, flight_phase_t, hover_reference

HOVER_MAX_DRIFT_M = 0.3
HOVER_MIN_HEIGHT_M = 0.8  # hover must really be in the air (altitude 1.2 m)
GROUND_TOL_M = 0.15
C1_COVERAGE = 0.90
C1_SURFACE_ERR_M = 0.10  # mean distance of occupied voxels to the nearest GT surface
C6_RECALL = 0.95
EST_ERR_MEAN_M = 0.15  # PX4 position estimate vs ground truth during the exploration, horizontal
EST_ERR_MAX_M = 0.30
EST_MIN_FLIGHT_Z = 0.4  # ground-truth height above ground below which samples are ignored
C2_CLEAR_M = 1.0
C2_MARGIN_M = 0.05
BODY_RADIUS_M = 0.39  # x500 circumscribed radius
CAPABILITY_SCENARIOS = ("explore", "goto_room")  # scored on held-out seeds for exit results


@dataclass
class Verdict:
    passed: bool
    reason: str
    metrics: dict = field(default_factory=dict)


# -- geometry (room polygons) ----------------------------------------------------------------
def point_in_polygon(p, poly) -> bool:
    inside = False
    n = len(poly)
    for k in range(n):
        x0, y0 = poly[k]
        x1, y1 = poly[(k + 1) % n]
        if (y0 > p[1]) != (y1 > p[1]) and p[0] < x0 + (p[1] - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside


def edge_distance(p, poly) -> float:
    best = math.inf
    n = len(poly)
    for k in range(n):
        ax, ay = poly[k]
        bx, by = poly[(k + 1) % n]
        dx, dy = bx - ax, by - ay
        t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / max(dx * dx + dy * dy, 1e-12)
        t = min(max(t, 0.0), 1.0)
        best = min(best, math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy)))
    return best


def inscribed_radius(poly, step: float = 0.05) -> float:
    """Largest distance from a polygon edge reached by any point inside it (grid search)."""
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    best = 0.0
    x = min(xs)
    while x <= max(xs):
        y = min(ys)
        while y <= max(ys):
            if point_in_polygon((x, y), poly):
                best = max(best, edge_distance((x, y), poly))
            y += step
        x += step
    return best


def c2_required_clearance(inscribed: float) -> float:
    """min(1.0, inscribed radius - 0.05): a room too small for 1 m inside is judged by what fits."""
    return min(C2_CLEAR_M, inscribed - C2_MARGIN_M)


# -- verdicts ----------------------------------------------------------------------------------
def hover_metrics(gt: list[dict], flight: list[dict], px4: list[dict], ground_z: float) -> dict:
    """Drift of the 10 s hover (ground truth) and how the run ended (PX4's own view)."""
    t_h = flight_phase_t(flight, "hover")
    m: dict = {"hover_reached": t_h is not None}
    if t_h is None:
        return m
    t_land = flight_phase_t(flight, "land", t_h) or (t_h + 10.0)
    ref = hover_reference(gt, t_h + 2.0, t_land)  # 2 s settle, as the S1 suite
    dev = deviation(gt, ref, t_h + 2.0, t_land)
    m["hover_height_m"] = None if ref is None else round(ref[2] - ground_z, 2)
    m["drift_3d_m"] = None if dev["n"] == 0 else round(dev["max_3d"], 3)
    m["drift_horiz_m"] = None if dev["n"] == 0 else round(dev["max_horiz"], 3)
    status = [e for e in px4 if e.get("k") == "status"]
    landed = [e for e in px4 if e.get("k") == "landed"]
    m["disarmed"] = bool(status) and status[-1]["arm"] == ARM_DISARMED
    m["landed"] = bool(landed) and bool(landed[-1]["landed"])
    last = gt[-1] if gt else None
    m["final_height_m"] = None if last is None else round(last["z"] - ground_z, 3)
    return m


def verdict_hover(m: dict) -> Verdict:
    if not m.get("hover_reached"):
        return Verdict(False, "hover phase never reached", m)
    if m["drift_3d_m"] is None:
        return Verdict(False, "no ground truth during the hover", m)
    if m["hover_height_m"] is None or m["hover_height_m"] < HOVER_MIN_HEIGHT_M:
        return Verdict(False, f"hover height {m['hover_height_m']} m too low", m)
    if m["drift_3d_m"] > HOVER_MAX_DRIFT_M:
        return Verdict(False, f"drift {m['drift_3d_m']} m > {HOVER_MAX_DRIFT_M} m", m)
    if not m["landed"] or not m["disarmed"]:
        return Verdict(False, f"landed={m['landed']} disarmed={m['disarmed']}", m)
    if m["final_height_m"] is None or m["final_height_m"] > GROUND_TOL_M:
        return Verdict(False, f"final height {m['final_height_m']} m, not on the ground", m)
    return Verdict(True, f"drift {m['drift_3d_m']} m, landed and disarmed", m)


def estimate_error(
    gt: list[dict], px4: list[dict], origin, t0: float, t1: float, ground_z: float
) -> dict:
    """PX4's own position estimate against ground truth over [t0, t1] (both wall-clock stamped).

    `gt` rows {t, x, y, z} in the Gazebo world frame; `px4` rows {k: "pos", t, x, y, z, valid} in
    PX4 local NED (x north, y east, z down). The estimate is converted to the map frame (ENU,
    origin = PX4 local origin = `origin`, z up from the floor) and compared with the ground truth
    interpolated at the estimate's own time; samples with the drone below EST_MIN_FLIGHT_Z are
    skipped. Returns horizontal and vertical error statistics (m), or {"est_samples": 0}.
    """
    pts = [(r["t"], r["x"], r["y"], r["z"]) for r in gt]
    pts.sort()
    hs: list[float] = []
    vs: list[float] = []
    j = 0
    for r in px4:
        if r.get("k") != "pos" or not r.get("valid") or not (t0 <= r["t"] <= t1):
            continue
        t = r["t"]
        while j + 1 < len(pts) and pts[j + 1][0] < t:
            j += 1
        if j + 1 >= len(pts) or pts[j][0] > t:
            continue
        a, b = pts[j], pts[j + 1]
        w = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
        gx = a[1] + w * (b[1] - a[1]) - origin[0]
        gy = a[2] + w * (b[2] - a[2]) - origin[1]
        gz = a[3] + w * (b[3] - a[3]) - ground_z
        if gz < EST_MIN_FLIGHT_Z:
            continue
        ex, ey, ez = r["y"], r["x"], -r["z"]  # NED -> ENU
        hs.append(math.hypot(ex - gx, ey - gy))
        vs.append(abs(ez - gz))
    if not hs:
        return {"est_samples": 0}
    hs_sorted = sorted(hs)
    return {
        "est_samples": len(hs),
        "est_err_horiz_mean_m": round(sum(hs) / len(hs), 3),
        "est_err_horiz_p95_m": round(hs_sorted[int(0.95 * (len(hs) - 1))], 3),
        "est_err_horiz_max_m": round(hs_sorted[-1], 3),
        "est_err_vert_mean_m": round(sum(vs) / len(vs), 3),
        "est_err_vert_max_m": round(max(vs), 3),
    }


def verdict_explore(m: dict) -> Verdict:
    """C1 (coverage, collisions, surface error) and C6 (frontier recall) of one exploration."""
    fails = []
    if m["coverage"] < C1_COVERAGE:
        fails.append(f"C1 coverage {m['coverage']:.3f} < {C1_COVERAGE}")
    if m["collisions"] > 0:
        fails.append(f"C1 {m['collisions']} collision(s)")
    if not m["surface_err_mean_m"] < C1_SURFACE_ERR_M:  # also catches NaN
        fails.append(f"C1 surface error {m['surface_err_mean_m']} m >= {C1_SURFACE_ERR_M} m")
    if not m["c6_recall"] >= C6_RECALL:
        fails.append(f"C6 recall {m['c6_recall']} < {C6_RECALL}")
    if m.get("est_samples"):  # estimate-vs-ground-truth targets of the visual odometry (S3b)
        if m["est_err_horiz_mean_m"] > EST_ERR_MEAN_M:
            fails.append(f"estimate error mean {m['est_err_horiz_mean_m']} m > {EST_ERR_MEAN_M} m")
        if m["est_err_horiz_max_m"] > EST_ERR_MAX_M:
            fails.append(f"estimate error max {m['est_err_horiz_max_m']} m > {EST_ERR_MAX_M} m")
    if fails:
        return Verdict(False, "; ".join(fails), m)
    return Verdict(
        True, f"coverage {m['coverage']:.3f}, 0 collisions, C6 recall {m['c6_recall']}", m
    )


def goto_metrics(manifest: dict, room_type: str, final_map_xy, origin) -> dict:
    """C2 geometry: where the drone ended against the GT room polygon (all in the map frame)."""
    room = next((r for r in manifest["rooms"] if r["type"] == room_type), None)
    if room is None:
        return {"room_type": room_type, "room_found": False}
    ox, oy = origin[0], origin[1]
    poly = [(x - ox, y - oy) for x, y in room["polygon"]]
    inside = point_in_polygon(final_map_xy, poly)
    insc = inscribed_radius(poly)
    return {
        "room_type": room_type,
        "room_found": True,
        "inside": inside,
        "edge_distance_m": round(edge_distance(final_map_xy, poly), 3) if inside else 0.0,
        "inscribed_m": round(insc, 3),
        "required_m": round(c2_required_clearance(insc), 3),
    }


def verdict_goto(m: dict) -> Verdict:
    if not m.get("room_found"):
        return Verdict(False, f"no {m.get('room_type')} in the ground-truth manifest", m)
    if not m.get("skill_succeeded", True):
        return Verdict(False, f"skill ended {m.get('skill_state')}: {m.get('skill_message')}", m)
    if not m["inside"]:
        return Verdict(False, f"drone ended outside the {m['room_type']}", m)
    if m["edge_distance_m"] < m["required_m"]:
        return Verdict(
            False,
            f"ended {m['edge_distance_m']} m inside, need {m['required_m']} m "
            f"(inscribed radius {m['inscribed_m']} m)",
            m,
        )
    return Verdict(True, f"ended {m['edge_distance_m']} m inside the {m['room_type']}", m)


# -- seed sets and summary -----------------------------------------------------------------------
def seed_set_of(seed: int) -> str:
    if 0 <= seed < 100_000:
        return "tuning"
    if 1_000_000 <= seed < 1_100_000:
        return "held_out"
    return "other"


def exit_result_warning(scenario: str, seed_sets: set[str]) -> str | None:
    """Capability results on tuning seeds are development numbers, never exit results."""
    if scenario in CAPABILITY_SCENARIOS and seed_sets - {"held_out"}:
        return (
            f"WARNING: scenario {scenario} on {sorted(seed_sets - {'held_out'})} seeds: these are "
            "development results, NOT exit results (exit tests use held_out seeds only)"
        )
    return None


def summarise(rows: list[dict], scenario: str) -> dict:
    ran = [r for r in rows if r["verdict"] in ("pass", "fail")]
    sets = {r["seed_set"] for r in rows}
    out = {
        "scenario": scenario,
        "runs": len(rows),
        "pass": sum(r["verdict"] == "pass" for r in rows),
        "fail": sum(r["verdict"] == "fail" for r in rows),
        "error": sum(r["verdict"] == "error" for r in rows),
        "scored": len(ran),
        "seed_sets": sorted(sets),
        "exit_result": scenario not in CAPABILITY_SCENARIOS or sets == {"held_out"},
    }
    warn = exit_result_warning(scenario, sets)
    if warn:
        out["warning"] = warn
    return out

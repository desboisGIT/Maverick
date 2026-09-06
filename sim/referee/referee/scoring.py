"""Pass/fail scoring for the failsafe suite (pure Python, unit-tested).

Criteria are the exit-test table in docs/s1-plan.md. Every verdict carries a reason that says
which number decided it. Positions come from Gazebo ground truth (ENU metres, `gt`); PX4's own
view (nav_state, arming, landed) comes from the px4 monitor events, which do not depend on the
relay or the host being alive.

Inputs are lists of dicts, each with a wall-clock "t":
  gt      {"t","x","y","z"}                  ground truth
  px4     {"k": "status","nav","arm","failsafe"} | {"k":"landed","landed"} | {"k":"sp","p":[..]}
  flight  host flight client stdout events ({"event": "phase"|"rejection"|"command"|"exit", ..})
  bridge  {"k": "bridge","forwarded","dropped_stale","age_ms"}
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

NAV_OFFBOARD = 14
NAV_AUTO_LAND = 18
ARM_DISARMED = 1
ARM_ARMED = 2

COM_OF_LOSS_T = 0.5  # sim/px4/airframes/4500_gz_maverick
F4_ENGAGE_MARGIN_S = 1.0
LAND_AFTER_FAILSAFE_S = 40.0
HOST_ABORT_LAND_S = 25.0  # F1 long, F3: from outage start / kill
SHORT_MAX_DEV_M = 0.5
MEDIUM_MAX_ALT_LOSS_M = 1.2
MEDIUM_MAX_HORIZ_M = 0.5
F2_MAX_DEV_M = 0.5
F2_MAX_AGE_MS = 200.0
F6_MAX_DEV_M = 0.5
F6_SETPOINT_TOL_M = 0.02
GROUND_TOL_M = 0.15  # "on the ground": within this of the spawn height
# F1 medium: gaps this short sit at PX4's detection edge (README measurement 2b: >= 1 s always
# engages, 1.0-1.4 s after the last setpoint), so engagement is only required above this gap.
MEDIUM_ENGAGE_REQUIRED_S = 1.5


@dataclass
class Verdict:
    passed: bool
    reason: str
    metrics: dict = field(default_factory=dict)


@dataclass
class RunData:
    case: str
    params: dict
    gt: list[dict]
    px4: list[dict]
    flight: list[dict]
    bridge: list[dict]
    fault: dict  # t_start, t_end (outage / netem window) or t_kill; "goals" for F6
    hover_ref: tuple[float, float, float] | None  # realised hold point in GT frame
    ground_z: float  # GT z of the drone on the ground before take-off


# -- helpers ---------------------------------------------------------------------------------
def window(rows: list[dict], t0: float, t1: float) -> list[dict]:
    return [r for r in rows if t0 <= r["t"] <= t1]


def hover_reference(gt: list[dict], t0: float, t1: float) -> tuple[float, float, float] | None:
    """Realised hold point: per-axis median of ground truth in [t0, t1]."""
    rows = window(gt, t0, t1)
    if len(rows) < 5:
        return None
    med = []
    for axis in ("x", "y", "z"):
        vals = sorted(r[axis] for r in rows)
        med.append(vals[len(vals) // 2])
    return (med[0], med[1], med[2])


def deviation(gt: list[dict], ref, t0: float, t1: float) -> dict:
    """Max 3D / horizontal distance from ref and the max drop below ref.z, over [t0, t1]."""
    rows = window(gt, t0, t1)
    if not rows or ref is None:
        return {"n": 0, "max_3d": math.nan, "max_horiz": math.nan, "max_alt_loss": math.nan}
    return {
        "n": len(rows),
        "max_3d": max(math.dist((r["x"], r["y"], r["z"]), ref) for r in rows),
        "max_horiz": max(math.hypot(r["x"] - ref[0], r["y"] - ref[1]) for r in rows),
        "max_alt_loss": max(ref[2] - r["z"] for r in rows),
    }


def px4_events(px4: list[dict], kind: str = "status") -> list[dict]:
    return [e for e in px4 if e.get("k") == kind]


def first_event(rows: list[dict], t_min: float, pred) -> dict | None:
    for r in rows:
        if r["t"] >= t_min and pred(r):
            return r
    return None


def landing_summary(run: RunData, t_after: float) -> dict:
    """Disarm / re-arm facts after t_after, from the PX4 monitor."""
    status = px4_events(run.px4)
    disarm = first_event(status, t_after, lambda e: e["arm"] == ARM_DISARMED)
    rearm = None
    if disarm is not None:
        rearm = first_event(status, disarm["t"], lambda e: e["arm"] == ARM_ARMED)
    last_gt = run.gt[-1] if run.gt else None
    on_ground = last_gt is not None and last_gt["z"] - run.ground_z < GROUND_TOL_M
    return {
        "t_disarm": disarm["t"] if disarm else None,
        "rearmed": rearm is not None,
        "on_ground_at_end": on_ground,
        "final_height_m": (last_gt["z"] - run.ground_z) if last_gt else math.nan,
    }


def crashed(run: RunData) -> str | None:
    """A disarm in the air, or ground truth below the floor, is a crash."""
    for e in px4_events(run.px4):
        if e["arm"] == ARM_DISARMED:
            g = first_event(run.gt, e["t"], lambda r: True)
            if g is not None and g["z"] - run.ground_z > 0.5:
                return f"disarmed at {g['z'] - run.ground_z:.2f} m above ground"
    if run.gt and min(r["z"] for r in run.gt) < run.ground_z - 0.3:
        return "ground truth below the floor"
    return None


def flight_result(flight: list[dict]) -> str | None:
    for e in reversed(flight):
        if e.get("event") == "exit":
            return e.get("result")
        if e.get("event") == "phase" and e.get("phase") == "done":
            return e.get("result")
    return None


def flight_phase_t(flight: list[dict], phase: str, t_min: float = 0.0) -> float | None:
    e = first_event(flight, t_min, lambda r: r.get("event") == "phase" and r.get("phase") == phase)
    return e["t"] if e else None


def _fail(reason: str, **metrics) -> Verdict:
    return Verdict(False, reason, metrics)


def _ok(reason: str, **metrics) -> Verdict:
    return Verdict(True, reason, metrics)


def _r(x, n=2):
    return round(x, n) if isinstance(x, float) and math.isfinite(x) else x


# -- F1 --------------------------------------------------------------------------------------
def score_f1(run: RunData) -> Verdict:
    kind = run.params["kind"]
    t0, t1 = run.fault["t_start"], run.fault["t_end"]
    d = t1 - t0
    status = px4_events(run.px4)
    nav18 = first_event(status, t0, lambda e: e["nav"] == NAV_AUTO_LAND)
    result = flight_result(run.flight)
    crash = crashed(run)
    if crash:
        return _fail(f"crash: {crash}", kind=kind)
    if kind == "short":
        failsafe = first_event(status, t0, lambda e: e["nav"] == NAV_AUTO_LAND and e["t"] <= t1 + 3)
        dev = deviation(run.gt, run.hover_ref, t0, t1 + 5)
        m = {"outage_s": _r(d), "max_dev_m": _r(dev["max_3d"]), "flight_result": result}
        if failsafe is not None:
            return _fail(f"short outage {d:.2f} s triggered a PX4 failsafe (nav_state 18)", **m)
        if not dev["max_3d"] <= SHORT_MAX_DEV_M:
            return _fail(f"deviation {dev['max_3d']:.2f} m > {SHORT_MAX_DEV_M} m", **m)
        if result != "ok":
            return _fail(f"mission did not complete (host result {result})", **m)
        return _ok(f"no failsafe, deviation {dev['max_3d']:.2f} m <= {SHORT_MAX_DEV_M} m", **m)
    if kind == "medium":
        t_land = flight_phase_t(run.flight, "land", t0) or (t1 + 15)
        dev = deviation(run.gt, run.hover_ref, t0, t_land)
        back = (
            first_event(status, nav18["t"], lambda e: e["nav"] == NAV_OFFBOARD) if nav18 else None
        )
        m = {
            "outage_s": _r(d),
            "failsafe": nav18 is not None,
            "back_to_offboard": back is not None,
            "alt_loss_m": _r(dev["max_alt_loss"]),
            "max_horiz_m": _r(dev["max_horiz"]),
            "flight_result": result,
        }
        if nav18 is None and d >= MEDIUM_ENGAGE_REQUIRED_S:
            return _fail(f"outage {d:.2f} s but PX4 never started landing (nav_state 18)", **m)
        if nav18 is not None and back is None:
            return _fail("PX4 did not return to offboard after setpoints resumed", **m)
        if result != "ok":
            return _fail(f"mission did not complete (host result {result})", **m)
        if not dev["max_alt_loss"] <= MEDIUM_MAX_ALT_LOSS_M:
            return _fail(
                f"altitude loss {dev['max_alt_loss']:.2f} m > {MEDIUM_MAX_ALT_LOSS_M} m", **m
            )
        if not dev["max_horiz"] <= MEDIUM_MAX_HORIZ_M:
            return _fail(
                f"horizontal deviation {dev['max_horiz']:.2f} m > {MEDIUM_MAX_HORIZ_M} m", **m
            )
        return _ok(
            f"landing {'started and ' if nav18 else 'not needed, '}back in offboard, mission "
            "completed, "
            f"alt loss {dev['max_alt_loss']:.2f} m, horizontal {dev['max_horiz']:.2f} m",
            **m,
        )
    # long
    return _score_host_abort(run, t0, f"long outage {d:.1f} s", extra={"outage_s": _r(d)})


def _score_host_abort(run: RunData, t_fault: float, what: str, extra: dict) -> Verdict:
    """F1 long and F3: lands and disarms within 25 s of the fault, never re-armed."""
    land = landing_summary(run, t_fault)
    m = dict(extra)
    m["t_to_disarm_s"] = _r(land["t_disarm"] - t_fault) if land["t_disarm"] else None
    m["rearmed"] = land["rearmed"]
    m["final_height_m"] = _r(land["final_height_m"])
    crash = crashed(run)
    if crash:
        return _fail(f"crash: {crash}", **m)
    if run.case == "F1":
        aborted = any(e.get("event") == "phase" and e.get("phase") == "aborted" for e in run.flight)
        m["host_aborted"] = aborted
        if not aborted:
            return _fail("host did not abort the mission (gap > 5 s)", **m)
        arm_after = [
            e
            for e in run.flight
            if e.get("event") == "command" and e.get("command") == "arm" and e["t"] > t_fault
        ]
        if arm_after:
            return _fail("host sent ARM after the outage started", **m)
    if land["t_disarm"] is None:
        return _fail(f"{what}: never disarmed", **m)
    if land["t_disarm"] - t_fault > HOST_ABORT_LAND_S:
        return _fail(
            f"{what}: disarmed {land['t_disarm'] - t_fault:.1f} s after the fault > "
            f"{HOST_ABORT_LAND_S:.0f} s",
            **m,
        )
    if land["rearmed"]:
        return _fail(f"{what}: vehicle was re-armed after the failsafe landing", **m)
    if not land["on_ground_at_end"]:
        return _fail(
            f"{what}: ground truth {land['final_height_m']:.2f} m above ground at end", **m
        )
    return _ok(
        f"{what}: landed and disarmed {land['t_disarm'] - t_fault:.1f} s after the fault, "
        "never re-armed",
        **m,
    )


# -- F2 --------------------------------------------------------------------------------------
STALE_SLACK_MS = 10.0  # monitor and relay read the clock at slightly different instants
COUNT_SLACK = 3  # window edges


def score_f2(run: RunData) -> Verdict:
    t0, t1 = run.fault["t_start"], run.fault["t_end"]
    br = window([b for b in run.bridge if b.get("k") == "bridge"], t0, t1 + 1.0)
    arrivals = window([b for b in run.bridge if b.get("k") == "arrive"], t0, t1 + 1.0)
    reached = [e for e in window(run.px4, t0, t1 + 1.0) if e.get("k") == "sp"]
    fresh = [a for a in arrivals if a["age_ms"] <= F2_MAX_AGE_MS + STALE_SLACK_MS]
    dev = deviation(run.gt, run.hover_ref, t0, t1 + 3)
    status = px4_events(run.px4)
    failsafes = [e for e in status if t0 <= e["t"] <= t1 + 3 and e["nav"] == NAV_AUTO_LAND]
    stale_drops = (br[-1]["dropped_stale"] - br[0]["dropped_stale"]) if len(br) >= 2 else None
    land = landing_summary(run, t0)
    m = {
        "duration_s": _r(t1 - t0),
        "max_dev_m": _r(dev["max_3d"]),
        "max_horiz_m": _r(dev["max_horiz"]),
        "alt_loss_m": _r(dev["max_alt_loss"]),
        "setpoints_arrived": len(arrivals),
        "arrived_older_than_200ms": len(arrivals) - len(fresh),
        "reached_px4": len(reached),
        "stale_dropped": stale_drops,
        "px4_failsafe_entries": len(failsafes),
        "flight_result": flight_result(run.flight),
    }
    crash = crashed(run)
    if crash:
        return _fail(f"crash: {crash}", **m)
    if not arrivals:
        return _fail("no setpoint arrived at the bridge during the fault", **m)
    if len(reached) > len(fresh) + COUNT_SLACK:
        return _fail(
            f"{len(reached)} setpoints reached PX4 but only {len(fresh)} arrived younger than "
            f"{F2_MAX_AGE_MS:.0f} ms: a stale setpoint got through",
            **m,
        )
    if not stale_drops:
        return _fail("no stale setpoint was dropped: the bad-link profile had no effect", **m)
    # "No flyaway" is scored on the horizontal distance from the hold point. Altitude loss is
    # reported, not scored: with the bad profile nearly every setpoint is stale, PX4 sees an
    # offboard loss and lands in place (see the F2 note in the S1 report).
    if not dev["max_horiz"] <= F2_MAX_DEV_M:
        return _fail(
            f"horizontal deviation {dev['max_horiz']:.2f} m from the hold point > {F2_MAX_DEV_M} m",
            **m,
        )
    if land["t_disarm"] is None or not land["on_ground_at_end"]:
        return _fail("did not end landed and disarmed", **m)
    return _ok(
        f"{len(arrivals) - len(fresh)} of {len(arrivals)} setpoints arrived stale, "
        f"{len(reached)} reached PX4 (<= {len(fresh)} fresh), {stale_drops} dropped, horizontal "
        f"deviation {dev['max_horiz']:.2f} m (alt loss {dev['max_alt_loss']:.2f} m), landed "
        f"({len(failsafes)} PX4 failsafe entries)",
        **m,
    )


# -- F3 / F4 ---------------------------------------------------------------------------------
def score_f3(run: RunData) -> Verdict:
    return _score_host_abort(run, run.fault["t_kill"], "host flight process killed", extra={})


def score_f4(run: RunData) -> Verdict:
    t_kill = run.fault["t_kill"]
    status = px4_events(run.px4)
    nav18 = first_event(status, t_kill, lambda e: e["nav"] == NAV_AUTO_LAND)
    limit = COM_OF_LOSS_T + F4_ENGAGE_MARGIN_S
    land = landing_summary(run, t_kill)
    m = {
        "t_to_failsafe_s": _r(nav18["t"] - t_kill) if nav18 else None,
        "t_failsafe_to_disarm_s": (
            _r(land["t_disarm"] - nav18["t"]) if nav18 and land["t_disarm"] else None
        ),
        "rearmed": land["rearmed"],
        "final_height_m": _r(land["final_height_m"]),
    }
    crash = crashed(run)
    if crash:
        return _fail(f"crash: {crash}", **m)
    if nav18 is None:
        return _fail("PX4 never entered Land (nav_state 18) after the relay was killed", **m)
    if nav18["t"] - t_kill > limit:
        return _fail(
            f"failsafe engaged {nav18['t'] - t_kill:.2f} s after the kill > {limit:.1f} s", **m
        )
    if land["t_disarm"] is None:
        return _fail("never disarmed", **m)
    if land["t_disarm"] - nav18["t"] > LAND_AFTER_FAILSAFE_S:
        return _fail(
            f"disarmed {land['t_disarm'] - nav18['t']:.1f} s after the failsafe > "
            f"{LAND_AFTER_FAILSAFE_S:.0f} s",
            **m,
        )
    if land["rearmed"]:
        return _fail("vehicle was re-armed after the failsafe landing", **m)
    if not land["on_ground_at_end"]:
        return _fail(f"ground truth {land['final_height_m']:.2f} m above ground at end", **m)
    return _ok(
        f"failsafe {nav18['t'] - t_kill:.2f} s after the kill (limit {limit:.1f} s), landed and "
        f"disarmed {land['t_disarm'] - nav18['t']:.1f} s later, never re-armed",
        **m,
    )


# -- F6 --------------------------------------------------------------------------------------
def score_f6(run: RunData) -> Verdict:
    goals = run.fault["goals"]  # [{"t": sent, "expect": reason, "goal": {...}}]
    t0 = goals[0]["t"] - 0.5
    t1 = goals[-1]["t"] + 3.0
    expected = [g["expect"] for g in goals]
    rejections = [e for e in run.flight if e.get("event") == "rejection" and e["t"] >= t0]
    got = [e["reason"] for e in rejections]
    sp_rows = [e for e in window(run.px4, t0, t1) if e.get("k") == "sp"]
    dev = deviation(run.gt, run.hover_ref, t0, t1)
    ref_sp = None
    pre = [e for e in window(run.px4, t0 - 1.5, t0) if e.get("k") == "sp"]
    if pre:
        ref_sp = pre[-1]["p"]
    max_sp_dev = max(math.dist(e["p"], ref_sp) for e in sp_rows) if ref_sp and sp_rows else math.nan
    held = all(
        e.get("setpoint") is not None
        and ref_sp is not None
        and math.dist(e["setpoint"], ref_sp) <= F6_SETPOINT_TOL_M
        for e in rejections
    )
    m = {
        "goals": len(goals),
        "rejected": len(rejections),
        "expected": expected,
        "got": got,
        "max_setpoint_dev_m": _r(max_sp_dev, 3),
        "max_dev_m": _r(dev["max_3d"]),
        "flight_result": flight_result(run.flight),
    }
    crash = crashed(run)
    if crash:
        return _fail(f"crash: {crash}", **m)
    if ref_sp is None or not sp_rows:
        return _fail("no setpoints seen at PX4 around the bad goals", **m)
    if got != expected:
        return _fail(f"rejections {got} != expected {expected}", **m)
    if not held:
        return _fail("a rejection event reports a setpoint different from the held one", **m)
    if not max_sp_dev <= F6_SETPOINT_TOL_M:
        return _fail(f"a setpoint {max_sp_dev:.3f} m from the hold setpoint reached PX4", **m)
    if not dev["max_3d"] <= F6_MAX_DEV_M:
        return _fail(f"deviation {dev['max_3d']:.2f} m > {F6_MAX_DEV_M} m", **m)
    if m["flight_result"] != "ok":
        return _fail(f"mission did not complete (host result {m['flight_result']})", **m)
    return _ok(
        f"all {len(goals)} bad goals rejected with the expected reasons, setpoint held "
        f"(max {max_sp_dev:.3f} m at PX4), deviation {dev['max_3d']:.2f} m",
        **m,
    )


SCORERS = {"F1": score_f1, "F2": score_f2, "F3": score_f3, "F4": score_f4, "F6": score_f6}

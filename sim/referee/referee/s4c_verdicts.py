"""S4c verdicts (pure Python, stdlib only): the `h2_text` scenario against the manifest.

The operator types "go check the kitchen for bananas, come back and tell us"; the drone's agent
answers by `say` text and (when it found them) sends a photo. Ground truth (the manifest, the
real banana, the person) enters here and only here (referee side). Checks, from
docs/simulation-plan.md §8 h2_bananas:

  answer    the spoken answer is yes when bananas are present, no when absent (rule based)
  photo     present: a photo was sent and it shows a banana (the object id it names is a banana
            in memory within 0.3 m of the real one); absent: no photo was sent
  distance  the drone ends within 1.5 m of the operator and never came closer than 1.0 m
  collisions  0 (ground-truth track against the apartment, computed in the tools container)
  time      the mission ended within 240 s of the command

Failures are attributed to a layer (agent decision, executive rejection, skill failure,
perception, flight/collision) for the S4c report.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field

ACK_TEXT = "On it."
TIME_LIMIT_S = 240.0
END_MAX_M = 1.5
MIN_M = 1.0
PHOTO_TOL_M = 0.3

_NEG = re.compile(
    r"\b(no|not|none|nothing|nope|never|without)\b|n't\b|\b(cannot|couldn|didn|isn|aren|wasn)\w*",
    re.IGNORECASE,
)
_POS = re.compile(r"\b(yes|yeah|yep|found|there (?:are|is)|i see|i saw|spotted|sure)\b", re.I)


def answer_polarity(text: str) -> str | None:
    """ "yes" / "no" / None (unclear) for a spoken answer to "are there bananas?"."""
    t = text.strip()
    if not t:
        return None
    neg, pos = bool(_NEG.search(t)), bool(_POS.search(t))
    # "Yes ..." / "No ..." at the start decides; otherwise a negation wins ("I did not find any")
    first = re.match(r"\W*(yes|yeah|yep|no|nope)\b", t, re.IGNORECASE)
    if first:
        return "yes" if first.group(1).lower() in ("yes", "yeah", "yep") else "no"
    if neg:
        return "no"
    return "yes" if pos else None


def spoken_answers(events: list[dict], t0: float) -> list[str]:
    """Maverick's spoken lines after the command, without the template acknowledgement and without
    photo captions."""
    out = []
    for e in events:
        d = e.get("data", {})
        if e.get("type") != "transcript" or e.get("t", 0) < t0 or d.get("role") != "maverick":
            continue
        if d.get("attachments") or d.get("text", "").strip() == ACK_TEXT:
            continue
        out.append(d.get("text", ""))
    return out


def photos_sent(events: list[dict], t0: float) -> list[dict]:
    return [
        a
        for e in events
        if e.get("type") == "transcript" and e.get("t", 0) >= t0
        for a in e.get("data", {}).get("attachments", [])
        if a.get("kind") == "photo"
    ]


@dataclass
class H2Verdict:
    passed: bool
    reason: str
    checks: dict[str, bool]
    layers: list[str] = field(default_factory=list)
    metrics: dict = field(default_factory=dict)

    def as_dict(self) -> dict:
        return {
            "verdict": "pass" if self.passed else "fail",
            "reason": self.reason,
            "checks": self.checks,
            "layers": self.layers,
            **self.metrics,
        }


def real_banana(manifest: dict) -> dict | None:
    for o in manifest.get("small_objects", []):
        if o["class"] == "banana":
            return o
    return None


def photo_shows_banana(photo: dict, objects: dict[str, dict], truth: dict | None, spawn_xy):
    """The photo names an object id; it must be a banana in memory near the real banana."""
    o = objects.get(str(photo.get("object_id")))
    if o is None or truth is None:
        return False, "photo names no known object"
    if o.get("label") != "banana":
        return False, f"photo shows a {o.get('label')}"
    px, py = o["position"][:2]
    err = math.hypot(
        px + spawn_xy[0] - truth["position"][0], py + spawn_xy[1] - truth["position"][1]
    )
    if err >= PHOTO_TOL_M:
        return False, f"photo object is {err:.2f} m from the real banana"
    return True, f"banana, {err:.2f} m off"


def last_plan_steps(events: list[dict]) -> list[dict]:
    steps: list[dict] = []
    for e in events:
        if e.get("type") == "agent_step":
            steps = e["data"].get("steps", [])
    return steps


def attribute_layers(
    checks: dict[str, bool], steps: list[dict], mission: dict | None, present: bool
) -> list[str]:
    """Which layer(s) the failed checks belong to (a heuristic over the agent's own plan)."""
    layers: list[str] = []

    def add(x: str) -> None:
        if x not in layers:
            layers.append(x)

    details = [(s.get("label", ""), s.get("status"), s.get("detail", "")) for s in steps]
    for label, status, detail in details:
        if status == "failed":
            if detail.startswith("invalid call"):
                add("agent decision")
            elif "rejected" in detail or "refused" in detail:
                add("executive rejection")
            elif label.startswith("Send photo"):
                add("agent decision")
            else:
                add("skill failure")
    if mission is None or mission.get("ended") != "end_turn":
        add("agent decision")
    if not checks.get("collisions", True) or not checks.get("distance", True):
        if "skill failure" not in layers and "executive rejection" not in layers:
            add("flight/collision")
    searched = next((d for lbl, st, d in details if lbl.startswith("Search")), None)
    search_found = searched is not None and searched.lower().startswith("found")
    search_miss = searched is not None and searched.lower().startswith("not found")
    if not checks.get("answer", True) or not checks.get("photo", True):
        if present and search_miss:
            add("perception")
        elif not present and search_found:
            add("perception")
        elif searched is None and not layers:
            add("agent decision")  # never searched
        elif not layers:
            add("agent decision")  # skills agreed with the truth but the agent said otherwise
    if not checks.get("time", True) and not layers:
        add("skill failure")
    return layers


def verdict_h2(
    manifest: dict,
    events: list[dict],
    t0: float,
    objects: list[dict],
    spawn_xy,
    end_dist: float | None,
    min_dist: float | None,
    n_collisions: int | None,
    mission_end: dict | None,
    mission_t: float | None,
) -> H2Verdict:
    truth = real_banana(manifest)
    present = truth is not None
    answers = spoken_answers(events, t0)
    pol = answer_polarity(answers[-1]) if answers else None
    photos = photos_sent(events, t0)
    by_id = {str(o.get("id")): o for o in objects}
    m: dict = {
        "present": present,
        "answers": answers,
        "polarity": pol,
        "n_photos": len(photos),
        "end_distance_m": None if end_dist is None else round(end_dist, 3),
        "min_distance_m": None if min_dist is None else round(min_dist, 3),
        "collisions": n_collisions,
        "mission": mission_end,
        "mission_s": None if mission_t is None else round(mission_t - t0, 1),
    }
    checks: dict[str, bool] = {}
    notes: list[str] = []
    checks["answer"] = pol == ("yes" if present else "no")
    if not checks["answer"]:
        notes.append(f"answer {pol!r} but bananas {'present' if present else 'absent'}")
    if present:
        res = [photo_shows_banana(p, by_id, truth, spawn_xy) for p in photos]
        checks["photo"] = any(ok for ok, _ in res)
        if not checks["photo"]:
            notes.append("no photo showing the banana" if not res else f"photo: {res[0][1]}")
    else:
        checks["photo"] = not photos
        if photos:
            notes.append("sent a photo although bananas are absent")
    checks["distance"] = (
        end_dist is not None and end_dist <= END_MAX_M and (min_dist is None or min_dist >= MIN_M)
    )
    if not checks["distance"]:
        notes.append(f"end {m['end_distance_m']} m, min {m['min_distance_m']} m from the operator")
    checks["collisions"] = n_collisions == 0
    if not checks["collisions"]:
        notes.append(f"{n_collisions} collision(s)")
    checks["time"] = mission_t is not None and mission_t - t0 <= TIME_LIMIT_S
    if not checks["time"]:
        notes.append(f"mission took {m['mission_s']} s" if mission_t else "mission never ended")
    ok = all(checks.values())
    layers = [] if ok else attribute_layers(checks, last_plan_steps(events), mission_end, present)
    m["steps"] = last_plan_steps(events)
    return H2Verdict(ok, "all checks pass" if ok else "; ".join(notes), checks, layers, m)


def summarise_h2(rows: list[dict]) -> dict:
    scored = [r for r in rows if r.get("verdict") in ("pass", "fail")]
    layer_counts: dict[str, int] = {}
    for r in scored:
        for lay in r.get("layers", []):
            layer_counts[lay] = layer_counts.get(lay, 0) + 1
    lat = [x for r in scored for x in r.get("model_latencies_s", [])]
    lat.sort()
    return {
        "runs": len(rows),
        "scored": len(scored),
        "passed": sum(r["verdict"] == "pass" for r in scored),
        "skipped": [r["seed"] for r in rows if r.get("verdict") == "skipped"],
        "errors": [r["seed"] for r in rows if r.get("verdict") == "error"],
        "failures_by_layer": layer_counts,
        "cost_eur": round(sum(r.get("cost_eur", 0.0) for r in rows), 4),
        "model_calls": sum(r.get("model_calls", 0) for r in rows),
        "model_latency_p50_s": lat[len(lat) // 2] if lat else None,
        "model_latency_p95_s": lat[min(len(lat) - 1, int(0.95 * len(lat)))] if lat else None,
        "mission_s_mean": round(
            sum(r.get("mission_s") or 0 for r in scored) / max(1, len(scored)), 1
        ),
    }

"""Run features that can be computed identically from a Gazebo run and from a kinsim run.

Input is the ground-truth track in the map frame (rows t, x, y, z; any sampling), the skill's
decision log (`skill_events.jsonl` rows: t relative to the skill start, `ev`, `xy` = the drone's
estimate, `wps` for plans) and the time of the skill start on the track's clock. Used for the
calibration (docs/kinsim.md): speed profile, tracking error along planned paths, stops.
"""

from __future__ import annotations

import math

import numpy as np


def align_offset(
    track: np.ndarray, events: list[dict], lo: float = 0.0, hi: float = 10.0, step: float = 0.1
) -> float:
    """Time of the skill start on the track clock, from where the events say the drone was.

    Every event carries the drone's estimated xy; the estimate follows the truth within a few cm,
    so the offset that minimises the distance between the two is the skill start (the track's
    clock is wall time, the events' clock is the skill's). `lo` / `hi` are relative to track[0, 0].
    """
    ev = [e for e in events if e.get("xy") is not None]
    if not ev or len(track) < 2:
        return float(track[0, 0]) if len(track) else 0.0
    t_e = np.array([e["t"] for e in ev])
    xy_e = np.array([e["xy"] for e in ev], float)
    best, best_off = math.inf, lo
    for off in np.arange(lo, hi + 1e-9, step):
        tt = track[0, 0] + off + t_e
        ok = (tt >= track[0, 0]) & (tt <= track[-1, 0])
        if ok.sum() < 3:
            continue
        x = np.interp(tt[ok], track[:, 0], track[:, 1])
        y = np.interp(tt[ok], track[:, 0], track[:, 2])
        cost = float(np.mean(np.hypot(x - xy_e[ok, 0], y - xy_e[ok, 1])))
        if cost < best:
            best, best_off = cost, off
    return float(track[0, 0] + best_off)


def _poly_dist(p: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Distance of points p (n, 2) to a polyline (m, 2)."""
    best = np.full(len(p), np.inf)
    for a, b in zip(poly[:-1], poly[1:], strict=True):
        d = b - a
        L2 = float(d @ d)
        t = np.zeros(len(p)) if L2 < 1e-12 else np.clip(((p - a) @ d) / L2, 0.0, 1.0)
        q = a + t[:, None] * d
        best = np.minimum(best, np.hypot(*(p - q).T))
    return best


def track_features(track: np.ndarray, events: list[dict], t_start: float, t_end: float) -> dict:
    """Speed profile and cross-track error of the drone over [t_start, t_end] (track clock)."""
    sel = track[(track[:, 0] >= t_start) & (track[:, 0] <= t_end)]
    out: dict = {}
    if len(sel) < 10:
        return out
    t, xy = sel[:, 0], sel[:, 1:3]
    # velocity over ~0.3 s windows
    w = max(1, int(round(0.3 / max(float(np.median(np.diff(t))), 1e-3))))
    v = np.hypot(*((xy[w:] - xy[:-w]).T)) / (t[w:] - t[:-w])
    mv = v[v > 0.05]
    out["path_len_m"] = round(float(np.sum(np.hypot(*np.diff(xy, axis=0).T))), 2)
    out["frac_moving"] = round(float(np.mean(v > 0.05)), 3)
    if len(mv):
        out["v_p50"] = round(float(np.percentile(mv, 50)), 3)
        out["v_p90"] = round(float(np.percentile(mv, 90)), 3)
        out["v_max"] = round(float(mv.max()), 3)
    a = np.diff(v) / (t[w + 1 :] - t[w:-1]) if len(v) > 2 else np.zeros(0)
    if len(a):
        out["acc_p95"] = round(float(np.percentile(np.abs(a), 95)), 3)
    # cross-track error along the planned polylines (explore plans carry `wps`)
    plans = [e for e in events if e.get("ev") == "plan" and e.get("wps")]
    errs: list[np.ndarray] = []
    stops = [e["t"] for e in events]
    for e in plans:
        t0 = t_start + e["t"]
        nxt = [s for s in stops if s > e["t"] + 0.05]
        t1 = t_start + (min(nxt) if nxt else t_end - t_start)
        m = (t >= t0 + 0.5) & (t <= t1)
        if m.sum() < 3:
            continue
        speed = np.concatenate([[0.0], np.hypot(*np.diff(xy, axis=0).T) / np.diff(t)])
        m &= speed > 0.2
        if m.sum() < 3:
            continue
        poly = np.array([e["xy"], *e["wps"]], float)
        errs.append(_poly_dist(xy[m], poly))
    if errs:
        c = np.concatenate(errs)
        out["xtrack_p50"] = round(float(np.percentile(c, 50)), 3)
        out["xtrack_p90"] = round(float(np.percentile(c, 90)), 3)
        out["xtrack_max"] = round(float(c.max()), 3)
    fp = next((e["t"] for e in events if e.get("ev") == "plan"), None)
    if fp is not None:
        out["first_plan_t"] = fp
    return out

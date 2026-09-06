from referee.scoring import (
    RunData,
    hover_reference,
    score_f1,
    score_f2,
    score_f3,
    score_f4,
    score_f6,
)

GROUND = -0.01
REF = (0.0, 0.0, 1.9)


def gt_hover(t0, t1, z=1.9, dz=None):
    rows, t = [], t0
    while t <= t1:
        rows.append({"t": t, "x": 0.0, "y": 0.0, "z": dz(t) if dz else z})
        t += 0.05
    return rows


def status(t, nav, arm):
    return {"t": t, "k": "status", "nav": nav, "arm": arm, "failsafe": False}


def run(case, params, fault, gt, px4, flight=(), bridge=()):
    return RunData(case, params, gt, list(px4), list(flight), list(bridge), fault, REF, GROUND)


def landing_gt(t_disarm):
    return gt_hover(0, t_disarm, dz=lambda t: max(GROUND, 1.9 - 0.15 * max(0, t - 100))) + gt_hover(
        t_disarm, t_disarm + 6, z=GROUND
    )


def test_hover_reference_is_median():
    gt = gt_hover(0, 3)
    assert hover_reference(gt, 1, 3) == (0.0, 0.0, 1.9)
    assert hover_reference(gt, 10, 11) is None


def test_f3_pass_and_slow_landing_and_rearm():
    px4 = [status(100, 14, 2), status(101, 18, 2), status(115, 18, 1)]
    r = run("F3", {}, {"t_kill": 100.0}, landing_gt(115), px4)
    assert score_f3(r).passed
    slow = [status(100, 14, 2), status(101, 18, 2), status(130, 18, 1)]
    v = score_f3(run("F3", {}, {"t_kill": 100.0}, landing_gt(130), slow))
    assert not v.passed and "25" in v.reason
    rearm = px4 + [status(120, 14, 2)]
    v = score_f3(run("F3", {}, {"t_kill": 100.0}, landing_gt(115), rearm))
    assert not v.passed and "re-armed" in v.reason


def test_f4_engagement_window():
    ok = [status(101.2, 18, 2), status(116, 18, 1)]
    assert score_f4(run("F4", {}, {"t_kill": 100.0}, landing_gt(116), ok)).passed
    late = [status(101.7, 18, 2), status(116, 18, 1)]
    v = score_f4(run("F4", {}, {"t_kill": 100.0}, landing_gt(116), late))
    assert not v.passed and "1.5" in v.reason
    none = [status(116, 1, 1)]
    assert not score_f4(run("F4", {}, {"t_kill": 100.0}, landing_gt(116), none)).passed


def test_f1_short_and_medium():
    gt = gt_hover(90, 125)
    flight = [{"t": 120, "event": "exit", "result": "ok"}]
    short = run(
        "F1", {"kind": "short"}, {"t_start": 100.0, "t_end": 100.3}, gt, [status(95, 14, 2)], flight
    )
    assert score_f1(short).passed
    trig = run(
        "F1",
        {"kind": "short"},
        {"t_start": 100.0, "t_end": 100.3},
        gt,
        [status(101, 18, 2)],
        flight,
    )
    assert not score_f1(trig).passed
    px4 = [status(95, 14, 2), status(101.2, 18, 2), status(103.3, 14, 2)]
    dip = gt_hover(90, 125, dz=lambda t: 1.9 - (0.4 if 101 < t < 104 else 0))
    med = run("F1", {"kind": "medium"}, {"t_start": 100.0, "t_end": 103.0}, dip, px4, flight)
    assert score_f1(med).passed
    deep = gt_hover(90, 125, dz=lambda t: 1.9 - (1.5 if 101 < t < 104 else 0))
    v = score_f1(
        run("F1", {"kind": "medium"}, {"t_start": 100.0, "t_end": 103.0}, deep, px4, flight)
    )
    assert not v.passed and "altitude loss" in v.reason
    noreturn = run(
        "F1", {"kind": "medium"}, {"t_start": 100.0, "t_end": 103.0}, gt, px4[:2], flight
    )
    assert not score_f1(noreturn).passed


def test_f1_long_requires_host_abort():
    px4 = [status(101, 18, 2), status(115, 18, 1)]
    flight = [{"t": 105, "event": "phase", "phase": "aborted"}]
    f = {"t_start": 100.0, "t_end": 114.0}
    r = run("F1", {"kind": "long"}, f, landing_gt(115), px4, flight)
    assert score_f1(r).passed
    assert not score_f1(run("F1", {"kind": "long"}, f, landing_gt(115), px4, [])).passed


def test_f2_stale_counts_and_deviation():
    br = [
        {"t": 100 + i, "k": "bridge", "forwarded": i * 4, "dropped_stale": i * 10, "age_ms": 400.0}
        for i in range(12)
    ]
    arrive = [
        {"t": 100 + i * 0.05, "k": "arrive", "age_ms": 100.0 if i % 4 == 0 else 350.0}
        for i in range(220)
    ]  # 55 fresh
    sp = [{"t": 100 + i * 0.2, "k": "sp", "p": [0, 0, -2]} for i in range(55)]
    px4 = [status(95, 14, 2), status(125, 18, 1)] + sp
    gt = gt_hover(90, 115) + gt_hover(115, 130, z=GROUND)
    f = {"t_start": 100.0, "t_end": 110.0}
    assert score_f2(run("F2", {}, f, gt, px4, bridge=br + arrive)).passed
    leaky = px4 + [{"t": 101 + i * 0.01, "k": "sp", "p": [0, 0, -2]} for i in range(30)]
    v = score_f2(run("F2", {}, f, gt, leaky, bridge=br + arrive))
    assert not v.passed and "stale setpoint got through" in v.reason


def test_f6_matches_reasons_and_setpoint():
    goals = [{"t": 100.0, "expect": "geofence"}, {"t": 101.0, "expect": "non_finite"}]
    fault = {"goals": goals}
    sp = [{"t": 99.5 + 0.05 * i, "k": "sp", "p": [0.0, 0.0, -2.0]} for i in range(100)]
    flight = [
        {"t": 100.01, "event": "rejection", "reason": "geofence", "setpoint": [0, 0, -2]},
        {"t": 101.01, "event": "rejection", "reason": "non_finite", "setpoint": [0, 0, -2]},
        {"t": 120, "event": "exit", "result": "ok"},
    ]
    gt = gt_hover(90, 110) + gt_hover(110, 125, z=GROUND)
    r = run("F6", {}, fault, gt, sp + [status(95, 14, 2)], flight)
    assert score_f6(r).passed
    wrong = [dict(flight[0], reason="jump")] + flight[1:]
    assert not score_f6(run("F6", {}, fault, gt, sp, wrong)).passed
    leaked = sp + [{"t": 102.0, "k": "sp", "p": [1.0, 0.0, -2.0]}]
    assert not score_f6(run("F6", {}, fault, gt, leaked, flight)).passed

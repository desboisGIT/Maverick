import math

from maverick_flight.safety_gate import (
    REASON_GEOFENCE,
    REASON_JUMP,
    REASON_NON_FINITE,
    REASON_SPEED,
    GateConfig,
    SafetyGate,
)


def test_accepts_goal_inside_box():
    assert SafetyGate().check_goal((1.0, -2.0, -2.0)) is None


def test_rejects_non_finite():
    gate = SafetyGate()
    for bad in ((math.nan, 0, -2), (0, math.inf, -2), (0, 0, -math.inf)):
        rej = gate.check_goal(bad)
        assert rej is not None and rej.reason == REASON_NON_FINITE


def test_rejects_outside_geofence_each_axis():
    gate = SafetyGate(GateConfig(geofence_min=(-1, -1, -3), geofence_max=(1, 1, 0)))
    for bad in ((2, 0, -1), (0, -2, -1), (0, 0, -3.5), (0, 0, 0.5)):
        rej = gate.check_goal(bad)
        assert rej is not None and rej.reason == REASON_GEOFENCE


def test_rejects_speed_over_limit_and_nonsense():
    gate = SafetyGate()
    assert gate.check_goal((0, 0, -2), 1.0) is None
    for speed in (1.5, 0.0, -1.0, math.nan):
        rej = gate.check_goal((0, 0, -2), speed)
        assert rej is not None and rej.reason == REASON_SPEED


def test_jump_check():
    gate = SafetyGate()
    assert gate.check_setpoint((0, 0, -2), (0.9, 0, -2)) is None
    rej = gate.check_setpoint((0, 0, -2), (1.5, 0, -2))
    assert rej is not None and rej.reason == REASON_JUMP


def test_step_towards_limits_speed_and_does_not_overshoot():
    gate = SafetyGate()
    p = (0.0, 0.0, 0.0)
    for _ in range(20):
        nxt = gate.step_towards(p, (3.0, 0.0, 0.0), 0.05)
        assert math.dist(p, nxt) <= 0.05 + 1e-9
        p = nxt
    assert math.isclose(p[0], 1.0, abs_tol=1e-9)
    assert gate.step_towards((2.9, 0, 0), (3.0, 0, 0), 0.5) == (3.0, 0, 0)
    assert gate.step_towards((0, 0, 0), (0, 0, 0), 0.05) == (0, 0, 0)


def test_step_never_exceeds_gate_speed_even_if_faster_requested():
    gate = SafetyGate()
    nxt = gate.step_towards((0, 0, 0), (10, 0, 0), 0.05, speed=5.0)
    assert math.dist((0, 0, 0), nxt) <= 0.05 + 1e-9


def test_reject_counts_and_remembers():
    gate = SafetyGate()
    gate.reject(gate.check_goal((9, 0, -2)))
    gate.reject(gate.check_goal((math.nan, 0, -2)))
    gate.reject(gate.check_goal((9, 0, -2)))
    assert gate.rejections == {REASON_GEOFENCE: 2, REASON_NON_FINITE: 1}
    assert gate.rejected_total == 3
    assert (
        gate.last_rejection.reason == REASON_NON_FINITE
        or gate.last_rejection.reason == REASON_GEOFENCE
    )


def test_geofence_is_relative_to_the_origin():
    from maverick_flight.safety_gate import SafetyGate

    g = SafetyGate()
    assert g.check_position((0.0, 0.0, -4.27)) is not None  # 4.27 m up: outside [-4, 0.3]
    g.origin = (2.1, -0.27, -2.27)  # estimator origin drifted: ground is at z = -2.27
    assert g.check_position((2.1, -0.27, -4.27)) is None  # 2 m above the take-off point
    assert g.check_position((2.1 + 5.5, -0.27, -4.27)) is not None  # 5.5 m from take-off

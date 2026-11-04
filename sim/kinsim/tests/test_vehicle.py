"""Vehicle model: tracking, limits, contact, estimator, determinism."""

import math

import kinsim  # noqa: F401
import numpy as np
import pytest
from kinsim.scene import KScene
from kinsim.vehicle import Vehicle, VehicleParams

DT = 0.025


@pytest.fixture(scope="module")
def scene():
    return KScene.from_seed(30)


class FreeScene:
    """Open space: dynamics without contact."""

    def contact(self, p, half_height=0.12):
        return math.inf, np.array([1.0, 0.0]), -1

    def floor_height(self, xy):
        return 0.0


def make(scene, p=None, pos=(0.0, 0.0, 1.2), yaw=0.0, seed=0):
    return Vehicle(scene, p or VehicleParams(), pos, yaw, np.random.default_rng(seed))


def free(p=None, seed=0):
    return make(FreeScene(), p, seed=seed)


def run(v, sp, secs, yaw_sp=None):
    out = []
    for _ in range(int(secs / DT)):
        v.step(DT, np.asarray(sp, float), yaw_sp)
        out.append(v.pos.copy())
    return np.array(out)


def test_converges_to_a_position_setpoint():
    v = free()
    run(v, (1.0, 0.0, 1.2), 12.0)
    assert np.hypot(*(v.pos[:2] - [1.0, 0.0])) < 0.03


def test_speed_and_acceleration_limits():
    p = VehicleParams()
    v = free(p)
    tr = run(v, (3.0, 0.0, 1.2), 10.0)
    speed = np.hypot(*(np.diff(tr[:, :2], axis=0) / DT).T)
    assert speed.max() <= p.v_max * 1.15
    acc = np.abs(np.diff(speed)) / DT
    assert acc.max() < p.a_max * 1.3


def test_tracking_lag_grows_with_speed():
    """A setpoint moving at 0.8 m/s is followed with a lag, not on top of it."""
    v = free()
    sp = np.array([0.0, 0.0, 1.2])
    lag = []
    for _ in range(int(5.0 / DT)):
        sp[0] += 0.8 * DT
        v.step(DT, sp, None)
        lag.append(sp[0] - v.pos[0])
    assert 0.05 < np.mean(lag[-40:]) < 1.0


def test_yaw_follows_setpoint_with_rate_limit():
    p = VehicleParams()
    v = free(p)
    ys = []
    for _ in range(int(4.0 / DT)):
        v.step(DT, np.array([0.0, 0.0, 1.2]), math.radians(170))
        ys.append(v.yaw)
    assert v.yaw == pytest.approx(math.radians(170), abs=0.05)
    rate = np.abs(np.diff(ys)) / DT
    assert rate.max() <= p.yaw_rate_max * 1.1


def test_contact_stops_the_vehicle_at_the_wall(scene):
    # fly towards the nearest wall from the spawn point; the body must stop at its radius
    c, n, _ = scene.contact(np.array([0.0, 0.0, 1.2]), 0.12)
    target = np.array([0.0, 0.0, 1.2])
    target[:2] -= n * (c + 1.0)  # a setpoint 1 m inside the wall
    v = make(scene, VehicleParams(contact_radius=0.1496, contact_jitter=0.0))
    run(v, target, 10.0)
    cl, _, _ = scene.contact(v.pos, 0.12)
    assert cl == pytest.approx(v.p.contact_radius, abs=2e-3)
    assert v.contacts >= 1
    # the referee's clearance of this position is below its 0.15 m threshold: it counts as a hit
    assert scene.ref_scene.horizontal_clearance(v.pos, 0.12) < 0.15


def test_estimate_error_is_small_and_deterministic():
    a, b = free(seed=5), free(seed=5)
    run(a, (0.5, 0.5, 1.2), 20.0)
    run(b, (0.5, 0.5, 1.2), 20.0)
    assert np.array_equal(a.est_pos, b.est_pos) and np.array_equal(a.pos, b.pos)
    assert np.hypot(*(a.est_pos[:2] - a.pos[:2])) < 0.1


def test_estimate_lags_truth_by_the_delay():
    p = VehicleParams(est_delay_s=0.2, est_sigma_m=0.0)
    v = free(p)
    run(v, (3.0, 0.0, 1.2), 3.0)
    # moving: the estimate is behind the truth by about speed x delay
    sp = float(np.hypot(*v.vel[:2]))
    assert 0.5 * sp * 0.2 < (v.pos[0] - v.est_pos[0]) < 1.5 * sp * 0.2 + 0.01

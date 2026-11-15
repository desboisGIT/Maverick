"""Vehicle model: PX4 offboard position control around a small quad, plus the position estimate.

Not a rigid-body simulation. The cascade PX4 runs for position-only offboard setpoints is
reproduced at the level that matters for navigation: tracking lag, overshoot, speed and
acceleration limits, a yaw loop, contact with walls and furniture, and the estimator's error.

    v_sp  = clip(kp_xy * (p_sp - p_est), v_max)                  position P (MPC_XY_P)
    a_cmd = clip(kv_xy * (v_sp - v_est), a_max)                  velocity loop, accel limit
    a'    = (a_cmd - a) / tau_a                                  attitude / thrust lag
    v' = a ;  p' = v

Vertical and yaw are first-order variants (limits MPC_Z_VEL_MAX_UP / DN, yaw rate). All numbers
live in `VehicleParams`; defaults are the calibrated ones (docs/kinsim.md).
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import asdict, dataclass, field, fields

import numpy as np

from .scene import KScene


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class VehicleParams:
    # horizontal cascade (PX4 defaults: MPC_XY_P 0.95, MPC_XY_VEL_P_ACC 1.8, MPC_XY_VEL_MAX 1.0)
    kp_xy: float = 0.95
    kv_xy: float = 1.8
    v_max: float = 1.0
    a_max: float = 3.0
    tau_a: float = 0.12
    # vertical
    kp_z: float = 1.0
    v_up: float = 1.0
    v_dn: float = 0.7
    tau_z: float = 0.15
    # yaw
    kp_yaw: float = 2.8
    yaw_rate_max: float = math.radians(120.0)
    tau_w: float = 0.10
    # contact: the body is a cylinder of this radius; the referee counts clearance < 0.15
    contact_radius: float = 0.1502
    contact_jitter: float = 0.0012  # per contact: the real contact depth varies by about a mm
    restitution: float = 0.2
    contact_friction: float = 0.5
    # a contact sometimes crashes the vehicle: the estimate jumps by a persistent bias (Gazebo: 4 of
    # 19 collisions, estimate meaningless afterwards, coverage 0.67-0.89)
    crash_prob: float = 0.0
    crash_bias_m: tuple[float, float] = (0.3, 0.9)
    crash_decay_s: float = 60.0
    # estimator (mocap through PX4's EKF): delayed truth plus slowly varying noise
    est_delay_s: float = 0.0
    est_sigma_m: float = 0.026
    est_tau_s: float = 1.0
    est_yaw_sigma: float = 0.003
    # disturbance on the true horizontal acceleration (attitude noise, ground effect, ...)
    dist_sigma_acc: float = 0.0
    dist_tau_s: float = 0.5
    # the pose paired with a depth frame is this much newer than the image (render -> bridge
    # receipt latency; the bridge stamps frames at receipt): moving fast smears the map
    pose_lead_s: float = 0.0
    map_latency_s: float = 0.10  # frame rendered -> integrated in the map (transport + node)
    frame_jitter_s: float = 0.02  # arrival jitter of the 10 Hz frames at the 5 Hz rate limiters
    start_z: float = 2.0  # the flight client hovers at 2.0 m, skills fly at 1.2 m


@dataclass
class KinParams:
    """Everything calibratable: vehicle, estimator, contact and sensor-path parameters."""

    vehicle: VehicleParams = field(default_factory=VehicleParams)
    map_stride: int = 2  # depth sampling of the mapping node (2 = the real launch file)
    snapshot_period_s: float = 34.0  # referee map dumps (30 s poll + dump time)
    start_delay_s: float = 0.0

    def to_dict(self) -> dict:
        return {
            "vehicle": asdict(self.vehicle),
            "map_stride": self.map_stride,
            "snapshot_period_s": self.snapshot_period_s,
            "start_delay_s": self.start_delay_s,
        }

    @staticmethod
    def from_dict(d: dict) -> KinParams:
        names = {f.name for f in fields(VehicleParams)}
        v = VehicleParams(**{k: val for k, val in d.get("vehicle", {}).items() if k in names})
        return KinParams(
            v,
            int(d.get("map_stride", 2)),
            float(d.get("snapshot_period_s", 34.0)),
            float(d.get("start_delay_s", 0.0)),
        )


class Vehicle:
    """Truth state (map frame, ENU) and the controller that follows a position setpoint."""

    def __init__(
        self,
        scene: KScene,
        p: VehicleParams,
        pos: tuple[float, float, float],
        yaw: float,
        rng: np.random.Generator,
    ) -> None:
        self.scene, self.p, self.rng = scene, p, rng
        self.pos = np.array(pos, float)
        self.vel = np.zeros(3)
        self.acc = np.zeros(2)  # actual horizontal acceleration (after the attitude lag)
        self._dist = np.zeros(2)
        self.vz_act = 0.0
        self.yaw = yaw
        self.omega = 0.0
        self.t = 0.0
        self.odometer = 0.0
        self.contacts = 0
        self.last_contact_t = -1e9
        self.in_contact = False
        self._r_contact = p.contact_radius
        # estimator state
        self._hist: deque[tuple[float, np.ndarray, np.ndarray, float]] = deque(maxlen=4000)
        self._noise = np.zeros(3)
        self._bias = np.zeros(2)
        self.crashes = 0
        self.est_pos = self.pos.copy()
        self.est_vel = np.zeros(3)
        self.est_yaw = yaw
        self._push_hist()

    # -- estimator ---------------------------------------------------------------------------
    def _push_hist(self) -> None:
        self._hist.append((self.t, self.pos.copy(), self.vel.copy(), self.yaw))

    def _at(self, t: float) -> tuple[np.ndarray, np.ndarray, float]:
        """Truth (pos, vel, yaw) at time t from the history (linear interpolation)."""
        h = self._hist
        if t <= h[0][0]:
            return h[0][1], h[0][2], h[0][3]
        if t >= h[-1][0]:
            return h[-1][1], h[-1][2], h[-1][3]
        lo, hi = 0, len(h) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if h[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        a, b = h[lo], h[hi]
        w = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
        return (
            a[1] + w * (b[1] - a[1]),
            a[2] + w * (b[2] - a[2]),
            a[3] + w * wrap_pi(b[3] - a[3]),
        )

    def estimate_at(self, t: float) -> tuple[np.ndarray, float]:
        """The estimate the stack would have at time t: delayed truth plus the noise state."""
        p, _, yaw = self._at(t - self.p.est_delay_s)
        return p + self._noise + np.array([*self._bias, 0.0]), yaw + self._yaw_noise

    _yaw_noise = 0.0

    def _update_estimate(self, dt: float) -> None:
        p = self.p
        # Ornstein-Uhlenbeck noise: stationary sigma est_sigma_m, correlation time est_tau_s
        k = math.exp(-dt / max(p.est_tau_s, 1e-3))
        s = p.est_sigma_m * math.sqrt(1.0 - k * k)
        self._noise[:2] = self._noise[:2] * k + self.rng.normal(size=2) * s
        self._noise[2] = self._noise[2] * k + self.rng.normal() * s * 0.5
        self._yaw_noise = self._yaw_noise * k + self.rng.normal() * p.est_yaw_sigma * math.sqrt(
            1.0 - k * k
        )
        pe, ve, ye = self._at(self.t - p.est_delay_s)
        self._bias *= math.exp(-dt / max(p.crash_decay_s, 1e-3))
        self.est_pos = pe + self._noise + np.array([*self._bias, 0.0])
        self.est_vel = ve.copy()
        self.est_yaw = ye + self._yaw_noise

    # -- dynamics ----------------------------------------------------------------------------
    def step(self, dt: float, sp_enu: np.ndarray, yaw_sp: float | None) -> None:
        """Advance dt seconds towards the setpoint (map frame) / yaw setpoint (ENU)."""
        p = self.p
        # horizontal cascade on the ESTIMATE
        err = sp_enu[:2] - self.est_pos[:2]
        v_sp = p.kp_xy * err
        n = float(np.hypot(*v_sp))
        if n > p.v_max:
            v_sp *= p.v_max / n
        a_cmd = p.kv_xy * (v_sp - self.est_vel[:2])
        n = float(np.hypot(*a_cmd))
        if n > p.a_max:
            a_cmd *= p.a_max / n
        self.acc += (a_cmd - self.acc) * min(dt / p.tau_a, 1.0)
        if p.dist_sigma_acc > 0.0:
            k = math.exp(-dt / max(p.dist_tau_s, 1e-3))
            self._dist = self._dist * k + self.rng.normal(size=2) * p.dist_sigma_acc * math.sqrt(
                1.0 - k * k
            )
        self.vel[:2] += (self.acc + self._dist) * dt
        # vertical: the rangefinder height follows low obstacles under the drone
        z_sp = sp_enu[2] + self.scene.floor_height(self.pos[:2])
        vz_cmd = max(-p.v_dn, min(p.v_up, p.kp_z * (z_sp - self.est_pos[2])))
        self.vz_act += (vz_cmd - self.vz_act) * min(dt / p.tau_z, 1.0)
        self.vel[2] = self.vz_act
        # yaw
        if yaw_sp is not None and math.isfinite(yaw_sp):
            w_cmd = max(-p.yaw_rate_max, min(p.yaw_rate_max, p.kp_yaw * wrap_pi(yaw_sp - self.yaw)))
        else:
            w_cmd = 0.0
        self.omega += (w_cmd - self.omega) * min(dt / p.tau_w, 1.0)
        self.yaw = wrap_pi(self.yaw + self.omega * dt)
        self.pos += self.vel * dt
        self.odometer += float(np.hypot(*self.vel[:2])) * dt
        self.t += dt
        self._contact()
        self._push_hist()
        self._update_estimate(dt)

    def _contact(self) -> None:
        p = self.p
        c, nrm, g = self.scene.contact(self.pos, 0.12)
        if g < 0 or c >= self._r_contact:
            if g < 0 or c > self._r_contact + 0.004:
                self.in_contact = False
                self._r_contact = p.contact_radius + float(self.rng.normal(0.0, p.contact_jitter))
            return
        pen = self._r_contact - c
        self.pos[:2] += nrm * pen
        vn = float(self.vel[:2] @ nrm)
        if vn < 0:  # moving into the surface: reflect with restitution, damp the tangent
            vt = self.vel[:2] - vn * nrm
            self.vel[:2] = (
                vt * (1.0 - p.contact_friction * min(1.0, -vn)) - p.restitution * vn * nrm
            )
            an = float(self.acc @ nrm)
            if an < 0:
                self.acc -= an * nrm
        if not self.in_contact:
            self.contacts += 1
            self.last_contact_t = self.t
            if p.crash_prob > 0 and self.rng.random() < p.crash_prob:
                ang = self.rng.uniform(0, 2 * math.pi)
                mag = self.rng.uniform(*p.crash_bias_m)
                self._bias += mag * np.array([math.cos(ang), math.sin(ang)])
                self.crashes += 1
        self.in_contact = True

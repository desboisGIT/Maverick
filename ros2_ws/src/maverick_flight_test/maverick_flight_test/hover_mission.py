"""Pure-Python take off / hover / land mission state machine (no ROS imports).

The ROS node feeds one Observation per tick (10 Hz) to HoverMission.step() and
executes the returned Action. Positions are NED metres, so altitude is -z.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

# Values from px4_msgs/msg/VehicleStatus (PX4 v1.17).
ARMING_STATE_DISARMED = 1
ARMING_STATE_ARMED = 2
NAV_STATE_OFFBOARD = 14

# Command names carried in Action.commands.
CMD_OFFBOARD = "offboard"
CMD_ARM = "arm"
CMD_LAND = "land"


@dataclass(frozen=True)
class Observation:
    t: float  # seconds, monotonic
    position: tuple[float, float, float] | None  # NED x, y, z; None if no valid estimate
    arming_state: int | None  # None until vehicle_status received
    nav_state: int | None
    landed: bool | None  # None until vehicle_land_detected received


@dataclass(frozen=True)
class Action:
    # Position setpoint (x, y, z) NED. None means: stop streaming offboard messages.
    setpoint: tuple[float, float, float] | None = None
    commands: tuple[str, ...] = ()


@dataclass(frozen=True)
class Config:
    altitude: float = 2.0
    hover_seconds: float = 10.0
    timeout: float = 90.0  # whole mission
    wait_data_timeout: float = 15.0
    arm_timeout: float = 10.0
    climb_timeout: float = 30.0
    land_timeout: float = 40.0
    reach_tol: float = 0.2
    settle_s: float = 2.0  # after reaching altitude, before hold quality is measured
    max_alt_error: float = 0.3
    max_xy_drift: float = 0.3
    min_setpoints: int = 10
    command_retry_s: float = 1.0


@dataclass(frozen=True)
class Verdict:
    passed: bool
    reason: str
    max_alt_error_m: float
    max_xy_drift_m: float
    duration_s: float


class Phase(Enum):
    WAIT_DATA = "wait_data"
    PREFLIGHT = "preflight"
    ARMING = "arming"
    CLIMB = "climb"
    SETTLE = "settle"
    HOVER = "hover"
    LAND = "land"
    DONE = "done"


class HoverMission:
    def __init__(self, config: Config | None = None) -> None:
        self.cfg = config or Config()
        self.phase = Phase.WAIT_DATA
        self.verdict: Verdict | None = None
        self._t0: float | None = None
        self._phase_t0 = 0.0
        self._home_xy = (0.0, 0.0)
        self._hover_xy = (0.0, 0.0)
        self._setpoints_sent = 0
        self._last_cmd_t = -math.inf
        self._fail: str | None = None  # first failure; mission still lands if airborne
        self._max_alt_err = 0.0
        self._max_drift = 0.0
        self._last_t = 0.0

    @property
    def done(self) -> bool:
        return self.phase is Phase.DONE

    def _enter(self, phase: Phase, t: float) -> None:
        self.phase = phase
        self._phase_t0 = t

    def _finish(self, t: float, reason: str) -> Action:
        self._enter(Phase.DONE, t)
        reason = self._fail or reason
        self.verdict = Verdict(
            passed=reason == "ok",
            reason=reason,
            max_alt_error_m=self._max_alt_err,
            max_xy_drift_m=self._max_drift,
            duration_s=t - (self._t0 if self._t0 is not None else t),
        )
        return Action()

    def _fail_airborne(self, t: float, reason: str) -> Action:
        """Record the first failure and land safely."""
        self._fail = self._fail or reason
        self._enter(Phase.LAND, t)
        self._last_cmd_t = t
        return Action(commands=(CMD_LAND,))

    def _expired(self, t: float, phase_timeout: float) -> bool:
        assert self._t0 is not None
        return t - self._phase_t0 > phase_timeout or t - self._t0 > self.cfg.timeout

    def step(self, obs: Observation) -> Action:
        t = obs.t
        self._last_t = t
        if self.phase is Phase.DONE:
            return Action()
        if self._t0 is None:
            self._t0 = t
            self._phase_t0 = t
        cfg = self.cfg
        pos = obs.position
        target_z = -cfg.altitude

        if self.phase is Phase.WAIT_DATA:
            if pos is not None and obs.arming_state is not None:
                self._home_xy = (pos[0], pos[1])
                self._enter(Phase.PREFLIGHT, t)
            elif self._expired(t, cfg.wait_data_timeout):
                return self._finish(t, "no_position")
            else:
                return Action()

        hold = (*self._home_xy, target_z)

        if self.phase is Phase.PREFLIGHT:
            self._setpoints_sent += 1
            if self._setpoints_sent >= cfg.min_setpoints:
                self._enter(Phase.ARMING, t)
                self._last_cmd_t = t
                return Action(hold, (CMD_OFFBOARD, CMD_ARM))
            return Action(hold)

        if self.phase is Phase.ARMING:
            if obs.arming_state == ARMING_STATE_ARMED and obs.nav_state == NAV_STATE_OFFBOARD:
                self._enter(Phase.CLIMB, t)
            elif self._expired(t, cfg.arm_timeout):
                return self._finish(t, "arm_rejected")
            elif t - self._last_cmd_t >= cfg.command_retry_s:
                self._last_cmd_t = t
                return Action(hold, (CMD_OFFBOARD, CMD_ARM))
            else:
                return Action(hold)

        airborne = self.phase in (Phase.CLIMB, Phase.SETTLE, Phase.HOVER)
        if airborne and obs.arming_state == ARMING_STATE_DISARMED:
            return self._finish(t, "unexpected_disarm")

        if self.phase is Phase.CLIMB:
            if pos is not None and abs(pos[2] - target_z) < cfg.reach_tol:
                self._enter(Phase.SETTLE, t)
            elif self._expired(t, cfg.climb_timeout):
                return self._fail_airborne(t, "climb_timeout")
            else:
                return Action(hold)

        if self.phase is Phase.SETTLE:
            # Let the climb overshoot die out so the hover window measures holding, not climbing.
            if t - self._phase_t0 < cfg.settle_s or pos is None:
                return Action(hold)
            self._hover_xy = (pos[0], pos[1])
            self._enter(Phase.HOVER, t)

        if self.phase is Phase.HOVER:
            if pos is not None:
                self._max_alt_err = max(self._max_alt_err, abs(pos[2] - target_z))
                drift = math.hypot(pos[0] - self._hover_xy[0], pos[1] - self._hover_xy[1])
                self._max_drift = max(self._max_drift, drift)
                if self._max_alt_err > cfg.max_alt_error:
                    return self._fail_airborne(t, "alt_error")
                if self._max_drift > cfg.max_xy_drift:
                    return self._fail_airborne(t, "drift")
            if t - self._phase_t0 >= cfg.hover_seconds:
                self._enter(Phase.LAND, t)
                self._last_cmd_t = t
                return Action(commands=(CMD_LAND,))
            return Action(hold)

        # Phase.LAND
        if obs.landed and obs.arming_state == ARMING_STATE_DISARMED:
            return self._finish(t, "ok")
        if self._expired(t, cfg.land_timeout):
            return self._finish(t, "land_timeout")
        if t - self._last_cmd_t >= cfg.command_retry_s * 5:
            self._last_cmd_t = t
            return Action(commands=(CMD_LAND,))
        return Action()

"""Pure-Python flight mission core (no ROS imports): take off, hover, land, link-gap policy.

The ROS node feeds one Observation per tick (20 Hz) to FlightMission.step() and executes the
returned Action. Every setpoint it returns went through the SafetyGate.

Link-gap policy (docs/s1-plan.md): if no PX4 telemetry arrived for more than gap_abort_s, the
mission is aborted: streaming stops, LAND is sent when the link returns, and the vehicle is
never armed again. Shorter gaps: keep streaming, so PX4 comes back to offboard by itself.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from maverick_flight.safety_gate import SafetyGate, Vec3

# px4_msgs/VehicleStatus values (PX4 v1.17).
ARMING_STATE_DISARMED = 1
ARMING_STATE_ARMED = 2
NAV_STATE_OFFBOARD = 14

CMD_OFFBOARD = "offboard"
CMD_ARM = "arm"
CMD_LAND = "land"


@dataclass(frozen=True)
class Observation:
    t: float  # seconds, monotonic
    position: Vec3 | None  # NED; None if no valid estimate
    arming_state: int | None
    nav_state: int | None
    landed: bool | None
    telemetry_age_s: float  # time since any PX4 telemetry arrived; inf before the first


@dataclass(frozen=True)
class Action:
    setpoint: Vec3 | None = None  # None: do not stream this tick
    commands: tuple[str, ...] = ()


@dataclass(frozen=True)
class MissionConfig:
    altitude: float = 2.0
    hover_seconds: float | None = 20.0  # None: hover until request_land()
    gap_abort_s: float = 5.0
    min_setpoints: int = 20  # streamed before OFFBOARD + ARM
    wait_data_timeout: float = 30.0
    arm_timeout: float = 10.0
    climb_timeout: float = 30.0
    land_timeout: float = 40.0
    abort_timeout: float = 300.0
    reach_tol: float = 0.2
    settle_s: float = 2.0
    command_retry_s: float = 1.0
    offboard_reengage_s: float = 1.0  # not in offboard this long with a live link: re-send OFFBOARD
    link_ok_age_s: float = 1.0


class Phase(Enum):
    WAIT_DATA = "wait_data"
    PREFLIGHT = "preflight"
    ARMING = "arming"
    TAKEOFF = "takeoff"
    SETTLE = "settle"
    HOVER = "hover"
    LAND = "land"
    ABORTED = "aborted"
    DONE = "done"


STREAMING = (Phase.PREFLIGHT, Phase.ARMING, Phase.TAKEOFF, Phase.SETTLE, Phase.HOVER)


class FlightMission:
    def __init__(self, config: MissionConfig | None = None, gate: SafetyGate | None = None) -> None:
        self.cfg = config or MissionConfig()
        self.gate = gate or SafetyGate()
        self.phase = Phase.WAIT_DATA
        self.result: str | None = None  # set when phase is DONE
        self.setpoint: Vec3 | None = None  # last setpoint streamed
        self.hold: Vec3 | None = None  # hover point
        self.goal: Vec3 | None = None
        self.goal_speed: float | None = None
        self.yaw: float = math.nan  # heading setpoint, PX4 NED radians; NaN = keep current
        self.events: list[dict] = []
        self._t0: float | None = None
        self._phase_t0 = 0.0
        self._last_t: float | None = None
        self._streamed = 0
        self._last_cmd_t = -math.inf
        self._not_offboard_since: float | None = None
        self._land_requested = False
        self._fail: str | None = None
        self.armed_ever = False

    # -- events ------------------------------------------------------------------------------
    def _emit(self, event: str, **fields) -> None:
        self.events.append({"event": event, "phase": self.phase.value, **fields})

    def drain_events(self) -> list[dict]:
        out, self.events = self.events, []
        return out

    def _enter(self, phase: Phase, t: float, **fields) -> None:
        self.phase = phase
        self._phase_t0 = t
        self._emit("phase", **fields)

    def _finish(self, t: float, reason: str) -> Action:
        self.result = self._fail or reason
        self._enter(Phase.DONE, t, result=self.result)
        return Action()

    @property
    def done(self) -> bool:
        return self.phase is Phase.DONE

    # -- inputs ------------------------------------------------------------------------------
    def on_goal(
        self, mode: str, position: Vec3, speed: float | None = None, yaw: float | None = None
    ) -> bool:
        """Offer a goal. Returns True if accepted. Goals only act while hovering.

        mode "goto": walk to `position` at `speed` (default: the gate's limit).
        mode "setpoint": use `position` as the next setpoint directly (rejected if it jumps).
        """
        rej = self.gate.check_goal(position, speed)
        if rej is None and mode == "setpoint" and self.setpoint is not None:
            rej = self.gate.check_setpoint(self.setpoint, position)
        if rej is None and self.phase not in (Phase.SETTLE, Phase.HOVER, Phase.TAKEOFF):
            return False
        if rej is not None:
            self.gate.reject(rej, f"goal {mode} {position}")
            self._emit(
                "rejection",
                reason=rej.reason,
                detail=rej.detail,
                setpoint=self.setpoint,
                rejections=dict(self.gate.rejections),
            )
            return False
        if mode == "setpoint":
            self.setpoint = position
            self.goal = position
        else:
            self.goal = position
            self.goal_speed = speed
        if yaw is not None and math.isfinite(yaw):
            self.yaw = yaw
        self._emit("goal", mode=mode, goal=position, speed=speed, yaw=yaw)
        return True

    def request_land(self) -> None:
        self._land_requested = True

    # -- tick --------------------------------------------------------------------------------
    def _stream(self, dt: float, goal: Vec3 | None) -> Vec3:
        assert self.setpoint is not None
        cand = self.setpoint
        if goal is not None:
            cand = self.gate.step_towards(self.setpoint, goal, dt, self.goal_speed)
        rej = self.gate.check_setpoint(self.setpoint, cand)
        if rej is not None:  # cannot happen with step_towards; keep the last valid setpoint
            self.gate.reject(rej, "setpoint")
            self._emit("rejection", reason=rej.reason, detail=rej.detail, setpoint=self.setpoint)
            cand = self.setpoint
        self.setpoint = cand
        self._streamed += 1
        return cand

    def step(self, obs: Observation) -> Action:
        t = obs.t
        dt = 0.0 if self._last_t is None else min(max(t - self._last_t, 0.0), 0.2)
        self._last_t = t
        if self.phase is Phase.DONE:
            return Action()
        if self._t0 is None:
            self._t0 = t
            self._phase_t0 = t
        cfg = self.cfg
        pos = obs.position
        armed = obs.arming_state == ARMING_STATE_ARMED
        disarmed = obs.arming_state == ARMING_STATE_DISARMED
        if armed:
            self.armed_ever = True

        # Link-gap policy.
        if self.phase in STREAMING and obs.telemetry_age_s > cfg.gap_abort_s:
            self._fail = "aborted_link_gap"
            self._enter(Phase.ABORTED, t, telemetry_gap_s=round(obs.telemetry_age_s, 2))
            return Action()

        if self.phase is Phase.ABORTED:
            if obs.telemetry_age_s <= cfg.link_ok_age_s:
                if disarmed and obs.landed:
                    return self._finish(t, "aborted_link_gap")
                if armed and t - self._last_cmd_t >= 2.0:
                    self._last_cmd_t = t
                    return Action(commands=(CMD_LAND,))
            if t - self._phase_t0 > cfg.abort_timeout:
                return self._finish(t, "aborted_link_gap")
            return Action()

        if self.phase is Phase.WAIT_DATA:
            if pos is not None and obs.arming_state is not None:
                start = (pos[0], pos[1], pos[2])
                self.gate.origin = start  # geofence relative to the take-off point
                self.setpoint = start
                # altitude is relative to the ground height at start (the estimator origin drifts)
                self.hold = (pos[0], pos[1], pos[2] - cfg.altitude)
                self._enter(Phase.PREFLIGHT, t, hold=self.hold)
            elif t - self._phase_t0 > cfg.wait_data_timeout:
                return self._finish(t, "no_telemetry")
            else:
                return Action()

        if self.phase is Phase.PREFLIGHT:
            sp = self._stream(dt, None)
            if self._streamed >= cfg.min_setpoints:
                self._enter(Phase.ARMING, t)
                self._last_cmd_t = t
                return Action(sp, (CMD_OFFBOARD, CMD_ARM))
            return Action(sp)

        if self.phase is Phase.ARMING:
            sp = self._stream(dt, None)
            if armed and obs.nav_state == NAV_STATE_OFFBOARD:
                self.goal = self.hold
                self.goal_speed = None
                self._enter(Phase.TAKEOFF, t)
            elif t - self._phase_t0 > cfg.arm_timeout:
                return self._finish(t, "arm_rejected")
            elif t - self._last_cmd_t >= cfg.command_retry_s:
                self._last_cmd_t = t
                return Action(sp, (CMD_OFFBOARD, CMD_ARM))
            else:
                return Action(sp)

        if self.phase in (Phase.TAKEOFF, Phase.SETTLE, Phase.HOVER) and disarmed:
            if self._land_requested:
                return self._finish(t, "ok")
            return self._finish(t, "unexpected_disarm")

        if self.phase is Phase.TAKEOFF:
            sp = self._stream(dt, self.goal)
            assert self.hold is not None
            if pos is not None and abs(pos[2] - self.hold[2]) < cfg.reach_tol:
                self._enter(Phase.SETTLE, t)
            elif t - self._phase_t0 > cfg.climb_timeout:
                self._fail = "climb_timeout"
                return self._begin_land(t)
            return Action(sp)

        if self.phase is Phase.SETTLE:
            sp = self._stream(dt, self.goal)
            if t - self._phase_t0 >= cfg.settle_s:
                self._enter(Phase.HOVER, t)
            return Action(sp)

        if self.phase is Phase.HOVER:
            if self._land_requested or (
                cfg.hover_seconds is not None and t - self._phase_t0 >= cfg.hover_seconds
            ):
                return self._begin_land(t)
            sp = self._stream(dt, self.goal)
            cmds: tuple[str, ...] = ()
            # Medium outage: PX4 left offboard (failsafe Land) and has not come back although
            # the link is up again: re-engage. PX4 normally does this by itself.
            if armed and obs.nav_state is not None and obs.nav_state != NAV_STATE_OFFBOARD:
                if self._not_offboard_since is None:
                    self._not_offboard_since = t
                if (
                    obs.telemetry_age_s <= cfg.link_ok_age_s
                    and t - self._not_offboard_since >= cfg.offboard_reengage_s
                    and t - self._last_cmd_t >= cfg.command_retry_s
                ):
                    self._last_cmd_t = t
                    cmds = (CMD_OFFBOARD,)
                    self._emit("reengage_offboard", nav_state=obs.nav_state)
            else:
                self._not_offboard_since = None
            return Action(sp, cmds)

        # Phase.LAND
        if obs.landed and disarmed:
            return self._finish(t, "ok")
        if t - self._phase_t0 > cfg.land_timeout:
            return self._finish(t, "land_timeout")
        if t - self._last_cmd_t >= cfg.command_retry_s * 5:
            self._last_cmd_t = t
            return Action(commands=(CMD_LAND,))
        return Action()

    def _begin_land(self, t: float) -> Action:
        self._enter(Phase.LAND, t)
        self._last_cmd_t = t
        return Action(commands=(CMD_LAND,))

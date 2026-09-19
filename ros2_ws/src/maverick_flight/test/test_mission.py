import math

from maverick_flight.mission import (
    ARMING_STATE_ARMED,
    ARMING_STATE_DISARMED,
    NAV_STATE_OFFBOARD,
    FlightMission,
    MissionConfig,
    Observation,
    Phase,
)

DT = 0.05


class Sim:
    """Drives a mission with a trivial vehicle model: z follows the setpoint when armed."""

    def __init__(self, **cfg):
        self.m = FlightMission(MissionConfig(**cfg))
        self.t = 0.0
        self.pos = [0.0, 0.0, 0.0]
        self.arm = ARMING_STATE_DISARMED
        self.nav = 4
        self.landed = True
        self.last_tel = 0.0
        self.link = True
        self.cmds: list[tuple[float, str]] = []
        self.setpoints: list = []

    def tick(self):
        self.t += DT
        if self.link:
            self.last_tel = self.t
        obs = Observation(
            self.t, tuple(self.pos), self.arm, self.nav, self.landed, self.t - self.last_tel
        )
        act = self.m.step(obs)
        for c in act.commands:
            self.cmds.append((self.t, c))
            if self.link:
                if c == "arm":
                    self.arm = ARMING_STATE_ARMED
                if c == "offboard":
                    self.nav = NAV_STATE_OFFBOARD
                if c == "land":
                    self.nav = 18
                    self.landed = True
                    self.arm = ARMING_STATE_DISARMED
        if act.setpoint is not None:
            self.setpoints.append(act.setpoint)
            if self.link and self.arm == ARMING_STATE_ARMED:
                self.pos = list(act.setpoint)
                self.landed = False
        return act

    def run(self, seconds):
        for _ in range(int(seconds / DT)):
            self.tick()


def test_full_mission_ok():
    s = Sim(hover_seconds=3.0)
    s.run(30)
    assert s.m.phase is Phase.DONE and s.m.result == "ok"
    assert [c for _, c in s.cmds] == ["offboard", "arm", "land"]
    assert math.isclose(s.pos[2], 0.0) or s.pos[2] <= -1.8


def test_setpoint_steps_never_exceed_speed_limit():
    s = Sim(hover_seconds=2.0)
    s.run(20)
    for a, b in zip(s.setpoints, s.setpoints[1:], strict=False):
        assert math.dist(a, b) <= 0.05 + 1e-9


def test_bad_goals_rejected_and_setpoint_held():
    s = Sim(hover_seconds=100.0)
    s.run(8)
    assert s.m.phase is Phase.HOVER
    hold = s.m.setpoint
    assert not s.m.on_goal("goto", (50, 0, -2))
    assert not s.m.on_goal("goto", (math.nan, 0, -2))
    assert not s.m.on_goal("setpoint", (hold[0] + 1.6, hold[1], hold[2]))
    assert not s.m.on_goal("goto", (1, 0, -2), speed=3.0)
    s.run(2)
    assert s.m.setpoint == hold
    assert s.m.gate.rejections == {"geofence": 1, "non_finite": 1, "jump": 1, "speed_limit": 1}
    events = [e for e in s.m.drain_events() if e["event"] == "rejection"]
    assert len(events) == 4 and all(e["setpoint"] == hold for e in events)


def test_good_goal_is_followed_at_limited_speed():
    s = Sim(hover_seconds=100.0)
    s.run(8)
    assert s.m.on_goal("goto", (2.0, 0.0, -2.0))
    s.run(1)
    assert 0.9 <= s.m.setpoint[0] <= 1.1
    s.run(3)
    assert s.m.setpoint[0] == 2.0


def test_short_gap_keeps_streaming():
    s = Sim(hover_seconds=100.0)
    s.run(8)
    n = len(s.setpoints)
    s.link = False
    s.run(4.0)
    s.link = True
    s.run(1)
    assert s.m.phase is Phase.HOVER
    assert len(s.setpoints) - n >= 4.9 / DT - 2


def test_long_gap_aborts_lands_and_never_rearms():
    s = Sim(hover_seconds=100.0)
    s.run(8)
    s.link = False
    s.run(5.5)
    assert s.m.phase is Phase.ABORTED
    n = len(s.setpoints)
    s.run(3)
    assert len(s.setpoints) == n  # stopped streaming
    assert not any(c == "land" for _, c in s.cmds)  # link still down: nothing sent
    s.arm = ARMING_STATE_ARMED  # PX4 failsafe still descending
    s.landed = False
    s.link = True
    s.run(5)
    assert [c for _, c in s.cmds].count("land") >= 1
    assert s.m.phase is Phase.DONE and s.m.result == "aborted_link_gap"
    assert [c for _, c in s.cmds].count("arm") == 1


def test_abort_after_px4_already_landed_just_finishes():
    s = Sim(hover_seconds=100.0)
    s.run(8)
    s.link = False
    s.run(7)
    s.arm = ARMING_STATE_DISARMED
    s.landed = True
    s.link = True
    s.run(1)
    assert s.m.result == "aborted_link_gap"
    assert [c for _, c in s.cmds].count("arm") == 1


def test_reengages_offboard_if_px4_does_not_return():
    s = Sim(hover_seconds=100.0)
    s.run(8)
    s.nav = 18  # PX4 failsafe landing, link alive, armed
    s.run(2)
    assert [c for _, c in s.cmds].count("offboard") == 2


def test_no_telemetry_times_out():
    m = FlightMission(MissionConfig(wait_data_timeout=1.0))
    t = 0.0
    while not m.done and t < 5:
        t += DT
        m.step(Observation(t, None, None, None, None, math.inf))
    assert m.result == "no_telemetry"


def test_arm_rejected_times_out():
    m = FlightMission(MissionConfig(arm_timeout=2.0))
    t = 0.0
    while not m.done and t < 10:  # vehicle ignores every command
        t += DT
        m.step(Observation(t, (0, 0, 0), 1, 4, True, 0.0))
    assert m.result == "arm_rejected"


def test_goal_yaw_is_kept_until_the_next_goal_with_a_yaw():
    s = Sim(hover_seconds=100.0)
    s.run(8)
    assert s.m.phase is Phase.HOVER
    assert math.isnan(s.m.yaw)  # no heading requested: PX4 keeps its own
    assert s.m.on_goal("goto", (1.0, 0.0, -2.0), yaw=1.2)
    assert s.m.yaw == 1.2
    assert s.m.on_goal("goto", (1.0, 1.0, -2.0))  # no yaw: keep the last one
    assert s.m.yaw == 1.2
    assert s.m.on_goal("goto", (1.0, 1.0, -2.0), yaw=float("inf"))  # non-finite: ignored
    assert s.m.yaw == 1.2


def test_climbs_when_the_estimator_origin_has_drifted():
    """Regression: on a reused stack the ground estimate drifted to z = -2.27 (flight 15)."""
    s = Sim(hover_seconds=100.0, altitude=2.0)
    s.pos = [2.1, -0.27, -2.27]
    s.run(8)
    assert s.m.phase is Phase.HOVER
    assert abs(s.pos[2] - (-4.27)) < 0.25
    assert s.m.gate.origin == (2.1, -0.27, -2.27)

from maverick_flight_test.hover_mission import (
    ARMING_STATE_ARMED,
    ARMING_STATE_DISARMED,
    NAV_STATE_OFFBOARD,
    Action,
    Config,
    HoverMission,
    Observation,
)

DT = 0.1
ALT = 2.0


class Sim:
    """Drives a mission with scripted vehicle behaviour at 10 Hz."""

    def __init__(self, cfg: Config | None = None) -> None:
        self.m = HoverMission(cfg or Config(altitude=ALT))
        self.t = 0.0
        self.pos: tuple[float, float, float] | None = (0.0, 0.0, 0.0)
        self.armed = False
        self.offboard = False
        self.landed = True
        self.log: list[Action] = []

    def tick(self) -> Action:
        arming = ARMING_STATE_ARMED if self.armed else ARMING_STATE_DISARMED
        nav = NAV_STATE_OFFBOARD if self.offboard else 4
        a = self.m.step(Observation(self.t, self.pos, arming, nav, self.landed))
        self.log.append(a)
        self.t += DT
        return a

    def run(self, seconds: float, vehicle) -> None:
        end = self.t + seconds
        while self.t < end and not self.m.done:
            vehicle(self, self.tick())

    @property
    def commands(self) -> list[str]:
        return [c for a in self.log for c in a.commands]


def good_vehicle(sim: Sim, a: Action, drift: float = 0.0, wobble: float = 0.0) -> None:
    """Obeys commands: arm, climb 1 m/s, hold, land 1 m/s."""
    if "offboard" in a.commands:
        sim.offboard = True
    if "arm" in a.commands and sim.offboard:
        sim.armed = True
        sim.landed = False
    x, y, z = sim.pos or (0.0, 0.0, 0.0)
    if "land" in a.commands:
        sim.offboard = False
        sim.target_land = True  # type: ignore[attr-defined]
    if getattr(sim, "target_land", False):
        z = min(0.0, z + DT)
        if z >= 0.0:
            sim.landed, sim.armed = True, False
    elif sim.armed:
        z = max(-ALT + wobble, z - DT)
        x += drift * DT
    sim.pos = (x, y, z)


def test_nominal_pass() -> None:
    sim = Sim()
    sim.run(90, good_vehicle)
    v = sim.m.verdict
    assert v is not None and v.passed and v.reason == "ok"
    assert v.max_alt_error_m <= 0.3 and v.max_xy_drift_m <= 0.3
    assert sim.commands.count("land") == 1
    # offboard + arm only after >= 10 setpoints streamed
    first = next(i for i, a in enumerate(sim.log) if a.commands)
    assert sum(a.setpoint is not None for a in sim.log[:first]) >= 9


def test_drift_fail_lands_and_reports() -> None:
    sim = Sim()
    sim.run(90, lambda s, a: good_vehicle(s, a, drift=0.2))
    v = sim.m.verdict
    assert v is not None and not v.passed and v.reason == "drift"
    assert v.max_xy_drift_m > 0.3
    assert "land" in sim.commands


def test_climb_timeout() -> None:
    sim = Sim()

    def stuck(s: Sim, a: Action) -> None:
        if "offboard" in a.commands:
            s.offboard = True
        if "arm" in a.commands:
            s.armed = True
        if "land" in a.commands:
            s.landed, s.armed = True, False

    sim.run(90, stuck)
    v = sim.m.verdict
    assert v is not None and not v.passed and v.reason == "climb_timeout"
    assert "land" in sim.commands


def test_land_timeout() -> None:
    sim = Sim()

    def never_lands(s: Sim, a: Action) -> None:
        good_vehicle(s, a)
        if "land" in a.commands or getattr(s, "target_land", False):
            s.pos = (0.0, 0.0, -ALT)
            s.landed, s.armed = False, True

    sim.run(120, never_lands)
    v = sim.m.verdict
    assert v is not None and not v.passed and v.reason == "land_timeout"


def test_arm_never_confirmed() -> None:
    sim = Sim()
    sim.run(60, lambda s, a: None)
    v = sim.m.verdict
    assert v is not None and not v.passed and v.reason == "arm_rejected"
    assert sim.commands.count("arm") > 1  # retried
    assert "land" not in sim.commands


def test_no_position() -> None:
    sim = Sim()
    sim.pos = None
    sim.run(60, lambda s, a: None)
    v = sim.m.verdict
    assert v is not None and v.reason == "no_position" and not v.passed


def test_settle_excludes_climb_tail_from_hold_error() -> None:
    """Error while still converging (within reach_tol) must not count as hover error."""
    reached_at: list[float] = []

    def converging(s: Sim, a: Action) -> None:
        good_vehicle(s, a)
        x, y, z = s.pos or (0.0, 0.0, 0.0)
        if s.armed and not getattr(s, "target_land", False) and z <= -ALT + 0.19:
            reached_at.append(s.t)
            # 0.19 m low for the first 1.5 s after reaching, then exactly on target
            s.pos = (x, y, -ALT + 0.19 if s.t - reached_at[0] < 1.5 else -ALT)

    sim = Sim()
    sim.run(90, converging)
    v = sim.m.verdict
    assert v is not None and v.passed
    assert v.max_alt_error_m < 0.01

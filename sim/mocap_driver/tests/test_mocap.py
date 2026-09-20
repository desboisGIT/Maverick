import math
import random

from maverick_bridge import mocap_protocol as p
from mocap_driver.driver import spawn_xy_from_env
from mocap_driver.noise import noisy_pose, sample_time_error


def test_roundtrip_and_size():
    s = p.MocapSample(5, 123.456, (1.0, -2.0, 0.5), (0.0, 0.0, 0.7071, 0.7071))
    data = p.encode(s)
    assert len(data) == p.PACKET.size
    assert p.decode(data) == s


def test_bad_packets_rejected():
    for bad in (
        b"",
        b"x" * p.PACKET.size,
        p.encode(p.MocapSample(1, 1, (0,) * 3, (0, 0, 0, 1)))[:-1],
    ):
        try:
            p.decode(bad)
        except p.MocapProtocolError:
            continue
        raise AssertionError("accepted")


def test_noise_levels():
    rng = random.Random(0)
    errs, angs = [], []
    for _ in range(3000):
        pos, q = noisy_pose(rng, (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))
        errs.append(pos[0] - 1.0)
        angs.append(2 * math.asin(min(1.0, math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2))))
        assert abs(sum(c * c for c in q) - 1) < 1e-9
    sd = math.sqrt(sum(e * e for e in errs) / len(errs))
    assert 0.0015 < sd < 0.0025
    assert 0.0 < sum(angs) / len(angs) < math.radians(1.5)
    assert all(abs(sample_time_error(rng)) <= 0.005 for _ in range(100))


def test_spawn_from_env():
    assert spawn_xy_from_env("") == (0.0, 0.0)
    assert spawn_xy_from_env("2.0000,6.6000,0.0000,0,0,-1.5708") == (2.0, 6.6)

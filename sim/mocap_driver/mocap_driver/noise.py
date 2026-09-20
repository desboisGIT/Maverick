"""Mocap measurement model (numpy-free): what a lab optical system adds to the true pose."""

from __future__ import annotations

import math
import random

POS_SIGMA_M = 0.002
ATT_SIGMA_RAD = math.radians(0.5)
LATENCY_JITTER_S = 0.005  # sample time error, uniform +-


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def small_rotation(rng: random.Random, sigma: float):
    """Quaternion (x, y, z, w) of a random rotation with ~sigma rad per axis."""
    rx, ry, rz = (rng.gauss(0.0, sigma) for _ in range(3))
    n = math.sqrt(rx * rx + ry * ry + rz * rz)
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sin(n / 2) / n
    return (rx * s, ry * s, rz * s, math.cos(n / 2))


def noisy_pose(rng: random.Random, pos, quat):
    p = tuple(c + rng.gauss(0.0, POS_SIGMA_M) for c in pos)
    q = quat_mul(small_rotation(rng, ATT_SIGMA_RAD), quat)
    n = math.sqrt(sum(c * c for c in q))
    return p, tuple(c / n for c in q)


def sample_time_error(rng: random.Random) -> float:
    return rng.uniform(-LATENCY_JITTER_S, LATENCY_JITTER_S)

"""Simulated motion-capture pose packets: mocap driver (sim container) -> bridge (drone network).

A lab mocap system sits in the room, not on the drone, and streams the drone's pose to it. In
simulation the driver in the `sim` container reads the model pose from Gazebo and sends one UDP
datagram per sample to the bridge on the drone network. The host never sees it: only PX4's own
estimate (fused from this pose) reaches the host. Pure Python, no ROS, no numpy.

Datagram (little endian, 77 bytes):
    4s  magic     b"MVMC"
    B   version   1
    I   seq
    d   t_s       capture time, wall clock (seconds)
    3d  position  metres, ENU, relative to the spawn point (= PX4 local origin)
    4d  quat      x y z w, body FLU -> ENU
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

MAGIC = b"MVMC"
VERSION = 1
PACKET = struct.Struct("<4sBId3d4d")
DEFAULT_PORT = 5601


class MocapProtocolError(ValueError):
    pass


@dataclass(frozen=True)
class MocapSample:
    seq: int
    t_s: float
    position_enu: tuple[float, float, float]
    quat_xyzw: tuple[float, float, float, float]


def encode(s: MocapSample) -> bytes:
    return PACKET.pack(MAGIC, VERSION, s.seq & 0xFFFFFFFF, s.t_s, *s.position_enu, *s.quat_xyzw)


def decode(data: bytes) -> MocapSample:
    if len(data) != PACKET.size:
        raise MocapProtocolError(f"packet is {len(data)} bytes, expected {PACKET.size}")
    magic, version, seq, t, px, py, pz, qx, qy, qz, qw = PACKET.unpack(data)
    if magic != MAGIC or version != VERSION:
        raise MocapProtocolError("bad magic or version")
    return MocapSample(seq, t, (px, py, pz), (qx, qy, qz, qw))

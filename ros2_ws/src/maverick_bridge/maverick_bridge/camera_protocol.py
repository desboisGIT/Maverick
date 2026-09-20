"""Camera stream wire format: sim camera driver (sim container) -> bridge (drone network).

Stands in for the USB cable of a real camera: plain length-prefixed frames over one TCP
connection, no Gazebo transport and no ground truth. Pure Python, no ROS and no numpy, so
both ends (and the tests) import it directly. The three images travel already encoded (JPEG
RGB, 16-bit PNG depth, 8-bit PNG semantic ids: see depth_codec), so the bridge forwards them
to the wifi link without touching pixels.

Frame on the wire (little endian):
    u32 length          bytes that follow
    header (struct HEADER):
        4s  magic       b"MVCF"
        B   version     2
        B   reserved    0
        u32 seq         driver counter, wraps
        u64 sim_ns      Gazebo simulation time of the paired frames
        d   t_wall      wall clock (s) when the driver had the full set: the capture time that
                        the pose is looked up at (containers share the host clock)
        u16 rgb_w, rgb_h       size of the decoded JPEG
        u16 depth_w, depth_h   size of the depth image (registered with the RGB image)
        u16 sem_w, sem_h       size of the semantic image
        u32 jpeg_len, depth_len, sem_len
    jpeg      jpeg_len bytes
    depth     depth_len bytes, PNG, 16-bit grey, millimetres, 0 = invalid
    semantic  sem_len bytes, PNG, 8-bit grey, worldgen class ids (0 = background)
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

MAGIC = b"MVCF"
VERSION = 3
HEADER = struct.Struct("<4sBBIQdHHHHHHIII")
LENGTH = struct.Struct("<I")
MAX_FRAME_BYTES = 4 * 1024 * 1024
DEFAULT_PORT = 5600


class ProtocolError(ValueError):
    """The byte stream is not a valid frame stream; the connection must be dropped."""


@dataclass(frozen=True)
class CameraFrame:
    seq: int
    sim_ns: int
    t_wall: float
    rgb_size: tuple[int, int]  # (width, height) of the JPEG
    depth_size: tuple[int, int]
    sem_size: tuple[int, int]
    jpeg: bytes
    depth_png: bytes
    semantic_png: bytes


def encode(frame: CameraFrame) -> bytes:
    header = HEADER.pack(
        MAGIC,
        VERSION,
        0,
        frame.seq & 0xFFFFFFFF,
        frame.sim_ns,
        frame.t_wall,
        *frame.rgb_size,
        *frame.depth_size,
        *frame.sem_size,
        len(frame.jpeg),
        len(frame.depth_png),
        len(frame.semantic_png),
    )
    body = header + frame.jpeg + frame.depth_png + frame.semantic_png
    return LENGTH.pack(len(body)) + body


def decode(body: bytes) -> CameraFrame:
    if len(body) < HEADER.size:
        raise ProtocolError("frame shorter than its header")
    magic, version, _, seq, sim_ns, t_wall, rw, rh, dw, dh, sw, sh, nj, nd, ns = HEADER.unpack_from(
        body
    )
    if magic != MAGIC:
        raise ProtocolError(f"bad magic {magic!r}")
    if version != VERSION:
        raise ProtocolError(f"unsupported version {version}")
    if len(body) != HEADER.size + nj + nd + ns:
        raise ProtocolError("frame length does not match its header")
    off = HEADER.size
    jpeg = body[off : off + nj]
    depth = body[off + nj : off + nj + nd]
    sem = body[off + nj + nd :]
    return CameraFrame(seq, sim_ns, t_wall, (rw, rh), (dw, dh), (sw, sh), jpeg, depth, sem)


class FrameReader:
    """Incremental parser: feed() arbitrary chunks, get complete frames back."""

    def __init__(self, max_frame_bytes: int = MAX_FRAME_BYTES) -> None:
        self._buf = bytearray()
        self._max = max_frame_bytes

    def feed(self, data: bytes) -> list[CameraFrame]:
        self._buf += data
        out: list[CameraFrame] = []
        while len(self._buf) >= LENGTH.size:
            (n,) = LENGTH.unpack_from(self._buf)
            if n > self._max or n < HEADER.size:
                raise ProtocolError(f"implausible frame length {n}")
            if len(self._buf) < LENGTH.size + n:
                break
            body = bytes(self._buf[LENGTH.size : LENGTH.size + n])
            del self._buf[: LENGTH.size + n]
            out.append(decode(body))
        return out

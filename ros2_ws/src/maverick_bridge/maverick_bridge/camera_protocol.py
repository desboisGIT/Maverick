"""Camera stream wire format: sim camera driver (sim container) -> bridge (drone network).

Stands in for the USB cable of a real camera: plain length-prefixed frames over one TCP
connection, no Gazebo transport and no ground truth. Pure Python, no ROS and no numpy, so
both ends (and the tests) import it directly.

Frame on the wire (little endian):
    u32 length          bytes that follow
    header (struct HEADER):
        4s  magic       b"MVCF"
        B   version     1
        B   reserved    0
        u32 seq         driver counter, wraps
        u64 sim_ns      Gazebo simulation time of the paired frames
        u16 rgb_w, rgb_h   size of the decoded JPEG
        u16 map_w, map_h   size of depth and semantic images
        u32 jpeg_len
    depth     map_w*map_h u16 little endian, millimetres, 0 = invalid
    semantic  map_w*map_h u8 class ids (worldgen classes, 0 = background)
    jpeg      jpeg_len bytes
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

MAGIC = b"MVCF"
VERSION = 1
HEADER = struct.Struct("<4sBBIQHHHHI")
LENGTH = struct.Struct("<I")
MAX_FRAME_BYTES = 4 * 1024 * 1024
DEFAULT_PORT = 5600


class ProtocolError(ValueError):
    """The byte stream is not a valid frame stream; the connection must be dropped."""


@dataclass(frozen=True)
class CameraFrame:
    seq: int
    sim_ns: int
    rgb_size: tuple[int, int]  # (width, height) of the JPEG
    map_size: tuple[int, int]  # (width, height) of depth and semantic
    depth_mm: bytes  # u16 little endian
    semantic: bytes  # u8
    jpeg: bytes


def encode(frame: CameraFrame) -> bytes:
    mw, mh = frame.map_size
    if len(frame.depth_mm) != mw * mh * 2:
        raise ProtocolError(f"depth is {len(frame.depth_mm)} bytes, expected {mw * mh * 2}")
    if len(frame.semantic) != mw * mh:
        raise ProtocolError(f"semantic is {len(frame.semantic)} bytes, expected {mw * mh}")
    header = HEADER.pack(
        MAGIC,
        VERSION,
        0,
        frame.seq & 0xFFFFFFFF,
        frame.sim_ns,
        frame.rgb_size[0],
        frame.rgb_size[1],
        mw,
        mh,
        len(frame.jpeg),
    )
    body = header + frame.depth_mm + frame.semantic + frame.jpeg
    return LENGTH.pack(len(body)) + body


def decode(body: bytes) -> CameraFrame:
    if len(body) < HEADER.size:
        raise ProtocolError("frame shorter than its header")
    magic, version, _, seq, sim_ns, rw, rh, mw, mh, jpeg_len = HEADER.unpack_from(body)
    if magic != MAGIC:
        raise ProtocolError(f"bad magic {magic!r}")
    if version != VERSION:
        raise ProtocolError(f"unsupported version {version}")
    n = mw * mh
    if len(body) != HEADER.size + n * 3 + jpeg_len:
        raise ProtocolError("frame length does not match its header")
    off = HEADER.size
    depth = body[off : off + n * 2]
    semantic = body[off + n * 2 : off + n * 3]
    jpeg = body[off + n * 3 :]
    return CameraFrame(seq, sim_ns, (rw, rh), (mw, mh), depth, semantic, jpeg)


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

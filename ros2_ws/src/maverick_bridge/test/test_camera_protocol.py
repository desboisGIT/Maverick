import random

import pytest
from maverick_bridge import camera_protocol as p


def make_frame(seq=1, sim_ns=1_234_567_890_123, jpeg=b"\xff\xd8jpeg\xff\xd9"):
    return p.CameraFrame(
        seq, sim_ns, (320, 240), (320, 240), (160, 120), jpeg, b"\x89PNGdepth", b"\x89PNGsem"
    )


def test_roundtrip():
    f = make_frame()
    wire = p.encode(f)
    assert int.from_bytes(wire[:4], "little") == len(wire) - 4
    assert p.decode(wire[4:]) == f


def test_real_sizes():
    f = p.CameraFrame(
        7, 5, (320, 240), (320, 240), (160, 120), b"x" * 15000, b"d" * 60000, b"s" * 2000
    )
    wire = p.encode(f)
    assert len(wire) == 4 + p.HEADER.size + 15000 + 60000 + 2000
    assert p.decode(wire[4:]) == f


def test_reader_any_chunking():
    frames = [make_frame(seq=i, sim_ns=i * 200_000_000) for i in range(5)]
    wire = b"".join(p.encode(f) for f in frames)
    rng = random.Random(3)
    for _ in range(20):
        r, got, i = p.FrameReader(), [], 0
        while i < len(wire):
            n = rng.randint(1, 50)
            got += r.feed(wire[i : i + n])
            i += n
        assert got == frames


def test_reader_partial_then_rest():
    wire = p.encode(make_frame())
    r = p.FrameReader()
    assert r.feed(wire[:-1]) == []
    assert len(r.feed(wire[-1:])) == 1


def test_bad_magic_version_length():
    wire = bytearray(p.encode(make_frame()))
    bad = bytearray(wire)
    bad[4:8] = b"XXXX"
    with pytest.raises(p.ProtocolError):
        p.FrameReader().feed(bytes(bad))
    bad = bytearray(wire)
    bad[8] = 99
    with pytest.raises(p.ProtocolError):
        p.FrameReader().feed(bytes(bad))
    with pytest.raises(p.ProtocolError):  # huge length prefix is rejected before buffering it
        p.FrameReader().feed((10**9).to_bytes(4, "little"))
    with pytest.raises(p.ProtocolError):  # length ok but header says a longer jpeg
        body = bytearray(wire[4:])
        body[-1:] = b""
        p.decode(bytes(body))

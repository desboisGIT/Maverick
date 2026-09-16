"""Gateway protocol (pure): envelopes, command parser, connect state, helpers, websocket server."""

import asyncio
import base64
import json
import os
import struct

import numpy as np
from maverick_gateway.protocol import (
    ChangeFilter,
    GatewayState,
    PoseTracker,
    agent_step_data,
    camera_frame_data,
    check_skill_progress,
    envelope,
    jpeg_size,
    link_quality,
    parse_command,
    parse_nvidia_smi,
    status_data,
)
from maverick_gateway.schema import validate
from maverick_gateway.wsserver import WebSocketServer, accept_key, encode_frame
from maverick_mapping import optical_rotation


def test_envelope_shape():
    e = json.loads(envelope("pose", {"a": 1}, 12.3456))
    assert e == {"type": "pose", "t": 12.346, "data": {"a": 1}}


def test_parse_explore_with_and_without_duration():
    c = parse_command("Explore the flat")
    assert (c.kind, c.request) == ("explore", {"skill": "explore", "budget_s": 360.0})
    assert parse_command("explore for 2 minutes").request["budget_s"] == 120.0
    assert parse_command("please map the apartment for 90 seconds").request["budget_s"] == 90.0
    assert parse_command("look around", 100.0).request["budget_s"] == 100.0


def test_parse_goto_room_synonyms():
    for text, room in [
        ("Go to the kitchen", "kitchen"),
        ("take me to the living room", "living"),
        ("fly to the lounge, please", "living"),
        ("head to the bathroom", "bathroom"),
        ("go to the bed room", "bedroom"),
        ("bring me to the hallway", "hallway"),
        ("go to the study", "office"),
    ]:
        c = parse_command(text)
        assert c.kind == "goto_room" and c.request["room_type"] == room, text
        assert c.reply.startswith("Heading to the")


def test_parse_stop_land_and_unknown():
    assert parse_command("stop").kind == "cancel"
    assert parse_command("stop exploring").request == {"skill": "cancel"}
    assert parse_command("land now").kind == "land"
    u = parse_command("make me a sandwich")
    assert u.kind == "unknown" and u.request is None and "explore" in u.reply


def test_jpeg_size_and_camera_frame():
    # SOI, APP0 (len 16), SOF0 (480 x 640)
    jpg = (
        b"\xff\xd8\xff\xe0\x00\x10JFIF\x00"
        + bytes(9)
        + b"\xff\xc0\x00\x11\x08\x01\xe0\x02\x80"
        + bytes(10)
    )
    assert jpeg_size(jpg) == (640, 480) and jpeg_size(b"nope") is None
    d = camera_frame_data(7, jpg, 73.0)
    assert (d["seq"], d["width"], d["height"], d["mime"]) == (7, 640, 480, "image/jpeg")
    assert base64.b64decode(d["data"]) == jpg and d["hfov_deg"] == 73.0


def test_pose_tracker_velocity_and_euler():
    pt = PoseTracker(smooth=0.0)
    r = optical_rotation(np.pi / 2)  # facing north
    pt.update(0.0, np.array([0.0, 0.0, 1.0]), r)
    d = pt.update(1.0, np.array([0.5, 0.0, 1.0]), r)
    assert abs(d["yaw"] - np.pi / 2) < 1e-3 and d["velocity"] == [0.5, 0.0, 0.0]
    assert d["position"] == [0.5, 0.0, 1.0] and abs(d["pitch"]) < 1e-6


def test_status_and_helpers():
    st = status_data(
        {"phase": "hover", "telemetry_age_s": 0.2},
        {"skill": "explore", "state": "running"},
        (3.0, 16.0),
        100.0,
    )
    assert st["mode"] == "hover: explore" and st["armed"] and st["link"]["quality"] == "good"
    lost = status_data(None, None, (0.0, 16.0), 100.0)
    assert lost["link"]["quality"] == "lost" and not lost["armed"]
    assert link_quality(2.0) == "degraded" and link_quality(9.0) == "lost"
    assert parse_nvidia_smi("4096, 16384\n") == (4.0, 16.0) and parse_nvidia_smi("N/A") is None


def test_skill_progress_validation_and_agent_step():
    assert check_skill_progress({"skill": "dance", "state": "running"}) is None
    d = check_skill_progress(
        {"skill": "explore", "state": "running", "progress": 0.3, "message": "x", "eta_s": 5}
    )
    assert d == {
        "skill": "explore",
        "state": "running",
        "progress": 0.3,
        "message": "x",
        "eta_s": 5,
    }
    a = agent_step_data(
        "p1", "explore", {"skill": "explore", "state": "succeeded", "message": "ok"}, 12.34
    )
    assert a["steps"][0]["status"] == "done" and a["steps"][0]["detail"] == "ok"


def test_connect_burst_order_and_map_reconstruction():
    st = GatewayState()
    assert st.connect_burst() == []
    st.map_delta(
        {
            "t": 1.0,
            "resolution": 0.1,
            "origin": [0.05] * 3,
            "reset": True,
            "add": [1, 2, 3, 4, 5, 6],
            "remove": [],
        }
    )
    st.map_delta(
        {
            "t": 2.0,
            "resolution": 0.1,
            "origin": [0.05] * 3,
            "reset": False,
            "add": [7, 8, 9],
            "remove": [1, 2, 3],
        }
    )
    st.put("status", {"battery_pct": 100})
    st.put("pose", {"position": [0, 0, 1]})
    st.put("rooms", {"rooms": []})
    st.put("transcript", {"id": "u1", "role": "user", "text": "hi"})
    st.put("camera_frame", {"seq": 1})  # not part of the connect state
    burst = [json.loads(f) for f in st.connect_burst()]
    assert [b["type"] for b in burst] == ["map_update", "rooms", "status", "pose", "transcript"]
    m = burst[0]["data"]
    assert m["reset"] is True and sorted(m["add"]) == [4, 5, 6, 7, 8, 9] and m["remove"] == []
    assert set(m) == {"resolution", "origin", "reset", "add", "remove"}


def test_change_filter():
    c = ChangeFilter()
    assert c.changed("rooms", [1]) and not c.changed("rooms", [1]) and c.changed("rooms", [2])
    assert c.changed("objects", [2])


# --- websocket server -------------------------------------------------------------------------
def test_accept_key_matches_rfc_example():
    assert accept_key("dGhlIHNhbXBsZSBub25jZQ==") == "s3pPLMBiTxaQ9kYGzzhZRbK+xOo="


def masked(payload: bytes, opcode: int = 0x1) -> bytes:
    key = os.urandom(4)
    n = len(payload)
    head = bytes([0x80 | opcode])
    head += bytes([0x80 | n]) if n < 126 else bytes([0x80 | 126]) + struct.pack("!H", n)
    return head + key + bytes(b ^ key[i & 3] for i, b in enumerate(payload))


async def read_text(reader) -> str:
    b0, b1 = await reader.readexactly(2)
    n = b1 & 0x7F
    if n == 126:
        (n,) = struct.unpack("!H", await reader.readexactly(2))
    assert b0 & 0x0F == 1
    return (await reader.readexactly(n)).decode()


def test_websocket_server_handshake_burst_broadcast_and_input():
    got = []
    srv = WebSocketServer("127.0.0.1", 0, lambda: ['{"type":"hello"}'], got.append)
    srv.start()

    async def client():
        r, w = await asyncio.open_connection("127.0.0.1", srv.port)
        w.write(
            b"GET / HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
            b"Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n"
        )
        head = await r.readuntil(b"\r\n\r\n")
        assert b"101" in head and b"s3pPLMBiTxaQ9kYGzzhZRbK+xOo=" in head
        assert json.loads(await read_text(r))["type"] == "hello"  # state on connect first
        srv.broadcast('{"type":"pose"}')
        assert json.loads(await read_text(r))["type"] == "pose"
        big = json.dumps({"type": "x", "pad": "y" * 70000})
        srv.broadcast(big)
        b0, b1 = await r.readexactly(2)
        assert b1 == 127
        (n,) = struct.unpack("!Q", await r.readexactly(8))
        assert len(await r.readexactly(n)) == len(big)
        w.write(masked(b'{"type":"text_command","data":{"text":"explore"}}'))
        w.write(masked(b"ping!", 0x9))
        b0, b1 = await r.readexactly(2)
        assert b0 & 0x0F == 0xA and await r.readexactly(b1) == b"ping!"
        w.write(masked(b"", 0x8))
        await w.drain()
        w.close()

    asyncio.run(asyncio.wait_for(client(), 10.0))
    srv.stop()
    assert got == ['{"type":"text_command","data":{"text":"explore"}}']


def test_websocket_server_answers_plain_http_and_survives_garbage():
    srv = WebSocketServer("127.0.0.1", 0, lambda: [], lambda t: None)
    srv.start()

    async def run():
        r, w = await asyncio.open_connection("127.0.0.1", srv.port)
        w.write(b"GET /health HTTP/1.1\r\nHost: x\r\n\r\n")
        assert b"426" in await r.readline()
        w.close()
        r, w = await asyncio.open_connection("127.0.0.1", srv.port)
        w.write(b"\x00\x01garbage\r\n\r\n")
        await w.drain()
        w.close()

    asyncio.run(asyncio.wait_for(run(), 10.0))
    srv.broadcast("still alive")
    srv.stop()


def test_encode_frame_lengths():
    assert encode_frame(b"a" * 5)[:2] == b"\x81\x05"
    assert encode_frame(b"a" * 200)[:4] == b"\x81\x7e\x00\xc8"


def test_schema_accepts_what_the_protocol_builds_and_rejects_broken_frames():
    frames = [
        envelope(
            "pose", {"position": [0, 0, 1], "yaw": 0, "pitch": 0, "roll": 0, "velocity": [0, 0, 0]}
        ),
        envelope("camera_frame", camera_frame_data(1, b"\xff\xd8", 73.0)),
        envelope(
            "detections",
            {
                "frame_seq": 1,
                "boxes": [{"label": "sofa", "confidence": 0.9, "box": [0.1, 0.2, 0.3, 0.4]}],
            },
        ),
        envelope(
            "map_update",
            {
                "resolution": 0.1,
                "origin": [0.05] * 3,
                "reset": True,
                "add": [1, 2, 3],
                "remove": [],
            },
        ),
        envelope("frontiers", {"frontiers": [{"id": "f0", "position": [1, 2, 1.2], "size": 5}]}),
        envelope(
            "path",
            {
                "path_id": "p",
                "status": "executing",
                "waypoints": [[0, 0, 1], [1, 1, 1]],
                "goal": [1, 1, 1],
            },
        ),
        envelope(
            "rooms",
            {
                "rooms": [
                    {
                        "id": "r1",
                        "type": "kitchen",
                        "label": "Kitchen",
                        "polygon": [[0, 0], [1, 0], [1, 1]],
                        "confidence": 0.8,
                        "visited": False,
                    }
                ]
            },
        ),
        envelope(
            "objects",
            {
                "objects": [
                    {
                        "id": "o1",
                        "label": "fridge",
                        "position": [1, 1, 1],
                        "size": [1, 1, 2],
                        "confidence": 0.9,
                        "last_seen": 1.0,
                        "status": "present",
                    }
                ]
            },
        ),
        envelope("transcript", {"id": "u1", "role": "user", "text": "hi"}),
        envelope("agent_step", agent_step_data("p1", "g", None, 1.0)),
        envelope(
            "skill_progress", {"skill": "goto", "state": "running", "progress": -1, "message": "m"}
        ),
        envelope("status", status_data({"phase": "hover"}, None, (1.0, 16.0), 100.0)),
        envelope("future_event", {"anything": 1}),
    ]
    for f in frames:
        assert validate(json.loads(f)) == [], f
    bad = [
        {"type": "pose", "data": {}},
        json.loads(
            envelope(
                "map_update",
                {
                    "resolution": 0.1,
                    "origin": [0, 0, 0],
                    "reset": False,
                    "add": [1, 2],
                    "remove": [],
                },
            )
        ),
        json.loads(
            envelope(
                "rooms",
                {
                    "rooms": [
                        {
                            "id": "r",
                            "type": "garage",
                            "label": "x",
                            "polygon": [],
                            "confidence": 1,
                            "visited": True,
                        }
                    ]
                },
            )
        ),
        json.loads(
            envelope(
                "skill_progress",
                {"skill": "dance", "state": "running", "progress": 0, "message": ""},
            )
        ),
    ]
    for f in bad:
        assert validate(f), f

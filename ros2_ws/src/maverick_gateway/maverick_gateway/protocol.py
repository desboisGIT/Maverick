"""Gateway event protocol v1 (docs/gateway-events.md): envelopes, state on connect, command parser.

Pure Python + numpy; no ROS and no sockets, so it is unit-tested directly.
"""

from __future__ import annotations

import base64
import json
import re
import time
from collections import deque
from dataclasses import dataclass

import numpy as np
from maverick_mapping.ros_conv import camera_euler
from maverick_mapping.voxel_delta import OccupiedSet

PROTOCOL_VERSION = 1
SKILLS = ("explore", "goto", "search", "inspect", "follow", "monitor", "return", "report")
SKILL_STATES = ("running", "succeeded", "failed", "cancelled")
PATH_STATES = ("planned", "executing", "done", "failed")


def envelope(type_: str, data: dict | list, t: float | None = None) -> str:
    return json.dumps(
        {"type": type_, "t": round(time.time() if t is None else t, 3), "data": data},
        separators=(",", ":"),
    )


# --- text commands ----------------------------------------------------------------------------
ROOM_WORDS = {
    "kitchen": "kitchen",
    "living room": "living",
    "livingroom": "living",
    "lounge": "living",
    "living": "living",
    "bedroom": "bedroom",
    "bed room": "bedroom",
    "bathroom": "bathroom",
    "bath room": "bathroom",
    "restroom": "bathroom",
    "toilet": "bathroom",
    "hallway": "hallway",
    "corridor": "hallway",
    "hall": "hallway",
    "office": "office",
    "study": "office",
}
ROOM_NAMES = {
    "living": "living room",
    "kitchen": "kitchen",
    "bedroom": "bedroom",
    "bathroom": "bathroom",
    "hallway": "hallway",
    "office": "office",
}


@dataclass
class Command:
    kind: str  # explore | goto_room | cancel | land | unknown
    request: dict | None  # a /maverick/skill/request payload (explore, goto_room, cancel)
    reply: str  # what Maverick says back


def _room_in(text: str) -> str | None:
    for word in sorted(ROOM_WORDS, key=len, reverse=True):
        if re.search(rf"\b{re.escape(word)}\b", text):
            return ROOM_WORDS[word]
    return None


def _duration_s(text: str) -> float | None:
    m = re.search(r"(\d+(?:\.\d+)?)\s*(minutes?|mins?|m|seconds?|secs?|s)\b", text)
    if not m:
        return None
    v = float(m.group(1))
    return v * 60.0 if m.group(2).startswith("m") else v


def parse_command(text: str, default_budget_s: float = 360.0) -> Command:
    """Tiny keyword parser (the LLM agent replaces it in S4). English only."""
    t = " ".join(text.lower().replace(",", " ").split())
    if re.search(r"\b(land|touch ?down)\b", t):
        return Command("land", None, "Landing.")
    if re.search(r"\b(stop|cancel|abort|halt|hold|freeze|wait)\b", t):
        return Command("cancel", {"skill": "cancel"}, "Stopping and holding position.")
    room = _room_in(t)
    if room and re.search(
        r"\b(go|goto|fly|head|take|bring|move|navigate|come|visit|check|to)\b", t
    ):
        return Command(
            "goto_room",
            {"skill": "goto_room", "room_type": room},
            f"Heading to the {ROOM_NAMES[room]}.",
        )
    if re.search(r"\b(explore|exploring|map|survey|scan|look around|discover)\b", t):
        budget = _duration_s(t) or default_budget_s
        req = {"skill": "explore", "budget_s": budget}
        return Command("explore", req, f"Exploring for up to {budget / 60:.0f} min.")
    return Command(
        "unknown",
        None,
        'I only understand "explore", "go to the <room>", "stop" and "land" for now.',
    )


# --- camera and pose --------------------------------------------------------------------------
def jpeg_size(data: bytes) -> tuple[int, int] | None:
    """(width, height) from a JPEG's SOF marker, or None."""
    if data[:2] != b"\xff\xd8":
        return None
    i = 2
    n = len(data)
    while i + 9 < n:
        if data[i] != 0xFF:
            i += 1
            continue
        m = data[i + 1]
        if m in (0xD8, 0x01) or 0xD0 <= m <= 0xD7:
            i += 2
            continue
        seg = int.from_bytes(data[i + 2 : i + 4], "big")
        if m in (0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF):
            h = int.from_bytes(data[i + 5 : i + 7], "big")
            w = int.from_bytes(data[i + 7 : i + 9], "big")
            return w, h
        i += 2 + seg
    return None


def camera_frame_data(
    seq: int, jpeg: bytes, hfov_deg: float, default_size: tuple[int, int] = (640, 480)
) -> dict:
    w, h = jpeg_size(jpeg) or default_size
    return {
        "seq": seq,
        "width": w,
        "height": h,
        "mime": "image/jpeg",
        "data": base64.b64encode(jpeg).decode("ascii"),
        "hfov_deg": round(hfov_deg, 2),
    }


class PoseTracker:
    """Camera pose stream -> gateway `pose` data with a smoothed velocity."""

    def __init__(self, smooth: float = 0.5) -> None:
        self.smooth = smooth
        self._last: tuple[float, np.ndarray] | None = None
        self.vel = np.zeros(3)

    def update(self, t: float, pos: np.ndarray, r_wc: np.ndarray) -> dict:
        if self._last is not None and t > self._last[0]:
            v = (pos - self._last[1]) / (t - self._last[0])
            if np.linalg.norm(v) < 10.0:
                self.vel = self.smooth * self.vel + (1 - self.smooth) * v
        self._last = (t, pos.copy())
        yaw, pitch, roll = camera_euler(r_wc)
        return {
            "position": [round(float(v), 3) for v in pos],
            "yaw": round(yaw, 4),
            "pitch": round(pitch, 4),
            "roll": round(roll, 4),
            "velocity": [round(float(v), 3) for v in self.vel],
        }


def link_quality(telemetry_age_s: float | None) -> str:
    if telemetry_age_s is None:
        return "lost"
    return "good" if telemetry_age_s < 1.0 else "degraded" if telemetry_age_s < 5.0 else "lost"


ARMED_PHASES = {"arming", "takeoff", "settle", "hover", "land", "aborted"}


def status_data(
    flight: dict | None,
    skill: dict | None,
    vram: tuple[float, float],
    battery_pct: float,
) -> dict:
    """Gateway `status` data from the flight client's status JSON and the active skill."""
    phase = str((flight or {}).get("phase", "unknown"))
    running = skill is not None and skill.get("state") == "running"
    mode = f"{phase}: {skill['skill']}" if running else phase
    age = (flight or {}).get("telemetry_age_s")
    return {
        "battery_pct": battery_pct,
        "link": {"quality": link_quality(age if flight is not None else None)},
        "mode": mode,
        "armed": phase in ARMED_PHASES,
        "vram_used_gb": round(vram[0], 2),
        "vram_total_gb": round(vram[1], 2),
    }


def parse_nvidia_smi(text: str) -> tuple[float, float] | None:
    """`nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader,nounits` -> GB."""
    try:
        used, total = (float(v) for v in text.strip().splitlines()[0].split(","))
    except (ValueError, IndexError):
        return None
    return used / 1024.0, total / 1024.0


def agent_step_data(plan_id: str, goal: str, skill: dict | None, elapsed_s: float) -> dict:
    """A one-step plan for the console, derived from the command and the skill's progress."""
    state = (skill or {}).get("state", "running")
    status = {
        "running": "running",
        "succeeded": "done",
        "failed": "failed",
        "cancelled": "skipped",
    }[state if state in SKILL_STATES else "running"]
    label = (skill or {}).get("skill", "skill")
    step = {
        "id": "s1",
        "label": f"{label}",
        "status": status,
        "elapsed_s": round(elapsed_s, 1),
    }
    if skill and skill.get("message"):
        step["detail"] = skill["message"]
    return {"plan_id": plan_id, "goal": goal, "steps": [step]}


def detections_data(det: dict, frame_seq: int) -> dict:
    return {"frame_seq": frame_seq, "boxes": det.get("boxes", [])}


def check_skill_progress(d: dict) -> dict | None:
    if d.get("skill") not in SKILLS or d.get("state") not in SKILL_STATES:
        return None
    out = {k: d[k] for k in ("skill", "state", "message", "eta_s") if k in d}
    out["progress"] = float(d.get("progress", -1))
    out.setdefault("message", "")
    return out


# --- state on connect -------------------------------------------------------------------------
LATEST_TYPES = (
    "rooms",
    "objects",
    "frontiers",
    "path",
    "agent_step",
    "skill_progress",
    "status",
    "pose",
)


class GatewayState:
    """What a newly connected console must be sent first, in the order of the protocol doc."""

    def __init__(self, transcript_len: int = 40) -> None:
        self.occupied = OccupiedSet()
        self.latest: dict[str, str] = {}
        self.transcript: deque[str] = deque(maxlen=transcript_len)
        self.map_t = 0.0

    def map_delta(self, d: dict) -> str:
        """Apply a voxel delta from the mapping node; returns the `map_update` frame to stream."""
        self.occupied.apply(d)
        self.map_t = float(d.get("t", time.time()))
        data = {k: d[k] for k in ("resolution", "origin", "reset", "add", "remove")}
        return envelope("map_update", data, self.map_t)

    def put(self, type_: str, data: dict | list, t: float | None = None) -> str:
        frame = envelope(type_, data, t)
        if type_ == "transcript":
            self.transcript.append(frame)
        elif type_ in LATEST_TYPES:
            self.latest[type_] = frame
        return frame

    def connect_burst(self) -> list[str]:
        out = []
        if len(self.occupied.codes) or self.map_t:
            out.append(envelope("map_update", self.occupied.snapshot(), self.map_t or None))
        out += [self.latest[k] for k in LATEST_TYPES if k in self.latest]
        out += list(self.transcript)
        return out


class ChangeFilter:
    """Pass a payload only when it differs from the previous one of its key."""

    def __init__(self) -> None:
        self._last: dict[str, str] = {}

    def changed(self, key: str, payload: object) -> bool:
        s = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        if self._last.get(key) == s:
            return False
        self._last[key] = s
        return True

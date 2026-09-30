"""Test doubles shared by the agent tests."""

from __future__ import annotations

import threading
import time

from maverick_agent.backends.base import Response
from maverick_agent.loop import SkillResult


class FakeExecutive:
    def __init__(self, results: dict[str, SkillResult] | None = None, hang: bool = False):
        self.results = results or {}
        self.hang = hang
        self.calls: list[tuple] = []
        self.spoken: list[str] = []
        self.lock = threading.Lock()
        self._cur = None
        self.mem_delay = 0.0

    def start_skill(self, name, args):
        self.calls.append(("start", name, args))
        self._cur = name

    def wait_skill(self, timeout_s):
        if self.hang:
            time.sleep(min(timeout_s, 0.01))
            return None
        return self.results.get(self._cur, SkillResult("succeeded", f"{self._cur} done"))

    def cancel_skill(self, reason):
        self.calls.append(("cancel", reason))

    def emergency(self, kind):
        self.calls.append(("emergency", kind))

    def memory_query(self, question):
        time.sleep(self.mem_delay)
        self.calls.append(("memory", question))
        return "kitchen known"

    def say(self, text):
        with self.lock:
            self.spoken.append(text)

    def send_photo(self, target, caption):
        self.calls.append(("photo", target, caption))
        return "queued"

    def state_block(self):
        return "rooms: kitchen(r1)"


class ListBackend:
    """Returns the given responses in order and records every request."""

    def __init__(self, responses):
        self.responses = list(responses)
        self.requests = []

    def create(self, system, tools, messages):
        self.requests.append((system, tools, messages))
        return self.responses.pop(0)


def tu(name, args, i="t1"):
    return {"type": "tool_use", "id": i, "name": name, "input": args}


def resp(blocks, stop="tool_use", usage=None):
    return Response(blocks, stop, usage or {})

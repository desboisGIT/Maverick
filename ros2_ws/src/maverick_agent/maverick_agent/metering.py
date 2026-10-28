"""Backend wrapper for live runs: latency, tokens and estimated cost per model call, a spend cap.

Writes one JSON line per call to `log_path` (when set). Pure Python, no ROS. Prices come from
eval_commands.PRICES (USD per million tokens, treated 1:1 as EUR, which over-estimates).
"""

from __future__ import annotations

import json
import threading
import time

from .backends.base import Backend, BackendError, Response
from .eval_commands import cost_of


class MeteredBackend:
    def __init__(self, inner: Backend, kind: str, cap_eur: float = 0.0, log_path: str = ""):
        self.inner, self.kind, self.cap, self.log_path = inner, kind, cap_eur, log_path
        self.cost = 0.0
        self.calls = 0
        self.usage: dict = {}
        self._lock = threading.Lock()

    def _log(self, row: dict) -> None:
        if not self.log_path:
            return
        try:
            with open(self.log_path, "a") as f:
                f.write(json.dumps(row) + "\n")
        except OSError:
            pass

    def create(self, system: str, tools: list[dict], messages: list[dict]) -> Response:
        if self.cap > 0 and self.cost >= self.cap:
            self._log({"t": time.time(), "error": "spend cap reached", "cum_cost_eur": self.cost})
            raise BackendError("spend cap reached")
        t0 = time.perf_counter()
        try:
            r = self.inner.create(system, tools, messages)
        except BackendError as exc:
            self._log({"t": time.time(), "error": str(exc)[:300], "cum_cost_eur": self.cost})
            raise
        dt = time.perf_counter() - t0
        c = cost_of(self.kind, r.usage)
        with self._lock:
            self.cost += c
            self.calls += 1
            for k, v in r.usage.items():
                if isinstance(v, int | float):
                    self.usage[k] = self.usage.get(k, 0) + v
        self._log(
            {
                "t": time.time(),
                "latency_s": round(dt, 3),
                "usage": {k: v for k, v in r.usage.items() if isinstance(v, int | float)},
                "cost_eur": round(c, 6),
                "cum_cost_eur": round(self.cost, 6),
                "stop_reason": r.stop_reason,
                "tools": [b["name"] for b in r.tool_uses()],
            }
        )
        return r

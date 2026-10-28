"""The manual agent loop (docs/s4-plan.md §2). Pure Python, no ROS imports.

One operator utterance -> a mission: acknowledgement from a template, then model calls until
`end_turn`. Each `tool_use` is validated and executed through the injected `Executive`; all
`tool_result`s of one model response go back in ONE user message. Motion tools run one at a time
and block until the skill ends; non-motion tools run in parallel. The mission is capped (calls and
wall time). "stop" / "land" / "abort" bypass the model.
"""

from __future__ import annotations

import logging
import re
import threading
import time
from collections import deque
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from typing import Protocol

from .backends.base import Backend, BackendError
from .events import agent_step_event, step_label, transcript_event
from .prompts import SYSTEM_PROMPT
from .tools import TOOLS, ToolError, tool_schemas, validate_call

log = logging.getLogger("maverick_agent.loop")

EMERGENCY_RE = re.compile(r"\b(stop|land|abort)\b", re.IGNORECASE)
APOLOGY = "Sorry, this is taking too long. I am stopping here."
LOST_CONNECTION = "I lost my connection, so I am holding position."
REFUSAL = "I can't do that one."
_OBJECT_ID_RE = re.compile(r"object_id (\S+?)[,)\s]")
_NEG_RE = re.compile(
    r"\b(no|not|none|nothing|nope|never|without|cannot)\b|n't\b|\bcouldn\w*|\bdidn\w*", re.I
)


def says_found(text: str, label: str) -> bool:
    """The model's sentence reports the object as found: starts with yes, or names the object
    without a negation."""
    if re.match(r"\W*(yes|yeah|yep)\b", text, re.I):
        return True
    stem = label.lower().replace("_", " ").rstrip("s")
    return bool(stem) and stem in text.lower() and not _NEG_RE.search(text)


@dataclass
class SkillResult:
    state: str  # succeeded | failed | cancelled
    message: str = ""


class Executive(Protocol):
    """What the loop needs from the skill executive (a ROS bridge in the node, a fake in tests)."""

    def start_skill(self, name: str, args: dict) -> None:
        """Start a motion skill; cancels a running one."""

    def wait_skill(self, timeout_s: float) -> SkillResult | None:
        """The result of the running skill, or None if it is still running after the timeout."""

    def cancel_skill(self, reason: str) -> None: ...

    def emergency(self, kind: str) -> None:
        """Immediate stop/land, never through the model."""

    def memory_query(self, question: str) -> str: ...

    def say(self, text: str) -> None: ...

    def send_photo(self, target: str, caption: str) -> str: ...

    def state_block(self) -> str:
        """Short text: known rooms, objects, people, current skill, battery."""


@dataclass
class LoopConfig:
    max_calls: int = 25
    max_seconds: float = 900.0
    poll_s: float = 0.5
    ack_text: str = "On it."
    max_history_utterances: int = 8
    parallel_workers: int = 4


@dataclass
class MissionResult:
    ended: str = "end_turn"  # end_turn | emergency | cap_calls | cap_time | refusal |
    # max_tokens | backend_error
    calls: int = 0
    tools: list[tuple[str, dict, bool]] = field(default_factory=list)  # name, args, is_error
    usage: dict = field(default_factory=dict)


class AgentLoop:
    def __init__(
        self,
        backend: Backend,
        executive: Executive,
        cfg: LoopConfig | None = None,
        emit: Callable[[str, dict], None] | None = None,
        clock: Callable[[], float] = time.monotonic,
        system: str = SYSTEM_PROMPT,
    ):
        self.backend, self.ex = backend, executive
        self.cfg = cfg or LoopConfig()
        self._emit_cb = emit or (lambda t, d: None)
        self.clock = clock
        self.system = system
        self.tools = tool_schemas()
        self.history: list[dict] = []
        self._inbox: deque[str] = deque()
        self._abort = threading.Event()
        self._lock = threading.Lock()
        self._plan: list[dict] = []
        self._goal = ""
        self._plan_n = 0
        self._spoke = False
        self._n_user = 0
        # evidence of the latest successful search: (object_id, label, photo already sent)
        self._evidence: list | None = None

    # -- input from other threads ------------------------------------------------------------
    def submit(self, text: str) -> bool:
        """Thread-safe. Emergency words act at once (returns True); anything else is queued for
        `process_inbox` or, during a mission, for the running wait to pick up."""
        self._user_event(text)
        if self._emergency(text):
            return True
        self._inbox.append(text)
        return False

    def _emergency(self, text: str) -> bool:
        m = EMERGENCY_RE.search(text)
        if m is None:
            return False
        kind = m.group(1).lower()
        self._abort.set()
        self.ex.emergency(kind)
        self._say("Landing." if kind == "land" else "Stopping.", template=True)
        return True

    def process_inbox(self) -> None:
        """Run queued utterances as missions, oldest first (called by one worker thread)."""
        while self._inbox:
            self.handle(self._inbox.popleft(), echo=False)

    # -- one mission -------------------------------------------------------------------------
    def handle(self, text: str, echo: bool = True) -> MissionResult:
        if echo:
            self._user_event(text)
            if self._emergency(text):
                return MissionResult("emergency")
        self._abort.clear()
        self._say(self.cfg.ack_text, template=True)
        self._trim_history()
        start = len(self.history)
        self.history.append({"role": "user", "content": text})
        self._plan_n += 1
        self._goal, self._plan, self._spoke = text, [], False
        self._evidence = None
        self._publish_plan()
        t0 = self.clock()
        res = self._run(start)
        self._finish_plan(res)
        self._emit(
            "mission_end",  # for logs and the referee; the gateway forwards only known types
            {
                "plan_id": f"plan-{self._plan_n}",
                "ended": res.ended,
                "calls": res.calls,
                "seconds": round(self.clock() - t0, 2),
                "tools": [[n, a, e] for n, a, e in res.tools],
                "usage": res.usage,
            },
        )
        return res

    def _run(self, start: int) -> MissionResult:
        res = MissionResult()
        t0 = self.clock()
        while True:
            if self._abort.is_set():
                res.ended = "emergency"
                return res
            if res.calls >= self.cfg.max_calls or self.clock() - t0 > self.cfg.max_seconds:
                res.ended = "cap_calls" if res.calls >= self.cfg.max_calls else "cap_time"
                self.ex.cancel_skill("mission cap reached")
                self._say(APOLOGY)
                return res
            res.calls += 1
            try:
                sent = self._with_state()
                resp = self.backend.create(self.system, self.tools, sent)
            except BackendError as exc:
                log.warning("backend failed: %s", exc)
                res.ended = "backend_error"
                self._say(LOST_CONNECTION)  # the running skill is left alone
                return res
            for k, v in resp.usage.items():
                if isinstance(v, int | float):
                    res.usage[k] = res.usage.get(k, 0) + v
            if resp.stop_reason in ("refusal", "max_tokens"):
                res.ended = resp.stop_reason
                del self.history[start:]  # drop the half-finished mission, keep the history valid
                self._say(REFUSAL if resp.stop_reason == "refusal" else "That got too long for me.")
                return res
            # Keep exactly what was sent (state block included): Claude binds thinking-block
            # signatures to the conversation prefix, so the prefix must not change afterwards.
            self.history[-1] = sent[-1]
            self.history.append({"role": "assistant", "content": resp.content})
            uses = resp.tool_uses()
            if not uses:
                if not self._spoke and resp.text():
                    self._say(resp.text())
                return res
            blocks = self._execute(uses, res)
            self.history.append({"role": "user", "content": blocks})

    # -- tools -------------------------------------------------------------------------------
    def _execute(self, uses: list[dict], res: MissionResult) -> list[dict]:
        for u in uses:
            self._plan.append(
                {
                    "id": u["id"],
                    "label": step_label(u["name"], u.get("input") or {}),
                    "status": "pending",
                }
            )
        self._publish_plan()
        out: dict[str, tuple[str, bool]] = {}
        parallel = [u for u in uses if not self._is_motion(u["name"])]
        interrupted = False
        with ThreadPoolExecutor(max_workers=self.cfg.parallel_workers) as pool:
            futs = {u["id"]: pool.submit(self._run_tool, u, res) for u in parallel}
            for u in uses:
                if self._is_motion(u["name"]):
                    if interrupted or self._abort.is_set():
                        out[u["id"]] = (
                            "skipped: the operator interrupted or stopped the mission",
                            True,
                        )
                        self._set_step(u["id"], "skipped")
                    else:
                        out[u["id"]] = self._run_tool(u, res)
                        interrupted = out[u["id"]][0].startswith("interrupted")
            for uid, f in futs.items():
                out[uid] = f.result()
        return [
            {
                "type": "tool_result",
                "tool_use_id": u["id"],
                "content": out[u["id"]][0],
                **({"is_error": True} if out[u["id"]][1] else {}),
            }
            for u in uses
        ]

    @staticmethod
    def _is_motion(name: str) -> bool:
        return name in TOOLS and TOOLS[name].motion

    def _run_tool(self, use: dict, res: MissionResult) -> tuple[str, bool]:
        name, uid = use["name"], use["id"]
        try:
            args = validate_call(name, use.get("input"))
        except ToolError as exc:
            res.tools.append((name, use.get("input") or {}, True))
            self._set_step(uid, "failed", str(exc))
            return f"invalid call: {exc}", True
        self._set_step(uid, "running")
        try:
            if name == "say":
                self._say(args["text"])
                text, err = "spoken", False
            elif name == "memory_query":
                text, err = self.ex.memory_query(args["question"]), False
            elif name == "send_photo":
                if self._evidence and args["target"] == self._evidence[0] and self._evidence[2]:
                    text, err = f"photo of {args['target']} was already sent", False
                else:
                    text, err = self.ex.send_photo(args["target"], args["caption"]), False
                    if self._evidence and args["target"] == self._evidence[0]:
                        self._evidence[2] = True
            else:
                text, err = self._run_skill(name, args)
                if name == "search" and not err:
                    m = _OBJECT_ID_RE.search(text)
                    self._evidence = [m.group(1), str(args.get("object", "")), False] if m else None
        except Exception as exc:  # an executive failure must not kill the loop
            log.exception("tool %s failed", name)
            text, err = f"{name} failed: {exc}", True
        res.tools.append((name, args, err))
        self._set_step(uid, "failed" if err else "done", text[:120])
        return text, err

    def _run_skill(self, name: str, args: dict) -> tuple[str, bool]:
        self.ex.start_skill(name, args)
        while True:
            if self._abort.is_set():
                self.ex.cancel_skill("emergency")
                return "cancelled: operator emergency stop", True
            r = self.ex.wait_skill(self.cfg.poll_s)
            if r is not None:
                if r.state == "succeeded":
                    return r.message or "succeeded", False
                return f"{r.state}: {r.message}".rstrip(": "), True
            if self._inbox:
                said = self._inbox.popleft()
                self._say(self.cfg.ack_text, template=True)
                return (
                    f"interrupted: the operator said {said!r}. The skill is still running; "
                    "start another skill to replace it, or carry on.",
                    False,
                )

    # -- helpers -----------------------------------------------------------------------------
    def _with_state(self) -> list[dict]:
        """History plus a fresh state block appended to the last message. `_run` stores that
        message once the model has answered, so earlier prefixes never change."""
        msgs = [dict(m) for m in self.history]
        last = msgs[-1]
        c = last["content"]
        blocks = [{"type": "text", "text": c}] if isinstance(c, str) else list(c)
        blocks.append({"type": "text", "text": f"<state>\n{self.ex.state_block()}\n</state>"})
        last["content"] = blocks
        return msgs

    def _trim_history(self) -> None:
        idx = [
            i
            for i, m in enumerate(self.history)
            if m["role"] == "user"
            and (
                isinstance(m["content"], str)
                or not any(b.get("type") == "tool_result" for b in m["content"])
            )
        ]
        if len(idx) >= self.cfg.max_history_utterances:
            del self.history[: idx[-(self.cfg.max_history_utterances - 1)]]

    def _say(self, text: str, template: bool = False) -> None:
        if not template:
            self._spoke = True
        self.ex.say(text)
        self._emit("transcript", transcript_event("maverick", text))
        if not template:
            self._attach_evidence(text)

    def _attach_evidence(self, text: str) -> None:
        """The report says the object was found and the search left a photo of it: send it with
        the report unless the model already did. The model decides what is reported; it does not
        have to remember the photo."""
        ev = self._evidence
        if not ev or ev[2] or not says_found(text, ev[1]):
            return
        ev[2] = True
        sid = f"auto-photo-{self._plan_n}"
        self._plan.append({"id": sid, "label": step_label("send_photo", {}), "status": "running"})
        self._publish_plan()
        try:
            note = self.ex.send_photo(ev[0], f"The {ev[1].replace('_', ' ')} I found")
            self._set_step(sid, "done", note[:120])
        except Exception as exc:  # no photo saved: the report stands without it
            log.warning("evidence photo failed: %s", exc)
            self._set_step(sid, "failed", str(exc)[:120])

    def _user_event(self, text: str) -> None:
        with self._lock:
            self._n_user += 1
            n = self._n_user
        self._emit("transcript", transcript_event("user", text, "text", f"user-{n}"))

    def _emit(self, typ: str, data: dict) -> None:
        try:
            self._emit_cb(typ, data)
        except Exception:
            log.exception("event sink failed")

    def _set_step(self, uid: str, status: str, detail: str | None = None) -> None:
        with self._lock:
            for s in self._plan:
                if s["id"] == uid:
                    s["status"] = status
                    if detail:
                        s["detail"] = detail
        self._publish_plan()

    def _publish_plan(self) -> None:
        with self._lock:
            steps = [dict(s) for s in self._plan]
        self._emit("agent_step", agent_step_event(f"plan-{self._plan_n}", self._goal, steps))

    def _finish_plan(self, res: MissionResult) -> None:
        if res.ended != "end_turn":
            with self._lock:
                for s in self._plan:
                    if s["status"] in ("pending", "running"):
                        s["status"] = "skipped"
        self._publish_plan()

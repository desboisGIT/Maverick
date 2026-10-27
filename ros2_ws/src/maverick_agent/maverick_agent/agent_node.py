"""ROS 2 node `agent_node`: operator text -> agent loop -> skill requests.

Topics
  in  /maverick/operator/text   std_msgs/String JSON {"text": "...", "speaker"?: "..."}
                                (a plain, non-JSON string is taken as the text)
  in  /maverick/skill/progress  std_msgs/String JSON (skill_node; a terminal state ends the tool)
  in  /maverick/memory/answer   std_msgs/String JSON (replies to memory_query)
  in  /maverick/memory/rooms, /maverick/memory/objects  (state block)
  out /maverick/skill/request   std_msgs/String JSON (maverick_agent.tools.skill_request)
  out /maverick/memory/query    std_msgs/String JSON
  out /maverick/agent/say       std_msgs/String JSON {"text": ...} (TTS / console)
  out /maverick/agent/event     std_msgs/String JSON {"type": "transcript"|"agent_step",
                                "t": ..., "data": {...}} (gateway envelope; the gateway forwards)

Also in  /maverick/camera/rgb/compressed (latest frame, for send_photo "current_view")
Logs (JSON lines, for the referee and for debugging): agent.event_log (every event, photo bytes
replaced by their length; includes `mission_end` with calls, seconds, tools, usage) and
agent.calls_log (one line per model call: latency, tokens, estimated EUR, running total).

Parameters: agent.backend (deepseek|claude|scripted, default deepseek; falls back to scripted
with a warning when the backend's API key is not in the environment), agent.model (empty:
backend default), agent.effort, agent.max_tokens, agent.cassette_path, agent.cap_eur (stop calling
the model after this estimated spend, 0 = no cap), agent.tts (none), agent.event_log,
agent.calls_log.
"""

from __future__ import annotations

import json
import os
import queue
import sys
import threading
import time

import rclpy
from maverick_mapping import topics as T
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from .backends import KEY_ENV, AgentConfig, make_backend
from .loop import AgentLoop, SkillResult
from .memory_bridge import format_answer, question_to_query, skill_result_text
from .metering import MeteredBackend
from .photos import photo_attachment, read_photo, strip_attachment_data
from .tools import PHOTO_CURRENT_VIEW, skill_request
from .tts import make_tts

OPERATOR_TEXT = "/maverick/operator/text"
AGENT_SAY = "/maverick/agent/say"
AGENT_EVENT = "/maverick/agent/event"

RELIABLE = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
LATCHED = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
)
TERMINAL = ("succeeded", "failed", "cancelled")


class RosExecutive:
    """Executive over ROS topics. Callbacks run on the spin thread; the loop thread waits."""

    def __init__(self, node: AgentNode):
        self.n = node
        self._result: queue.Queue[SkillResult] = queue.Queue()
        self._active = False
        self._ignore_cancel = False
        self._rid: str | None = None  # id of the request in flight (the executive echoes it)
        self._rn = 0
        self._answers: dict[str, dict] = {}
        self._answer_ev = threading.Event()
        self._qn = 0
        self.rooms: list = []
        self.objects: list = []
        self.progress: dict = {}
        self.photos: dict[str, dict] = {}  # object_id -> {path, label}, from search results
        self.last_frame: bytes | None = None

    def start_skill(self, name: str, args: dict) -> None:
        while not self._result.empty():
            self._result.get_nowait()
        self._ignore_cancel = self._active  # skill_node reports the replaced skill as cancelled
        self._active = True
        self._rn += 1
        self._rid = f"agent-{self._rn}"
        req = {**skill_request(name, args), "id": self._rid}
        if name == "return_home":
            req["land"] = True  # the tool's contract: fly back to the take-off point and land
        self.n.skill_pub.publish(String(data=json.dumps(req)))

    def wait_skill(self, timeout_s: float) -> SkillResult | None:
        try:
            return self._result.get(timeout=timeout_s)
        except queue.Empty:
            return None

    def cancel_skill(self, reason: str) -> None:
        if self._active:
            self.n.skill_pub.publish(String(data=json.dumps({"skill": "cancel"})))

    def emergency(self, kind: str) -> None:
        # stop and land both go through the executive as a cancel; PX4 holds position (CLAUDE.md)
        self._active = False
        self.n.skill_pub.publish(String(data=json.dumps({"skill": "cancel"})))
        self.n.get_logger().warn(f"emergency keyword: {kind}")

    def on_progress(self, d: dict) -> None:
        self.progress = d
        state = d.get("state")
        if not self._active or state not in TERMINAL:
            return
        rid = d.get("request_id")
        if rid is not None and self._rid is not None and rid != self._rid:
            return  # a result of an earlier request
        if state == "cancelled" and self._ignore_cancel:
            self._ignore_cancel = False
            return
        res = d.get("result")
        if isinstance(res, dict) and res.get("object_id") and res.get("photo"):
            self.photos[str(res["object_id"])] = {
                "path": str(res["photo"]),
                "label": str(res.get("object", "")),
            }
        self._active = False
        self._result.put(SkillResult(state, skill_result_text(d)))

    def memory_query(self, question: str) -> str:
        self._qn += 1
        qid = f"agent-q{self._qn}"
        q = question_to_query(question, qid)
        if q is None:
            return "I can only look up rooms by type and objects by name."
        self._answer_ev.clear()
        self.n.query_pub.publish(String(data=json.dumps(q)))
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            if qid in self._answers:
                return format_answer(self._answers.pop(qid))
            self._answer_ev.wait(0.1)
        return "memory did not answer in time"

    def on_answer(self, d: dict) -> None:
        if d.get("id"):
            self._answers[str(d["id"])] = d
            self._answer_ev.set()

    def say(self, text: str) -> None:
        self.n.say_pub.publish(String(data=json.dumps({"text": text})))
        try:
            self.n.tts.speak(text)
        except Exception:
            self.n.get_logger().warn("tts failed", throttle_duration_sec=5.0)

    def send_photo(self, target: str, caption: str) -> str:
        """A transcript message with the photo attached (crop memory saved, or the current view)."""
        if target == PHOTO_CURRENT_VIEW:
            if self.last_frame is None:
                raise RuntimeError("no camera frame yet")
            att = photo_attachment(self.last_frame, caption)
        else:
            ref = self.photos.get(target)
            jpeg = read_photo(ref["path"]) if ref else None
            if ref is None or jpeg is None:
                raise RuntimeError(
                    f"no photo for {target}: use the object_id from a successful search result"
                )
            att = photo_attachment(jpeg, caption, target, ref["label"])
        self.n.event(
            "transcript",
            {
                "id": f"photo-{time.time_ns()}",
                "role": "maverick",
                "text": caption,
                "attachments": [att],
            },
        )
        return f"photo of {target} sent"

    def state_block(self) -> str:
        rooms = ", ".join(f"{r.get('type')}({r.get('id')})" for r in self.rooms) or "none yet"
        objs = (
            ", ".join(f"{o.get('label')}@{o.get('room_id') or '?'}" for o in self.objects[:20])
            or "none yet"
        )
        p = self.progress
        cur = f"{p.get('skill')} {p.get('state')}: {p.get('message', '')}" if p else "idle"
        return f"rooms: {rooms}\nobjects seen: {objs}\ncurrent skill: {cur}"


class AgentNode(Node):
    def __init__(self) -> None:
        super().__init__("agent_node")
        p = self.declare_parameter
        cfg = AgentConfig(
            backend=str(p("agent.backend", "deepseek").value),
            model=str(p("agent.model", "").value),
            effort=str(p("agent.effort", "low").value),
            max_tokens=int(p("agent.max_tokens", 16000).value),
            cassette_path=str(p("agent.cassette_path", "").value),
        )
        cap_eur = float(p("agent.cap_eur", 0.0).value)
        self.event_log = str(p("agent.event_log", "/tmp/agent_events.jsonl").value)
        calls_log = str(p("agent.calls_log", "/tmp/agent_calls.jsonl").value)
        self.tts = make_tts(str(p("agent.tts", "none").value))
        key_env = KEY_ENV.get(cfg.backend)
        if key_env and not os.environ.get(key_env):
            self.get_logger().warning(
                f"{key_env} not set: agent.backend {cfg.backend} unavailable, using scripted"
            )
            cfg.backend = "scripted"
        self.skill_pub = self.create_publisher(String, T.SKILL_REQUEST, RELIABLE)
        self.query_pub = self.create_publisher(String, T.MEM_QUERY, RELIABLE)
        self.say_pub = self.create_publisher(String, AGENT_SAY, RELIABLE)
        self.event_pub = self.create_publisher(String, AGENT_EVENT, RELIABLE)
        self.ex = RosExecutive(self)
        self.backend = MeteredBackend(make_backend(cfg), cfg.backend, cap_eur, calls_log)
        self.loop = AgentLoop(self.backend, self.ex, emit=self.event)
        self.create_subscription(String, OPERATOR_TEXT, self._on_text, RELIABLE)
        self.create_subscription(String, T.SKILL_PROGRESS, self._on_progress, LATCHED)
        self.create_subscription(String, T.MEM_ANSWER, self._on_answer, RELIABLE)
        self.create_subscription(String, T.MEM_ROOMS, lambda m: self._set("rooms", m), LATCHED)
        self.create_subscription(String, T.MEM_OBJECTS, lambda m: self._set("objects", m), LATCHED)
        self.create_subscription(
            CompressedImage, T.CAM_RGB, self._on_frame, qos_profile_sensor_data
        )
        self._wake = threading.Event()
        threading.Thread(target=self._worker, daemon=True).start()
        self.get_logger().info(f"agent_node up (backend {cfg.backend})")

    def event(self, typ: str, data: dict) -> None:
        env = {"type": typ, "t": time.time(), "data": data}
        self.event_pub.publish(String(data=json.dumps(env)))
        try:
            logged = strip_attachment_data(data) if typ == "transcript" else data
            with open(self.event_log, "a") as f:
                f.write(json.dumps({"type": typ, "t": env["t"], "data": logged}) + "\n")
        except OSError:
            pass

    def _on_frame(self, msg: CompressedImage) -> None:
        self.ex.last_frame = bytes(msg.data)

    def _set(self, attr: str, msg: String) -> None:
        try:
            setattr(self.ex, attr, json.loads(msg.data))
        except ValueError:
            pass

    def _on_progress(self, msg: String) -> None:
        try:
            self.ex.on_progress(json.loads(msg.data))
        except ValueError:
            pass

    def _on_answer(self, msg: String) -> None:
        try:
            self.ex.on_answer(json.loads(msg.data))
        except ValueError:
            pass

    def _on_text(self, msg: String) -> None:
        try:
            text = str(json.loads(msg.data).get("text", ""))
        except (ValueError, AttributeError):
            text = msg.data
        if text.strip():
            if not self.loop.submit(text.strip()):
                self._wake.set()

    def _worker(self) -> None:
        while True:
            self._wake.wait()
            self._wake.clear()
            try:
                self.loop.process_inbox()
            except Exception:
                self.get_logger().error("agent loop crashed", throttle_duration_sec=1.0)


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = AgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

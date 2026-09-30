import threading
import time

from fakes import FakeExecutive, ListBackend, resp, tu
from maverick_agent.backends import ScriptedBackend
from maverick_agent.backends.base import BackendError
from maverick_agent.loop import (
    APOLOGY,
    LOST_CONNECTION,
    REFUSAL,
    AgentLoop,
    LoopConfig,
    SkillResult,
)


def make(backend, ex=None, **cfg):
    ex = ex or FakeExecutive()
    events = []
    loop = AgentLoop(backend, ex, LoopConfig(**cfg), emit=lambda t, d: events.append((t, d)))
    return loop, ex, events


def test_h2_flow_scripted():
    loop, ex, events = make(ScriptedBackend())
    r = loop.handle("Check the kitchen for bananas and come back and tell us")
    assert r.ended == "end_turn" and r.calls == 5
    assert [t[0] for t in r.tools] == ["goto_room", "search", "come_to", "say"]
    assert not any(t[2] for t in r.tools)
    starts = [c[1:] for c in ex.calls if c[0] == "start"]
    assert starts[0] == ("goto_room", {"room": "kitchen"})
    assert starts[1][0] == "search" and starts[1][1]["object"] == "bananas"
    assert starts[2] == ("come_to", {"person": "operator"})
    assert ex.spoken[0] == "On it." and ex.spoken[-1].startswith("I checked the kitchen")
    assert {"transcript", "agent_step"} <= {t for t, _ in events}
    last_plan = [d for t, d in events if t == "agent_step"][-1]
    assert [s["status"] for s in last_plan["steps"]] == ["done"] * 4


def test_ack_is_first_and_not_from_model():
    loop, ex, _ = make(ListBackend([resp([], "end_turn")]))
    loop.handle("hello")
    assert ex.spoken == ["On it."]


def test_state_block_last_and_prefix_stable():
    be = ListBackend([resp([tu("say", {"text": "hi"})]), resp([], "end_turn")])
    loop, ex, _ = make(be)
    loop.handle("hello")
    (s1, t1, m1), (s2, t2, m2) = be.requests
    assert s1 == s2 and t1 == t2
    assert m1[-1]["content"][-1]["text"].startswith("<state>")
    assert m2[-1]["content"][0]["type"] == "tool_result"
    assert m2[-1]["content"][-1]["text"].startswith("<state>")
    assert all("<state>" not in str(m) for m in loop.history)  # never stored


def test_invalid_call_is_error_result_and_no_skill_started():
    be = ListBackend([resp([tu("goto_room", {"room": "garage"})]), resp([], "end_turn")])
    loop, ex, _ = make(be)
    r = loop.handle("go to the garage")
    res = be.requests[1][2][-1]["content"][0]
    assert res["is_error"] is True and "one of" in res["content"]
    assert not [c for c in ex.calls if c[0] == "start"]
    assert r.tools[0][2] is True


def test_failed_skill_result_is_error():
    ex = FakeExecutive({"goto_room": SkillResult("failed", "no kitchen known yet")})
    be = ListBackend([resp([tu("goto_room", {"room": "kitchen"})]), resp([], "end_turn")])
    loop, _, _ = make(be, ex)
    loop.handle("go")
    res = be.requests[1][2][-1]["content"][0]
    assert res["is_error"] and "no kitchen known yet" in res["content"]


def test_parallel_non_motion_one_message_and_really_parallel():
    ex = FakeExecutive()
    ex.mem_delay = 0.2
    blocks = [
        tu("memory_query", {"question": "any kitchen?"}, "a"),
        tu("memory_query", {"question": "any bedroom?"}, "b"),
        tu("say", {"text": "checking"}, "c"),
    ]
    be = ListBackend([resp(blocks), resp([], "end_turn")])
    loop, _, _ = make(be, ex)
    t = time.monotonic()
    loop.handle("what do you know")
    assert time.monotonic() - t < 0.35  # the two 0.2 s queries overlapped
    last = be.requests[1][2][-1]
    assert last["role"] == "user"
    ids = [b["tool_use_id"] for b in last["content"] if b["type"] == "tool_result"]
    assert ids == ["a", "b", "c"]


def test_motion_serialised_in_order():
    blocks = [
        tu("goto_room", {"room": "kitchen"}, "a"),
        tu("come_to", {"person": "operator"}, "b"),
        tu("say", {"text": "x"}, "c"),
    ]
    be = ListBackend([resp(blocks), resp([], "end_turn")])
    loop, ex, _ = make(be)
    loop.handle("go")
    assert [c[1] for c in ex.calls if c[0] == "start"] == ["goto_room", "come_to"]
    blocks = be.requests[1][2][-1]["content"]
    assert [b["type"] for b in blocks].count("tool_result") == 3


def test_call_cap_ends_with_apology():
    be = ListBackend([resp([tu("memory_query", {"question": "q"}, f"t{i}")]) for i in range(40)])
    loop, ex, _ = make(be)
    r = loop.handle("loop forever")
    assert r.ended == "cap_calls" and r.calls == 25
    assert ex.spoken[-1] == APOLOGY and ("cancel", "mission cap reached") in ex.calls


def test_time_cap_ends_with_apology():
    now = [0.0]

    class B:
        def create(self, s, t, m):
            now[0] += 400
            return resp([tu("memory_query", {"question": "q"}, f"t{now[0]}")])

    ex = FakeExecutive()
    r = AgentLoop(B(), ex, clock=lambda: now[0]).handle("x")
    assert r.ended == "cap_time" and ex.spoken[-1] == APOLOGY and r.calls == 3


def test_emergency_bypasses_model():
    be = ListBackend([])
    loop, ex, _ = make(be)
    for word, say in (
        ("Stop!", "Stopping."),
        ("please land now", "Landing."),
        ("abort", "Stopping."),
    ):
        ex.calls.clear()
        r = loop.handle(word)
        assert r.ended == "emergency" and ex.calls[0][0] == "emergency"
        assert ex.spoken[-1] == say
    assert be.requests == []


def test_emergency_during_running_skill():
    ex = FakeExecutive(hang=True)
    be = ListBackend([resp([tu("explore", {"budget_s": 60})]), resp([], "end_turn")])
    loop, _, _ = make(be, ex)
    out = []
    th = threading.Thread(target=lambda: out.append(loop.handle("explore")))
    th.start()
    time.sleep(0.1)
    assert loop.submit("stop") is True
    th.join(2)
    assert out[0].ended == "emergency"
    assert ("emergency", "stop") in ex.calls and ("cancel", "emergency") in ex.calls


def test_new_utterance_during_skill_interrupts_wait():
    ex = FakeExecutive(hang=True)
    be = ListBackend([resp([tu("explore", {"budget_s": 60})]), resp([], "end_turn")])
    loop, _, _ = make(be, ex)
    th = threading.Thread(target=lambda: loop.handle("explore"))
    th.start()
    time.sleep(0.1)
    assert loop.submit("actually check the bedroom") is False
    th.join(2)
    res = be.requests[1][2][-1]["content"][0]
    assert "interrupted" in res["content"] and "bedroom" in res["content"]
    assert "is_error" not in res


def test_refusal():
    loop, ex, _ = make(ListBackend([resp([], "refusal")]))
    r = loop.handle("do something bad")
    assert r.ended == "refusal" and ex.spoken[-1] == REFUSAL
    assert loop.history == []  # nothing dangling


def test_max_tokens():
    loop, ex, _ = make(ListBackend([resp([tu("say", {"text": "x"})], "max_tokens")]))
    r = loop.handle("hi")
    assert r.ended == "max_tokens" and loop.history == []
    assert not [c for c in ex.calls if c[0] == "start"]


def test_backend_error_reports_and_leaves_flight():
    class B:
        def create(self, *a):
            raise BackendError("down")

    loop, ex, _ = make(B())
    r = loop.handle("hi")
    assert r.ended == "backend_error" and ex.spoken[-1] == LOST_CONNECTION
    assert not [c for c in ex.calls if c[0] in ("cancel", "emergency")]


def test_assistant_content_appended_unchanged_and_usage_summed():
    think = {"type": "thinking", "thinking": "hm", "signature": "sig"}
    be = ListBackend(
        [
            resp([think, tu("say", {"text": "hi"})], usage={"cache_read_input_tokens": 5}),
            resp([], "end_turn", {"cache_read_input_tokens": 7}),
        ]
    )
    loop, _, _ = make(be)
    r = loop.handle("hello")
    assert loop.history[1]["content"][0] is think
    assert r.usage["cache_read_input_tokens"] == 12


def test_final_text_spoken_when_model_never_used_say():
    loop, ex, _ = make(ListBackend([resp([{"type": "text", "text": "All done."}], "end_turn")]))
    loop.handle("hi")
    assert ex.spoken[-1] == "All done."


def test_history_trimmed_at_utterance_boundaries():
    be = ListBackend([resp([], "end_turn") for _ in range(10)])
    loop, _, _ = make(be, max_history_utterances=3)
    for i in range(5):
        loop.handle(f"u{i}")
    assert isinstance(loop.history[0]["content"], str)
    assert sum(1 for m in loop.history if isinstance(m["content"], str)) <= 3

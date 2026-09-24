import json
import types

import pytest
from fakes import FakeExecutive
from maverick_agent.backends import (
    AgentConfig,
    CassetteBackend,
    Rule,
    ScriptedBackend,
    make_backend,
)
from maverick_agent.backends.base import BackendError
from maverick_agent.backends.claude import ClaudeBackend
from maverick_agent.loop import AgentLoop


class RateLimitError(Exception):
    pass


class APIConnectionError(Exception):
    pass


class APIStatusError(Exception):
    def __init__(self, code):
        self.status_code = code


FAKE_SDK = types.SimpleNamespace(
    RateLimitError=RateLimitError,
    APIConnectionError=APIConnectionError,
    APIStatusError=APIStatusError,
    Anthropic=lambda: None,
)


class Block:
    def __init__(self, d):
        self.d = d

    def model_dump(self, exclude_none=False):
        return dict(self.d)


class FakeClient:
    def __init__(self, outcomes):
        self.outcomes = list(outcomes)
        self.kwargs = []
        self.messages = self

    def create(self, **kw):
        self.kwargs.append(kw)
        o = self.outcomes.pop(0)
        if isinstance(o, Exception):
            raise o
        return o


def raw(content, stop="tool_use", cache=0):
    u = types.SimpleNamespace(
        input_tokens=10, output_tokens=5, cache_read_input_tokens=cache,
        cache_creation_input_tokens=None,
    )  # fmt: skip
    return types.SimpleNamespace(content=[Block(b) for b in content], stop_reason=stop, usage=u)


def backend(outcomes, **kw):
    c = FakeClient(outcomes)
    return ClaudeBackend(client=c, sdk=FAKE_SDK, sleep=lambda s: None, **kw), c


def test_request_shape():
    be, c = backend([raw([{"type": "text", "text": "hi"}], "end_turn")])
    tools = [{"name": "say"}]
    r = be.create("SYS", tools, [{"role": "user", "content": "hi"}])
    kw = c.kwargs[0]
    assert kw["model"] == "claude-opus-5-5" and kw["max_tokens"] == 16000
    assert kw["system"] == [{"type": "text", "text": "SYS", "cache_control": {"type": "ephemeral"}}]
    assert kw["output_config"] == {"effort": "low"}
    assert kw["tools"] == tools and kw["messages"][0]["content"] == "hi"
    assert not {"thinking", "temperature", "tool_choice"} & set(kw)
    assert r.stop_reason == "end_turn" and r.text() == "hi"


def test_usage_cache_read_logged(caplog):
    be, _ = backend([raw([], "end_turn", cache=1234)])
    with caplog.at_level("INFO", logger="maverick_agent.claude"):
        r = be.create("s", [], [])
    assert r.usage["cache_read_input_tokens"] == 1234
    assert "cache_read=1234" in caplog.text


def test_thinking_block_preserved():
    th = {"type": "thinking", "thinking": "x", "signature": "s"}
    be, _ = backend([raw([th, {"type": "tool_use", "id": "1", "name": "say", "input": {}}])])
    assert be.create("s", [], []).content[0] == th


def test_retries_then_succeeds():
    outcomes = [RateLimitError(), APIStatusError(503), APIConnectionError(), raw([], "end_turn")]
    be, c = backend(outcomes)
    assert be.create("s", [], []).stop_reason == "end_turn" and len(c.kwargs) == 4


def test_gives_up_after_retries_as_backend_error():
    be, c = backend([APIConnectionError()] * 4)
    with pytest.raises(BackendError, match="lost connection"):
        be.create("s", [], [])
    assert len(c.kwargs) == 4


def test_client_error_not_retried():
    be, c = backend([APIStatusError(400)])
    with pytest.raises(BackendError, match="rejected"):
        be.create("s", [], [])
    assert len(c.kwargs) == 1


def test_refusal_stop_reason_passed_through():
    be, _ = backend([raw([], "refusal")])
    assert be.create("s", [], []).stop_reason == "refusal"


def test_cassette_round_trip(tmp_path):
    path = tmp_path / "c.jsonl"
    script = [
        raw([{"type": "tool_use", "id": "t1", "name": "say", "input": {"text": "hello"}}]),
        raw([{"type": "text", "text": "done"}], "end_turn"),
    ]
    be, _ = backend(script, cassette_path=str(path))
    ex1 = FakeExecutive()
    AgentLoop(be, ex1).handle("hi")
    lines = [json.loads(x) for x in path.read_text().splitlines()]
    assert len(lines) == 2 and lines[0]["request"]["model"] == "claude-opus-5-5"

    ex2 = FakeExecutive()
    replay = CassetteBackend(path)
    r = AgentLoop(replay, ex2).handle("hi")
    assert ex2.spoken == ex1.spoken == ["On it.", "hello"] and r.calls == 2
    with pytest.raises(BackendError):
        replay.create("", [], [])


def test_make_backend(tmp_path):
    assert isinstance(make_backend(AgentConfig()), ScriptedBackend)
    with pytest.raises(ValueError):
        make_backend(AgentConfig(backend="local"))


def test_scripted_parallel_step_final_text_and_no_match():
    sb = ScriptedBackend(
        [Rule("hi", [[("say", {"text": "a"}), ("memory_query", {"question": "q"})]], "bye")]
    )
    r = sb.create("", [], [{"role": "user", "content": "hi"}])
    assert [b["name"] for b in r.tool_uses()] == ["say", "memory_query"]
    results = [{"type": "tool_result", "tool_use_id": "x", "content": "ok"}]
    msgs = [
        {"role": "user", "content": "hi"},
        {"role": "assistant", "content": r.content},
        {"role": "user", "content": results},
    ]
    assert sb.create("", [], msgs).text() == "bye"
    assert sb.create("", [], [{"role": "user", "content": "zzz"}]).content == []

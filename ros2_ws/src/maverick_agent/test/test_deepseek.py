import json
import types

import pytest
from maverick_agent.backends import AgentConfig, make_backend
from maverick_agent.backends.base import BackendError
from maverick_agent.backends.deepseek import DeepSeekBackend, to_openai_messages


class RateLimitError(Exception):
    pass


class APIConnectionError(Exception):
    pass


class APIStatusError(Exception):
    def __init__(self, code):
        self.status_code = code


SDK = types.SimpleNamespace(
    RateLimitError=RateLimitError,
    APIConnectionError=APIConnectionError,
    APIStatusError=APIStatusError,
)
NS = types.SimpleNamespace


class FakeClient:
    def __init__(self, outcomes):
        self.outcomes = list(outcomes)
        self.kwargs = []
        self.chat = NS(completions=self)

    def create(self, **kw):
        self.kwargs.append(kw)
        o = self.outcomes.pop(0)
        if isinstance(o, Exception):
            raise o
        return o


def raw(content=None, calls=(), finish="stop", reasoning=None):
    tcs = [
        NS(
            id=i,
            type="function",
            function=NS(name=n, arguments=a if isinstance(a, str) else json.dumps(a)),
        )
        for i, n, a in calls
    ]
    msg = NS(content=content, tool_calls=tcs or None, reasoning_content=reasoning)
    usage = NS(
        prompt_tokens=100,
        completion_tokens=7,
        prompt_cache_hit_tokens=60,
        prompt_cache_miss_tokens=40,
    )
    return NS(choices=[NS(message=msg, finish_reason=finish)], usage=usage)


def backend(outcomes, **kw):
    c = FakeClient(outcomes)
    return DeepSeekBackend(client=c, sdk=SDK, sleep=lambda s: None, **kw), c


TOOLS = [
    {
        "name": "say",
        "description": "d",
        "strict": True,
        "input_schema": {"type": "object", "properties": {"text": {"type": "string"}}},
    }
]


def test_request_shape():
    be, c = backend([raw("hi")])
    r = be.create("SYS", TOOLS, [{"role": "user", "content": "yo"}])
    kw = c.kwargs[0]
    assert kw["model"] == "deepseek-flash"
    assert kw["messages"] == [
        {"role": "system", "content": "SYS"},
        {"role": "user", "content": "yo"},
    ]
    assert kw["tools"][0] == {
        "type": "function",
        "function": {"name": "say", "description": "d", "parameters": TOOLS[0]["input_schema"]},
    }
    assert kw["extra_body"] == {"thinking": {"type": "disabled"}}
    assert r.text() == "hi" and r.stop_reason == "end_turn"
    assert r.usage["input_tokens"] == 40 and r.usage["cache_read_input_tokens"] == 60
    assert r.usage["output_tokens"] == 7


def test_parallel_tool_calls_to_blocks():
    calls = [("c1", "say", {"text": "a"}), ("c2", "memory_query", {"question": "q"})]
    be, _ = backend([raw(None, calls, "tool_calls")])
    r = be.create("s", TOOLS, [])
    assert r.stop_reason == "tool_use"
    assert [(b["id"], b["name"], b["input"]) for b in r.tool_uses()] == [
        ("c1", "say", {"text": "a"}),
        ("c2", "memory_query", {"question": "q"}),
    ]


def test_bad_json_arguments_do_not_crash():
    be, _ = backend([raw(None, [("c1", "say", "{oops")], "tool_calls")])
    assert "_unparsable_arguments" in be.create("s", TOOLS, []).tool_uses()[0]["input"]


def test_history_translation():
    msgs = [
        {"role": "user", "content": "go"},
        {
            "role": "assistant",
            "content": [
                {"type": "text", "text": "ok"},
                {"type": "tool_use", "id": "c1", "name": "say", "input": {"text": "a"}},
                {"type": "tool_use", "id": "c2", "name": "explore", "input": {"budget_s": 60}},
            ],
        },
        {
            "role": "user",
            "content": [
                {"type": "tool_result", "tool_use_id": "c1", "content": "spoken"},
                {"type": "tool_result", "tool_use_id": "c2", "content": "failed", "is_error": True},
                {"type": "text", "text": "<state>\nrooms\n</state>"},
            ],
        },
    ]
    o = to_openai_messages("S", msgs)
    assert [m["role"] for m in o] == ["system", "user", "assistant", "tool", "tool", "user"]
    a = o[2]
    assert a["content"] == "ok" and [t["id"] for t in a["tool_calls"]] == ["c1", "c2"]
    assert json.loads(a["tool_calls"][1]["function"]["arguments"]) == {"budget_s": 60}
    assert o[3] == {"role": "tool", "tool_call_id": "c1", "content": "spoken"}
    assert o[4]["content"].startswith("ERROR: ")
    assert o[5]["content"].startswith("<state>")


def test_reasoning_round_trip():
    be, _ = backend(
        [raw(None, [("c1", "say", {"text": "a"})], "tool_calls", reasoning="hmm")], thinking=True
    )
    r = be.create("s", TOOLS, [])
    assert r.content[0] == {"type": "thinking", "thinking": "hmm"}
    o = to_openai_messages("s", [{"role": "assistant", "content": r.content}])
    assert o[1]["reasoning_content"] == "hmm"


def test_retries_then_succeeds():
    be, c = backend([RateLimitError(), APIStatusError(503), APIConnectionError(), raw("ok")])
    assert be.create("s", [], []).text() == "ok" and len(c.kwargs) == 4


def test_gives_up_as_backend_error():
    be, c = backend([APIConnectionError()] * 4)
    with pytest.raises(BackendError, match="lost connection"):
        be.create("s", [], [])
    assert len(c.kwargs) == 4


def test_client_error_not_retried():
    be, c = backend([APIStatusError(400)])
    with pytest.raises(BackendError, match="rejected"):
        be.create("s", [], [])
    assert len(c.kwargs) == 1


def test_missing_key(monkeypatch):
    monkeypatch.delenv("DEEPSEEK_API_KEY", raising=False)
    with pytest.raises(BackendError, match="not set"):
        DeepSeekBackend(sdk=SDK)


def test_make_backend_deepseek(monkeypatch):
    pytest.importorskip("openai")
    monkeypatch.setenv("DEEPSEEK_API_KEY", "x")
    assert isinstance(make_backend(AgentConfig(backend="deepseek")), DeepSeekBackend)

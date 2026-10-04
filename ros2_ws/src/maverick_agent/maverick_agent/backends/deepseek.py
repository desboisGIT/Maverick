"""DeepSeek backend: OpenAI-compatible chat completions with tool calling.

Facts from api-docs.deepseek.com (checked 2026-11): base URL https://api.deepseek.com, models
`deepseek-flash` / `deepseek-v4-pro`; thinking is ON by default and is turned off with
`extra_body={"thinking": {"type": "disabled"}}`. With thinking on and tools present, every
`reasoning_content` must be passed back, so it is kept as a "thinking" block in the history.
Tool `arguments` is a JSON string. Usage has prompt_cache_hit_tokens / prompt_cache_miss_tokens.

Our history is Anthropic-shaped (text / tool_use / tool_result blocks); this module translates it
to OpenAI chat messages and back, keeping tool_use ids.
"""

from __future__ import annotations

import json
import logging
import os
import time
from pathlib import Path

from .base import BackendError, Response

log = logging.getLogger("maverick_agent.deepseek")

BASE_URL = "https://api.deepseek.com"
FINISH_TO_STOP = {
    "stop": "end_turn",
    "tool_calls": "tool_use",
    "length": "max_tokens",
    "content_filter": "refusal",
}


def to_openai_tools(tools: list[dict]) -> list[dict]:
    return [
        {
            "type": "function",
            "function": {
                "name": t["name"],
                "description": t.get("description", ""),
                "parameters": t.get("input_schema", {"type": "object", "properties": {}}),
            },
        }
        for t in tools
    ]


def to_openai_messages(system: str, messages: list[dict]) -> list[dict]:
    out: list[dict] = [{"role": "system", "content": system}]
    for m in messages:
        c = m["content"]
        if m["role"] == "user":
            if isinstance(c, str):
                out.append({"role": "user", "content": c})
                continue
            for b in c:  # tool results first: they must follow the assistant tool_calls
                if b.get("type") == "tool_result":
                    content = b.get("content", "")
                    if isinstance(content, list):
                        content = " ".join(x.get("text", "") for x in content)
                    if b.get("is_error"):
                        content = f"ERROR: {content}"
                    out.append(
                        {"role": "tool", "tool_call_id": b["tool_use_id"], "content": content}
                    )
            text = "\n".join(b["text"] for b in c if b.get("type") == "text")
            if text:
                out.append({"role": "user", "content": text})
        else:
            if isinstance(c, str):
                out.append({"role": "assistant", "content": c})
                continue
            msg: dict = {
                "role": "assistant",
                "content": "\n".join(b["text"] for b in c if b.get("type") == "text") or None,
            }
            think = "".join(b.get("thinking", "") for b in c if b.get("type") == "thinking")
            if think:
                msg["reasoning_content"] = think
            calls = [
                {
                    "id": b["id"],
                    "type": "function",
                    "function": {"name": b["name"], "arguments": json.dumps(b.get("input") or {})},
                }
                for b in c
                if b.get("type") == "tool_use"
            ]
            if calls:
                msg["tool_calls"] = calls
            out.append(msg)
    return out


def from_openai_message(msg) -> list[dict]:
    """An OpenAI assistant message (SDK object) -> normalised content blocks."""
    blocks: list[dict] = []
    reasoning = getattr(msg, "reasoning_content", None)
    if reasoning:
        blocks.append({"type": "thinking", "thinking": reasoning})
    if getattr(msg, "content", None):
        blocks.append({"type": "text", "text": msg.content})
    for tc in getattr(msg, "tool_calls", None) or []:
        try:
            args = json.loads(tc.function.arguments or "{}")
        except json.JSONDecodeError:
            args = {"_unparsable_arguments": str(tc.function.arguments)[:200]}
        blocks.append({"type": "tool_use", "id": tc.id, "name": tc.function.name, "input": args})
    return blocks


class DeepSeekBackend:
    def __init__(
        self,
        model: str = "deepseek-flash",
        thinking: bool = False,
        max_tokens: int = 2048,
        retries: int = 3,
        backoff_s: float = 1.0,
        cassette_path: str | None = None,
        base_url: str = BASE_URL,
        client=None,
        sdk=None,
        sleep=time.sleep,
    ):
        """`client`/`sdk` are injectable for tests; `openai` is imported only when needed."""
        self.model, self.thinking, self.max_tokens = model, thinking, max_tokens
        self.retries, self.backoff_s = retries, backoff_s
        self.cassette_path = cassette_path
        self._sleep = sleep
        if client is None:
            key = os.environ.get("DEEPSEEK_API_KEY")
            if not key:
                raise BackendError("DEEPSEEK_API_KEY is not set")
            if sdk is None:
                import openai as sdk
            client = sdk.OpenAI(api_key=key, base_url=base_url)
        if sdk is None:
            import openai as sdk
        self._sdk = sdk
        self.client = client

    def request(self, system: str, tools: list[dict], messages: list[dict]) -> dict:
        return {
            "model": self.model,
            "max_tokens": self.max_tokens,
            "messages": to_openai_messages(system, messages),
            "tools": to_openai_tools(tools),
            "tool_choice": "auto",
            "extra_body": {"thinking": {"type": "enabled" if self.thinking else "disabled"}},
        }

    def _retryable(self, exc: Exception) -> bool:
        sdk = self._sdk
        if isinstance(exc, sdk.RateLimitError | sdk.APIConnectionError):
            return True
        return isinstance(exc, sdk.APIStatusError) and getattr(exc, "status_code", 0) >= 500

    def create(self, system: str, tools: list[dict], messages: list[dict]) -> Response:
        req = self.request(system, tools, messages)
        sdk = self._sdk
        last: Exception | None = None
        for attempt in range(self.retries + 1):
            try:
                raw = self.client.chat.completions.create(**req)
                break
            except (sdk.RateLimitError, sdk.APIStatusError, sdk.APIConnectionError) as exc:
                last = exc
                if not self._retryable(exc):
                    raise BackendError(f"request rejected: {exc}") from exc
                log.warning("model call failed (%s), attempt %d", type(exc).__name__, attempt + 1)
                if attempt < self.retries:
                    self._sleep(self.backoff_s * 2**attempt)
        else:
            raise BackendError(f"lost connection: {last}") from last

        choice = raw.choices[0]
        u = raw.usage
        hit = getattr(u, "prompt_cache_hit_tokens", None) or 0
        prompt = getattr(u, "prompt_tokens", 0) or 0
        usage = {
            "input_tokens": getattr(u, "prompt_cache_miss_tokens", None) or max(prompt - hit, 0),
            "output_tokens": getattr(u, "completion_tokens", 0) or 0,
            "cache_read_input_tokens": hit,
            "cache_creation_input_tokens": 0,
        }
        log.info(
            "usage in=%d out=%d cache_read=%d stop=%s",
            usage["input_tokens"],
            usage["output_tokens"],
            hit,
            choice.finish_reason,
        )
        content = from_openai_message(choice.message)
        fr = choice.finish_reason or "stop"
        stop = FINISH_TO_STOP.get(fr, fr)
        if any(b["type"] == "tool_use" for b in content) and stop == "end_turn":
            stop = "tool_use"
        resp = Response(content, stop, usage)
        if self.cassette_path:
            self._record(req, resp)
        return resp

    def _record(self, req: dict, resp: Response) -> None:
        line = {
            "request": req,
            "response": {
                "content": resp.content,
                "stop_reason": resp.stop_reason,
                "usage": resp.usage,
            },
        }
        p = Path(self.cassette_path)
        p.parent.mkdir(parents=True, exist_ok=True)
        with p.open("a") as f:
            f.write(json.dumps(line, default=str) + "\n")

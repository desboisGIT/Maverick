"""Claude backend (Anthropic Python SDK, `client.messages.create`).

Rules for claude-opus-5-5 (docs/s4-plan.md §2): thinking is always on, so `thinking` and
`temperature` are never sent; tool_choice stays auto; the full `response.content` (thinking blocks
included) goes back into the history unchanged. The system prompt carries the cache breakpoint
(render order is tools, then system), so tools + system form the cached prefix.
"""

from __future__ import annotations

import json
import logging
import time
from pathlib import Path

from .base import BackendError, Response

log = logging.getLogger("maverick_agent.claude")


class ClaudeBackend:
    def __init__(
        self,
        model: str = "claude-opus-5-5",
        effort: str = "low",
        max_tokens: int = 16000,
        retries: int = 3,
        backoff_s: float = 1.0,
        cassette_path: str | None = None,
        client=None,
        sdk=None,
        sleep=time.sleep,
    ):
        """`client`/`sdk` are injectable for tests; `anthropic` is imported only when needed."""
        self.model, self.effort, self.max_tokens = model, effort, max_tokens
        self.retries, self.backoff_s = retries, backoff_s
        self.cassette_path = cassette_path
        self._sleep = sleep
        if sdk is None:
            import anthropic as sdk
        self._sdk = sdk
        self.client = client if client is not None else sdk.Anthropic()

    def request(self, system: str, tools: list[dict], messages: list[dict]) -> dict:
        return {
            "model": self.model,
            "max_tokens": self.max_tokens,
            "system": [{"type": "text", "text": system, "cache_control": {"type": "ephemeral"}}],
            "tools": tools,
            "messages": messages,
            "output_config": {"effort": self.effort},
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
                raw = self.client.messages.create(**req)
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

        content = [
            b if isinstance(b, dict) else b.model_dump(exclude_none=True) for b in raw.content
        ]
        u = raw.usage
        usage = {
            k: getattr(u, k, None) or 0
            for k in (
                "input_tokens",
                "output_tokens",
                "cache_read_input_tokens",
                "cache_creation_input_tokens",
            )
        }
        log.info(
            "usage in=%d out=%d cache_read=%d cache_write=%d stop=%s",
            usage["input_tokens"],
            usage["output_tokens"],
            usage["cache_read_input_tokens"],
            usage["cache_creation_input_tokens"],
            raw.stop_reason,
        )
        resp = Response(content, raw.stop_reason or "end_turn", usage)
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

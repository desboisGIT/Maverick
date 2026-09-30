"""Backend interface: one call in, one normalised response out."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol


class BackendError(RuntimeError):
    """The model could not be reached (after retries). The loop reports it; flight is untouched."""


@dataclass
class Response:
    """`content` is a list of block dicts: {"type": "text"|"tool_use"|"thinking"|...}.

    Blocks go back into the history unchanged (thinking blocks included).
    """

    content: list[dict]
    stop_reason: str = "end_turn"  # end_turn | tool_use | refusal | max_tokens | ...
    usage: dict = field(default_factory=dict)

    def tool_uses(self) -> list[dict]:
        return [b for b in self.content if b.get("type") == "tool_use"]

    def text(self) -> str:
        return " ".join(b.get("text", "") for b in self.content if b.get("type") == "text").strip()


class Backend(Protocol):
    def create(self, system: str, tools: list[dict], messages: list[dict]) -> Response: ...

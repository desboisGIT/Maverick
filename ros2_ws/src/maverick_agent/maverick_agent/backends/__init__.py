"""Model backends, picked by config `agent.backend` (`scripted` | `claude`)."""

from __future__ import annotations

from dataclasses import dataclass

from .base import Backend, BackendError, Response
from .scripted import CassetteBackend, Rule, ScriptedBackend

__all__ = [
    "AgentConfig",
    "Backend",
    "BackendError",
    "CassetteBackend",
    "Response",
    "Rule",
    "ScriptedBackend",
    "make_backend",
]


@dataclass
class AgentConfig:
    backend: str = "scripted"  # scripted | claude  (no local backend yet)
    model: str = "claude-opus-5-5"
    effort: str = "low"
    max_tokens: int = 16000
    cassette_path: str = ""  # claude: record here; scripted: replay from here if set


def make_backend(cfg: AgentConfig) -> Backend:
    if cfg.backend == "scripted":
        return CassetteBackend(cfg.cassette_path) if cfg.cassette_path else ScriptedBackend()
    if cfg.backend == "claude":
        from .claude import ClaudeBackend

        return ClaudeBackend(
            model=cfg.model,
            effort=cfg.effort,
            max_tokens=cfg.max_tokens,
            cassette_path=cfg.cassette_path or None,
        )
    raise ValueError(f"unknown agent.backend {cfg.backend!r} (scripted | claude)")

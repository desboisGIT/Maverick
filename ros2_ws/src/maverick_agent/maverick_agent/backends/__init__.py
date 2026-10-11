"""Model backends, picked by config `agent.backend` (`scripted` | `deepseek` | `claude`).

The ROS node defaults to `deepseek` (owner decision, docs/s4-plan.md §2); the pure config keeps
`scripted` so tests and CI never need a key or the network.
"""

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
    "DEFAULT_MODELS",
    "KEY_ENV",
    "make_backend",
]


@dataclass
class AgentConfig:
    backend: str = "scripted"  # scripted | deepseek | claude  (no local backend yet)
    model: str = ""  # empty: the backend's default (DEFAULT_MODELS)
    effort: str = "low"
    max_tokens: int = 16000
    cassette_path: str = ""  # claude: record here; scripted: replay from here if set


DEFAULT_MODELS = {"claude": "claude-opus-5-5", "deepseek": "deepseek-flash"}
KEY_ENV = {"claude": "ANTHROPIC_API_KEY", "deepseek": "DEEPSEEK_API_KEY"}


def make_backend(cfg: AgentConfig) -> Backend:
    if cfg.backend == "scripted":
        return CassetteBackend(cfg.cassette_path) if cfg.cassette_path else ScriptedBackend()
    if cfg.backend == "claude":
        from .claude import ClaudeBackend

        return ClaudeBackend(
            model=cfg.model or DEFAULT_MODELS["claude"],
            effort=cfg.effort,
            max_tokens=cfg.max_tokens,
            cassette_path=cfg.cassette_path or None,
        )
    if cfg.backend == "deepseek":
        from .deepseek import DeepSeekBackend

        return DeepSeekBackend(
            model=cfg.model if cfg.model.startswith("deepseek") else DEFAULT_MODELS["deepseek"],
            max_tokens=min(cfg.max_tokens, 4096),
            cassette_path=cfg.cassette_path or None,
        )
    raise ValueError(f"unknown agent.backend {cfg.backend!r} (scripted | claude | deepseek)")

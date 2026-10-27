"""Backends without a network: pattern rules -> tool-call sequences, and cassette replay."""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path

from .base import BackendError, Response


@dataclass
class Rule:
    """`pattern` is a regex searched in the utterance (case-insensitive).

    `steps`: a list; each item is a tool call `(name, args)` or a list of them (issued in one
    response, i.e. parallel). String args are `str.format`ed with the regex groups: `{1}`, `{2}`.
    After the last step the backend ends the turn with `final_text` (if any).
    """

    pattern: str
    steps: list
    final_text: str = ""


def _calls(step):
    return step if isinstance(step, list) else [step]


def _result_texts(messages: list[dict], after: int) -> list[str]:
    """Text of every tool_result in the messages after index `after` (this mission's results)."""
    out = []
    for m in messages[after + 1 :]:
        if m["role"] == "user" and not isinstance(m["content"], str):
            for b in m["content"]:
                if b.get("type") == "tool_result":
                    c = b.get("content", "")
                    out.append(c if isinstance(c, str) else json.dumps(c))
    return out


def h2_report(results: list[str]):
    """Step after search + come_to: the spoken answer (and the photo when found)."""
    found = bool(results) and results[0].lower().startswith("found")
    if not found:
        ok = bool(results) and results[0].lower().startswith("not found")
        text = "No, I looked and there are none." if ok else "Sorry, I could not finish checking."
        return ("say", {"text": text})
    m = re.search(r"object_id (\S+?)[,)]", results[0])
    calls = [("say", {"text": "Yes, I found them. Here is a photo."})]
    if m:
        calls.append(("send_photo", {"target": m.group(1), "caption": "found it"}))
    return calls


def _fill(v, groups: tuple[str, ...]):
    if isinstance(v, str):
        try:
            return v.format("", *groups)
        except (IndexError, KeyError):
            return v
    return v


DEFAULT_RULES = [
    Rule(
        r"check (?:in |on )?(?:the )?(\w+?) (?:if|whether) there (?:are|is) (?:any |some |a |an )?"
        r"(\w+?)s?\b",
        [
            ("search", {"object": "{2}", "region": "{1}", "budget_s": 120}),
            ("come_to", {"person": "operator"}),
            h2_report,
        ],
    ),
    Rule(
        r"check (?:the )?(\w+?) for (?:some |a |an )?(\w+).*(?:come back|return)",
        [
            ("goto_room", {"room": "{1}"}),
            ("search", {"object": "{2}", "region": "{1}", "budget_s": 120}),
            ("come_to", {"person": "operator"}),
            ("say", {"text": "I checked the {1} for {2}."}),
        ],
    ),
    Rule(r"(?:go|fly) to (?:the )?(\w+)", [("goto_room", {"room": "{1}"})]),
    Rule(r"explore|look around|map", [("explore", {"budget_s": 120})]),
    Rule(r"come (?:here|to me)", [("come_to", {"person": "operator"})]),
    Rule(r"(?:go|come) home|return", [("return_home", {})]),
    Rule(r".*", [("say", {"text": "I did not understand that."})]),
]


class ScriptedBackend:
    """Stateless: the position in the script is derived from the message history."""

    def __init__(self, rules: list[Rule] | None = None):
        self.rules = rules if rules is not None else DEFAULT_RULES

    @staticmethod
    def _utterance(messages: list[dict]) -> tuple[str, int, int]:
        """The latest user text, how many assistant turns followed it, and its index."""
        for i in range(len(messages) - 1, -1, -1):
            m = messages[i]
            if m["role"] != "user":
                continue
            c = m["content"]
            if isinstance(c, str):
                return c, sum(1 for x in messages[i + 1 :] if x["role"] == "assistant"), i
            if not any(b.get("type") == "tool_result" for b in c):
                text = " ".join(
                    b["text"]
                    for b in c
                    if b.get("type") == "text" and not b["text"].startswith("<state>")
                )
                return text, sum(1 for x in messages[i + 1 :] if x["role"] == "assistant"), i
        return "", 0, -1

    def create(self, system: str, tools: list[dict], messages: list[dict]) -> Response:
        text, turn, at = self._utterance(messages)
        for rule in self.rules:
            m = re.search(rule.pattern, text, re.IGNORECASE)
            if m is None:
                continue
            if turn >= len(rule.steps):
                content = [{"type": "text", "text": rule.final_text}] if rule.final_text else []
                return Response(content, "end_turn")
            blocks = []
            step = rule.steps[turn]
            if callable(step):
                step = step(_result_texts(messages, at))
            for j, (name, args) in enumerate(_calls(step)):
                blocks.append(
                    {
                        "type": "tool_use",
                        "id": f"toolu_script_{turn}_{j}",
                        "name": name,
                        "input": {k: _fill(v, m.groups()) for k, v in args.items()},
                    }
                )
            return Response(blocks, "tool_use")
        return Response([], "end_turn")


class CassetteBackend:
    """Replays the `response` of each JSONL line in order (written by ClaudeBackend)."""

    def __init__(self, path: str | Path):
        self.entries = [
            json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()
        ]
        self.i = 0

    def create(self, system: str, tools: list[dict], messages: list[dict]) -> Response:
        if self.i >= len(self.entries):
            raise BackendError("cassette exhausted")
        r = self.entries[self.i]["response"]
        self.i += 1
        return Response(r["content"], r.get("stop_reason", "end_turn"), r.get("usage", {}))

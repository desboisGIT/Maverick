"""Text to speech behind one interface (S4c). Default: the null backend (nothing is played).

TODO(S4d): a Kokoro CPU backend (kokoro-onnx, about 80 MB model, no VRAM) that renders `say` text
to audio for the console and the simulated drone speaker. Not added in S4c: the dependency is not
in the image and the H2 text scenario needs no sound. Add it as another class here and a branch in
`make_tts`; the node already calls `tts.speak(text)` for every spoken line.
"""

from __future__ import annotations

from typing import Protocol


class TTS(Protocol):
    def speak(self, text: str) -> None: ...


class NullTTS:
    def speak(self, text: str) -> None:
        return None


def make_tts(name: str) -> TTS:
    if name in ("", "none", "null"):
        return NullTTS()
    raise ValueError(f"unknown agent.tts {name!r} (only 'none'; Kokoro is a TODO, see tts.py)")

"""S4c pieces: photo attachments, result text, metering, scripted H2 flow, mission_end event."""

import json

from fakes import FakeExecutive
from maverick_agent.backends import ScriptedBackend
from maverick_agent.backends.base import BackendError, Response
from maverick_agent.loop import AgentLoop, LoopConfig, SkillResult
from maverick_agent.memory_bridge import skill_result_text
from maverick_agent.metering import MeteredBackend
from maverick_agent.photos import photo_attachment, read_photo, strip_attachment_data
from maverick_agent.tts import NullTTS, make_tts

H2 = (
    "Maverick, we're sitting down. Can you go check in the kitchen if there are bananas for us, "
    "and come back and tell us?"
)
JPEG = b"\xff\xd8\xff\xe0" + b"x" * 50


def test_photo_attachment_and_log_strip():
    att = photo_attachment(JPEG, "found", "obj_3", "banana")
    assert att["kind"] == "photo" and att["object_id"] == "obj_3" and att["label"] == "banana"
    d = strip_attachment_data({"id": "x", "attachments": [att]})
    assert "data" not in d["attachments"][0] and d["attachments"][0]["data_len"] > 0
    assert "data" in att  # the original is untouched


def test_read_photo_rejects_bad_files(tmp_path):
    good, bad, big = tmp_path / "a.jpg", tmp_path / "b.jpg", tmp_path / "c.jpg"
    good.write_bytes(JPEG)
    bad.write_bytes(b"not a jpeg")
    big.write_bytes(b"\xff\xd8" + b"0" * 500_000)
    assert read_photo(good) == JPEG
    assert read_photo(bad) is None and read_photo(big) is None
    assert read_photo(tmp_path / "missing.jpg") is None


def test_skill_result_text_names_object_id():
    res = {"found": True, "object_id": "obj_7", "photo": "/tmp/x.jpg"}
    p = {"message": "found the banana in the kitchen", "result": res}
    assert skill_result_text(p).endswith("(object_id obj_7)")
    res["photo"] = None
    assert "no photo" in skill_result_text(p)
    assert skill_result_text({"message": "not found, searched kitchen"}) == (
        "not found, searched kitchen"
    )


def test_null_tts():
    assert isinstance(make_tts("none"), NullTTS)
    try:
        make_tts("kokoro")
    except ValueError:
        pass
    else:
        raise AssertionError("kokoro is a TODO")


class Inner:
    def __init__(self, fail=False):
        self.fail = fail

    def create(self, system, tools, messages):
        if self.fail:
            raise BackendError("boom")
        return Response([], "end_turn", {"input_tokens": 1000, "output_tokens": 100})


def test_metered_backend_logs_cost_and_caps(tmp_path):
    log = tmp_path / "calls.jsonl"
    m = MeteredBackend(Inner(), "deepseek", cap_eur=0.0005, log_path=str(log))
    m.create("", [], [])
    row = json.loads(log.read_text().splitlines()[0])
    assert row["usage"]["input_tokens"] == 1000 and row["cost_eur"] > 0
    assert row["cum_cost_eur"] == round(m.cost, 6)
    m.create("", [], [])  # 0.00042 so far: still under the cap
    try:
        m.create("", [], [])
    except BackendError as exc:
        assert "cap" in str(exc)
    else:
        raise AssertionError("cap not enforced")


def test_metered_backend_error_is_logged(tmp_path):
    log = tmp_path / "c.jsonl"
    m = MeteredBackend(Inner(fail=True), "deepseek", log_path=str(log))
    try:
        m.create("", [], [])
    except BackendError:
        pass
    assert "boom" in log.read_text()


def run_h2(search_msg: str):
    ex = FakeExecutive({"search": SkillResult("succeeded", search_msg)})
    events = []
    loop = AgentLoop(
        ScriptedBackend(), ex, LoopConfig(poll_s=0.01), emit=lambda t, d: events.append((t, d))
    )
    res = loop.handle(H2)
    return res, ex, events


def test_scripted_h2_present_sends_photo_after_coming_back():
    res, ex, events = run_h2("found the banana in the kitchen (object_id obj_9)")
    assert [t[0] for t in res.tools] == ["search", "come_to", "say", "send_photo"]
    starts = [c[1:] for c in ex.calls if c[0] == "start"]
    assert starts[0][1]["object"] == "banana" and starts[0][1]["region"] == "kitchen"
    assert starts[1] == ("come_to", {"person": "operator"})
    assert [c[1] for c in ex.calls if c[0] == "photo"] == ["obj_9"]
    assert ex.spoken[-1].startswith("Yes")


def test_scripted_h2_absent_says_no_and_sends_no_photo():
    res, ex, _ = run_h2("not found, searched kitchen")
    assert [t[0] for t in res.tools] == ["search", "come_to", "say"]
    assert ex.spoken[-1].startswith("No")


def test_mission_end_event_is_emitted_last():
    res, _, events = run_h2("not found, searched kitchen")
    assert events[-1][0] == "mission_end"
    d = events[-1][1]
    assert d["ended"] == "end_turn" and d["calls"] == res.calls and d["plan_id"] == "plan-1"

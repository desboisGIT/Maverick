import json

import pytest
from fakes import ListBackend, resp, tu
from maverick_agent import eval_commands as ev
from maverick_agent.backends.base import BackendError, Response


def test_twenty_scenarios_unique():
    assert len(ev.SCENARIOS) == 20 and len({s.id for s in ev.SCENARIOS}) == 20


def test_cost_estimate():
    u = {"input_tokens": 1_000_000, "output_tokens": 100_000, "cache_read_input_tokens": 0}
    assert abs(ev.cost_of("claude", u) - 6.0) < 1e-9
    assert ev.cost_of("scripted", u) == 0


def test_correct_run_scores_ok():
    sc = next(s for s in ev.SCENARIOS if s.id == "goto_known")
    script = [
        resp([tu("goto_room", {"room": "bedroom"})]),
        resp([tu("say", {"text": "I am in the bedroom."}, "t2")]),
        resp([{"type": "text", "text": "done"}], "end_turn"),
    ]
    row = ev.run_scenario(sc, ev.Metered(ListBackend(script), "scripted", 1.0))
    assert row["correct"], row["fails"]


def test_invalid_call_counted():
    sc = next(s for s in ev.SCENARIOS if s.id == "goto_known")
    script = [
        resp([tu("goto_room", {"room": "attic"})]),
        resp([{"type": "text", "text": "sorry"}], "end_turn"),
    ]
    row = ev.run_scenario(sc, ev.Metered(ListBackend(script), "scripted", 1.0))
    assert not row["correct"] and row["invalid_calls"] == 1


def test_cap_stops_backend():
    class B:
        def create(self, *a):
            return Response([], "end_turn", {"output_tokens": 1_000_000})

    m = ev.Metered(B(), "claude", 1.0)
    m.create("", [], [])
    with pytest.raises(BackendError, match="cap"):
        m.create("", [], [])


def test_main_scripted(tmp_path):
    assert ev.main(["--backend", "scripted", "--out", str(tmp_path)]) == 0
    s = json.loads((tmp_path / "summary.json").read_text())
    assert s["scenarios"] == 20 and s["cost_eur_est"] == 0

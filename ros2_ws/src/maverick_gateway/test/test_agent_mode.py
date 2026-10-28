"""Gateway agent mode (pure helpers): which agent events reach the console, operator text JSON."""

import json

from maverick_gateway.protocol import agent_event_frame, operator_text_payload


def test_agent_event_frame_forwards_only_console_types():
    data = {"id": "a", "role": "maverick", "text": "hi"}
    tr = json.dumps({"type": "transcript", "t": 5.5, "data": data})
    assert agent_event_frame(tr) == ("transcript", data, 5.5)
    assert agent_event_frame(json.dumps({"type": "mission_end", "t": 1, "data": {}})) is None
    assert agent_event_frame("not json") is None
    assert agent_event_frame(json.dumps({"type": "agent_step", "t": 1, "data": []})) is None


def test_operator_text_payload():
    assert json.loads(operator_text_payload("go", "me")) == {"text": "go", "speaker": "me"}
    assert json.loads(operator_text_payload("go")) == {"text": "go"}

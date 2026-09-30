import json

import pytest
from maverick_agent.tools import (
    ROOM_TYPES,
    TOOLS,
    ToolError,
    skill_request,
    tool_schemas,
    validate_call,
)


def test_every_schema_is_strict_and_closed():
    schemas = tool_schemas()
    assert {s["name"] for s in schemas} == {
        "explore", "goto_room", "search", "come_to", "return_home",
        "memory_query", "say", "send_photo",
    }  # fmt: skip
    for s in schemas:
        assert s["strict"] is True
        sch = s["input_schema"]
        assert sch["type"] == "object" and sch["additionalProperties"] is False
        assert set(sch["required"]) <= set(sch["properties"])
        json.dumps(s)
        for p in sch["properties"].values():
            assert not {"minimum", "maximum"} & set(p)  # strict schemas reject bounds


def test_no_tool_takes_coordinates():
    bad = {"x", "y", "z", "xy", "position", "pose", "coords", "coordinates", "yaw", "velocity"}
    for s in tool_schemas():
        assert not bad & set(s["input_schema"]["properties"])


def test_room_enum_matches_memory_vocabulary():
    memory = pytest.importorskip("maverick_memory.room_typing")
    enum = TOOLS["goto_room"].to_api()["input_schema"]["properties"]["room"]["enum"]
    assert tuple(enum) == tuple(memory.TYPES) == tuple(ROOM_TYPES)


def test_region_is_optional():
    sch = TOOLS["search"].to_api()["input_schema"]
    assert "region" in sch["properties"] and "region" not in sch["required"]


def test_valid_calls_normalise():
    assert validate_call("goto_room", {"room": " Kitchen "}) == {"room": "kitchen"}
    assert validate_call("search", {"object": "banana", "budget_s": 60})["budget_s"] == 60
    assert validate_call("return_home", {}) == {}
    assert validate_call("send_photo", {"target": "current_view", "caption": "hi"})


@pytest.mark.parametrize(
    "name,args,frag",
    [
        ("goto_room", {"room": "garage"}, "one of"),
        ("goto_room", {}, "missing"),
        ("goto_room", {"room": "kitchen", "x": 1}, "unexpected"),
        ("explore", {"budget_s": 5}, "at least"),
        ("explore", {"budget_s": 99999}, "at most"),
        ("explore", {"budget_s": "60"}, "number"),
        ("explore", {"budget_s": True}, "number"),
        ("say", {"text": ""}, "empty"),
        ("say", {"text": "x" * 500}, "longer"),
        ("search", {"object": "cup", "budget_s": 60, "region": "attic"}, "one of"),
        ("send_photo", {"target": "a b;c", "caption": "x"}, "object id"),
        ("fly_to", {"x": 1}, "unknown tool"),
        ("say", "hello", "object"),
    ],
)
def test_rejections(name, args, frag):
    with pytest.raises(ToolError, match=frag):
        validate_call(name, args)


def test_skill_requests():
    assert skill_request("goto_room", {"room": "kitchen"}) == {
        "skill": "goto_room",
        "room_type": "kitchen",
    }
    assert skill_request("return_home", {}) == {"skill": "return"}
    req = skill_request("search", {"object": "cup", "budget_s": 30, "region": "kitchen"})
    assert req["region"] == "kitchen"
    with pytest.raises(ToolError):
        skill_request("say", {"text": "x"})

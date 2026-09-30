from maverick_agent.memory_bridge import format_answer, question_to_query


def test_room_question():
    q = question_to_query("Do we know where the kitchen is?", "q1")
    assert q == {"query": "goal_for_type", "room_type": "kitchen", "id": "q1"}


def test_find_question():
    q = question_to_query("Where is the banana?", "q2")
    assert q["query"] == "find" and q["label"] == "banana"


def test_unmappable():
    assert question_to_query("how are you", "q3") is None


def test_format_answers():
    assert "no kitchen" in format_answer({"ok": False, "reason": "no kitchen known yet"})
    out = format_answer({"ok": True, "query": "find", "objects": [{"id": "o1", "room_id": "r2"}]})
    assert "o1" in out and "r2" in out

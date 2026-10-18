import threading
import time

import pytest
from referee import stackcfg
from referee.failsafe import summarise_cases
from referee.parallel import run_queue
from referee.stack import rtf_summary


def test_stack_zero_is_the_original_layout():
    env = stackcfg.stack_env(0)
    assert env["COMPOSE_PROJECT_NAME"] == "maverick"
    assert (env["MAVERICK_DRONE_NET"], env["MAVERICK_WIFI_NET"], env["MAVERICK_WORLD_NET"]) == (
        "10.231.1",
        "10.231.2",
        "10.231.3",
    )
    assert env["MAVERICK_GATEWAY_PORT"] == "8765"
    assert env["MAVERICK_XRCE_AGENT_IP_INT"] == "182911235"  # 10.231.1.3, the old constant


def test_stack_n_addresses():
    env = stackcfg.stack_env(2)
    assert env["COMPOSE_PROJECT_NAME"] == "maverick-s2"
    assert env["MAVERICK_DRONE_NET"] == "10.231.21"
    assert env["MAVERICK_WIFI_NET"] == "10.231.22"
    assert env["MAVERICK_WORLD_NET"] == "10.231.23"
    assert env["MAVERICK_GATEWAY_PORT"] == "8767"
    assert env["MAVERICK_XRCE_AGENT_IP_INT"] == str((10 << 24) + (231 << 16) + (21 << 8) + 3)


def test_stacks_never_overlap():
    seen = set()
    for n in range(stackcfg.MAX_STACK + 1):
        for prefix in stackcfg.prefixes(n).values():
            assert prefix not in seen
            seen.add(prefix)
    assert len(seen) == 3 * (stackcfg.MAX_STACK + 1)
    with pytest.raises(ValueError):
        stackcfg.prefixes(stackcfg.MAX_STACK + 1)
    with pytest.raises(ValueError):
        stackcfg.prefixes(-1)


def test_parse_stacks():
    assert stackcfg.parse_stacks(1, None) == [0]
    assert stackcfg.parse_stacks(3, None) == [1, 2, 3]
    assert stackcfg.parse_stacks(1, "2") == [2]
    assert stackcfg.parse_stacks(3, "1,2,3") == [1, 2, 3]
    with pytest.raises(ValueError):
        stackcfg.parse_stacks(2, "1,2,3")
    with pytest.raises(ValueError):
        stackcfg.parse_stacks(2, "1,1")
    with pytest.raises(ValueError):
        stackcfg.parse_stacks(0, None)


def test_run_queue_distributes_and_keeps_order():
    seen: dict[int, list[int]] = {}
    lock = threading.Lock()

    def make_worker(sid):
        def work(i, item):
            time.sleep(0.01 * (1 + item % 3))
            with lock:
                seen.setdefault(sid, []).append(i)
            return {"item": item}

        return work

    got = []
    res = run_queue(list(range(12)), [1, 2, 3], make_worker, lambda i, s, r: got.append(i))
    assert [r["item"] for r in res] == list(range(12))
    assert sorted(i for v in seen.values() for i in v) == list(range(12))  # each item once
    assert len(seen) == 3
    assert sorted(got) == list(range(12))
    assert all(r["stack"] in (1, 2, 3) for r in res)


def test_run_queue_single_stack_is_sequential():
    order = []

    def make_worker(sid):
        def work(i, x):
            order.append(x)
            return {"x": x}

        return work

    res = run_queue(["a", "b", "c"], [0], make_worker)
    assert order == ["a", "b", "c"]
    assert [r["stack"] for r in res] == [0, 0, 0]


def test_run_queue_worker_error_stops_others_and_reraises():
    def make_worker(sid):
        def work(i, item):
            if item == 1:
                raise RuntimeError("boom")
            time.sleep(0.05)
            return {}

        return work

    with pytest.raises(RuntimeError, match="boom"):
        run_queue(list(range(50)), [1, 2], make_worker)


def test_run_queue_stop_event():
    stop = threading.Event()

    def make_worker(sid):
        def work(i, item):
            stop.set()
            return {}

        return work

    res = run_queue(list(range(10)), [1], make_worker, stop=stop)
    assert res[0] is not None and all(r is None for r in res[1:])


def test_summarise_cases_merges_stacks():
    rows = [
        {"case": "F1", "verdict": "pass", "stack": 1},
        {"case": "F1", "verdict": "fail", "stack": 2},
        {"case": "F3", "verdict": "error", "stack": 3},
    ]
    s = summarise_cases(rows, ["F1", "F3"])
    assert s["F1"] == {"runs": 2, "pass": 1, "fail": 1, "error": 0}
    assert s["F3"] == {"runs": 1, "pass": 0, "fail": 0, "error": 1}


def test_rtf_summary():
    assert rtf_summary([]) is None
    s = rtf_summary([{"rtf": 1.0}, {"rtf": 0.9}, {"t": 1}, {"rtf": 1.1}])
    assert s["min"] == 0.9 and s["n"] == 3 and s["mean"] == 1.0

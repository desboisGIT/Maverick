"""Run a list of work items over several stacks (pure Python, stdlib only).

Each stack runs one item at a time, pulled from a shared queue, so a slow run on one stack does
not hold up the others. `run_queue` returns the results in item order.
"""

from __future__ import annotations

import queue
import threading
from collections.abc import Callable, Sequence
from typing import Any


def run_queue(
    items: Sequence[Any],
    stack_ids: Sequence[int],
    make_worker: Callable[[int], Callable[[int, Any], dict]],
    on_result: Callable[[int, int, dict], None] | None = None,
    stop: threading.Event | None = None,
) -> list[dict | None]:
    """Distribute `items` over `stack_ids`.

    make_worker(stack_id) runs in that stack's thread and returns worker(index, item) -> dict
    (it must catch its own run errors). The dict gets a `stack` field. `on_result(index,
    stack_id, rec)` is called under a lock (write results.jsonl, print). Items not run because
    `stop` was set stay None. A worker that raises ends only its own thread: it sets `stop`, the
    others wind down, and the error is re-raised at the end.
    """
    stop = stop or threading.Event()
    todo: queue.Queue[tuple[int, Any]] = queue.Queue()
    for pair in enumerate(items):
        todo.put(pair)
    results: list[dict | None] = [None] * len(items)
    lock = threading.Lock()
    errors: list[BaseException] = []

    def loop(sid: int) -> None:
        try:
            worker = make_worker(sid)
            while not stop.is_set():
                try:
                    i, item = todo.get_nowait()
                except queue.Empty:
                    return
                rec = dict(worker(i, item))
                rec["stack"] = sid
                with lock:
                    results[i] = rec
                    if on_result:
                        on_result(i, sid, rec)
        except BaseException as exc:  # noqa: BLE001
            with lock:
                errors.append(exc)
            stop.set()

    threads = [threading.Thread(target=loop, args=(s,), daemon=True) for s in stack_ids]
    for t in threads:
        t.start()
    try:
        while any(t.is_alive() for t in threads):
            for t in threads:
                t.join(0.5)
    except BaseException:  # Ctrl-C: tell the workers to stop pulling items, then propagate
        stop.set()
        raise
    if errors:
        raise errors[0]
    return results

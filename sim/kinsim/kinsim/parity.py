"""Pin the ROS nodes' glue that kinsim re-implements.

kinsim runs the real pure modules, but the nodes' own scheduling code (skill_node's `_tick`,
`_guard_tick`, `_emit`, ..., mapping_node's `_integrate`, memory_node's `_process`) cannot be
imported without ROS, so `stack.py` / `memory.py` repeat it. This module hashes the AST of those
methods; `tests/test_parity.py` fails when one of them changes, which means: read the change, copy
it into kinsim, then run `python3 -m kinsim.parity --update`.
"""

from __future__ import annotations

import ast
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PINNED = Path(__file__).resolve().parents[1] / "data" / "parity.json"
SRC = ROOT / "ros2_ws" / "src"

WATCH = {
    "maverick_skills/maverick_skills/skill_node.py": {
        "SkillNode": [
            "_on_pose",
            "_on_rooms",
            "_on_objects",
            "_on_request",
            "_tick",
            "_guard_tick",
            "_emit",
            "_on_status",
        ],
    },
    "maverick_mapping/maverick_mapping/mapping_node.py": {
        "MappingNode": ["_integrate", "_publish_clearance", "_publish_grid"],
    },
    "maverick_memory/maverick_memory/memory_node.py": {
        "MemoryNode": ["_process", "_update_rooms", "_on_query"],
    },
    "maverick_gateway/launch/s3_host.launch.py": {},
    "maverick_flight/maverick_flight/flight_node.py": {
        "FlightClient": ["_on_goal", "_observe", "_tick"],
    },
}


def _digest(node: ast.AST) -> str:
    return hashlib.sha256(ast.dump(node, annotate_fields=True).encode()).hexdigest()[:16]


def current() -> dict[str, str]:
    out: dict[str, str] = {}
    for rel, classes in WATCH.items():
        tree = ast.parse((SRC / rel).read_text())
        if not classes:  # whole file (launch file parameters: stride, altitude, ...)
            out[rel] = _digest(tree)
            continue
        for cls, methods in classes.items():
            node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls)
            for fn in node.body:
                if isinstance(fn, ast.FunctionDef) and fn.name in methods:
                    out[f"{rel}::{cls}.{fn.name}"] = _digest(fn)
    return out


def diff() -> list[str]:
    pinned = json.loads(PINNED.read_text()) if PINNED.exists() else {}
    cur = current()
    bad = [k for k in sorted(set(pinned) | set(cur)) if pinned.get(k) != cur.get(k)]
    return bad


def main(argv: list[str] | None = None) -> int:
    if "--update" in (argv if argv is not None else sys.argv[1:]):
        PINNED.write_text(json.dumps(current(), indent=1, sort_keys=True) + "\n")
        print(f"pinned {len(current())} entries in {PINNED}")
        return 0
    bad = diff()
    print("\n".join(bad) if bad else "in sync")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())

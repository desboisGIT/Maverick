"""Addresses of parallel stacks (the one place that derives them; pure Python, stdlib only).

Stack 0 is today's stack: project `maverick`, subnets 10.231.1.0/24 (drone), 10.231.2.0/24
(wifi), 10.231.3.0/24 (world), gateway port 8765. Stack N >= 1 uses project `maverick-s<N>`,
subnets 10.231.(10N+1|2|3).0/24 with the same host parts, gateway 127.0.0.1:(8765+N).
docker/compose.yaml reads the MAVERICK_* variables returned by `stack_env` (each with the
stack-0 default); scripts/compose.sh and referee.stack.Stack both take them from here.

  python3 -m referee.stackcfg N     # prints `export` lines (used by scripts/compose.sh)
"""

from __future__ import annotations

import ipaddress
import sys

MAX_STACK = 24  # 10N+3 must stay below 256
BASE_PORT = 8765


def prefixes(index: int) -> dict[str, str]:
    """Network name -> the first three octets of its /24."""
    if not 0 <= index <= MAX_STACK:
        raise ValueError(f"stack index must be 0..{MAX_STACK}, got {index}")
    if index == 0:
        return {"drone": "10.231.1", "wifi": "10.231.2", "world": "10.231.3"}
    return {
        name: f"10.231.{10 * index + k}" for k, name in ((1, "drone"), (2, "wifi"), (3, "world"))
    }


def project_name(index: int) -> str:
    return "maverick" if index == 0 else f"maverick-s{index}"


def gateway_port(index: int) -> int:
    return BASE_PORT + index


def ip_as_int(ip: str) -> int:
    return int(ipaddress.IPv4Address(ip))


def stack_env(index: int) -> dict[str, str]:
    """Environment for docker compose (compose.yaml variables) for stack `index`."""
    p = prefixes(index)
    bridge_drone = f"{p['drone']}.3"
    return {
        "COMPOSE_PROJECT_NAME": project_name(index),
        "MAVERICK_STACK": str(index),
        "MAVERICK_DRONE_NET": p["drone"],
        "MAVERICK_WIFI_NET": p["wifi"],
        "MAVERICK_WORLD_NET": p["world"],
        "MAVERICK_GATEWAY_PORT": str(gateway_port(index)),
        "MAVERICK_XRCE_AGENT_IP_INT": str(ip_as_int(bridge_drone)),
    }


def parse_stacks(jobs: int, stacks: str | None) -> list[int]:
    """Stack indices to use: an explicit `--stacks 1,2,3` list, else 0 for one job, 1..K for K."""
    if stacks:
        ids = [int(s) for s in stacks.split(",") if s.strip()]
        if not ids:
            raise ValueError("--stacks is empty")
        if len(set(ids)) != len(ids):
            raise ValueError(f"--stacks has duplicates: {stacks}")
        if jobs != 1 and jobs != len(ids):
            raise ValueError(f"--jobs {jobs} does not match --stacks {stacks}")
    else:
        if jobs < 1:
            raise ValueError("--jobs must be >= 1")
        ids = [0] if jobs == 1 else list(range(1, jobs + 1))
    for i in ids:
        prefixes(i)  # range check
    return ids


def main(argv: list[str]) -> int:
    index = int(argv[0]) if argv else 0
    for k, v in stack_env(index).items():
        print(f"export {k}={v}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

"""Start one stack with the agent for a live, typed mission (S4c run-it guide in docs/s4-plan.md).

  just ws-build                                     # once, and after code changes
  just live-s4 --seed 12                            # stack 0, DeepSeek agent, port 8765
  MAVERICK_STACK=3 just live-s4 --seed 12 --stack 3 --backend scripted   # stack 3, port 8768
  just down                                         # (MAVERICK_STACK=3 just down for stack 3)

Generates apartment `--seed` with bananas, bread and a mannequin, starts PX4 + Gazebo + the host
stack with `agent_node` (host launch `s4`), waits for take-off and prints where to point the
console. Then type in the console, e.g. "Explore the flat", then "Come to me", then the H2
mission. Nothing here runs a scenario: no referee, no verdicts.
"""

from __future__ import annotations

import argparse
import sys

from referee.s3 import LOCALISATIONS, configure_stack, make_world
from referee.stack import Stack
from referee.stackcfg import gateway_port


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--seed", type=int, default=12)
    ap.add_argument("--stack", type=int, default=0, help="stack index (port 8765 + index)")
    ap.add_argument("--backend", choices=("scripted", "deepseek", "claude"), default="deepseek")
    ap.add_argument("--cap-eur", type=float, default=0.5, help="agent spend cap (estimate)")
    ap.add_argument("--localisation", choices=LOCALISATIONS, default="mocap")
    args = ap.parse_args(argv)
    stack = Stack(index=args.stack)
    world = make_world(args.seed, extras=True)
    configure_stack(stack, world, "s4", args.localisation)
    stack.env["MAVERICK_LAUNCH_ARGS"] = (
        f"agent_backend:={args.backend} agent_cap_eur:={args.cap_eur:.3f}"
    )
    stack.restart()
    port = gateway_port(args.stack)
    people = world["manifest"].get("people") or []
    print(f"\nStack {args.stack} is up: apartment seed {args.seed}, agent backend {args.backend}.")
    print(f"Gateway websocket: ws://localhost:{port}")
    print(f"Console: http://localhost:5173/?gateway=ws://localhost:{port}  (see console/README.md)")
    print(f"Operator mannequin: {'yes' if people else 'none in this apartment, pick another seed'}")
    print("The drone takes off by itself. Type: 'explore the flat', then 'come to me'.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

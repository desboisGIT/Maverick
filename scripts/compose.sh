#!/usr/bin/env bash
# docker compose for the Maverick stack selected by MAVERICK_STACK (default 0 = project
# `maverick`, the original addresses). Addresses come from sim/referee/referee/stackcfg.py.
#   MAVERICK_STACK=2 scripts/compose.sh up -d
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
eval "$(cd "$ROOT/sim/referee" && python3 -m referee.stackcfg "${MAVERICK_STACK:-0}")"
exec docker compose -f "$ROOT/docker/compose.yaml" "$@"

#!/usr/bin/env bash
# S0 exit test: N runs of take off -> hover -> land, each on a freshly started simulator.
# Usage: scripts/hover_trials.sh [N=20]
# Results: runs/s0-hover-<timestamp>/results.jsonl (one JSON line per trial) and per-trial logs.
set -euo pipefail

N="${1:-20}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
COMPOSE=(docker compose -f "$ROOT/docker/compose.yaml")
OUT="$ROOT/runs/s0-hover-$(date +%Y%m%d-%H%M%S)"
READY_TIMEOUT_S=180
mkdir -p "$OUT"

in_sim() { "${COMPOSE[@]}" exec -T sim bash -lc "$1"; }

stop_sim() { "${COMPOSE[@]}" down --timeout 10 >/dev/null 2>&1 || true; }
trap stop_sim EXIT

start_sim() {
  HEADLESS=1 "${COMPOSE[@]}" up -d sim >/dev/null
  local waited=0
  until in_sim "test -f /tmp/px4_ready" 2>/dev/null; do
    sleep 2; waited=$((waited + 2))
    if (( waited >= READY_TIMEOUT_S )); then
      echo "PX4 not ready after ${READY_TIMEOUT_S}s" >&2
      return 1
    fi
  done
}

pass=0
for i in $(seq 1 "$N"); do
  log="$OUT/trial-$(printf %02d "$i").log"
  stop_sim
  if ! start_sim; then
    result='{"result": "fail", "reason": "sim_not_ready"}'
    "${COMPOSE[@]}" logs sim >"$log" 2>&1 || true
  else
    if (( i == 1 )); then
      in_sim "cd /maverick/ros2_ws && colcon build --symlink-install --packages-select maverick_flight_test" \
        >"$OUT/colcon.log" 2>&1 || { echo "colcon build failed, see $OUT/colcon.log" >&2; exit 1; }
    fi
    in_sim "ros2 run maverick_flight_test hover_test --altitude 2.0 --hover-seconds 10 --timeout 90" \
      >"$log" 2>&1 || true
    result="$(grep -E '^\{.*"result"' "$log" | tail -n 1 || true)"
    [[ -n "$result" ]] || result='{"result": "fail", "reason": "no_result_line"}'
    "${COMPOSE[@]}" logs sim >"${log%.log}-sim.log" 2>&1 || true
  fi
  echo "$result" >>"$OUT/results.jsonl"
  if [[ "$result" == *'"result": "pass"'* ]]; then pass=$((pass + 1)); status=PASS; else status=FAIL; fi
  echo "trial $i/$N: $status $result"
done

echo "S0 hover: $pass/$N passed. Results in $OUT"
(( pass == N ))

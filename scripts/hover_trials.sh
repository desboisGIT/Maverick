#!/usr/bin/env bash
# S0/S1 hover test: N runs of take off -> hover -> land through the full bridge path
# (hover_test --via bridge on the host container), each on a freshly started stack.
# Usage: scripts/hover_trials.sh [N=20]
# Results: runs/s1-hover-<timestamp>/results.jsonl (one JSON line per trial) and per-trial logs.
set -euo pipefail

N="${1:-20}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
COMPOSE=(docker compose -f "$ROOT/docker/compose.yaml")
OUT="$ROOT/runs/s1-hover-$(date +%Y%m%d-%H%M%S)"
READY_TIMEOUT_S=180
mkdir -p "$OUT"

DOCKER_GID="$(stat -c %g /var/run/docker.sock)"
export DOCKER_GID
in_sim() { "${COMPOSE[@]}" exec -T sim bash -lc "$1"; }
in_host() { "${COMPOSE[@]}" exec -T host bash -lc "$1"; }

stop_sim() { "${COMPOSE[@]}" down --timeout 10 >/dev/null 2>&1 || true; }
trap stop_sim EXIT

start_sim() {
  HEADLESS=1 "${COMPOSE[@]}" up -d >/dev/null
  local waited=0
  until in_sim "test -f /tmp/px4_ready" 2>/dev/null; do
    sleep 2; waited=$((waited + 2))
    if (( waited >= READY_TIMEOUT_S )); then
      echo "PX4 not ready after ${READY_TIMEOUT_S}s" >&2
      return 1
    fi
  done
}

# Build the workspace (maverick_interfaces, maverick_bridge, maverick_flight_test) once.
"${COMPOSE[@]}" --profile tools run --rm -T tools bash -lc "cd /maverick/ros2_ws && colcon build --symlink-install" \
  >"$OUT/colcon.log" 2>&1 || { echo "colcon build failed, see $OUT/colcon.log" >&2; exit 1; }

pass=0
for i in $(seq 1 "$N"); do
  log="$OUT/trial-$(printf %02d "$i").log"
  stop_sim
  if ! start_sim; then
    result='{"result": "fail", "reason": "sim_not_ready"}'
    "${COMPOSE[@]}" logs >"$log" 2>&1 || true
  else
    # Wait for the bridge relay and the zenoh link: the host must see PX4 telemetry.
    in_host "for _ in \$(seq 60); do ros2 topic list 2>/dev/null | grep -q /maverick/bridge/status && exit 0; sleep 1; done; exit 1" \
      >>"$log" 2>&1 || true
    in_host "ros2 run maverick_flight_test hover_test --via bridge --altitude 2.0 --hover-seconds 10 --timeout 90" \
      >>"$log" 2>&1 || true
    result="$(grep -E '^\{.*"result"' "$log" | tail -n 1 || true)"
    [[ -n "$result" ]] || result='{"result": "fail", "reason": "no_result_line"}'
    in_sim "python3 /maverick/scripts/ulog_rates.py" >"${log%.log}-ulog.txt" 2>&1 || true
    "${COMPOSE[@]}" logs >"${log%.log}-stack.log" 2>&1 || true
  fi
  echo "$result" >>"$OUT/results.jsonl"
  if [[ "$result" == *'"result": "pass"'* ]]; then pass=$((pass + 1)); status=PASS; else status=FAIL; fi
  echo "trial $i/$N: $status $result"
done

echo "Hover via bridge: $pass/$N passed. Results in $OUT"
(( pass == N ))

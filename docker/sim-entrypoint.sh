#!/usr/bin/env bash
# Maverick sim container entrypoint.
#   no args      : PX4 SITL + Gazebo Harmonic (the XRCE agent runs in the bridge container)
#   with args    : exec them (e.g. `docker compose run sim bash`)
# Env: HEADLESS=1 (default) no Gazebo GUI; HEADLESS=0 shows it via X11.
#      PX4_SIM_MODEL (default gz_maverick, set by compose). ROS_DOMAIN_ID is picked up as UXRCE_DDS_DOM_ID.
#      PX4's uXRCE-DDS client reaches the agent via PX4_PARAM_UXRCE_DDS_AG_IP (compose).
set -uo pipefail

if [ "$#" -gt 0 ]; then
    exec "$@"
fi

PX4_BUILD="${PX4_DIR:-/opt/PX4-Autopilot}/build/px4_sitl_default"
PX4_LOG=/tmp/px4.log
READY_FILE=/tmp/px4_ready
HEADLESS="${HEADLESS:-1}"

rm -f "$READY_FILE" "$PX4_LOG"
: > "$PX4_LOG"

# px4-rc.gzsim starts the Gazebo GUI whenever HEADLESS is *unset or empty*
# (it only tests -z), so HEADLESS=0 must mean "unset" here.
if [ "$HEADLESS" = "0" ]; then
    unset HEADLESS
    echo "[entrypoint] Gazebo GUI enabled, DISPLAY=${DISPLAY:-unset}"
else
    export HEADLESS=1
fi

PX4_PID=""

cleanup() {
    trap '' TERM INT
    echo "[entrypoint] shutting down"
    [ -n "$PX4_PID" ] && kill -TERM "$PX4_PID" 2>/dev/null
    # wait up to ~10 s for PX4 to exit, then make sure Gazebo is gone too
    for _ in $(seq 1 20); do
        { [ -z "$PX4_PID" ] || ! kill -0 "$PX4_PID" 2>/dev/null; } && break
        sleep 0.5
    done
    pkill -TERM -f "gz sim" 2>/dev/null
    [ -n "$PX4_PID" ] && kill -KILL "$PX4_PID" 2>/dev/null
    exit "${1:-0}"
}
trap 'cleanup 0' TERM INT

# 1. PX4 SITL with a Gazebo model, same as `make px4_sitl <model>` but running the prebuilt
#    binary. The uXRCE-DDS client is started by rcS and reads UXRCE_DDS_AG_IP at that moment,
#    so the agent IP must already be set then: rcS applies PX4_PARAM_* before starting it.
#    -d: no interactive pxh shell, so no TTY is needed.
#    The binary has its rootfs baked in (build/px4_sitl_default/rootfs).
export PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_maverick}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
cd "$PX4_BUILD/rootfs" || exit 1
stdbuf -oL -eL "$PX4_BUILD/bin/px4" -d </dev/null > >(tee -a "$PX4_LOG") 2>&1 &
PX4_PID=$!

# 2. Readiness watcher: PX4 commander prints "Ready for takeoff!" once the
#    preflight checks (EKF, home position, ...) pass.
#    Before that, re-apply PX4_PARAM_* overrides once startup has finished: rcS applies
#    them *before* the airframe file, whose `param set-default` can override a value
#    that equalled the old default (e.g. NAV_DLL_ACT=0 vs x500's default of 2).
#    Stage S1 moves these into Maverick's own airframe file.
(
    params_applied=0
    while kill -0 "$PX4_PID" 2>/dev/null; do
        if [ "$params_applied" = 0 ] && grep -q "Startup script returned" "$PX4_LOG" 2>/dev/null; then
            while IFS='=' read -r name value; do
                echo "[entrypoint] param set ${name#PX4_PARAM_} $value"
                "$PX4_BUILD/bin/px4-param" set "${name#PX4_PARAM_}" "$value" >/dev/null
            done < <(env | grep '^PX4_PARAM_')
            params_applied=1
        fi
        if grep -q "Ready for takeoff" "$PX4_LOG" 2>/dev/null; then
            touch "$READY_FILE"
            echo "[entrypoint] PX4 ready -> $READY_FILE"
            exit 0
        fi
        sleep 0.5
    done
) &

# Block until PX4 exits; `wait` returns early on a trapped signal.
wait "$PX4_PID"
RC=$?
echo "[entrypoint] PX4 exited with code $RC"
cleanup "$RC"

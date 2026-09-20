#!/usr/bin/env bash
# Maverick sim container entrypoint.
#   no args      : PX4 SITL + Gazebo Harmonic (the XRCE agent runs in the bridge container), then the
#                  camera driver (CAMERA_DRIVER=0 disables it, CAMERA_BRIDGE_ADDR default 10.231.1.3)
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
    pkill -TERM -f "camera_driver.driver" 2>/dev/null
    pkill -TERM -f "mocap_driver.driver" 2>/dev/null
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
# Localisation source: flow (default: PX4 optical flow + rangefinder only), mocap (simulated motion
# capture pose fused as external vision), vo (host visual odometry). mocap and vo switch EV fusion on.
export MAVERICK_LOCALISATION="${MAVERICK_LOCALISATION:-flow}"
case "$MAVERICK_LOCALISATION" in
    mocap|vo) export MAVERICK_EV_CTRL="${MAVERICK_EV_CTRL:-9}" ;;
    *) export MAVERICK_EV_CTRL="${MAVERICK_EV_CTRL:-0}" ;;
esac
export PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_maverick}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
cd "$PX4_BUILD/rootfs" || exit 1
# px4-rc.gzsim sources rootfs/gz_env.sh, which *overwrites* PX4_GZ_WORLDS with PX4's own worlds
# directory. Patch it so a PX4_GZ_WORLDS from the environment (compose) wins. [F] checked in the image.
if [ -n "${PX4_GZ_WORLDS:-}" ]; then
    if [ ! -f "$PX4_GZ_WORLDS/${PX4_GZ_WORLD:-default}.sdf" ]; then
        echo "[entrypoint] world file $PX4_GZ_WORLDS/${PX4_GZ_WORLD:-default}.sdf not found" >&2
        exit 1
    fi
    sed -i "s|^export PX4_GZ_WORLDS=.*|export PX4_GZ_WORLDS=$PX4_GZ_WORLDS|" gz_env.sh
    echo "[entrypoint] world ${PX4_GZ_WORLD:-default} from $PX4_GZ_WORLDS, model pose '${PX4_GZ_MODEL_POSE:-}'"
fi
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
            # Camera driver ("the camera hardware"): Gazebo sensor topics -> TCP to the bridge on
            # the drone network. Gazebo transport stays on the world network (GZ_IP).
            if [ "$MAVERICK_LOCALISATION" = mocap ]; then
                ( cd /maverick/sim/mocap_driver \
                    && exec python3 -u -m mocap_driver.driver --host "${CAMERA_BRIDGE_ADDR:-10.231.1.3}" ) &
            fi
            if [ "${CAMERA_DRIVER:-1}" = 1 ]; then
                cd /maverick/sim/camera_driver \
                    && exec python3 -u -m camera_driver.driver --host "${CAMERA_BRIDGE_ADDR:-10.231.1.3}"
            fi
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

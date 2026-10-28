#!/usr/bin/env bash
# Host-PC container: zenoh-bridge-ros2dds (connects to the bridge over wifi) and, optionally,
# Maverick's host software. Pass args to run something else.
set -uo pipefail
if [ "$#" -gt 0 ]; then exec "$@"; fi
export CYCLONEDDS_URI=file:///maverick/docker/zenoh/cyclonedds-localhost.xml
LAUNCH_PID=""
trap 'kill -TERM "$ZENOH_PID" $LAUNCH_PID 2>/dev/null; exit 0' TERM INT

# Optional host software (MAVERICK_HOST_LAUNCH): "none" (default; the S1 failsafe suite
# starts its own flight client), "s3" (mapping, memory, skills, gateway, flight client) or "s4"
# (s3 plus agent_node; the agent backend comes from MAVERICK_LAUNCH_ARGS, e.g.
# "agent_backend:=scripted agent_cap_eur:=0.1", default deepseek).
# Needs the workspace built first (`just ws-build`).
case "${MAVERICK_HOST_LAUNCH:-none}" in
    s3|s4)
        ( set +u; . /maverick/ros2_ws/install/setup.bash \
          && exec ros2 launch maverick_gateway s3_host.launch.py \
            vo:=$([ "${MAVERICK_LOCALISATION:-flow}" = vo ] && echo true || echo false) \
            require_ev:=$([ "${MAVERICK_LOCALISATION:-flow}" = flow ] && echo false || echo true) \
            agent:=$([ "${MAVERICK_HOST_LAUNCH}" = s4 ] && echo true || echo false) \
            ${MAVERICK_LAUNCH_ARGS:-} ) &
        LAUNCH_PID=$!
        echo "[host] started ${MAVERICK_HOST_LAUNCH} host launch (pid $LAUNCH_PID)"
        ;;
    none) ;;
    *) echo "[host] unknown MAVERICK_HOST_LAUNCH=${MAVERICK_HOST_LAUNCH}" >&2 ;;
esac

# Endpoints in the json5 are written for stack 0 (10.231.2.x); WIFI_PREFIX moves them per stack.
sed "s/10\.231\.2\./${WIFI_PREFIX:-10.231.2.}/g" /maverick/docker/zenoh/host.json5 >/tmp/zenoh.json5
while true; do   # restart if it ever exits (e.g. the bridge was not up yet)
    zenoh-bridge-ros2dds -c /tmp/zenoh.json5 &
    ZENOH_PID=$!
    wait "$ZENOH_PID"
    sleep 1
done

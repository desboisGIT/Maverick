#!/usr/bin/env bash
# Bridge-board container: Micro XRCE-DDS Agent (drone network) + zenoh-bridge-ros2dds (wifi
# network, allow-listed topics) + the maverick_bridge relay. Pass args to run something else.
set -o pipefail   # no -u: ROS setup.bash reads unset variables
if [ "$#" -gt 0 ]; then exec "$@"; fi

# shellcheck disable=SC1091
. /etc/profile.d/maverick.sh

# PX4 <-> agent <-> relay DDS lives in its own domain (PX4_DDS_DOMAIN_ID, set for sim via
# ROS_DOMAIN_ID), apart from domain 0 where the relay and the zenoh bridge face the host.
# zenoh-bridge-ros2dds only routes entities declared by ROS nodes, and the agent's are not, so
# the relay republishes the allow-listed /fmu/out topics into domain 0.
MicroXRCEAgent udp4 -p 8888 &
AGENT_PID=$!
export CYCLONEDDS_URI=file:///maverick/docker/zenoh/cyclonedds-localhost.xml
# Endpoints in the json5 are written for stack 0 (10.231.2.x); WIFI_PREFIX moves them per stack.
sed "s/10\.231\.2\./${WIFI_PREFIX:-10.231.2.}/g" /maverick/docker/zenoh/bridge.json5 >/tmp/zenoh.json5
zenoh-bridge-ros2dds -c /tmp/zenoh.json5 &
ZENOH_PID=$!

# The relay needs the workspace built (`just ws-build`). It is deliberately not restarted if it
# dies: the failsafe tests kill it and expect PX4's offboard-loss failsafe to take over.
(
    until [ -f /maverick/ros2_ws/install/setup.bash ] \
        && ros2 pkg prefix maverick_bridge >/dev/null 2>&1; do
        . /etc/profile.d/maverick.sh
        sleep 2
    done
    MOCAP_LISTEN="${MOCAP_LISTEN:-10.231.1.3}" exec ros2 run maverick_bridge relay
) &

# Camera relay: receives the camera driver's TCP stream from the sim container on the drone
# network only (listen address = our drone-network IP) and publishes /maverick/camera/*.
(
    until [ -f /maverick/ros2_ws/install/setup.bash ] \
        && ros2 pkg prefix maverick_bridge >/dev/null 2>&1; do
        . /etc/profile.d/maverick.sh
        sleep 2
    done
    CAMERA_LISTEN="${CAMERA_LISTEN:-10.231.1.3}" exec ros2 run maverick_bridge camera_relay
) &

cleanup() { trap '' TERM INT; kill -TERM "$AGENT_PID" "$ZENOH_PID" 2>/dev/null; pkill -TERM -f "maverick_bridge" 2>/dev/null; pkill -TERM -f camera_relay 2>/dev/null; exit 0; }
trap cleanup TERM INT
wait -n "$AGENT_PID" "$ZENOH_PID"
echo "[bridge] agent or zenoh bridge exited"
cleanup

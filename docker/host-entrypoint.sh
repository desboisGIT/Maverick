#!/usr/bin/env bash
# Host-PC container: zenoh-bridge-ros2dds (connects to the bridge over wifi), then idle.
# Later stages add the flight nodes. Pass args to run something else.
set -uo pipefail
if [ "$#" -gt 0 ]; then exec "$@"; fi
export CYCLONEDDS_URI=file:///maverick/docker/zenoh/cyclonedds-localhost.xml
trap 'kill -TERM "$ZENOH_PID" 2>/dev/null; exit 0' TERM INT
while true; do   # restart if it ever exits (e.g. the bridge was not up yet)
    zenoh-bridge-ros2dds -c /maverick/docker/zenoh/host.json5 &
    ZENOH_PID=$!
    wait "$ZENOH_PID"
    sleep 1
done

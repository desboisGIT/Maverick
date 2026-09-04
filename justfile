set shell := ["bash", "-euo", "pipefail", "-c"]

compose := "docker compose -f docker/compose.yaml"

# List available recipes
default:
    @just --list

# Check host prerequisites (docker, compose, nvidia runtime, DISPLAY)
setup-check:
    #!/usr/bin/env bash
    rc=0
    report() { if [ "$2" = 0 ]; then echo "OK       $1"; else echo "MISSING  $1"; rc=1; fi; }
    command -v docker >/dev/null 2>&1; report docker $?
    docker compose version >/dev/null 2>&1; report "docker compose" $?
    docker info --format '{{ "{{json .Runtimes}}" }}' 2>/dev/null | grep -q nvidia; report "nvidia runtime" $?
    [ -n "${DISPLAY:-}" ]; report DISPLAY $?
    exit "$rc"

# Build the simulation image
build:
    {{compose}} build sim

# Run the simulation in the foreground (gui=1 shows the Gazebo GUI)
sim gui="0":
    #!/usr/bin/env bash
    if [ "{{gui}}" = "1" ]; then xhost +local:docker >/dev/null 2>&1 || true; fi
    HEADLESS=$(( 1 - {{gui}} )) {{compose}} up sim

# Stop and remove the simulation containers
down:
    {{compose}} down

# Open a login shell in the running sim container
shell:
    {{compose}} exec sim bash -l

# Build the ROS 2 workspace inside the sim container
ws-build:
    {{compose}} exec sim bash -lc "cd /maverick/ros2_ws && colcon build --symlink-install"

# Run one hover trial
hover:
    scripts/hover_trials.sh 1

# Run n hover trials
hover-trials n="20":
    scripts/hover_trials.sh {{n}}

# Run unit tests in the sim container
test-unit:
    {{compose}} run --rm --no-deps sim bash -lc "cd /maverick && python3 -m pytest"

# Lint the repo in the sim container
lint:
    {{compose}} run --rm --no-deps sim bash -lc "cd /maverick && ruff check ."

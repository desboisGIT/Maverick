set shell := ["bash", "-euo", "pipefail", "-c"]

compose := "scripts/compose.sh"

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

# Build the image (shared by sim, bridge, host and referee)
build:
    {{compose}} build sim

# Start the whole stack in the background (builds the ROS workspace first)
up gui="0": ws-build
    #!/usr/bin/env bash
    if [ "{{gui}}" = "1" ]; then xhost +local:docker >/dev/null 2>&1 || true; fi
    export DOCKER_GID=$(stat -c %g /var/run/docker.sock)
    HEADLESS=$(( 1 - {{gui}} )) {{compose}} up -d

# Run the simulator in the foreground (gui=1 shows the Gazebo GUI); needs the bridge for XRCE
sim gui="0":
    #!/usr/bin/env bash
    if [ "{{gui}}" = "1" ]; then xhost +local:docker >/dev/null 2>&1 || true; fi
    HEADLESS=$(( 1 - {{gui}} )) {{compose}} up bridge host -d
    HEADLESS=$(( 1 - {{gui}} )) {{compose}} up sim

# Stop and remove the whole stack
down:
    {{compose}} down --timeout 10

# Bring down every parallel stack (compose projects maverick-s*); never touches `maverick` (stack 0)
stacks-down:
    #!/usr/bin/env bash
    for p in $(docker compose ls -a --format json | python3 -c 'import json,sys; [print(x["Name"]) for x in json.load(sys.stdin) if x["Name"].startswith("maverick-s")]'); do
        docker compose -p "$p" down --timeout 10
    done

# Open a login shell in a running service (sim, bridge, host or referee)
shell service="sim":
    {{compose}} exec {{service}} bash -l

# Apply a wifi link profile to bridge and host: lab, apartment, bad, outage or clear
netem profile:
    {{compose}} exec -T bridge bash /maverick/docker/netem/apply.sh {{profile}}
    {{compose}} exec -T host bash /maverick/docker/netem/apply.sh {{profile}}

# Build the ROS 2 workspace (uid 1000, into ros2_ws/build and install); works without a stack
ws-build:
    {{compose}} --profile tools run --rm -T tools bash -lc "cd /maverick/ros2_ws && colcon build --symlink-install"

# Run one hover trial
hover:
    scripts/hover_trials.sh 1

# Run n hover trials
hover-trials n="20":
    scripts/hover_trials.sh {{n}}

# Run unit tests in the sim container
test-unit:
    {{compose}} --profile tools run --rm -T tools bash -lc "cd /maverick && python3 -m pytest"

# Lint the repo in the sim container
lint:
    {{compose}} --profile tools run --rm -T tools bash -lc "cd /maverick && ruff check ."

# Randomised failsafe suite F1/F2/F3/F4/F6 (runs on the host, drives docker compose).
# e.g. `just test-failsafe --runs-per-case 3 --seed 1`, `--cases F3,F4`;
# in parallel: `--jobs 3 --stacks 1,2,3` (stacks 1.. only; stack 0 is the default `maverick`)
test-failsafe *args:
    PYTHONPATH=sim/referee python3 -m referee.failsafe {{args}}

# S3 scenarios in generated apartments (runs on the host, drives docker compose):
# e.g. `just test-s3 --scenario hover --seed-set held_out --runs 10`,
# `just test-s3 --scenario explore --seeds 1000000 --budget 360`, `--scenario goto_room ...`
# in parallel: `--jobs 3` (stacks 1..3) or `--stacks 1,2`
test-s3 *args:
    PYTHONPATH=sim/referee python3 -m referee.s3 {{args}}

# S4b: search (bananas, bread), come_to and return in generated apartments, tuning seeds only:
# e.g. `just test-s4 --seeds 10-14 --stacks 3`
test-s4 *args:
    PYTHONPATH=sim/referee python3 -m referee.s4 {{args}}

# S4c h2_text (mission by text through the agent), tuning seeds: `just test-s4c --seeds 10-19 --stacks 3 --backend scripted`
test-s4c *args:
    PYTHONPATH=sim/referee python3 -m referee.s4c {{args}}

# S4c live: one stack with the agent for typed missions in the console (see docs/s4-plan.md),
# e.g. `just live-s4 --seed 12` or `just live-s4 --seed 12 --stack 3 --backend scripted`
live-s4 *args:
    PYTHONPATH=sim/referee python3 -m referee.live {{args}}

# Generate random apartment world(s) into sim/worlds/generated/, e.g. `just worldgen 7` or
# `just worldgen 7 --count 5 --no-ceiling`
worldgen seed="7" *args:
    PYTHONPATH=sim/worldgen python3 -m worldgen --seed {{seed}} --out sim/worlds/generated {{args}}

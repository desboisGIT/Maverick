# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Maverick is agentic drone software. The goal: full autonomy with spatial reasoning. The drone should understand complex instructions given by voice or in natural language, and explore, map and reason about its surroundings.

Design docs: `docs/architecture-study.md` (phase 1: architecture and feasibility) and `docs/simulation-plan.md` (phase 2: simulation, live demo, build stages S0–S8). Stage S0 (Docker + PX4 SITL + Gazebo + scripted offboard hover) is the current code.

## Commands

Everything runs in Docker through `just` (recipes in `justfile`). Host setup once: `sudo bash scripts/setup-host.sh`, then `just setup-check`.

- `just build`: build the `maverick-sim:dev` image (PX4 v1.17.0 prebuilt, Gazebo Harmonic, ROS 2 Jazzy, px4_msgs underlay at `/opt/px4_ws`). The first build takes about 30 min.
- `just sim` / `just sim gui=1`: run PX4 SITL + Gazebo (x500), headless or with the Gazebo window. `just down` stops it, `just shell` opens a shell in it.
- `just ws-build`: `colcon build` the repo's ROS 2 workspace (`ros2_ws/`) inside the running sim container.
- `just hover` / `just hover-trials 20`: S0 exit test. Each trial restarts the sim, runs `ros2 run maverick_flight_test hover_test`, and appends one JSON line to `runs/s0-hover-<timestamp>/results.jsonl` (per-trial logs alongside).
- `just test-unit`: pytest in the container. `just lint`: ruff. Config in `pyproject.toml` (pytest only collects `test_*.py`).
- Single test: `docker compose -f docker/compose.yaml run --rm --no-deps sim bash -lc "cd /maverick && python3 -m pytest ros2_ws/src/maverick_flight_test/test/test_hover_mission.py::<test_name>"`. Pure-Python tests also run on the host with pytest installed.
- Inside the container, `bash -lc` sources ROS 2, the px4_msgs underlay and `ros2_ws/install` (via `/etc/profile.d/maverick.sh`).

## Code layout

- `docker/`: `sim.Dockerfile`, `compose.yaml` (service `sim`, repo bind-mounted at `/maverick`), `sim-entrypoint.sh` (starts Micro XRCE-DDS Agent on UDP 8888 and PX4 SITL; creates `/tmp/px4_ready` on "Ready for takeoff").
- `ros2_ws/src/maverick_flight_test/`: S0 hover test. Pattern to follow in later packages: mission logic in a pure-Python module with no ROS imports (`hover_mission.py`, unit-tested on the host), and a thin rclpy node around it (`hover_test.py`).
- `scripts/`: host setup and the trial runner.
- Planned layout for later stages: `docs/simulation-plan.md` §5.

## PX4 facts learned in S0 (verified by running)

- PX4 v1.17.0 SITL publishes **versioned** topic names for versioned messages, e.g. `/fmu/out/vehicle_status_v1` and `/fmu/out/vehicle_local_position_v1`, but `/fmu/out/vehicle_land_detected` and `/fmu/in/*` are unversioned. Check the `uxrce_dds_client` lines in the sim log before trusting a topic name.
- PX4 parameter overrides go in `compose.yaml` as `PX4_PARAM_<NAME>`. The entrypoint re-applies them after startup, because rcS applies them before the airframe's `param set-default`, which otherwise wins. `NAV_DLL_ACT=0` is required to arm without a ground-control station.

## Decisions code must respect

- Simulation is Gazebo Harmonic + PX4 SITL + ROS 2 Jazzy, run in Ubuntu 24.04 Docker containers. The host runs Ubuntu 26.04, which PX4/Harmonic/Isaac Sim don't list as supported. Isaac Sim is a later, optional stage.
- The simulated drone is the small indoor quad (3.5 inch ducted class, 0.30 m across with guards, about 0.5 kg; `MAVERICK_DRONE=small`, default); the x500 is kept as an option (`MAVERICK_DRONE=x500`). One switch selects model, airframe, worldgen profile and the planner/referee body radius.
- LLMs and VLMs only call a typed skill API (explore, goto, search, inspect, follow, monitor, return, report). They never send setpoints. Flight-critical loops stay on PX4.
- Losing Wi-Fi or the GPU host must never cause uncontrolled flight. There is no onboard companion computer: PX4 holds position on optical flow + rangefinder indoors (GPS outdoors), and its offboard-loss failsafe holds, then lands. A small bridge board only streams sensors and relays setpoints (dropping stale ones); VIO, mapping, obstacle checks and all models run offboard.
- The 16 GB VRAM budget is shared between the simulator and all local models, so check VRAM before adding a model.
- Cloud models are allowed and are the way to relieve VRAM. Each model role has one interface with local and cloud backends picked by config. Nothing flight-critical depends on the cloud.
- The drone hears its operator through its own mic array; the bridge streams raw audio and all speech processing (keyword spotting included) runs on the host. The RC kill switch is the manual emergency stop. Commands are English only.
- Outdoor flight is in scope under ideal conditions (no wind, bright daylight): the drone has GPS, and outdoors it relies on GPS + barometer, not optical flow.
- The drone must work in any home with no prior map. Develop on randomly generated apartments and report results only on held-out layouts, never on layouts used for tuning (including the owner's flat).
- The drone's stack (bridge, host containers) must never read simulator ground truth; only the referee does (simulation plan §2.1).
- Design docs tag claims [F] verified, [E] estimate, [H] hypothesis, [NV] not verified. Keep that convention.

As later stages land, add the high-level architecture here: how voice/NLU, the agent, the executive (skills), mapping/perception and flight control connect.

# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Maverick is agentic drone software. The goal: full autonomy with spatial reasoning. The drone should understand complex instructions given by voice or in natural language, and explore, map and reason about its surroundings.

## Current state

Phase 1 (architecture and feasibility study) is in `docs/architecture-study.md`. Phase 2 (simulation and live demo plan: containers, referee with ground truth isolated from the drone's stack, world generator, Mission Console, build stages S0–S8) is in `docs/simulation-plan.md`. There is no source code, build system or test suite yet, so there are no build, lint or test commands to document.

Decisions in the study that code must respect:
- Simulation is Gazebo Harmonic + PX4 SITL + ROS 2 Jazzy, run in Ubuntu 24.04 Docker containers. The host runs Ubuntu 26.04, which PX4/Harmonic/Isaac Sim don't list as supported. Isaac Sim is a later, optional stage.
- LLMs and VLMs only call a typed skill API (explore, goto, search, inspect, follow, monitor, return, report). They never send setpoints. Flight-critical loops stay on PX4.
- Losing Wi-Fi or the GPU host must never cause uncontrolled flight. There is no onboard companion computer: PX4 holds position on optical flow + rangefinder indoors (GPS outdoors), and its offboard-loss failsafe holds, then lands. A small bridge board only streams sensors and relays setpoints (dropping stale ones); VIO, mapping, obstacle checks and all models run offboard.
- The 16 GB VRAM budget is shared between the simulator and all local models, so check VRAM before adding a model.
- Cloud models are allowed and are the way to relieve VRAM. Each model role has one interface with local and cloud backends picked by config. Nothing flight-critical depends on the cloud.
- The drone hears its operator through its own mic array; the bridge streams raw audio and all speech processing (keyword spotting included) runs on the host. The RC kill switch is the manual emergency stop. Commands are English only.
- Outdoor flight is in scope under ideal conditions (no wind, bright daylight): the drone has GPS, and outdoors it relies on GPS + barometer, not optical flow.
- The drone must work in any home with no prior map. Develop on randomly generated apartments and report results only on held-out layouts, never on layouts used for tuning (including the owner's flat).
- The study tags claims [F] verified, [E] estimate, [H] hypothesis, [NV] not verified. Keep that convention in design docs.

When the first code lands (language, build system, test runner, project layout), update this file with:
- How to build, lint and run tests, including how to run a single test
- The high-level architecture, such as how the voice/NLU, planning/reasoning, mapping/perception and flight-control parts connect

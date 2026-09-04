# Maverick: simulation and live demo plan (phase 2)

Status: plan for review, written 2026-11-11. No implementation yet. Builds on [architecture-study.md](architecture-study.md) ("the study"); section numbers like "study §2.5" point there.

Claims use the study's tags: **[F]** verified (sources at the end), **[E]** estimate, **[H]** hypothesis, **[NV]** not verified.

---

## 0. Summary

1. **One simulated world, three machines, one referee.** Docker containers mirror the real system: `sim` (Gazebo + PX4 = the physical world and the drone), `bridge` (the small board on the drone), `host` (the PC: perception, map, memory, agent, voice). A fourth container, the **referee**, sees the simulator's ground truth, runs scenarios, injects faults and scores results. The drone's software can never read ground truth: it runs on a separate network.
2. **You are in the simulation.** In the live demo you control an avatar of yourself: walk with the keyboard or a gamepad, point by clicking, and talk through your real microphone. Your voice is played into the drone's simulated microphones from your avatar's position, mixed with propeller noise. Friends like "Adrien" are other avatars you can spawn.
3. **Skills are ROS 2 actions, the agent is one tool-calling LLM.** Each skill (explore, search, goto, inspect, follow, come_to, monitor, say, photo, return, land) is a ROS 2 action with a typed goal, live feedback and a result. The agent's tools map one to one onto these actions, plus memory queries and "remember this person".
4. **Worlds are generated, never hand-made.** A seeded generator builds a random furnished apartment (and, for outdoor scenarios, a house with a roof and garden) with a ground-truth manifest: rooms, objects, people.
5. **The Mission Console is the demo.** A web app showing the drone's camera with detections, a 3D view of what the drone has mapped and remembered, the conversation, the agent's current plan, and photo reports. Gazebo's own window gives the third-person view.
6. **One command to run anything:** `just demo`, `just scenario bananas --seed 7`, `just test failsafe`, `just bench`.
7. **First milestone that feels like Maverick (end of stage S4, about 11–13 weeks at 10 h/week [E]):** "We're sitting down. Can you check the kitchen for bananas and come back and tell me?" in a random apartment, by voice, with the drone answering out loud and showing a photo.
8. **Outdoor flight is in scope, under ideal conditions** (owner decision, 2026-11-11): no wind, bright daylight, good GPS. It needs GPS and height, which the indoor setup (optical flow + short-range rangefinder) does not cover, so the drone gets a GPS module. Outdoor is stage S6, after the indoor demo works. Wind, rain and low light are out of scope for now and can be added later as harder scenario settings. Drone regulations still apply to real outdoor flights [NV].

---

## 1. Hero scenarios

These are the owner's examples, turned into concrete scenarios. Each becomes a scored test (§8) and a demo act (§7).

| # | What you say | What Maverick must do | New capability beyond the study |
|---|---|---|---|
| H1 | "Explore my flat, find the kitchen, tell me if there's bread on the table, then come back to me and report." | Explore, recognise the kitchen, inspect the table, return to your current position (you may have moved), speak the answer, show a photo | Returning to a person, not a place |
| H2 | "We're sitting down. Can you go check in the kitchen if there are bananas for us, and come back and tell us?" | Same as H1, but knowing the kitchen from earlier, and "us" means the group | Using memory from earlier in the session |
| H3 | "Follow me on my right shoulder while I work." | Hold a position to your right, keep a safe distance, follow through doorways | Safety limit: "shoulder" is mapped to about 1 m to the right and slightly behind, never closer than the study's 1.0 m minimum (C3). The executive enforces it, not the LLM |
| H4 | "That's my friend Adrien. Can you say hi?" | Work out who "that" is (pointing, gaze or the only other person), remember his appearance under the name Adrien, fly to a polite distance, face him, say hi. Hold a short conversation | **Person enrolment and re-identification**; conversational speech |
| H5 | "You see that roof up there?" (pointing) "Go check around it, send me a picture and tell me if there's any damage." | Resolve the pointing gesture to a roof far away, plan a safe viewpoint above it, orbit, take photos, judge damage, report with photos | **Outdoor flight**, long-range pointing, damage assessment |

---

## 2. Architecture of the simulation

### 2.1 Containers and networks

```
                   ┌────────────────────────────── referee ────────────────────────────────┐
                   │ scenario runner · ground-truth reader · scorer · fault injector       │
                   │ avatar controller (you, Adrien, scripted people) · voice emitter       │
                   └──────┬──────────────────────────────────────────────┬─────────────────┘
          ground truth,   │ net: world (sim + referee only)              │ tc netem on the
          actor control   ▼                                              ▼ drone↔host link
 ┌────────────────────── sim ──────────────────────┐   ┌──────────── bridge ─────────────┐
 │ Gazebo Harmonic: generated world, actors        │   │ sensor streaming (H.264, audio)  │
 │ drone model: RGB cam, (depth), IMU, flow, range │──►│ setpoint relay, staleness drop   │
 │ simulated mic array (audio mixer node)          │   │ same code as the real bridge     │
 │ PX4 SITL (failsafes, flow-based hold)           │◄──│                                  │
 └─────────────────────────────────────────────────┘   └───────────────┬─────────────────┘
                                                                       │ net: wifi (lossy)
 ┌──────────────────────────────────────── host (GPU) ─────────────────▼─────────────────────────┐
 │ perception: YOLOE-26, pose/person re-ID, VLM (local or cloud), pointing resolver              │
 │ state: VIO/SLAM → map (OctoMap/nvblox) → scene graph memory (rooms, objects, people)          │
 │ executive: skill action servers (behaviour trees) · safety checks on every setpoint            │
 │ agent: tool-calling LLM · voice: denoise → wake word → ASR · TTS                              │
 │ gateway: one websocket API for the console                                                     │
 └──────────────────────────────────────────────┬─────────────────────────────────────────────────┘
                                                │ net: ui
                                   ┌────────────▼────────────┐
                                   │ Mission Console (browser)│  your mic, keyboard, gamepad
                                   └──────────────────────────┘
```

**The isolation rule.** The `host` and `bridge` containers are not on the `world` network, so they cannot see ground truth: object positions, room labels, people's identities. This is enforced by Docker networks and separate Zenoh routers, not by convention. It keeps the tests honest: the drone only knows what it sensed.

The console talks to the host gateway (what the drone thinks) and to the referee (what you do as an avatar, and the director controls in §7).

### 2.2 The `sim` container

- **Gazebo Harmonic + PX4 SITL v1.17**, talking over PX4's Gazebo bridge (gz transport, not MAVLink) [F].
- **Drone model:** starts from PX4's `x500_depth` [F, study]. We derive a `maverick` model that matches the planned hardware (study §7, tier A):
  - Wide-angle RGB camera, IMU, barometer, rangefinder, downward optical flow, GPS. An optional depth camera for tier B.
  - PX4 ships an optical-flow setup for Gazebo Classic; a Gazebo (gz) flow model was not confirmed [NV]. If none exists, we write a small flow sensor system that derives flow from the true velocity and height, with noise and a texture-quality term.
  - GPS is fitted but only usable in outdoor worlds; indoors it has no fix, as in reality.
  - Size and mass of a 3.5–5" ducted frame, not the large x500.
- **Simulated microphone array:** a node that takes "voice emissions" from the referee (audio clip + 3D position) and outputs 4 channels as the real mic array would: attenuated by distance, delayed per mic so the direction can be estimated, mixed with propeller noise scaled by motor speed [E]. Placeholder noise is synthetic until experiment X8 records real propellers.
- **Speaker:** the drone's speech is published with the drone's position; the console plays it, louder when the camera is near the drone.

### 2.3 The `bridge` container

Identical to the code that will run on the real bridge board: forward camera (H.264), IMU and PX4 telemetry up; forward setpoints down; drop setpoints older than 200 ms. Nothing else (study §2.5).

### 2.4 The `host` container

ROS 2 Jazzy, Python nodes unless speed demands C++. GPU through the NVIDIA Container Toolkit.

| Component | Job | First implementation |
|---|---|---|
| `state` | Pose, map | Ground-truth-free VIO later; start with PX4's own estimate (flow + simulated noise) so mapping works on day one. OctoMap first, nvblox if Blackwell works |
| `perception` | Objects, people, poses | YOLOE-26 with a text-prompt list driven by the agent ("bananas"); a pose model for people (keypoints give pointing direction); appearance embeddings for re-identification |
| `memory` | Scene graph | Rooms (from map segmentation + VLM labels on keyframes), objects with 3D position, last-seen time and a photo crop, people with name, embedding and last-seen position. Persists across missions in a session |
| `pointing` | "That one" | Shoulder→wrist ray from pose keypoints, cast into the map; if the hit is ambiguous or beyond the map, ask the VLM/Molmo to point in the image along that direction |
| `executive` | Skills | Each skill is a ROS 2 action server running a behaviour tree (py_trees [NV: Jazzy packaging]). Every motion goes through the safety checks (speed, geofence, distance to people, collision) |
| `agent` | Understanding and dialogue | One tool-calling LLM with a persona. Tools = skill actions + `memory.query` + `person.remember` + `say` + `photo.send`. Local or cloud behind one interface (study §2.2) |
| `voice` | Hearing and talking | Beamforming → denoise → wake word ("Maverick") → ASR (Parakeet or cloud) · TTS (Kokoro) |
| `gateway` | Console API | FastAPI + websockets: one curated event stream (transcript, agent steps, skill progress, pose, detections, photos, map updates) and one input channel (your audio, text) |

### 2.5 How H2 flows through the system

```
you (console mic) ──► referee: play at avatar position ──► sim mic array (+ prop noise)
  ──► bridge ──► host voice: beamform → denoise → wake word → ASR
  ──► agent: "check kitchen for bananas, come back, tell us"
        memory.query("kitchen") → known, room 3
        search(object="bananas", region="kitchen", budget_s=120)   ← ROS 2 action
            executive: goto(room 3) → scan pattern → YOLOE "banana" → VLM verify → photo
            feedback: "in kitchen", "candidate on counter", "confirmed (0.91)"
        come_to(person="operator")   ← last known position + re-ID, may need a short search
        say("Yes, three bananas on the counter. Sending you a photo.") + photo.send
  ──► console: speech audio, transcript, photo card
referee: was the claim true? did it end within 1.5 m of the speaker? any collision?
```

---

## 3. Worlds

### 3.1 Apartment generator (indoor)

Our own Python generator, deterministic from a seed:

1. **Room programme:** pick a flat type (studio to 4 rooms) and room list: living room, kitchen (or open kitchen), bedroom(s), bathroom, hallway, office.
2. **Floor plan:** subdivide a footprint (rectangles, L-shapes) into rooms by recursive splitting with adjacency rules (bathroom off the hallway, kitchen next to the living room). Doors ≥70 cm (study §5), ceilings 2.4–2.7 m [E].
3. **Geometry:** walls, floors and doorways as SDF boxes and meshes, with random wall and floor textures. Floors always get some texture so optical flow works, but some seeds use low-texture floors to test the flow-quality risk (study §8).
4. **Furnishing:** per-room rules place furniture from an asset library (kitchen: counter, fridge, table, chairs; living room: sofa, table, TV). Rule-based, not learned.
5. **Scenario objects:** bread, bananas, keys, mugs, clocks set to a time, placed on surfaces according to the scenario (likely places and unlikely ones).
6. **Lighting:** day/evening/lamp-only presets.
7. **Manifest:** a JSON file with every room polygon and label, every object with class and pose, every door. Only the referee reads it.

Tuning seeds and held-out seeds are separate ranges (study §5). The generator is tested on its own: every room reachable, doors wide enough, no overlapping furniture.

**Assets:** start with simple, permissively licensed models (Gazebo Fuel [NV: check each licence], plus our own simple meshes). Track every asset's licence in `sim/assets/LICENSES.md`. Infinigen Indoors (BSD licence, exports OBJ/FBX/USD, built for photoreal scenes) [F] is the source for the later Isaac Sim stage; it has no confirmed Gazebo export [F: none found], so it is not used for Gazebo.

### 3.2 Outdoor world (for H5)

A house exterior on a lot: pitched roof with tiles, gutters, chimney; garden; trees; a fence. The generator randomises roof shape and places **damage**: missing or shifted tiles, a cracked tile, a blocked gutter, debris, moss — or no damage at all, so "no damage found" is also tested. Conditions are ideal by assumption: no wind, bright daylight, clear sky, GPS with good accuracy. The manifest lists each damage item and its position.

- **Indoor ↔ outdoor:** outdoor scenarios start the drone outside (for example in the garden next to you). Flying from inside to outside through a door or window is not planned.
- **Estimation outdoors:** EKF2 uses GPS and the barometer. Optical flow stops helping at roof height and the short rangefinder runs out of range [NV: check the chosen sensor's range], so neither is relied on above a few metres.
- **Later, harder settings:** wind (Gazebo wind effects [NV: check gz and PX4 model support]), low sun, overcast light. These are scenario options that default to off.

### 3.3 People (actors)

- Gazebo actors with skeletal animations from COLLADA or BVH files [F]: walk, idle, sit, wave, point, talk.
- Gazebo Sim has a `FollowActor` system, so actor pose and animation components can be driven by a system at runtime [F]. We write one small C++ system plugin, `ActorPuppet`, that takes a target pose, speed and animation name from a topic. That single plugin serves scripted people, Adrien and your avatar [H: feasibility of switching animations live is the first thing to prototype].
- Fallback if live animation switching fails: a simple articulated mannequin whose arm joints we control directly. Less pretty, fully controllable.
- Each person has a distinct appearance (clothes, colours) so re-identification is possible. Simulated faces are too low-detail for face recognition, so recognition uses whole-body appearance [H].

---

## 4. Simulating the hard parts honestly

| Real-world problem | How the simulation represents it |
|---|---|
| Wi-Fi latency, jitter, loss, outages | `tc netem` profiles between `bridge` and `host`: `lab`, `apartment`, `bad`, plus scripted outages (study F1–F2) |
| Propeller noise on the mics | Mic mixer with motor-speed-dependent noise; real recordings from X8 replace the synthetic noise |
| Camera image quality | Gazebo rendering with random lighting, motion blur and noise added in the bridge; Gazebo images look simpler than real ones, so real-footage gates (study G1) still decide |
| Pose estimation errors | No ground-truth pose on the host. Start with PX4's flow-based estimate, then VIO |
| Model latency | Real models in the loop, real cloud calls, the simulation runs in real time |
| People behaving unpredictably | Scripted paths with random pauses and detours; you, live, in the demo |
| Faults | The referee kills processes, injects VIO jumps, garbles agent output (study F3–F8) |

Time: the simulation runs at real time with PX4 lockstep [NV], because LLM, VLM and cloud latency are part of what we test. Faster-than-real-time runs are only for the failsafe suite, which has no models in the loop.

---

## 5. Repository layout and languages

```
maverick/
├── justfile                      # every command (just demo, just test, ...)
├── docker/
│   ├── compose.yaml              # services: sim, bridge, host, referee, console; profiles: demo, headless, ci
│   ├── sim.Dockerfile            # Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, PX4 v1.17
│   ├── host.Dockerfile           # + CUDA, model servers
│   └── netem/                    # link profiles
├── ros2_ws/src/
│   ├── maverick_interfaces/      # skill actions, messages, services (the contract)
│   ├── maverick_bridge/          # runs on the real drone too
│   ├── maverick_state/           # pose, map
│   ├── maverick_perception/      # detector, people, pointing, VLM client
│   ├── maverick_memory/          # scene graph
│   ├── maverick_executive/       # skill action servers, safety checks
│   ├── maverick_agent/           # LLM agent, tools, persona
│   ├── maverick_voice/           # beamforming, denoise, wake word, ASR, TTS
│   └── maverick_gateway/         # console API
├── sim/
│   ├── worldgen/                 # apartment and outdoor generators, manifests
│   ├── models/                   # maverick drone, props, people
│   ├── plugins/                  # ActorPuppet, flow sensor (C++ gz systems)
│   ├── audio/                    # mic mixer, noise recordings
│   └── referee/                  # scenario runner, scorer, fault injector
├── scenarios/                    # YAML: world seed, people, utterances, success criteria
├── console/                      # web app (TypeScript, Vite, three.js)
└── docs/
```

- **Python 3.12** for ROS nodes, the agent, the generator and the referee: fastest to build, all model tooling is Python. Lint and format with `ruff`, tests with `pytest`, type checks with `pyright` [NV: pick versions in S0].
- **C++** only for Gazebo plugins and anything that proves too slow.
- **TypeScript** for the console.
- `maverick_interfaces` is written first and changed carefully: it is the contract between the agent, the executive and the console.

---

## 6. Running it

| Command | What it does |
|---|---|
| `just build` | Build the Docker images and the ROS workspace |
| `just demo` | Start everything with the Gazebo window, open the Mission Console, random apartment |
| `just demo --world outdoor --seed 3` | Same, with a chosen world |
| `just scenario h2_bananas --seed 7` | Run one scored scenario headless, print the score, save a recording |
| `just test unit` | Python unit tests (generator, safety checks, agent tools with a fake LLM) |
| `just test failsafe` | Study suite F1–F8, headless, no models in the loop, in CI on every change |
| `just test capability` | Hero scenarios and C1–C11 on held-out seeds, real models (nightly) |
| `just replay <run>` | Open a saved run (MCAP recording) in Foxglove |
| `just bench` | VRAM and latency benchmark (study X4) |

- **Docker Compose profiles:** `demo` (GUI + console), `headless` (no windows), `ci` (headless, failsafe suite).
- **Gazebo window from a container:** X11 socket forwarding to the host's Wayland session through XWayland [NV: test in S0]. The console works in any browser and needs no forwarding.
- **Recording:** every run writes an MCAP file of all topics. Foxglove, documented in the ROS 2 docs for visualising ROS 2 data [F], opens them for debugging.
- **LLMs in CI:** per-change CI uses recorded LLM responses ("cassettes") so tests are fast, free and deterministic. Nightly runs use the real models.

---

## 7. Live demo: the Mission Console

### 7.1 Screen

```
┌──────────────────────────────┬───────────────────────────────┬──────────────────────────────┐
│ DRONE CAMERA                 │ MAVERICK'S MIND (3D)          │ WORLD (third person)         │
│ live feed + detection boxes  │ map built so far, frontiers,  │ chase camera or top-down,    │
│ + person skeletons + labels  │ rooms coloured by label,      │ you and friends as avatars,  │
│ click = point here           │ objects as pins with photos,  │ click = walk here            │
│                              │ people, planned path          │                              │
├──────────────────────────────┴───────────────────────────────┴──────────────────────────────┤
│ CONVERSATION                               │ AGENT PLAN                │ STATUS              │
│ you: "check the kitchen for bananas"       │ ✓ memory: kitchen = room 3│ battery 78% · link  │
│ maverick: "On it."                         │ ▶ search bananas (42 s)   │ good · mode: search │
│ maverick: "Three bananas on the counter."  │ ○ come back to you        │ VRAM 9.1/16 GB      │
│ [photo card]                               │ ○ report                  │ [director ▾]        │
│ (hold SPACE to talk · or say "Maverick")   │                           │                     │
└────────────────────────────────────────────┴───────────────────────────┴─────────────────────┘
```

- **Drone camera:** what the drone sees, with what it understood drawn on top.
- **Maverick's mind:** a three.js view of the drone's own map and memory, built live as it explores. This is the "spatial reasoning" made visible, and the most impressive part of the demo. It shows only what the drone knows, never ground truth.
- **World:** the Gazebo view (in the Gazebo window at first; streamed into the console later).
- **Agent plan:** the tool calls as a live checklist, with skill feedback.

### 7.2 Interaction

- **Your avatar:** WASD or a gamepad to walk; click in the world or drone view to point there (your avatar plays the pointing animation towards it); sit/stand keys.
- **Voice:** hold Space or say "Maverick". Your real voice goes into the simulation at your avatar's position, so standing far away or behind the drone really makes hearing harder. Typed commands also work, for quiet rooms.
- **Friends:** "Add person" spawns Adrien (or anyone) as an avatar; you can switch which avatar you control and speak as them.
- **Director menu:** move or remove objects (take the bananas away), cut the Wi-Fi for 5 s, drain the battery. (Wind outdoors comes later, with the harder settings in §3.2.) These show the drone adapting and the failsafes working, live.

### 7.3 Demo script (target for S7)

1. **Wake up in a new flat:** "Maverick, explore the flat." The mind view fills in, rooms get labels.
2. **H1/H2:** "We're sitting down, check the kitchen for bananas." Answer with a photo. Director removes them; ask again; it says they're gone.
3. **H4:** spawn Adrien, point at him: "That's my friend Adrien, say hi." Short chat.
4. **H3:** "Follow me on my right." Walk through two rooms.
5. **Robustness:** cut the Wi-Fi mid-flight: it holds, then lands; restore and resume.
6. **H5 (outdoor world):** point at the roof, get photos and a damage report.

---

## 8. Scenarios and scoring

Scenarios are YAML files, so a new test is data, not code:

```yaml
id: h2_bananas
world: { type: apartment, seed_set: heldout }
people:
  - { id: operator, controlled_by: script, start: living_room.sofa, animation: sit }
  - { id: guest, start: living_room.sofa, animation: sit }
setup:
  - place: { object: bananas, count: 3, in: kitchen, on: counter, probability: 0.7 }   # absent 30% of runs
script:
  - at: 5s
    say: { who: operator, text: "Maverick, we're sitting down. Can you check the kitchen for bananas and come back and tell us?" }
success:
  - spoken_answer_matches_truth: bananas_present          # yes/no and roughly how many
  - photo_sent_shows: bananas                              # only when present
  - ends_within_m: { of: operator, distance: 1.5 }
  - time_limit_s: 240
  - collisions: 0
```

- `say` uses TTS voices for scripted people (several voices and accents [E]) and real recordings in a subset of runs.
- The referee checks answers against the manifest. Free-text answers ("three bananas on the counter") are checked by a small LLM judge against the truth, with a rule-based check for yes/no.
- **CI tiers:** every change: unit tests + failsafe suite + two short scenarios with LLM cassettes (target under 15 min [E]). Nightly: all hero scenarios and C1–C11 on 10 held-out seeds each, real models. Results go to a simple dashboard (pass rate per capability over time).

---

## 9. Build order

Each stage ends with something that runs and a measurable exit test. Durations at about 10 h/week [E].

| Stage | Content | Exit test |
|---|---|---|
| **S0 Foundations** (1–2 wk) | Docker, NVIDIA toolkit, images, `justfile`, PX4 SITL in Gazebo, Gazebo window from a container, CI skeleton | 20/20 take off → hover → land runs from a script (study P0) |
| **S1 Drone and link** (2 wk) | `maverick` drone model with flow + range, bridge container, netem profiles, PX4 failsafe parameters, referee fault injection | Failsafe suite F1–F6 passes 50/50 randomised runs |
| **S2 Worlds** (2 wk) | Apartment generator v1, manifests, asset library, generator tests | 100 seeds generate, all valid; drone flies in each |
| **S3 See and map** (3 wk) | Map, frontier exploration, YOLOE, room labelling, scene graph, first console with camera + mind view | C1, C2, C6 on held-out seeds; mind view shows rooms and objects live |
| **S4 First real demo: bananas** (3 wk) | Skill actions, executive, agent with tools, gateway, typed commands then voice (no noise first, then the mic mixer), TTS, `ActorPuppet` with sitting/standing operator | H1 and H2 pass 8/10 held-out seeds, by voice; demo acts 1–2 work live |
| **S5 People** (3 wk) | Pose, re-ID, pointing resolver, follow, come_to, person enrolment, conversation, avatar control in the console | H3, H4, C3, C4, C10 meet thresholds; demo acts 3–4 |
| **S6 Outdoor** (2–3 wk) | Outdoor generator with roof damage, GPS, ideal conditions (no wind, daylight), inspect orbit, damage report | H5: correct damage / no-damage call in ≥8/10 seeds, photos show the damage |
| **S7 Demo polish** (2 wk) | Director menu, world view in the console, demo script, recorded demo video | Full demo script runs twice in a row with no restart |
| **S8 Photoreal (optional)** | Isaac Sim 6.x + Infinigen scenes or a scan of a real flat (study §3.1) | Perception scores on photoreal scenes |

The first time it "feels like Maverick" is the end of S4, about 11–13 weeks of part-time work [E], sooner if S2 and S3 overlap. The fastest throwaway path to a first spoken mission is the study's vertical slice (about 3 weeks), and S0–S1 are built so that slice can be shown along the way.

---

## 10. Compute during the demo

| Process | VRAM [E] |
|---|---|
| Gazebo rendering (drone camera + Gazebo window) | 1–2 GB |
| YOLOE-26 + pose model | 1–2 GB |
| Re-ID embeddings | <0.5 GB |
| OctoMap (CPU) or nvblox | 0–1 GB |
| ASR (Parakeet) + TTS (Kokoro) + denoiser | 1–2 GB, or CPU |
| VLM + agent LLM | **cloud** by default during the demo |
| **Total** | **about 4–8 GB**, leaving room to bring the VLM local for the S3/S4 benchmarks |

Conversation feel: the target is under about 2 s from end of speech to the start of the drone's reply for simple exchanges [E]. A quick "On it" acknowledgement is spoken immediately while the agent plans, so long missions never feel frozen.

---

## 11. Risks and decisions for the owner

| Risk | Impact | Plan |
|---|---|---|
| **Outdoor flight (H5)** | In scope under ideal conditions. Optical flow and a short-range rangefinder don't work at roof height, so outdoors relies on GPS. Real days are rarely windless, and a small ducted indoor drone may struggle in wind [E]. EU drone rules apply outdoors [NV] | Simulate it at S6 with no wind. Hardware: add a GPS module (small cost [E]). At G2, check the frame copes with at least light wind, even though sim assumes none. Check the regulations before any outdoor flight |
| Live actor animation switching in Gazebo | Without it, pointing and sitting look fake or fail | Prototype `ActorPuppet` in week 1 of S4; mannequin fallback |
| Gazebo flow sensor support | Needed to test the no-companion design | Own flow sensor system if none exists [NV] |
| Gazebo looks plain | Perception works better in sim than reality | Real-footage gate (study G1); Isaac Sim stage S8 |
| Following "on your shoulder" | A drone near a head is dangerous | Executive minimum distance 1.0 m, enforced in code and tested |
| Person enrolment and privacy | Storing appearances of guests | Stored locally only, list and delete from the console; real hardware: only on the operator's request |
| Asset licences | Can't publish a demo video with unclear assets | Licence file per asset from day one |
| Cloud LLM latency | Conversation feels slow | Immediate acknowledgements; small fast model for chit-chat, bigger one for planning [E] |

**Decisions needed before S0:**

1. ~~Confirm outdoor flight (H5) is in scope.~~ Decided 2026-11-11: yes, assuming no wind and bright daylight.
2. **Which cloud LLM provider for the agent and VLM by default?** The original brief mentioned OpenAI; the interface stays provider-neutral either way.
3. **Maverick's voice and personality:** name, tone (helpful and brief by default), and whether it should make small talk with guests.

---

## Sources

Checked on 2026-11-11, in addition to the study's sources.

- Gazebo Sim `FollowActor` system: https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1FollowActor.html , https://gazebosim.org/api/sim/9/classgz_1_1sim_1_1systems_1_1FollowActor.html
- Gazebo actors (COLLADA and BVH animations): https://gazebosim.org/docs/all/actors/
- PX4 simulation and the Gazebo bridge: https://docs.px4.io/v1.17/en/simulation/
- Infinigen Indoors (formats, licence, exports): https://zuoym15.github.io/publications/Infinigen_indoors , https://arxiv.org/abs/2406.11824v1 , https://80.lv/articles/this-tool-generates-photorealistic-procedural-interiors
- Foxglove for ROS 2 (official ROS docs): https://docs.ros.org/en/jazzy/How-To-Guides/Visualizing-ROS-2-Data-With-Foxglove-Studio.html
- Rerun with ROS 2 point clouds (2024 example, considered and not chosen): https://medium.com/@tfoldi/ros-2-live-depth-cam-point-cloud-visualization-with-rerun-66534561557a

# Maverick: architecture and feasibility study (phase 1)

Status: draft for review, written 2026-11-10 and revised on 2026-11-10 and 2026-11-11 with the owner's answers (section 10). Simulation and demo plan: [simulation-plan.md](simulation-plan.md). No implementation yet.

**How claims are labelled**

| Tag | Meaning |
|---|---|
| **[F]** | Verified fact, with a source link in the [Sources](#sources) section. Checked on 2026-11-10. |
| **[E]** | Estimate. Engineering judgement or a rough calculation, not yet measured. |
| **[H]** | Hypothesis. Something we believe but must test (see [Experiments](#9-research-experiments)). |
| **[NV]** | Not verified in this study. Recalled from general knowledge; check it before relying on it. |

Facts about the host machine come from this machine on 2026-11-10: Ubuntu 26.04.1 LTS, RTX 5070 Ti with 16,303 MiB VRAM, NVIDIA driver 595.99.02, 16 CPU threads, about 30 GB RAM, Docker not installed.

---

## 0. Summary

1. **The project is feasible in simulation on this PC, with one change: use Gazebo first, not Isaac Sim.**
   - Isaac Sim's minimum tier already assumes 16 GB of VRAM for the simulator alone [F].
   - The RTX 5070 Ti is not in its listed GPU tiers, and Ubuntu 26.04 is not a listed OS [F].
   - Running Isaac Sim *and* the local AI models on one 16 GB card is the biggest compute risk [E].
   - Gazebo Harmonic with PX4 SITL is the CI-tested, well-trodden path [F] and leaves most of the GPU free for the models.
   - Isaac Sim becomes an optional later stage for photoreal perception testing.
2. **Run the robotics stack in Docker, on Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic.**
   - The host runs Ubuntu 26.04. There, the newest ROS 2 (Lyrical) pairs with Gazebo Jetty [F], while PX4 only CI-tests Harmonic [F].
   - Containers avoid fighting the host OS, and they also let us simulate the drone and the GPU host as separate machines joined by a lossy link.
3. **Language models never fly the drone.**
   - LLMs and VLMs choose among a small, typed set of *skills*: explore, go to a place, search for an object, follow a person, inspect, return, report.
   - Deterministic robotics code carries the skills out: VIO/SLAM, a voxel map, frontier exploration and a trajectory planner.
   - PX4 handles attitude and position control and the failsafes.
4. **Safety rule: the Wi-Fi link and the GPU host are optional for safe flight.**
   - Everything needed to hold position or land runs on the flight controller itself: PX4 holds position on an optical-flow sensor and a rangefinder, and its offboard-loss failsafe holds then lands when the host goes quiet.
   - Everything else is offboard, including VIO and obstacle checks (owner decision, section 10). The drone carries only the flight controller, its sensors and a small bridge board that streams camera and microphone data over Wi-Fi. No AI runs on the drone.
5. **Keep the model count small:** five to six models (table below). Each extra model costs VRAM, latency and integration work.
   - Cloud models are allowed. Use them as a pressure valve: whatever doesn't fit in 16 GB of VRAM (most likely the VLM) moves to the cloud, behind the same interface as the local version.
   - Anything that must keep working without the network (hover, land) never depends on the cloud.
6. **The drone must hear its operator through its own microphone.** Propeller noise makes this the hardest perception problem in the project [H]. It gets its own design (section 2.7), its own test (V1) and its own experiment (X8).
7. **Maverick must work in any flat, not just the owner's.** The drone knows nothing about a home in advance: it explores, maps and works out the layout itself. Development and tests use randomly generated apartment layouts, with a strict split between layouts used for tuning and held-out layouts used to measure results (section 5). The owner's flat is just one more unseen test case.
8. **A DIY drone of about €400–650 is enough [E].** Because all computing is offboard, the drone needs only a frame, a PX4 flight controller, flow and range sensors, a camera, a mic array and a small bridge board (section 7).
   - Integrated research drones such as ModalAI Starling 2 (about $2,950 [F]) carry onboard compute we don't need.
   - Don't buy hardware until the simulation and real-video perception milestones pass (decision gate G2 in section 4).

**Proposed model set (initial picks, each to be benchmarked):**

| Role | First choice | Alternative | Rate |
|---|---|---|---|
| Open-vocabulary detection and segmentation | YOLOE-26 (Ultralytics) [F] | SAM 3 for offline or slow, high-quality masks [F] | 10–30 Hz |
| Vision-language reasoning (scene questions, rooms, "is there bread?") | Qwen3.5-9B, natively multimodal [F] | Gemma 4 E4B (smaller) [F] or Qwen3-VL-8B [F] | Event-driven, 0.2–2 Hz |
| Pointing and grounding ("what am I pointing at", precise object location) | Molmo 2 (4B/8B) [F] | Qwen3.5 grounding output | Event-driven |
| Speech-to-text (English only), fed by the drone's microphone after noise suppression | Parakeet, English model [F] | Whisper large-v3-turbo [F] or Gemma 4 E2B/E4B native audio [F][H] | Streaming |
| Keyword spotting ("Maverick", "stop", "land") | Small keyword-spotting model on the host [NV: pick in X8] | Push-to-talk on a phone, as a fallback | Always on |
| Speech enhancement (propeller noise removal) | Beamforming on a mic array, then a neural denoiser [H, see X8] | Cloud ASR that tolerates noise [H] | Streaming |
| Text-to-speech | Kokoro-82M [F] | Piper [NV] | On demand |
| Metric depth, only if the camera has no stereo depth | Depth Anything 3 metric [F] | Use the stereo sensor directly (preferred) | 10–30 Hz |
| Cloud offload: VLM when VRAM is short, plus hard reasoning | OpenAI GPT-6.1 Terra or Sol (vision input) [F] | Local only, with smaller models | As needed |

Person detection and pose for follow-me and pointing: start with an Ultralytics pose model [NV: check the YOLO26 pose variants].

---

## 1. Challenging the brief

| Assumption in the brief | Challenge | Recommendation |
|---|---|---|
| "Isaac Sim, because it's the best simulator" | Its minimum spec is a 16 GB GPU for the simulator alone [F]. Ours is 16 GB and must also host the models. 5070 Ti and Ubuntu 26.04 are not in the requirement tables [F]. Forum reports show Blackwell driver crashes on 5.1 [F]. | Gazebo Harmonic first. Isaac Sim later, for perception realism only, with the VLM moved to the cloud during those runs if VRAM runs out. |
| "Offboard GPU inference over Wi-Fi" | Fine for semantics. Not acceptable for state estimation or collision avoidance. A Wi-Fi dropout in a hallway must not mean "no position estimate". | Onboard: only PX4 with optical flow and a rangefinder, which can hold and land by itself, plus a small bridge board that streams sensors. Offboard: everything else, including VIO and obstacle checks (owner decision). When the link drops, the drone holds on flow and then lands. |
| "Use many modern open-source models" | Each model costs VRAM, latency and failure modes. Several jobs (mapping, avoidance, following) are solved better by classical algorithms. | One VLM, one open-vocabulary detector, one pointing model, ASR, TTS. Classical SLAM, mapping and planning. |
| "Voice commands to the drone" | Propeller noise makes onboard speech capture hard [H]. An external mic would be easier, but the owner requires the drone to hear them directly (section 10). | Onboard mic array with beamforming and denoising (section 2.7). Keyword spotting runs on the host: during a link outage the drone already holds and lands, and the RC kill switch is the manual emergency stop. A phone push-to-talk app is the fallback, not the main path. Measure feasibility early (X8). |
| "Read a clock" | VLMs are reported to be unreliable at reading analog clocks [H, see experiment X3]. | Have the drone approach for a close-up, then read. Test digital and analog separately. Report uncertainty. |
| "Pondering whether to investigate further" | Unbounded LLM deliberation is slow and unsafe. | The executive computes the costs (remaining frontier area, battery, elapsed time, prior likelihood of the object in each room). The LLM makes a bounded go/no-go choice. Hard limits are enforced in code. |
| "Budget-conscious drone" | Small indoor drones with onboard compute and depth sensing are payload-limited and short on flight time [E]. | Keep the drone light: no onboard compute beyond the flight controller and a small bridge board. Choose hardware after simulation, with a measured payload list. |
| "Use OpenAI models" | Cloud calls add latency and cost. The owner accepts in-home video going to the cloud. | Local where the GPU can handle it, cloud wherever it relieves VRAM. Every model sits behind one interface, so local and cloud can be swapped by configuration. Nothing flight-critical uses the cloud. |
| "Explore, map and understand the house" | Tuning on one known flat would overfit: the system would learn that flat, not how to understand flats. | No prior map or room list. Develop on randomly generated layouts and score only on held-out layouts the system has never seen (section 5). |

---

## 2. Technical architecture

### 2.1 Layers and timing

```
                  ┌──────────────────── OFFBOARD HOST (RTX 5070 Ti) ─────────────────────┐
 drone mic audio  │  ASR ─► Agent (LLM/VLM) ─► Task executive (behaviour tree)            │
 ───────────────► │           ▲  tool calls      │  skills: explore / goto / search /     │
 ◄─────────────── │  TTS ◄────┘                  │  follow / inspect / return / report    │
 to drone speaker │                              ▼                                        │
                  │  Semantic memory / scene graph ◄── Perception (YOLOE-26, VLM, Molmo)  │
                  │  (rooms, objects, timestamps)                                         │
                  │  VIO / SLAM on camera + IMU ─► pose (also sent back to PX4)           │
                  │  Voxel/ESDF map (nvblox or OctoMap) ◄── depth + pose                  │
                  │  Exploration (frontiers) + global planner ─► waypoint/trajectory goals│
                  └───────────────▲──────────────────────────────┬────────────────────────┘
               compressed images, │ Wi-Fi (Zenoh)                │ setpoints, VIO pose
                  pose, mic audio │ lossy, variable latency      ▼
                  ┌───────────────┴──────── ONBOARD BRIDGE (small Linux board, no AI) ────┐
                  │ Camera (+ optional depth) ─► H.264 stream to the host                 │
                  │ Mic array ─► speech-gated audio stream to the host                    │
                  │ Relays PX4 telemetry up and host setpoints down                       │
                  │ Drops setpoints older than 200 ms; no other logic                     │
                  └───────────────┬───────────────────────────────────────────────────────┘
                                  │ serial, uXRCE-DDS (not over Wi-Fi)
                  ┌───────────────▼──────── FLIGHT CONTROLLER (PX4) ──────────────────────┐
                  │ EKF2, rate/attitude/position control, geofence, offboard-loss failsafe│
                  │ Optical flow + rangefinder ─► EKF2: holds position with no host       │
                  │ Offboard-loss failsafe: host or link lost ⇒ hold ⇒ land               │
                  │ RC kill switch, battery failsafe                                      │
                  └───────────────────────────────────────────────────────────────────────┘
```

| Loop | Where it runs | Rate [E] | Can depend on Wi-Fi? |
|---|---|---|---|
| Rate and attitude control | PX4 | ≥250 Hz | Never |
| Position hold (EKF2 + optical flow + rangefinder) | PX4 | IMU ≥200 Hz, flow 20–50 Hz | Never |
| Link-loss failsafe (hold, then land) | PX4 | Timeout about 0.5–1 s | Never |
| VIO / SLAM pose, fused into EKF2 as external vision | Host | 20–30 Hz | Yes: EKF2 falls back to flow |
| Trajectory setpoints and obstacle checks | Host | 10–50 Hz | Yes: link loss ⇒ PX4 holds, then lands |
| Keyword spotting ("stop", "land") | Host | Always on | Yes: link loss ⇒ PX4 holds, then lands |
| Mapping (voxels/ESDF) | Host | 5–30 Hz | Yes: degraded mode is "hold" |
| Exploration and global planning | Host | 1–5 Hz | Yes |
| Detection (YOLOE-26) | Host | 10–30 Hz | Yes |
| VLM / pointing | Host | Event-driven, ≤2 Hz | Yes |
| Agent / LLM | Host (or cloud) | Seconds per decision | Yes |
| Full speech recognition (ASR) | Host (or cloud) | Streaming | Yes |

### 2.2 Interfaces

- **PX4 ↔ bridge:** uXRCE-DDS over serial, using PX4's ROS 2 interface [F: PX4 docs]. Offboard mode needs setpoints or `OffboardControlMode` heartbeats above 2 Hz [F].
- **Bridge ↔ host:** ROS 2 over `rmw_zenoh`. The bridge only forwards; it runs no planning or perception.
  - Zenoh is designed for lossy wireless links, PX4 v1.17 has Zenoh compatibility work [F], and Arm publishes Zenoh-over-Wi-Fi tuning guidance [F].
  - Compress images (JPEG for perception frames, H.264 for the monitoring stream).
  - In one study, raw images took 1401 ms round trip versus 38 ms with H.264 [F]. That was a cloud setup, so treat it as indicative only.
  - VIO now runs on the host, so camera frames and PX4's IMU data must be time-stamped on the drone and synchronised on the host [H: accuracy to be measured in X6]. EKF2 can compensate for a fixed vision delay (`EKF2_EV_DELAY`) [NV]; jitter is the harder part.
  - Video bandwidth: a 720p H.264 stream needs a few Mbit/s [E], so the bridge needs a hardware video encoder and preferably 5 GHz Wi-Fi [NV: check the chosen board].
  - Audio goes up only while voice activity is detected, as a compressed stream (for example Opus [NV]), a few tens of kbit/s [E].
- **Host internal:** ROS 2 Jazzy nodes. Each model runs as its own inference server (for example vLLM or llama.cpp for the VLM), wrapped by a thin ROS 2 node.
  - Each role (VLM, ASR, TTS, reasoning) has one interface with a local and a cloud backend, chosen by configuration. Moving a model to the cloud to free VRAM is a config change, not a code change.
  - A cloud call can fail or be slow, so every cloud-backed skill has a timeout and a degraded answer ("I couldn't check that") rather than blocking the executive.
- **Agent ↔ executive:** a typed skill API, as JSON schemas used for tool calls. The executive validates every call (geofence, battery, no-fly rooms, rate limits) before acting.

### 2.3 Skill API (sketch)

| Skill | Implemented by | Ends when |
|---|---|---|
| `explore(region?, budget_s)` | Frontier exploration over the voxel map | Frontiers exhausted, budget spent, or abort |
| `goto(place \| object \| pose)` | Scene-graph lookup, then global and local planner | Within tolerance, or unreachable |
| `search(object_query, budget_s)` | Explore, ordered by a room prior (for example bread → kitchen), with detector + VLM verification | Found with confidence ≥ τ, or budget spent |
| `inspect(target, question)` | Approach for a close-up view, then VLM question | Answer plus confidence |
| `follow(person, offset)` | Person tracker, offset setpoint, collision checks | Command, lost target, or safety stop |
| `come_to(person)` | Direction of the voice from the mic array [H], confirmed by person detection, then `goto` | Within 1.5 m |
| `monitor(target \| area, duration, condition)` | Hover or perch at a viewpoint, detector diffing, VLM on change | Duration, event, or battery |
| `return_home()`, `land()`, `report(text)` | Planner / PX4 / TTS | Always available |

No skill takes a prior map or a list of rooms. Places and objects are known only once the drone has observed them, so "go to the kitchen" in a new home means explore until a room is recognised as a kitchen, then go there.

"Should I keep looking?": `search` returns a structured progress record (area covered, rooms left with prior probabilities, battery, elapsed time). The agent picks continue, widen or give up, inside hard limits set by the user or config.

### 2.4 Memory

- **Metric layer:** a TSDF/ESDF voxel map.
  - nvblox supports ROS 2 Jazzy on x86 with an NVIDIA GPU [F]; its Blackwell support is unconfirmed.
  - Fallback: OctoMap [NV], which runs on the CPU.
- **Semantic layer:** a lightweight scene graph (rooms → objects, each with 3D position, label, embedding, last-seen time and image crop).
  - Start with the "Bare Necessities" style of simple open-vocabulary scene graph, reported as more than 3× faster than ConceptGraphs or HOV-SG at similar segmentation quality [F].
  - Change detection = diff object sets and per-voxel occupancy against a stored snapshot.

### 2.5 Failsafe design

| Fault | Detection | Response | Owner |
|---|---|---|---|
| Wi-Fi drop shorter than the timeout (about 0.5 s) [E] | None needed | PX4 keeps flying to the last position setpoint, which is at most a short hop away | PX4 |
| Wi-Fi drop longer than the timeout | PX4 offboard-loss timeout `COM_OF_LOSS_T` [F] | `COM_OBL_RC_ACT` / `COM_OBL_ACT`: hold position on flow + range, then land [F for the parameters; NV whether PX4 chains hold then land, otherwise set Land directly] | PX4 |
| Bridge board crash or freeze | Same timeout: setpoints stop arriving | Same as a long Wi-Fi drop | PX4 |
| Host VIO stale, lost or diverging | EKF2 innovation checks, vision data timeout | EKF2 falls back to flow + range; the host pauses the mission until VIO recovers [H: check the PX4 behaviour] | PX4 + host |
| GPU host crash or model hang | Executive timeouts; if setpoints stop, the PX4 timeout | Host stops sending motion; PX4 holds, then lands | Host, then PX4 |
| Low battery | PX4 battery failsafe | Land or return | PX4 |
| Pilot intervention | RC transmitter switch | Kill or manual takeover, highest priority | PX4 |

Rule: the host is the only source of setpoints, so it checks each one before sending: bounded (speed, acceleration, geofence) and collision-checked against the latest depth data. The bridge drops any setpoint older than 200 ms, and PX4's own speed limits and geofence are the last layer. The host sends position setpoints as short hops and keeps indoor speed low (about 1 m/s [E]), so a stale command cannot carry the drone far before the failsafe triggers.

Trade-off accepted by the owner: with no onboard computer, obstacle checks depend on the link. A latency spike means the drone flies on slightly old information; short hops and low speed bound the damage, and PX4's distance-sensor collision prevention may add a last check [NV: whether it applies in offboard mode].

### 2.6 Simulation topology (mirrors the real system)

```
[container: sim]    Gazebo Harmonic + PX4 SITL (x500_depth or a custom small frame)
                    world = one randomly generated apartment per run (section 5)
[container: drone]  "bridge" nodes (sensor and audio streaming, setpoint relay, staleness drop)
                    simulated mic = recorded speech + recorded propeller noise, mixed by distance
        │  tc netem: latency, jitter, loss, outages  ← fault injection
[container: host]   VIO/SLAM, mapping, exploration, perception, agent, voice; GPU via NVIDIA Container Toolkit
        │  optional: cloud model APIs
```

Running the bridge code in its own container from day one means the same code later runs on the real bridge board, and the Wi-Fi failsafes are tested in every CI run.

Gazebo does not simulate sound [NV], so the simulated microphone is a node that mixes a speech clip with propeller noise recorded from a real motor and propeller, scaled by the speaker's distance and the throttle. This is crude, so the real numbers come from X8.

### 2.7 Onboard voice (the drone hears its operator)

The owner requires the drone itself to hear commands. This is the riskiest perception requirement: the microphones sit centimetres from the propellers, and the operator may be several metres away [H].

- **Hardware:** a small mic array (4 or more mics) wired to the bridge board, mounted as far from the propellers as the frame allows, with foam and vibration isolation [E].
- **Pipeline:**
  1. Beamforming steers towards the speaker and suppresses noise from the propeller directions [H].
  2. A neural denoiser cleans the result [H].
  3. An always-on keyword spotter on the host recognises the wake word and the safety words "stop" and "land", which go straight to the executive.
  4. After the wake word, the same audio goes to full English ASR (Parakeet locally, or a cloud ASR if it copes better with noise).

  Steps 1–4 all run on the host: the bridge only streams the raw multichannel audio. If the link drops, the drone holds and lands on its own, and the RC kill switch remains the manual emergency stop.
- **Help from flight behaviour:** when the drone hears its wake word, it can stop and hover, which is quieter than manoeuvring, and turn towards the voice before listening to the rest of the command [H].
- **Bonus:** the same mic array gives the direction of the voice, which `come_to` uses to find the speaker.
- **Fallback:** a phone push-to-talk app, through the same ASR interface, for when onboard listening fails.
- **Language:** English only, so English-only ASR and TTS models are fine.

Whether this works at all is unknown. Experiment X8 measures it on a real motor and propeller before the hardware is chosen, and the result decides the mic placement, frame size and whether a quieter (larger, slower) propeller is needed.

---

## 3. Comparison matrices

### 3.1 Simulators

| Option | Status (Nov 2026) | PX4 / ROS 2 | GPU cost on 16 GB | Visual realism for VLMs | Verdict |
|---|---|---|---|---|---|
| **Gazebo Harmonic** | PX4's CI-tested Gazebo version. v1.17 also builds against Jetty [F] | Native PX4 SITL, `gz_x500_depth` model [F]. ROS 2 via `ros_gz` (Jazzy branch, release 2026-02-04) [F] | Low to moderate [E] | Low: simple textures [E] | **Primary** for control, mapping, exploration, failsafe and CI |
| Gazebo Jetty | Released; Lyrical binary pairing [F] | PX4 v1.17 can build against it, not CI-tested [F] | Low to moderate [E] | Low [E] | Later migration path |
| **Isaac Sim 6.x** | 6.0 GA June 2026; 6.1 Sept 2026 [F, second source]. Ships NuRec Gaussian splatting [F] | PX4 via Pegasus Simulator, whose latest release found targets Isaac Sim **4.5** [F]. Compatibility with 6.x not established | High: min tier is a 16 GB GPU, driver 595.58.03 on 6.x [F]. 5070 Ti and Ubuntu 26.04 not listed [F] | High, plus Gaussian-splat scenes of a real apartment [F] | **Secondary**, phase 6+: perception realism and sim-to-real |
| Project AirSim / Colosseum | Original AirSim archived by Microsoft [F]. Successors exist; maintenance unclear [F] | PX4 support historically present [NV] | Unreal Engine: high [E] | High [E] | Not recommended: ecosystem risk |
| Flightmare | Older, Unity-based [F] | Limited | Moderate | Medium | Not recommended |
| CARLA-Air | New (arXiv, March 2026) [F] | Unknown | High [E] | High (outdoor-focused) [E] | Not suited to indoor use |

### 3.2 Middleware and flight stack

| Item | Options | Choice |
|---|---|---|
| ROS 2 distro | Jazzy (LTS, Ubuntu 24.04, supported to May 2029 [F, second source]) vs Lyrical (LTS, Ubuntu 26.04 Tier 1, May 2026; some packages such as slam_toolbox were late at launch [F]) | **Jazzy in containers**: the PX4, Gazebo Harmonic and nvblox ecosystem is validated there [F]. Revisit Lyrical in 2027. |
| RMW over Wi-Fi | Default DDS vs `rmw_zenoh` | **Zenoh** for the bridge ↔ host link; measure it (experiment X5) |
| Autopilot | PX4 vs ArduPilot | **PX4**: offboard ROS 2 interface, documented offboard failsafes [F], used by ModalAI VOXL [F]. ArduPilot not evaluated [NV]. |

### 3.3 Localisation, mapping, exploration

| Need | Candidates | Evidence | Pick |
|---|---|---|---|
| VIO (on the host, over Wi-Fi) | OpenVINS, VINS-Fusion, ORB-SLAM3 | One 2025 comparison: OpenVINS lost tracking under motion blur; VINS-Fusion better on short sequences [F]. No drone-plus-PX4 head-to-head found | **OpenVINS** first, VINS-Fusion as fallback. In sim, start with ground-truth pose and add VIO in phase 2 |
| Global SLAM / loop closure | RTAB-Map | Mature graph SLAM, failed to initialise on some EuRoC sequences [F] | Optional, for relocalisation across sessions |
| 3D map / costs | nvblox (GPU), OctoMap (CPU) | nvblox: Jazzy, x86 and Jetson, ESDF output [F] | **nvblox**, with OctoMap fallback if Blackwell or containers cause problems |
| Exploration | FUEL, FALCON | FUEL is ROS 1; a ROS 2/PX4 port is documented in a forum post, release unconfirmed [F]. FALCON's code status is unclear [F] | **Start with a simple frontier planner** (our own, about 500 lines). Port FUEL only if coverage speed matters |
| Local planning | EGO-Planner family, MPPI | A 2026 ROS 2 Jazzy + PX4 project uses D* Lite with CUDA MPPI [F] | Decide in phase 2 |

### 3.4 Perception and reasoning models

| Model | Type | Open weights / licence | Size | Fits beside the others? [E] | Notes |
|---|---|---|---|---|---|
| YOLOE-26 | Open-vocabulary detection and segmentation | Ultralytics [F]; licence AGPL-3.0 [NV] | Small to X | Yes, about 1 GB | Text, visual or prompt-free (4,585 classes). LVIS AP 40.6 (x, text) [F] |
| SAM 3 / 3.1 | Concept segmentation | Gated checkpoints on Hugging Face [F] | 848M [F] | Possibly | Use for offline labelling or slow verification |
| Qwen3.5-9B | Native multimodal LLM | Apache 2.0 [F] | 9B | Yes at 4-bit, about 7–9 GB with cache [E] | Reported best sub-15B multimodal model (Artificial Analysis) [F]. No first-party int4 [F] |
| Qwen3-VL-8B | VLM with grounding | Open [F] | 8B; FP8 card exists [F] | Yes at FP8/4-bit | Grounding and pointing supported [F] |
| Gemma 4 E4B / E2B | Multimodal, audio in | Apache 2.0 [F] | About 4B / 2B effective | Yes | Native audio could merge ASR and VLM [H] |
| Gemma 4 26B MoE | Multimodal | Apache 2.0 [F] | 26B total, 3.8B active [F] | Tight at 4-bit [E] | Better reasoning; VRAM risk |
| Molmo 2 (4B/8B) | Pointing and tracking VLM | Open (Ai2) [F] | 4B / 8B | Yes at 4-bit [E] | Reported to beat Gemini 3 Pro on video pointing (author numbers) [F] |
| Depth Anything 3 metric | Monocular metric depth | Apache 2.0 [F] | 0.35B [F] | Yes | 50 FPS reported with TensorRT on a Quadro RTX 6000 [F]. Only needed without stereo depth |
| Parakeet / Whisper large-v3-turbo | ASR | Open [F] | 0.6B / 809M [F] | Yes | Whisper turbo is batch-only; use a streaming-native model for low latency [F] |
| Kokoro-82M | TTS | Commercially usable licence [F] | 82M [F] | Yes | |
| OpenAI GPT-6.x | Cloud multimodal | Closed | n/a | n/a | Flagship IDs `gpt-6-astra`, `gpt-6.1-sol`, `gpt-6.1-terra`, `gpt-6-luna`; all accept image input [F]. Realtime voice models GPT-Realtime-2.1 / GPT-Live-1 [F]. Pricing and latency not checked |

### 3.5 Onboard hardware candidates (for later; do not buy yet)

| Item | Option | Price [F unless marked] | Weight | Notes |
|---|---|---|---|---|
| Integrated dev drone | ModalAI Starling 2 | $2,949.99 | 280 g | VOXL 2, VIO tracking camera, ToF, PX4, 40+ min claimed |
| Bridge board | Small Linux board with a hardware H.264 encoder and 5 GHz Wi-Fi (Raspberry Pi Zero 2 W / Radxa Zero 3W class) | [NV] | Light | Streams sensors only. Check encoder, Wi-Fi band and USB bandwidth before buying |
| Fallback companion | Raspberry Pi 5 8 GB | about $80 | Light | Only if X6 shows offboard VIO over Wi-Fi is not good enough |
| Camera | Wide-angle global-shutter RGB camera module | [NV] | Light | Enough for VIO and perception; depth comes from Depth Anything 3 on the host if there is no depth camera |
| Depth camera | OAK-D Pro W | $499 / €568 | 91 g | Stereo with active IR; wide FOV helps VIO |
| Depth camera | OAK-D Lite | About €198; discontinued at one retailer; Luxonis price change July 2026 | 61 g | Optional upgrade: real depth for obstacle checks. Needs USB bandwidth on the bridge |
| Flow + range | ARK Flow-class sensor | [NV] | Light | Big improvement over PX4 Vision stock sensors, per one owner report [F] |
| Microphones | Small 4–6 mic USB or I²S array (ReSpeaker-class) | [NV] | Light [E] | Needed for onboard voice. Choose after X8 |
| Speaker | Small speaker for spoken reports | [NV] | Light [E] | May be hard to hear over the propellers; reports can also go to a phone |

---

## 4. Implementation roadmap

Durations assume about 10 hours a week [E]. Each phase ends at a measurable gate.

| Phase | Content | Exit milestone (measurable) |
|---|---|---|
| **P0 Environment** (1–2 weeks) | Install Docker + NVIDIA Container Toolkit. Containers: Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic + PX4 v1.17 SITL. Repo skeleton, CI that runs headless sim | 20/20 scripted runs of take off → hover 10 s → land via ROS 2 offboard, with no manual steps |
| **P1 Safety skeleton** (2 weeks) | Bridge container: setpoint relay with staleness drop. Host setpoint checks and geofence. `tc netem` fault injection. PX4 failsafe params and flow-based position hold | Failsafe suite F1–F6 (section 5) passes 100% over 50 randomised runs. F7 and F8 join the suite in P4, once voice and cloud backends exist |
| **P2 Map and move** (3–4 weeks) | Random apartment generator (section 5), tuning and held-out layout sets. Depth → nvblox/OctoMap. Global and local planner. Frontier exploration. Start with ground-truth pose, then switch to VIO | Capabilities 1, 6, 7 meet their thresholds |
| **P3 Perception and memory** (3 weeks) | YOLOE-26 node, 3D object lifting, scene graph, room labelling (VLM on keyframes) | Capabilities 2, 5 meet their thresholds in sim |
| **P4 Agent and voice** (3 weeks) | Skill API, behaviour-tree executive, VLM/LLM server with local and cloud backends, simulated noisy mic, keyword spotter, ASR/TTS, search budgeting | End-to-end: "explore a bit, find the kitchen, tell me if you see bread, come back" passes 8/10 seeded scenarios on held-out layouts, spoken through the simulated mic |
| **G1 Real-world gate** (1–2 weeks) | Perception: phone video at drone height from several different homes, plus public indoor datasets. Voice: X8 on a real motor and propeller | Detection, room ID and clock reading meet thresholds on real footage from homes not used for tuning (X1–X3). Onboard voice meets V1 thresholds in X8, or the voice design is revised |
| **P5 People** (3 weeks) | Person detection and tracking, follow with offset, come-to-me, pointing (Molmo 2 + 3D ray cast) | Capabilities 3, 4, 10 |
| **P6 Monitoring and photoreal** (3+ weeks) | Change detection, monitor skill, clock reading. Optional Isaac Sim 6.x + Gaussian splat of the apartment | Capabilities 8, 9, 11 |
| **G2 Hardware decision** | Use measured compute, payload and link needs to choose the platform (section 7) | Written bill of materials within budget, reviewed |
| **P7 Hardware bring-up** | Tethered or netted tests, offboard VIO over real Wi-Fi, onboard mic tests in flight, then supervised indoor flights | Failsafe suite repeated on hardware; capability 7 in a real room; V1 in flight |
| **P8 Real homes** | First full missions in a home the system has never seen (the owner's flat counts as one) | Capabilities 1, 2 and 5 in a real flat, with no prior map |

**Fastest path to a working simulation (vertical slice, about 3 weeks):** P0 → minimal P1 (PX4 link-loss failsafe only) → a "fly to the [object]" demo. Use ground-truth pose, Gazebo's depth camera, an OctoMap, straight-line A* to a goal, and the YOLOE-26 text prompt to pick the goal. This proves the full loop (voice or text → skill → planner → PX4) before investing in VIO, exploration or VLMs.

---

## 5. Automated validation tests

All tests run headless in containers with fixed random seeds. A test fails on any collision (contact sensor in sim) unless stated otherwise. Thresholds are initial targets [E], to be tightened as we learn.

**Random apartments, not one known flat.** The system must work in any home, so it is never tuned on the home it is scored on.

- **Generator:** each run builds a random apartment: number and type of rooms, layout, door widths (≥70 cm), ceiling height, furniture, lighting, textures and object placement, all from a seed.
  - Options to evaluate: our own floor-plan generator exported to Gazebo SDF, or existing procedural indoor generators and scanned-home datasets (ProcTHOR-style generators, Infinigen Indoors, HM3D or similar) converted to Gazebo [NV: licences and conversion effort not checked].
- **Split:** seeds are divided into a tuning set (used while developing and choosing thresholds) and a held-out set (used only to report results). Capability scores count only on the held-out set. Anything trained, such as a fine-tuned detector, uses tuning layouts only.
- **Real homes:** a 3D scan of the owner's flat is welcome later as one extra held-out test case and for an Isaac Sim Gaussian-splat twin. It is never used for tuning.
- **Other assets:** placeable objects (keys, bread, analog and digital clocks with programmable time, mugs) and a scripted human actor that can speak (a recorded or TTS voice fed to the simulated mic). Gazebo indoor assets need sourcing or building [NV].

| # | Capability | Scenario | Metric and pass threshold |
|---|---|---|---|
| C1 | Map and explore | Start at a random pose, `explore(budget=6 min)` | ≥90% of reachable free volume observed; 0 collisions; map surface error <10 cm against the ground-truth mesh |
| C2 | Find a named room | "Go to the kitchen" in an unseen layout | Correct room in ≥9/10 seeds; ends ≤1 m inside the room |
| C3 | Follow me (right side) | Actor walks a scripted 3-minute path with turns and doorways | Lateral offset error ≤0.5 m for ≥90% of the time; never <1.0 m from the actor; target lost <2× |
| C4 | What am I pointing at | Actor points at one of N labelled objects | Correct object ≥80% (N=5 distractors) |
| C5 | Find object (keys) | Keys at a random location, possibly occluded | Found within budget in ≥80%; position error <0.3 m; false-positive report rate <5% |
| C6 | Detect unexplored areas | Partial map | Frontier set recall ≥95% against ground-truth unknown regions |
| C7 | Obstacle avoidance | Random clutter, narrow doorways (≥70 cm), 100 goal runs | 0 collisions; minimum clearance ≥0.25 m; ≥95% goals reached |
| C8 | Read a clock | Clocks at random set times, digital and analog | Digital: exact minute ≥95%. Analog: within ±2 min ≥80%. Confidence must flag errors |
| C9 | Detect changes | Between two runs, move, add or remove k objects | Precision ≥0.9, recall ≥0.8 at k=3 |
| C10 | Come to me | Actor in a random room calls the drone | Arrives within 1.5 m in <60 s (≥9/10) |
| C11 | Monitor an object/area | Watch for 10 min; event (object removed, person enters) at a random time | Detected within 5 s; ≤1 false alarm per hour |
| V1 | Hear the operator | English commands spoken at 1–4 m while the drone hovers and while it moves (simulated mic in sim, then X8 on real hardware) | Wake word detected ≥95% at ≤3 m while hovering, ≤1 false wake per hour; "stop"/"land" ≥99% with no Wi-Fi; command word error rate ≤10% after the wake word [E: initial targets] |
| A1 | Agent end-to-end | Ten scripted multi-step instructions | ≥8/10 complete; no unsafe tool calls accepted by the executive |
| A2 | Search budgeting | Object absent from the house | Gives up within its budget and reports "not found, searched rooms X, Y" |

Failsafe suite (run in CI on every change):

| # | Injected fault | Pass condition |
|---|---|---|
| F1 | Link outage of 0.3, 2 and 15 s | Hover within 0.5 m drift on flow for short outages [E]; lands for long ones; never collides |
| F2 | Latency 300 ms ± 200 ms, 20% packet loss | No stale setpoint executed (age >200 ms rejected); flight stays stable |
| F3 | Kill the host GPU/agent process | Same as F1 |
| F4 | Kill the bridge process | PX4 offboard-loss failsafe lands within `COM_OF_LOSS_T` + margin |
| F5 | VIO reset or divergence (injected pose jump) | Detected; EKF2 falls back to flow, host pauses; no flyaway beyond 1 m |
| F6 | Malicious or garbled agent output (out-of-geofence goal, NaN, 10 m/s) | Rejected by the host setpoint checks or PX4 limits; logged |
| F7 | Spoken "stop" while flying, link up | Drone stops and hovers within 1 s of the end of the word |
| F8 | Cloud model timeout or error mid-mission | Skill returns a degraded result within its timeout; no stall, no unsafe action |

---

## 6. Compute requirements (host, RTX 5070 Ti 16 GB)

All figures are estimates [E] until measured in experiment X4.

| Process | VRAM | Notes |
|---|---|---|
| Gazebo Harmonic rendering (1 RGB-D camera) | 0.5–1.5 GB | Physics on CPU |
| nvblox | 0.5–1 GB | Depends on voxel size and house size |
| YOLOE-26 s/m (TensorRT or PyTorch) | 0.5–1.5 GB | |
| Qwen3.5-9B, 4-bit, about 8k context | 7–9 GB | Weights plus vision encoder plus KV cache. Serve with vLLM or llama.cpp |
| Molmo 2 4B, 4-bit (loaded on demand) | 3–4 GB | Or time-share with the VLM |
| ASR (Parakeet or Whisper turbo) | 1–2 GB | Could run on the CPU |
| Kokoro TTS | <0.5 GB | CPU is enough |
| **Total, everything resident** | **about 13–19 GB** | **Over budget in the worst case.** |
| **Total with the VLM in the cloud** | **about 6–10 GB** | Comfortable. This is the planned fallback now that cloud models are allowed |
| Isaac Sim instead of Gazebo | +about 8–14 GB | Not compatible with the full local model set on one 16 GB card. Use the cloud VLM (and cloud pointing model if needed) during Isaac runs |

Order of offloading when VRAM runs out: first the VLM to the cloud (biggest single item), then the pointing model, then ASR. Detection and mapping stay local, because they run at 10–30 Hz and need low latency. X4 measures whether any offload is needed at all.

CPU and RAM: PX4 SITL, Gazebo physics, ROS 2 and planners fit in 16 threads and 30 GB [E]. 30 GB is Isaac Sim's minimum alone [F], another point against running it alongside the full stack.

Onboard: nothing beyond PX4 and the bridge board. VIO and SLAM run on the host CPU or GPU (about 1–2 CPU threads for OpenVINS [E]). If no depth camera is fitted, Depth Anything 3 metric adds about 1 GB of VRAM [E].

---

## 7. Preliminary hardware and budget (indicative; buy nothing until gate G2)

| Tier | Approach | Rough cost [E] | Pros | Cons |
|---|---|---|---|---|
| **A: Offboard minimal (recommended)** | 3.5–5" ducted frame, PX4 flight controller (Pixhawk 6C Mini class), flow + range sensor, GPS module (outdoor flights), RGB camera, bridge board, mic array, small speaker, RC transmitter and receiver, 3–4 batteries, charger, spares | €400–650 | Cheap and light; all intelligence upgradable on the PC | Depends on Wi-Fi quality; VIO latency over the link; monocular depth only |
| B: A plus depth camera | Tier A with an OAK-D Lite | €600–850 | Real depth for obstacle checks and mapping | More weight and USB bandwidth on the bridge |
| C: Integrated (≈€2,800+) | ModalAI Starling 2 ($2,949.99 [F]) | Not needed | Works out of the box | Pays for onboard compute we don't use |

Non-negotiable for any tier: an RC transmitter with a kill switch, prop guards or ducts, and a safety net or a padded test space for first flights. EU/French drone rules for indoor private flight [NV]: check before flying.

---

## 8. Risks and open questions

| Risk | Likelihood / impact [E] | Mitigation |
|---|---|---|
| VRAM contention (VLM + sim + detector) | Medium / Medium (was High / High before cloud was allowed) | Gazebo first, on-demand model loading, VLM to the cloud |
| Cloud latency, outages or cost | Medium / Medium | Timeouts and degraded answers (F8); nothing flight-critical in the cloud; track monthly API spend |
| Overfitting to the simulator's apartments | Medium / High | Random generator with a held-out set; varied textures and lighting; real footage from several homes at G1 |
| Blackwell / Ubuntu 26.04 compatibility (Isaac Sim, nvblox, CUDA wheels) | Medium / Medium | Containers; driver 595.99 meets Isaac 6.x's ≥595.58.03 [F]; smoke-test each dependency in P0 |
| Pegasus not ported to Isaac Sim 6.x | High / Low (Isaac is optional) | Defer Isaac. Write our own PX4 bridge only if needed |
| VLM hallucination ("I see bread") | High / Medium | Detector + VLM agreement, close-up `inspect`, confidence in reports |
| Clock reading accuracy | Medium / Low | Close-up approach; experiment X3 |
| Gazebo-to-real perception gap | High / Medium | Gate G1 on real phone footage; later Isaac Gaussian splats |
| Real Wi-Fi in the apartment (dead zones, interference) | Medium / High (higher now that VIO and obstacle checks are offboard) | PX4 hold-then-land failsafe; X5 measures it; a dedicated 5 GHz access point for the drone |
| Offboard VIO too laggy or jittery over Wi-Fi | Medium / High | Flow + range fallback in EKF2; low speed; X6 measures it. If it fails, add a Pi 5 companion for VIO only (about €80) |
| Camera and IMU time sync across the link | Medium / Medium | Time-stamp on the drone; X6 |
| Optical flow on plain or dark floors | Medium / High | Flow sensor with its own illumination [NV]; low speed; land if flow quality drops [H] |
| VIO on blank walls or in low light | Medium / High | Active-IR stereo (OAK-D Pro), flow + range fallback, slow speeds |
| Short flight time, payload limits | High / Medium | Measure in sim; size the hardware at G2 |
| Propeller noise breaks onboard voice | High / High (onboard hearing is now a requirement) | Mic array, beamforming, denoising, hover-to-listen, RC kill switch; measure early (X8); phone push-to-talk fallback |
| Privacy of in-home video | n/a | The owner accepts cloud processing. Still send only the frames a skill needs, not a continuous stream |

Open questions remaining:
1. Is there a monthly budget for cloud API calls? This sets how freely the VLM can be offloaded.
2. What wake word should the drone answer to? ("Maverick" is the working assumption.)
3. Should spoken reports come from the drone's speaker, a phone, or both? A small onboard speaker may be hard to hear over the propellers.

---

## 9. Research experiments

| ID | Question | Method | Decides |
|---|---|---|---|
| X1 | Does YOLOE-26 find everyday objects in homes it has never seen? | Phone video at 1–1.5 m height from several different homes, plus public indoor datasets; label 200 frames; measure recall at fixed precision | Detector choice or fine-tuning need |
| X2 | Can the VLM name rooms and answer "is there bread?" reliably in unfamiliar homes? | Same footage, 100 questions; compare Qwen3.5-9B, Gemma 4 E4B, Qwen3-VL-8B, one cloud model | VLM choice, local or cloud |
| X3 | Clock reading accuracy versus distance and angle | Real and synthetic clocks; VLMs vs Molmo 2 vs a specialised pipeline (hand detection + geometry) | `read_clock` design |
| X4 | Real VRAM, latency and throughput of the full model set on the 5070 Ti | Benchmark script, all models resident and on-demand | Model sizes and quantisation |
| X5 | Wi-Fi link quality in the flat | Laptop at drone positions: RTT, jitter, loss, Zenoh image throughput | Failsafe timeouts, image rates |
| X6 | Is VIO on the host good enough over Wi-Fi? | In sim with `tc netem` set to the latency and jitter measured in X5, then on recorded real data replayed over real Wi-Fi: compare position error and hold quality against flow alone | Whether the offboard design holds, or a Pi 5 companion is needed for VIO |
| X7 | Is the LLM's search budgeting sensible? | 50 synthetic search states; compare against an oracle cost policy | How much autonomy to give the agent |
| X8 | Can a mic on the drone hear the operator over its propellers? | Bench rig: one or more real motors and propellers at hover throttle, a mic array at candidate positions, English commands spoken at 1–4 m from several directions. Measure wake-word recall, false wakes and word error rate with and without beamforming and denoising, local vs cloud ASR. Also record the noise for the simulated mic | Mic placement, frame and propeller choice, whether hover-to-listen is needed, whether onboard voice is viable at all |

---

## 10. Owner decisions (2026-11-10)

| Question | Answer | Effect on this study |
|---|---|---|
| Command language | English only | English ASR and TTS models (Parakeet English, Kokoro) |
| Cloud models for in-home imagery | Accepted, especially to relieve the GPU | Every model role has a local and a cloud backend; the VLM is the first to offload (section 6) |
| Voice capture | The drone must hear the operator itself | Onboard mic array, host-side keyword spotter (section 2.7), test V1, failsafe F7, experiment X8 |
| Known flat vs any flat | The drone must adapt to any home; it is not about knowing the owner's flat | No prior maps; random apartment generator with a held-out test set (section 5); real-world gate uses several homes |
| 3D scan of the owner's flat | Acceptable | Used later as one held-out test case and an Isaac Sim twin, never for tuning |
| Onboard compute (2026-11-11) | Only what's needed to fly; all computing offboard, no need for an expensive drone | No companion computer: PX4 holds and lands on flow + range, a small bridge board streams sensors, VIO and obstacle checks move to the host. Budget about €400–650 (section 7). X6 tests the risk |
| Outdoor flight (2026-11-11) | In scope, assuming ideal conditions: no wind, bright daylight | GPS module added to the drone; outdoors relies on GPS + barometer rather than optical flow. Simulated as stage S6 of [simulation-plan.md](simulation-plan.md) (roof inspection). Wind and low light are later scenario options. Regulations to check before real outdoor flights [NV] |
| Flat size and layout | Not needed: design for generic homes | Assume doorways ≥70 cm and normal ceiling heights; the generator covers the range |

---

## Sources

Simulation
- Isaac Sim 5.1 requirements: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html
- Isaac Sim 6.x requirements (updated Oct 14, 2026): https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/requirements.html
- Isaac Sim 6.0 GA announcement: https://forums.developer.nvidia.com/t/isaac-sim-6-0-general-availability/372621
- Isaac Sim 6.0.1 release notes: https://docs.isaacsim.omniverse.nvidia.com/6.0.1/overview/release_notes.html
- Isaac Sim 6.0 NuRec Gaussian splatting: https://radiancefields.com/nvidia-s-isaac-sim-6.0-ships-with-nurec-gaussian-splatting
- Isaac Sim 6.1 vs 6.0 (third party): https://openelab.io/es/blogs/learn/isaac-sim-6-1-vs-6-0-upgrade
- 5070 Ti crash on Isaac Sim 5.1: https://forums.developer.nvidia.com/t/isaac-sim-5-1-gui-crash-access-violation-on-rtx-5070-ti-blackwell-fixed-by-driver-downgrade-to-591-74/365335
- Pegasus Simulator: https://github.com/PegasusSimulator/PegasusSimulator , https://arxiv.org/abs/2307.05263v1
- PX4 Gazebo simulation: https://docs.px4.io/main/en/sim_gazebo_gz/index
- PX4 v1.17 release notes: https://docs.px4.io/main/en/releases/1.17.html , https://px4.io/px4-autopilot-release-v1-17-what-you-need-to-know/
- ros_gz (version pairings): https://github.com/gazebosim/ros_gz , https://gazebosim.org/docs/all/ros_installation
- PX4 ROS 2 Jazzy + Harmonic + CUDA MPPI project: https://discourse.openrobotics.org/t/px4-ros2-drone-nav-map-free-3d-navigation-with-a-persistent-d-lite-planner-and-cuda-mppi-ros-2-jazzy-px4-sitl-gazebo-harmonic/58190
- AirSim archived: https://www.microsoft.com/en-us/research/project/aerial-informatics-robotics-platform
- Colosseum: https://GitHub.com/CodexLabsLLC/Colosseum
- CARLA-Air: https://arxiv.org/html/2603.28032v1

Middleware and flight control
- ROS 2 Lyrical Luth release: https://discourse.openrobotics.org/t/ros-2-lyrical-luth-released/55021 , https://docs.ros.org/en/lyrical/Releases/Release-Lyrical-Luth.html
- Jazzy vs Lyrical (third party): https://openelab.io/es/blogs/learn/ros-2-lyrical-vs-jazzy
- PX4 offboard mode and failsafes: https://docs.px4.io/main/en/flight_modes/offboard
- Zenoh over Wi-Fi: https://learn.arm.com/learning-paths/cross-platform/tuning-zenoh-ros2-lp3/4-measure-real-wifi/
- FogROS2 image latency: https://arxiv.org/pdf/2205.09778
- Clearpath on ROS 2 video over Wi-Fi: https://clearpathrobotics.com/blog/2025/04/optimizing-video-streaming-in-ros-2/

Mapping, SLAM, exploration
- Isaac ROS nvblox: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox
- VIO comparison (OpenVINS, VINS-Fusion, RTAB-Map): https://arxiv.org/pdf/2507.11241
- RTAB-Map: https://arxiv.org/pdf/2403.06341
- FUEL: https://arxiv.org/abs/2010.11561v2
- FUEL on ROS 2 + PX4: https://discuss.px4.io/t/ai-assisted-reproduction-of-fuel-on-ros-2-and-px4-offboard-simulation-to-real-indoor-uav-exploration/49185
- FALCON: https://www.themoonlight.io/review/falcon-fast-autonomous-aerial-exploration-using-coverage-path-guidance
- Open-vocabulary scene graphs: https://arxiv.org/pdf/2412.01539 , https://arxiv.org/html/2403.17846v2 , https://arxiv.org/html/2410.11989v3

Models
- OpenAI models: https://developers.openai.com/api/docs/models
- GPT-Realtime-2.1: https://community.openai.com/t/new-realtime-models-on-the-api-gpt-realtime-2-1-and-gpt-realtime-2-1-mini/1385896
- Qwen3.5-9B: https://huggingface.co/Qwen/Qwen3.5-9B , https://artificialanalysis.ai/articles/qwen3-5-small-models
- Qwen3-VL-8B FP8: https://huggingface.co/Qwen/Qwen3-VL-8B-Instruct-FP8 , https://docs.unsloth.ai/models/qwen3-vl-run-and-fine-tune
- Gemma 4: https://blog.google/innovation-and-ai/technology/developers-tools/gemma-4/
- Molmo 2: https://allenai.org/blog/molmo2 , https://arxiv.org/pdf/2601.10611
- SAM 3: https://cdn.jsdelivr.net/gh/facebookresearch/sam3@main/README.md
- YOLOE / YOLOE-26 / YOLO26: https://docs.ultralytics.com/models/yoloe , https://docs.ultralytics.com/models/yolo26/
- Depth Anything 3: https://cdn.jsdelivr.net/gh/bytedance-seed/depth-anything-3@main/README.md , https://huggingface.co/APaul1/DA3METRIC-LARGE , https://discourse.openrobotics.org/t/depth-anything-3-ros-2-wrapper-development/51030
- Streaming ASR: https://arxiv.org/html/2604.14493v2
- ASR and TTS overviews: https://spokenly.app/blog/parakeet-vs-whisper , https://d-central.tech/local-voice-ai-models/

Hardware
- ModalAI Starling 2: https://modalai.com/starling-2 , https://docs.modalai.com/starling-2/
- Jetson vs Pi 5: https://computingforgeeks.com/edge-llm-jetson-orin-vs-ai-hat-2-vs-hailo-10h/ , https://specpicks.com/reviews/pi-5-hailo-8-vs-jetson-orin-nano-super-vision-2026
- OAK-D Pro W: https://shop.luxonis.com/products/oak-d-pro-w , https://botland.store/22256-luxonis-oak-d-pro-w-imx378-ai-set-for-image-recognition-fixed-focus.html
- OAK-D Lite: https://botland.store/20955-oak-d-lite.html , https://shop.luxonis.com/pages/pricing-change-oak-lite-oak-1
- PX4 Vision kit: https://docs.px4.io/main/en/complete_vehicles_mc/px4_vision_kit.html , https://discuss.px4.io/t/px4-vision-1-5-setup-and-thoughts/31994

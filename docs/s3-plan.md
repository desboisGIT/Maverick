# Stage S3: see and map

Status: plan, written 2026-11-11. Parent plan: [simulation-plan.md](simulation-plan.md) §2.4, §7 and §9 (stage S3). Tags as in the study: [F] verified, [E] estimate, [H] hypothesis, [NV] not verified.

## Goal and exit test

The drone explores an unseen generated apartment on its own, builds a 3D map, finds its rooms and recognises what they are, and the Mission Console shows its camera and its "mind" live.

**Exit test** (held-out worldgen seeds, ground truth from the manifest):

| # | Capability | Pass threshold |
|---|---|---|
| C1 | `explore(budget = 6 min)` from the spawn point | ≥ 90% of reachable free volume observed; 0 collisions; map surface error < 10 cm |
| C6 | Frontier detection on partial maps | Frontier recall ≥ 95% against ground-truth unknown-but-reachable regions |
| C2 | "Go to the kitchen" (by command, no agent yet) | Correct room in ≥ 9/10 seeds; ends ≤ 1 m inside the room |
| Console | Live demo view | Camera with detections and the 3D mind view (map, rooms, objects, path) update live during exploration |

## Decisions

- **Depth camera in simulation.** The drone model gets a forward RGB-D camera (PX4's OAK-D Lite Gazebo model, as used by `x500_depth`). This separates mapping from depth estimation. Tier A hardware has no depth camera, so monocular depth (Depth Anything 3) stays a later experiment behind the same `DepthSource` interface.
- **Pose:** PX4's own estimate (optical flow + rangefinder), with its real drift. No ground truth pose on the host, ever. VIO comes later if drift is a problem. `MAVERICK_LOCALISATION` = `flow` (default) | `mocap` | `vo`. `mocap` is a simulated motion-capture system (like an indoor lab): a process in the `sim` container reads the drone's Gazebo pose, adds mocap noise (2 mm, 0.5 deg, +-5 ms) and sends it to the bridge, which feeds it to PX4 as external vision; the host still only sees PX4's estimate. Results with mocap are labelled "with mocap" and are NOT S3 exit results (the no-ground-truth rule stands for exit results); they separate localisation faults from planning and flight faults.
- **Map: our own numpy voxel map**, not OctoMap or nvblox. Flats are small (≤ 110 m² × 2.7 m ≈ 30 k voxels at 0.1 m per metre of height layer), numpy is in the image, and owning the code makes it testable offline. nvblox can replace it later behind the same interface if needed [E].
- **Room recognition: furniture first, VLM later.** A room's type comes from the objects detected in it (fridge/oven/counter → kitchen, bed → bedroom, toilet/shower → bathroom, sofa/TV → living room) with a rule-based scorer. The VLM joins in S4 for ambiguous rooms. This keeps S3 deterministic and free of large model downloads.
- **Simulated detector in S3.** Worldgen furniture is built from primitives, which a real detector such as YOLOE would not recognise. In simulation the detector is Gazebo's segmentation camera (every model carries a class label) plus a noise model (misses, false positives, box jitter, confidence), behind the same `Detector` interface YOLOE will implement. The real detector is judged on real footage (gate G1) and on photoreal scenes later (S8). This is oracle-quality perception with realistic occlusion, and results must be reported as such.
- **Drone size.** The simulated drone is PX4's x500 (circumscribed radius 0.39 m), so worldgen's default `x500` profile uses 1.1–1.3 m doors and 1.2 m corridors; the `small` profile keeps real 0.8–1.0 m doors for a future small-drone model.
- **Offline harness first.** Mapping and exploration are developed against a synthetic simulator: depth images ray-cast from worldgen manifests, a kinematic drone following the planner's waypoints, and pose noise. Hundreds of explorations run in seconds without Gazebo; Gazebo then confirms.

## Components

| Package | Content | Owner |
|---|---|---|
| `maverick_mapping` | `VoxelMap` (log-odds, depth integration, ray-casting in numpy), 2D projections for the flight band (0.3–2.0 m), frontier detection and clustering, exploration planner (frontier choice by gain / path cost), A* on an inflated 2D grid, path smoothing to waypoints; offline harness (`synthetic.py`: manifest → depth images; kinematic drone; metrics C1/C6) | agent E |
| `console/` | Mission Console web app (TypeScript, Vite, three.js): camera panel with detection overlay, 3D mind view (voxels, rooms, objects, path, frontiers), conversation, plan and status panels; gateway event contract; mock gateway for development | agent G |
| `maverick_memory` | Scene graph: rooms (segmented from the 2D map, typed by the furniture scorer), objects with 3D positions and last-seen times | after E (needs the map interfaces) |
| `maverick_perception` | YOLOE-26 detector node (GPU), detections lifted to 3D with depth | after the S1 exit test (needs the GPU and the stack) |
| Integration | Camera on the drone model, Gazebo → ROS image bridging (`ros_gz_bridge`), streaming over the bridge, gateway node, explore skill in the flight client | orchestrator |

## Contracts

- **2D occupancy grid:** numpy `int8` array, `-1` unknown, `0` free, `100` occupied, with `resolution` (m) and `origin` (x, y of cell [0, 0]) in the map frame: ROS REP 103 ENU (x east, y north, z up, metres, right-handed; yaw counter-clockwise from +x). Conversion from PX4 local NED in one place: x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned, yaw_enu = pi/2 - yaw_ned. Gazebo worlds and worldgen manifests are ENU too.
- **Waypoints to the flight client:** goals on `/maverick/flight/goal` as in S1 (`goto` mode, ≤ 1 m/s).
- **Gateway events (console):** JSON over a websocket, one envelope `{"type": ..., "t": ..., "data": ...}`, types: `pose`, `camera_frame` (JPEG base64 or a URL), `detections`, `map_update` (voxel blocks or a downsampled occupied-voxel list), `frontiers`, `path`, `rooms`, `objects`, `transcript`, `agent_step`, `skill_progress`, `status`. Agent G defines the exact schema in `console/src/gateway/types.ts` and a matching `docs/gateway-events.md`; the ROS gateway node follows it.

## Integration contract (sensor path → host)

The camera is part of the drone, so the sensor path mirrors hardware: Gazebo renders in the `sim` container ("the camera"), a small camera driver there sends frames to the `bridge` container over the `drone` network (standing in for the USB cable; plain length-prefixed TCP, no Gazebo transport and no ground truth on that network), and the bridge relay publishes them to the host through Zenoh.

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/maverick/camera/rgb/compressed` | `sensor_msgs/CompressedImage` (JPEG) | 10 Hz | 320x240 |
| `/maverick/camera/depth/png` | `CompressedImage`, format `16UC1; png` (lossless, mm, 0 = invalid) | 10 Hz | 320x240, registered with RGB, same stamp |
| `/maverick/camera/semantic/png` | `CompressedImage`, format `mono8; png` (worldgen class ids) | 10 Hz | 160x120, **simulation only** |
| `/maverick/camera/info` | `sensor_msgs/CameraInfo` | 10 Hz | intrinsics of the 320x240 images; the 160x120 semantic image scales by 1/2 |
| `/maverick/camera/pose` | `geometry_msgs/PoseStamped` | 10 Hz | camera optical-frame pose in the map frame (ENU), PX4 estimate at frame time, computed on the bridge |
| `/maverick/vision/odometry` | `nav_msgs/Odometry` (host -> bridge, best-effort) | ~10 Hz | body (FLU) pose in `map` (ENU), stamp = camera frame stamp; the relay drops it when older than 200 ms and converts to `px4_msgs/VehicleOdometry` on `/fmu/in/vehicle_visual_odometry` |

Measured bandwidth (camera, bridge to host): about 15 kB per frame, 1.1 Mbit/s at 9 Hz [F], far below the ~10 Mbit/s budget (depth PNG of the generated apartments is only a few kB). Raw images exist only on the host: `maverick_vo/camera_unpack` decodes them to `/maverick/camera/{rgb,depth,semantic}` (host-local, not in the Zenoh lists), which mapping, memory and the odometry read. Details of the odometry: sim/px4/README.md "Visual odometry".

The camera topics carry the same `header.stamp` per frame (bridge wall clock at receipt). Frame ids: `map`, `camera_optical`. Host nodes: `mapping_node` (VoxelMap, frontiers, 2D grid), `memory_node` (simulated detector → SceneGraph), `explore` skill node (frontier exploration through `/maverick/flight/goal`), `gateway` (websocket per docs/gateway-events.md).

## Closed-loop diagnosis and fixes with mocap (2026-11-11)

All results "with mocap" (not exit results), x500, explore budget 360 s, 0.03 m estimate error [F].

**Root causes found** (before: held-out coverage 0.83/0.60/0.98/0.81/0.82, collisions 2/1/0/20/5, goto_room 0/5):
1. [F] The skills planned from the camera pose, 0.12 m ahead of the body (camera_mount.json); the body deviated from the planned path by 0.16-0.26 m (p90) in straight legs and corners (seed 1000001 replay). The node now subtracts the mount offset.
2. [F] Doors of 1.1 m are impassable with hard radius 0.39 + margin (needs 1.1 + 2 x margin): 8 of 24 held-out doors were blocked in the planning grid at margin 0.16 (map-final replay). Explore then ended with "explored everything reachable" at coverage 0.6 and approached blocked doors "as close as possible", where the drone hit the frame (seed 1000001, 0.40 m from the door corner, then a crash); goto_room failed with "no path" / "no goal". The planner's start-escape zone also allowed viewpoints inside the blocked inflation next to the frame.
3. [F] Height follows the rangefinder, so flying over furniture below the 0.8 m obstacle band lifted the drone by the furniture height (+0.5 m over a coffee table, 2.2 m near the 2.55 m ceiling in seed 1000003), with 25 collisions after the first contact. Obstacle band is now 0.35-1.6 m (map and referee).
4. [F] Typical remaining contacts are 0.33-0.39 m clearance at door frames (threshold 0.39); a prop strike at that distance can start a crash and then a meaningless estimate (seeds 1000001, 1000003 after the first hit).
5. [F] Camera behind the lens: the body cell is never ray-traced, so with the body-centred start the robot stood on unknown cells (explore ended after 11 s in a first attempt); unobserved cells within 0.45 m of the robot are now free.

**Fixes**: planner margin 0.22; door "squeezes" (cells between tight radius 0.46 and 0.61 m of an obstacle, only where obstacles are on both sides: penalised, never a viewpoint, tight line-of-sight, crossed at 0.4 m/s with 0.08 m arrival radius; radius relaxed to 0.43 then 0.40 when nothing is reachable; last-resort path along the drone's own flown track); final waypoint of a path is slow; memory goal connectivity with 0.3 m clearance and an unconnected fallback goal; goal 1.1 m from the room edge; skill decision log (`skill_events.jsonl` per run).

**Results**, with mocap (tuning seeds 10-14 were used to develop; the held-out numbers are one run of the final code):

| Set | Scenario | Coverage | Collisions | Outcome |
|---|---|---|---|---|
| tuning 10-14 (intermediate code) | explore | 0.92 / 0.99 / 0.99 / 0.68 / 0.99 | 1 / 2 / 0 / 2 / 1 | 0/5 pass (collisions 0.33-0.39 m clearance, surface error 0.09-0.2 m) |
| tuning 10-14 (intermediate code) | goto_room | 0.95-0.99 | 1-69 | 1/5 |
| held-out 1000000-04 | explore | 0.99 / 0.96 / 0.98 / 0.82 / 0.98 | 4 / 14 / 1 / 10 / 1 | 0/5 pass |
| held-out 1000000-04 | goto_room | 0.72 / 0.84 / 0.98 / 0.99 / 0.69 | 39 / 11 / 1 / 9 / 9 | 0/5 pass |

**Open issues**
- [F] Door-frame clearance: tracking error at doors (0.05-0.1 m) eats the 0.07 m budget between the planned tight radius and the 0.39 m body radius. Ideas [H]: tilt the camera or add a side-looking sensor to see door frames while passing, door-axis alignment with a dedicated approach controller, a smaller simulated drone (`small` worldgen profile) for the 0.8-1.0 m doors.
- [F] The map walls are fat/noisy at doors (spurious occupied cells 0.2-0.4 m inside a 1.1 m gap, seed 10): goto then plans along the flown track. Depth edge handling in the camera driver (nearest valid of each 4x4 block) is the suspect [H].
- [F] goto_room: room typing of partially seen open-plan flats gave goals in the living room or bedroom (confidence 0.47), goals only 0.8-0.9 m inside the room, and after a crash the estimate says "arrived" while the drone is elsewhere. Prefer the most confident kitchen over the nearest [H].
- [E] Replanning can dither between two distant frontiers early on (seed 11: reversal after 3 s); stronger hysteresis or a commit-to-door rule would help.
- [F] `test_websocket_server_handshake_burst_broadcast_and_input` in maverick_gateway times out in the container, with or without these changes.

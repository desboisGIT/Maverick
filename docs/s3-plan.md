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
- **Pose:** PX4's own estimate (optical flow + rangefinder), with its real drift. No ground truth pose on the host, ever. VIO comes later if drift is a problem.
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

| Topic (host side) | Type | Rate | Notes |
|---|---|---|---|
| `/maverick/camera/rgb/compressed` | `sensor_msgs/CompressedImage` (JPEG) | 5 Hz | 640x480 |
| `/maverick/camera/depth` | `sensor_msgs/Image` `16UC1` (mm, 0 = invalid) | 5 Hz | 160x120, same stamp as RGB |
| `/maverick/camera/semantic` | `sensor_msgs/Image` `mono8` (worldgen class ids) | 5 Hz | 160x120, **simulation only**: feeds the simulated detector, does not exist on hardware |
| `/maverick/camera/info` | `sensor_msgs/CameraInfo` | latched / 1 Hz | intrinsics for the 160x120 depth/semantic images; RGB intrinsics scale by 4 |
| `/maverick/camera/pose` | `geometry_msgs/PoseStamped` | 5 Hz | camera optical-frame pose in the map frame (ENU), PX4 estimate at frame time, computed on the bridge from vehicle_local_position + vehicle_attitude + the fixed camera mount |

All five carry the same `header.stamp` per frame (bridge wall clock at receipt). Frame ids: `map`, `camera_optical`. Host nodes: `mapping_node` (VoxelMap, frontiers, 2D grid), `memory_node` (simulated detector → SceneGraph), `explore` skill node (frontier exploration through `/maverick/flight/goal`), `gateway` (websocket per docs/gateway-events.md).

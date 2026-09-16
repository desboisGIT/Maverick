# S3 host stack: nodes, topics, launch

Status: implemented 2026-11-11, smoke-tested on synthetic topics (not yet with Gazebo). Tags as in the study: [F] verified, [E] estimate, [H] hypothesis, [NV] not verified. Parent: [s3-plan.md](s3-plan.md) "Integration contract"; events: [gateway-events.md](gateway-events.md).

Pattern: pure core + thin rclpy node. The cores (`VoxelMap`, `SceneGraph`, `ExplorationPlanner`, skill state machines, gateway protocol) have no ROS imports and are unit-tested; the nodes only convert messages and call them.

## Packages and nodes

| Package | Node (executable) | Core it wraps |
|---|---|---|
| `maverick_mapping` | `mapping_node` | `VoxelMap`, `detect_frontiers`; `ros_conv.py` (messages <-> numpy, `SyncBuffer`), `voxel_delta.py` (map_update deltas) |
| `maverick_memory` | `memory_node` | `SceneGraph`; `semantic_detector.py` (class-id image -> detections + noise model), `queries.py` |
| `maverick_skills` (new) | `skill_node` | `skills.py`: `ExploreSkill`, `GotoRoomSkill`, `WaypointFollower` |
| `maverick_gateway` (new) | `gateway` | `protocol.py`, `schema.py`, `wsserver.py` (stdlib websocket server) |

`maverick_skills` is its own package, not part of `maverick_flight`: the flight client is the small, safety-relevant piece (gate, failsafes, only PX4 messages) and must not depend on the planner, memory or numpy-heavy code; the S4 executive will host many skills and replace this node. Skills only publish goals on `/maverick/flight/goal`.

## Topic graph

```
bridge/Zenoh ──► /maverick/camera/{depth,semantic,info,pose,rgb/compressed}
  depth+info+pose ─► mapping_node ─► /maverick/map/grid (OccupancyGrid, 1 Hz, latched)
                                  ─► /maverick/map/frontiers (JSON, 1 Hz, latched)
                                  ─► /maverick/map/voxels (JSON deltas <= 4 Hz; reset snapshot at start,
                                     on /maverick/map/voxels_request and every 30 s)
  semantic+depth+info+pose, map/grid ─► memory_node ─► /maverick/memory/{detections,rooms,objects}
                     /maverick/memory/query ─► memory_node ─► /maverick/memory/answer (goal_for_type, find, room_of)
  /maverick/skill/request (JSON) ─► skill_node ─► /maverick/flight/goal (goto, PX4 NED, + "yaw")
     in: map/grid, camera/pose, flight/status (starts at phase "hover"), memory/answer
     out: /maverick/skill/{progress,path} (gateway formats), /maverick/memory/query
  gateway ◄─ everything above + flight/status; ─► websocket :8765; text_command ─► skill/request
```

Requests: `{"skill":"explore","budget_s":360}`, `{"skill":"goto_room","room_type":"kitchen"}`, `{"skill":"cancel"}`. Test from a shell: `ros2 topic pub --once /maverick/skill/request std_msgs/msg/String "{data: '{\"skill\":\"explore\"}'}"`.

Frames: map and `camera/pose` are PX4-local ENU (REP 103); `skill_node` converts goals to NED with `maverick_mapping.frames`, the only conversion point. The drone position is taken from the camera pose (mount offset ignored, [E] under 0.2 m).

Explore: 360 degree yaw scan (6 x 60 degree yaw goals, camera yaw as feedback) at the start and every 4 m of travel, then `ExplorationPlanner.plan`, one waypoint goal at a time, replan every 3 s, a viewpoint blacklist on stall (12 s without 0.15 m progress) or when a frontier does not resolve after a 1.5 s look. Ends with `succeeded` when no reachable frontier is left or the budget is used up. `goto_room` asks memory for `goal_for_type`, plans A* on the inflated grid, replans every 3 s, succeeds within 0.4 m of the goal, fails with a spoken reason when the room is unknown, there is no path for 10 s, or after 180 s.

Gateway text commands (tiny keyword parser, the S4 agent replaces it): "explore [for N minutes]", "go to the kitchen / living room / bedroom / bathroom / hallway / office", "stop", "land" (publishes `land` on `/maverick/flight/mission`).

## Launch

```
ros2 launch maverick_gateway s3_host.launch.py                # flight client + 4 nodes, ws on :8765
ros2 launch maverick_gateway s3_host.launch.py flight:=false  # flight client started elsewhere
```
Arguments: `flight`, `port`, `altitude` (1.2), `seed` (detector noise). Per-node parameters are listed in each node's module docstring; pass floats as floats (`max_rate_hz:=5.0`). Build: `colcon build --packages-up-to maverick_gateway` (needs the px4_msgs underlay because of `maverick_flight`).

For the orchestrator, `docker/host-entrypoint.sh` should start, after the zenoh bridge (background, restart on exit): `source /maverick/ros2_ws/install/setup.bash && ros2 launch maverick_gateway s3_host.launch.py`. `docker/compose.yaml`, service `host`: `ports: ["8765:8765"]` for the Mission Console. No new Python packages are needed: the websocket server is standard library (the image has no `websockets`), JPEG frames are passed through undecoded.

## Smoke test (no Gazebo)

```
docker compose -f docker/compose.yaml --profile tools run --rm -T tools bash -lc \
  "cd /maverick && python3 ros2_ws/src/maverick_gateway/e2e/s3_smoke.py --seed 1000003"
```
`e2e/s3_smoke.py` starts the four real nodes as processes, plays the drone (renders depth and class ids from the worldgen scene with `maverick_mapping.synthetic`, 80 x 60 px, follows the goto goals with the kinematic model, 3x real time) and a console (websocket client). It checks: the map grows, the drone flies, rooms and objects appear, every frame validates against `schema.py`, a late console gets `map_update{reset}` first with a matching voxel count, "explore"/"stop"/"go to the <room>" work. One JSON line, exit 0 on pass. Result (seed 1000003, 75 s wall): 14.5k occupied voxels at 8 s, 26.9k at 75 s, 5 rooms (living/kitchen, hallway, 2 bedrooms, bathroom) with correct-looking types, 13 object classes, "go to the kitchen" succeeded, 0 invalid frames. Perception is oracle quality (segmentation ids plus the noise model): this checks the plumbing, not the exit-test metrics C1/C2/C6, which come from the offline harness and, later, from Gazebo.

Unit tests: `python3 -m pytest` (126 tests, the new ones are `test_ros_glue.py` in mapping and memory, `maverick_skills/test`, `maverick_gateway/test`).

## Known gaps

- The flight client ignores `yaw` in goals (it streams yaw 0). Scans then time out (3 s per step, about 18 s per scan) and the camera always looks north (+y ENU). Set `scan_every_m` to 0 in `SkillConfig` until the client forwards `yaw` (`FlightSetpoint.yaw` exists). Exploration quality without yaw control is [NV] and likely poor.
- The flight client's default geofence (+-5 m) and takeoff altitude (2.0 m) do not fit an apartment explored at 1.2 m: set `geofence_*` and `altitude`.
- The battery in `status` is a placeholder (`battery_pct` parameter, 100): the simulation has no battery model. VRAM comes from `nvidia-smi` when the host container has it, else 0 / 16.
- `detections` boxes carry no `object_id` yet; detections are matched to camera frames by header stamp (a detection for a frame the gateway dropped is dropped).
- Depth integration of one 160 x 120 frame took about 33 ms (8 ms at stride 2) on the dev machine in a small map [E]; `mapping_node` has `stride` and `max_rate_hz` (default 5). Cost on a full map and on the target host is [NV].

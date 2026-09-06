# Stage S1: drone, link and failsafes

Status: plan, written 2026-11-11. Parent plan: [simulation-plan.md](simulation-plan.md) §9 (stage S1). Safety design: [architecture-study.md](architecture-study.md) §2.5 and failsafe suite §5. Tags as in the study: [F] verified, [E] estimate, [H] hypothesis, [NV] not verified.

## Goal and exit test

Split the S0 single container into the real system's shape (drone ↔ bridge board ↔ lossy Wi-Fi ↔ host PC), put the indoor drone model and Maverick's PX4 failsafe parameters in place, and prove the link-loss safety with automated fault injection.

**Exit test:** failsafe cases F1, F2, F3, F4 and F6 each pass 50/50 randomised runs, and the S0 hover still passes 20/20 through the full bridge path. F5 (VIO fault) waits for VIO in S3; F7 (spoken stop) and F8 (cloud timeout) wait for S4.

| # | Injected fault (randomised) | Pass condition |
|---|---|---|
| F1 | Wi-Fi outage of 0.2–0.4 s, 1–4 s, or 12–20 s while hovering | Short: no failsafe, deviation ≤ 0.5 m. Medium: PX4 starts landing, returns to offboard when setpoints resume, mission completes; altitude loss ≤ 1.2 m, horizontal deviation ≤ 0.5 m. Long: the host aborts the mission (gap > 5 s), the drone lands and disarms within 25 s of the outage start, and is never re-armed |
| F2 | Latency 300 ± 200 ms and 20% loss for 10–30 s | No setpoint older than 200 ms reaches PX4 (bridge stats); no flyaway (≤ 0.5 m from the commanded hold point); hold or continue, never crash |
| F3 | Kill the host flight process mid-hover | Same outcome as a long outage: lands and disarms within 25 s, never re-armed |
| F4 | Kill the bridge relay process mid-hover | PX4 offboard-loss failsafe (Land, nav_state 18) engages within `COM_OF_LOSS_T` + 1.0 s (measured 1.0–1.4 s), then lands and disarms |
| F6 | Bad goals sent to the host flight client: outside the geofence, NaN, a jump > 1 m, speed request > limit | Rejected by the host safety gate and logged; drone holds its last valid setpoint; nothing reaches PX4 |

"Lands" means: vehicle_land_detected landed and disarmed, within 40 s of the failsafe engaging. All positions for scoring come from Gazebo ground truth, read only by the referee.

## Topology

```
              network: drone (stands in for the serial cable; no netem)
 ┌─────────────── sim ───────────────┐        ┌────────────── bridge ──────────────┐
 │ Gazebo + PX4 SITL, model maverick │◄──────►│ Micro XRCE-DDS Agent (UDP 8888)     │
 │ (uxrce client → bridge IP)        │  XRCE  │ maverick_bridge relay (staleness)   │
 └───────────────┬───────────────────┘        │ zenoh-bridge-ros2dds (selected      │
                 │ network: world             │   topics only)                       │
 ┌───────────────┴───────────────────┐        └──────────────────┬──────────────────┘
 │ referee: ground truth, scenarios, │                           │ network: wifi
 │ fault injection, scoring          │                           │ tc netem on both ends
 └───────────────────────────────────┘        ┌──────────────────┴──────────────────┐
                                              │ host: zenoh-bridge-ros2dds,          │
                                              │ maverick_flight (safety gate,        │
                                              │ mission client)                      │
                                              └──────────────────────────────────────┘
```

- DDS never crosses a container boundary: every container sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. Only Zenoh crosses the `wifi` network, carrying an explicit allow-list of topics. Only XRCE crosses the `drone` network.
- The host and bridge are not on the `world` network, so they cannot read ground truth.
- One image (`maverick-sim:dev`) serves all four services for now, with different commands. Slimmer per-service images come later.
- Clocks: containers share the host kernel clock, so wall-clock stamps are comparable across containers. On real hardware this needs time sync (chrony) [E].

## Contracts

### Messages: `ros2_ws/src/maverick_interfaces`

- `FlightSetpoint`: host → bridge. `stamp` (host wall clock when issued), `position_ned` (m, PX4 local NED), `yaw` (rad, NaN = keep current).
- `FlightCommand`: host → bridge. `stamp`, `command` ∈ {ARM, DISARM, OFFBOARD, LAND, HOLD}.
- `BridgeStatus`: bridge → host, 1 Hz. Counters of forwarded and dropped (stale/invalid) setpoints and commands, plus the age of the last setpoint.

### Topics across the link (Zenoh allow-list)

| Topic | Type | Direction |
|---|---|---|
| `/maverick/flight/setpoint` | `maverick_interfaces/FlightSetpoint` | host → bridge |
| `/maverick/flight/command` | `maverick_interfaces/FlightCommand` | host → bridge |
| `/maverick/bridge/status` | `maverick_interfaces/BridgeStatus` | bridge → host |
| `/fmu/out/vehicle_local_position_v1` | `px4_msgs/VehicleLocalPosition` | bridge → host |
| `/fmu/out/vehicle_status_v1` | `px4_msgs/VehicleStatus` | bridge → host |
| `/fmu/out/vehicle_land_detected` | `px4_msgs/VehicleLandDetected` | bridge → host |
| `/fmu/out/vehicle_command_ack` | `px4_msgs/VehicleCommandAck` | bridge → host |

The host never publishes `/fmu/in/*`. Only the bridge does.

### Bridge relay rules (`maverick_bridge`)

- Telemetry: the four `/fmu/out` topics are re-published by the relay from PX4's DDS domain (1) into the ROS domain (0) that the Zenoh bridge reads, because zenoh-bridge-ros2dds only routes topics published by ROS nodes and the XRCE agent's are not. The relay is therefore on both paths; if it dies, PX4 loses setpoints (failsafe) and the host loses telemetry (gap policy).
- On each `FlightSetpoint`: drop it if older than 200 ms, stamped in the future by more than 50 ms, or containing NaN in the position. Otherwise publish `OffboardControlMode` (position) + `TrajectorySetpoint` to PX4 immediately.
- Never publish anything on its own: no setpoint repetition, no heartbeat. If fresh setpoints stop arriving, PX4's offboard-loss failsafe must trigger. This is what makes link loss safe.
- `FlightCommand`: same staleness rule (500 ms), translated to `VehicleCommand`.
- Mission logic in a pure-Python core with unit tests; thin rclpy node around it (the S0 pattern).

### PX4 configuration: `sim/` drone model and airframe

- World: optical flow needs a textured floor. On PX4's plain default world the EKF reports zero motion while ground truth shows the drone drifting away at > 1 m/s [F: measured]. S1 uses `sim/worlds/textured.sdf` (random floor tiles); every later world must have floor texture.
- Model `maverick`: starts as PX4's `x500_flow` (x500 + optical flow + LW20 downward lidar, GPS off) [F: present in PX4 v1.17.0]. Camera comes in S3, GPS for outdoor worlds in S6.
- Airframe file `sim/px4/airframes/4500_gz_maverick` (sources 4021_gz_x500_flow), installed into the PX4 rootfs. It carries Maverick's parameters with `param set-default`, which replaces the S0 `PX4_PARAM_*` workaround:
  - `NAV_DLL_ACT 0` (no ground station needed)
  - Offboard loss: `COM_OF_LOSS_T 0.5`, `COM_OBL_RC_ACT 4` (Land). Measured in S1 [F]: Hold is not available without GPS (it silently becomes Land), Position/Altitude take no action without RC (unsafe), and `COM_FAIL_ACT_T` has no effect on this path. So PX4's reaction to link loss is **land immediately, recoverable**: if setpoints resume during the descent, PX4 returns to offboard by itself. The planned "hold, then land after 10 s" is replaced by this. The host decides whether to resume (gap ≤ 5 s) or abort (§ host flight client)
  - Indoor limits: horizontal speed about 1 m/s [E], climb/descent limits, a local geofence if PX4 supports one without GPS [NV]
  - The values come from measurement, documented in the airframe file comments.

### Host flight client (`maverick_flight`)

- `SafetyGate` (pure Python): rejects NaN/inf, goals outside a configured box geofence, jumps > 1 m from the current setpoint, and enforces a maximum speed by stepping towards goals. Every rejection is logged with a reason.
- Mission client node: streams setpoints at 20 Hz to the bridge, sends commands, re-engages offboard after a medium outage if the mission is still active, and never re-arms after a failsafe landing.
- Link-gap policy: if telemetry from the bridge was missing for more than 5 s, the mission is aborted: the client stops streaming setpoints, sends LAND when the link returns, and never re-arms. Shorter gaps: keep streaming so PX4 resumes offboard.
- Accepts goals on `/maverick/flight/goal` (the stand-in for the S4 executive), which is how F6 injects bad goals.

### Referee (`sim/referee`)

- Python package run in the `referee` container on the `world` network.
- Ground truth: drone pose from Gazebo transport.
- Fault injection: `tc netem` on the `wifi` interfaces and process kills, via the Docker CLI with the Docker socket mounted. This is for local development only, because the socket is root-equivalent.
- Scenario runner: `just test failsafe --runs 50 --seed N` runs randomised F1/F2/F3/F4/F6 cases, each on a fresh stack, writes `runs/s1-failsafe-<timestamp>/results.jsonl` and a summary.

## Work split

| Package | Owner | Depends on |
|---|---|---|
| `maverick_interfaces` | orchestrator | none |
| Drone model, airframe, PX4 failsafe measurement | agent A | none |
| Compose topology, `maverick_bridge`, Zenoh link, netem profiles | agent B | interfaces |
| Integration of A + B, hover regression through the bridge | orchestrator | A, B |
| `maverick_flight`, referee, failsafe suite | agent C | integrated stack |
| 50-run exit test, docs, commit | orchestrator | all |

Agents running simulations at the same time must use different `COMPOSE_PROJECT_NAME`s so their stacks don't collide.

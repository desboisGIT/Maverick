# Maverick PX4 airframe and drone model

Files:
- `sim/px4/airframes/4500_gz_maverick`: airframe (sources `4021_gz_x500_flow`, then `param set-default` for Maverick's parameters, each with its measured reason).
- `sim/px4/airframes/4501_gz_maverick_small`, `sim/models/maverick_small/`: the DEFAULT drone, the small indoor quad (section "Small drone" below). `MAVERICK_DRONE=small|x500` (default small) selects it; the x500 files below stay as they are.
- `sim/models/maverick/{model.config,model.sdf}`: Gazebo model `maverick`, a plain include of PX4's `x500_flow` (x500 + optical flow + LW20 downward rangefinder, no GPS). Plus the S3 forward camera (RGB, depth, semantic; mount in `camera_mount.json`).

## Installing into the sim container (bind mounts)

PX4 SITL starts with `PX4_SIM_MODEL=gz_maverick`. `rcS` finds the airframe by file name (`<id>_gz_maverick`) and `px4-rc.gzsim` spawns `file://$PX4_GZ_MODELS/maverick/model.sdf`, so mount:

```
/abs/sim/px4/airframes/4500_gz_maverick -> /opt/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/4500_gz_maverick  (ro)
/abs/sim/models/maverick               -> /opt/PX4-Autopilot/Tools/simulation/gz/models/maverick                                  (ro)
env: PX4_SIM_MODEL=gz_maverick
```

The airframe sets no `PX4_PARAM_*` workaround any more; `NAV_DLL_ACT 0` is in the file. The Gazebo model instance is named `maverick_0` (x500) or `maverick_small_0` (small). Compose mounts both drones; the entrypoint sets `PX4_SIM_MODEL` (`gz_maverick_small` or `gz_maverick`) from `MAVERICK_DRONE`, an explicit `PX4_SIM_MODEL` wins.

## World requirement (important)

PX4's `default` world has a plain grey ground plane. The optical-flow camera then reports zero flow with quality 255, so the EKF believes the vehicle is stationary while it really drifts away. A scripted hover in `default` reported 0.04 m drift from `vehicle_local_position` while Gazebo ground truth showed the drone leaving at over 1 m/s (50 m in 40 s). The S0 hover test cannot see this because it only reads PX4's own estimate.
In a world with a textured floor (a generated test world with about 360 randomly coloured 4 mm tiles, name `textured`) the same hover stays within 0.07 m of its mean position in ground truth. A world used with this model needs a textured floor. For `PX4_GZ_WORLD=<name>` the file must be `worlds/<name>.sdf` and `<world name="<name>">` must match.

## Measurements (PX4 v1.17.0 SITL, model gz_maverick, textured world, no GPS)

| # | Question | Result |
|---|---|---|
| 1 | Flow position hold | Hover 20-30 s at 2 m: PX4 local position drift 0.02-0.11 m, ground truth within about 0.07 m. Only in a textured world (see above). |
| 2a | Gap 0.3 s, `COM_OF_LOSS_T` 0.5 | Nothing happens (10/10 runs, nav_state stays 14). |
| 2b | Gap >= 1 s | nav_state 18 (AUTO_LAND), failsafe flag 1, 1.0-1.4 s after the last offboard message. Position held within 0.03 m. Descent about 0.1-0.25 m/s, so 2 s costs 0.2 m, 4 s costs 0.9 m. |
| 2c | Hold as the offboard-loss action (`COM_OBL_RC_ACT` 5) | Not available: nav_state goes to 18 (Land), identical to 4. `commander mode auto:loiter` is also refused. Hold needs a global position. |
| 2d | `COM_OBL_RC_ACT` 0 (Position) / 1 (Altitude) | No failsafe at all in 6 s: nav_state stays 14 on stale setpoints, vehicle keeps the last setpoint. Unsafe, not used. |
| 2e | `COM_FAIL_ACT_T` | No effect on the offboard-loss path: Land starts at once, with 10 s or 5 s. |
| 2f | Setpoints resume during the Land | PX4 returns to offboard (nav 14) by itself within 0.1-0.4 s, no `DO_SET_MODE` needed, and the vehicle climbs back to the commanded setpoint. |
| 2g | Long gap | Lands and disarms 14.7-15.2 s after the last setpoint (from 2 m). Setpoints resuming after the disarm do not re-arm. |
| 3 | Geofence without GPS | `GF_MAX_HOR_DIST` 2 / `GF_ACTION` 2: no action when flown 4 m out. One run with `GF_MAX_VER_DIST` 3 refused to arm ("ekf2 missing data"), not repeated. GF_* uses the global position, so it does not work. The host safety gate is the geofence. |
| 4 | Limits | 4 m step goto, default limits (12 m/s, 3 m/s2): peak 2.96 m/s, 0.73 m overshoot. `MPC_XY_VEL_MAX` 1.0, `MPC_ACC_HOR` 1.5: peak 1.08 m/s, 0.15 m overshoot, 8 s to 0.2 m. Climb 2 to 4.5 m peak 0.53 m/s, descent 2 to 0.8 m peak 0.38 m/s (limited by acceleration, below the 1.0/0.7 caps). |

Validation with the final airframe only (no `PX4_PARAM_*` overrides), 5 runs each:

| Gap | Result |
|---|---|
| 0.3 s | 5/5: no failsafe, max horizontal deviation 0.02 m, no altitude loss. |
| 2 s | 5/5: Land started 1.0-1.3 s after the last setpoint, back in offboard 0.1-0.4 s after setpoints resumed, altitude loss 0.16-0.24 m, horizontal deviation 0.02 m, final z back to -2.0 m, still armed. |
| 15 s | 5/5: Land started 1.0-1.4 s after the last setpoint, disarmed at 14.7-15.2 s, horizontal deviation 0.09-0.33 m, no re-arm. |

Consequence for the design: there is no PX4 "hold, then land after 10 s". The PX4 backstop is "land at once, recoverable by resuming setpoints". Any hold-in-place behaviour for 1-4 s outages has to come from the host flight client (keep the last position setpoint) and from the link recovering fast enough; PX4 itself starts a slow descent after 0.5 s plus about 0.5 s of detection latency.

## Take-off/landing drift

Setup: full SITL (`maverick-sim:s3`, textured world, no GPS), a scratch rclpy client flying take-off (offboard, position setpoint at 2 m from the floor at once) -> 3 s hover -> PX4 Land -> disarm -> 4 s on the floor, 5 cycles back to back in one PX4 session, each cycle holding the estimated position at arm. Ground truth from `gz topic /world/textured/dynamic_pose/info`, both logged at 10 Hz. Estimate error per cycle = (estimate displacement) - (ground-truth displacement) over the cycle, horizontal. Height-origin drift = same for z (estimate minus truth), so a physical hover that sinks while the estimate says 2.0 m shows up here. Three runs of 5 cycles each per row.

| | Before (4500 airframe as it was) | After |
|---|---|---|
| Height-origin drift per cycle | -0.146 to -0.163 m, every cycle (hover above the floor 1.52 -> 0.91 m over 5 flights at the same setpoint) | -0.013 to -0.003 m (hover 1.80-1.86 m, constant) |
| Physical displacement per cycle (mean / median / max) | 0.18 / 0.17 / 0.45 m (15 cycles) | 0.15 / 0.15 / 0.27 m (13 cycles) |
| Estimate error per cycle (mean / median / max) | 0.28 / 0.22 / 0.62 m | 0.18 / 0.16 / 0.44 m |

Cause of the height drift [F]: `EKF2_MIN_RNG` is the range the terrain estimator assumes while on the ground (default 0.01 m) but the LW20 reads 0.17 m at rest. At lift-off `dist_bottom` fell to 0.02 m and the 0.15-0.16 m mismatch was absorbed into z, once per flight, which equals the measured per-cycle step. `EKF2_HGT_REF` = range (with `EKF2_RNG_CTRL` 2) and `EKF2_BARO_CTRL` 0 did not change it (same -0.165 m), so it is not the baro. Fix: `EKF2_MIN_RNG 0.17` (tested 0.15 and 0.17, both remove it).

Horizontal: NOT at the target (<= 0.1 m physical, <= 0.2 m estimate per cycle). Medians are 0.15 m and 0.16 m but single cycles reach 0.27 m and 0.44 m, and run-to-run noise is as large as any parameter effect I could resolve with 15 cycles per setting. What the data says [E]:
- The estimate tracks truth to within 0.05-0.15 m while the drone is above about 1.2 m and fails below it: regressing estimated against true velocity gives slope 1.0 above 1.2 m and 0.4-0.7 below, in the first 0.3 m of climb and the last 0.5 m of descent (rmse about as large as the speed itself). Both phases contribute: roughly 0.2-0.4 m of climb and 0.2-0.3 m of landing displacement in the bad cycles.
- Tried and rejected (no gain, or worse, over 5-cycle runs): `EKF2_OF_GATE` 10 and 5, `EKF2_OF_DELAY` 0, `EKF2_OF_N_MIN/MAX` 0.03-0.05/0.1-0.2, `EKF2_HGT_REF` range, a denser noise-textured floor (so the sparse tiles of the `textured` world are not the cause), `MPC_ACC_UP_MAX` 1.5 with `MPC_JERK_MAX` 2, a 0.3 m/s ramped climb setpoint, `MPC_XY_VEL_I_ACC` 0, flow/rangefinder mounting offsets (`EKF2_OF_POS_X/Z`, `EKF2_RNG_POS_Z`).
- Kept: `MPC_TKO_RAMP_T` 0.5 (the 3 s default thrust ramp left the drone on the floor 1.9-4.7 s with the land detector already "in air" and the controllers active; 0.5 gives about 1.2 s) and `EKF2_OF_GYR_SRC` 1. Both are weakly supported: the runs with them (Y/Z/Q family, 8 runs) were no worse and the displacement distribution a little tighter, but not significantly.
- `MPC_LAND_SPEED` 0.4 gave one run of 0.10-0.23 m displacement / 0.09-0.30 m estimate error per cycle; not enough to adopt, not repeated.
- Where to go next: the flow-based velocity near the floor is the weak link (the Gazebo flow sensor reports quality 0 on about half its packets at dist_bottom < 0.3 m). A real fix is probably outside PX4 parameters: a longer-focus or higher-resolution flow sensor in the model, or the S1 flight client climbing/landing in a position-hold with a low horizontal gain until 0.5 m.
- One run in 15 refused to arm after the 3rd landing with "Preflight Fail: horizontal velocity unstable" (cycles 4 and 5 not flown); the check also flickers in the other runs. Not seen before the changes in 15 cycles, one case is not enough to blame the parameters.

S1 checks re-run with the new airframe: hover quality does not degrade (ground-truth hover span 0.03-0.15 m over 3 s, as before). Offboard-loss unchanged, 2 runs each: gap 2 s: Land (nav 18) 0.9-1.0 s after the last setpoint, altitude loss 0.12-0.21 m, horizontal deviation 0.05-0.10 m, back in offboard and at the commanded height when setpoints resume; gap 15 s: Land 0.9-1.0 s after, disarmed 14.0-14.1 s after the last setpoint, horizontal deviation 0.31-0.34 m (before: 14.7-15.2 s, 0.09-0.33 m).

## Camera (S3)

The model carries `camera_link` (5 g, so flight is unchanged) with three co-located sensors at 5 Hz, 640x480, horizontal FOV 1.274 rad (the OAK-D Lite depth FOV), clip 0.2 to 19.1 m: `rgb` (camera), `depth` (depth_camera, float32 m) and `semantic` (segmentation, semantic type, worldgen class ids). Gazebo topics (global names, set by `<topic>`): `/maverick/camera/rgb`, `/maverick/camera/depth`, `/maverick/camera/semantic/labels_map`. Mount (single source of truth, `camera_mount.json`; `model.sdf` is tested against it): sensor frame at x 0.12, y 0, z 0.10 m from `base_link` (FLU), level, looking forward. The bridge turns it into the optical frame (z forward, x right, y down); `maverick_bridge.camera_pose` holds the maths.

Data path (updated for S3b: 320x240 at 10 Hz, PNG depth, see below): `sim/camera_driver` (sim container, started by `sim-entrypoint.sh` once PX4 is ready) pairs the three images by sim time, downsamples depth (nearest valid return of each 4x4 block) and semantic (block-centre sample) to 160x120, JPEG-encodes RGB (quality 80) and streams `maverick_bridge.camera_protocol` frames over TCP to the bridge (10.231.1.3:5600, drone network only). `maverick_bridge camera_relay` attaches the PX4 pose and publishes `/maverick/camera/{rgb/compressed,depth,semantic,info,pose}` with one stamp; Zenoh forwards them best-effort. The map frame origin is the PX4 local origin (the spawn point), not the Gazebo world origin. Disable the driver with `CAMERA_DRIVER=0`.

## Visual odometry (S3b)

Camera stream: all sensors render 320x240 at 10 Hz (was 640x480 at 5 Hz). Wire to host: RGB JPEG q80 + lossless 16-bit PNG depth (320x240) + PNG semantic ids (160x120), about 15 kB per frame, 1.1 Mbit/s at 9 Hz [F] (budget 10 Mbit/s). Why 3.8 Hz instead of 5 Hz [F]: the rate is in *simulation* time; with 640x480 RGB + depth + segmentation the sim ran at real-time factor 0.62-0.76 (frames arrived in matching counts at the driver, nothing was lost in pairing or the link), so 5 Hz sim = 3.1-3.8 Hz wall. At 320x240 / 10 Hz the sim delivers 8.8 Hz wall (RTF 0.88); the render path (Gazebo ogre2 + CPU image copies; GPU utilisation 0%), not our code, is the limit.

Method, measured on a 35 s recording of the generated apartment seed 1000000 replayed offline against ground truth (rtabmap 0.23.13): ORB features (rgbd_odometry, also with a lower FAST threshold, 1000 features, MinInliers 8) diverge within 10 s (the walls, ceiling and furniture are untextured, "Not enough inliers 6/12, matches 27"); vision+ICP likewise; **ICP on the depth point cloud (icp_odometry, point-to-plane)** tracks 4 m of travel with 0.05-0.07 m drift, 5 ms per frame [F, one recording]. Using PX4's own pose as motion guess made it worse in replay. Chosen: `vo.launch.py mode:=icp`. Output anchored once to PX4's camera pose, converted by `vo_adapter`; the bridge relay converts to `VehicleOdometry` on `/fmu/in/vehicle_visual_odometry` (no `_v1` suffix; checked on the live uXRCE topic list; `timestamp_sample` is timesynced by the XRCE client, EKF fuses it).

Airframe: `EKF2_EV_CTRL` = 9 (horizontal position + yaw; enabled through `MAVERICK_EV_CTRL`, the S3 runner sets it; default 0 because with 9 and no vision data 2 of 6 S1 hover missions ended in land_timeout), `EKF2_EV_DELAY` 50 ms (guess, not measured; estimator_aid_src_ev_pos showed fusion, innovation about 3 mm in hover), `EKF2_EVP_NOISE` / `EKF2_EVA_NOISE` 0.05. Velocity and EV height not used.

Results (held-out seeds 1000000-1000002, explore budget 360 s, all FAIL): coverage 0.53 / 0.80 / 0.75, collisions 5 / 5 / 14, estimate error mean 0.54 / 1.59 / 3.79 m, max 5.4 / 4.7 / 10.3 m (flow only before: C1 0.44, 47 collisions). In hover the odometry error is 2 mm; in motion it is 0.03-0.1 m until the drone translates parallel to a plain wall, where the depth geometry is degenerate (a +0.9 m sideways move registered as -0.15 m); the drone then hits a door frame (doors are 1.1 m, the x500 needs 0.78 m), crashes, and the estimate after the crash is meaningless. The estimate-error numbers above therefore include the post-crash flight. Open: the forward-only camera cannot constrain sliding along plain walls (options: tilt the camera down to see floor tiles, a downward second camera, use PX4 flow velocity inside the ICP, lower speed near doors).

## Simulated mocap localisation (`MAVERICK_LOCALISATION=mocap`)

`sim/mocap_driver` (sim container) reads maverick_0's pose from Gazebo (about 22-30 Hz), adds 2 mm / 0.5 deg noise and +-5 ms sample-time jitter, sends UDP (`maverick_bridge.mocap_protocol`) to the relay on the drone network; the relay converts like the VO path (same staleness rule) to `/fmu/in/vehicle_visual_odometry`. Position origin = spawn (x, y from `PX4_GZ_MODEL_POSE`). `mocap` and `vo` set `EKF2_EV_CTRL` 9; `flow` keeps 0.

- `EKF2_EV_DELAY` measured on seed 1000000, 60 s explore, estimate error before the first crash (p95 / mean): delay 0: 0.77 m p95; 40 ms: 0.07 m p95, 0.067 m mean; 100 ms: 1.95 m p95. Set 40 ms.
- Hover with mocap on (3 seeds): drift 0.03-0.06 m, lands and disarms every time (the EV-without-data land_timeout does not occur when data flows).
- Mocap stops mid-hover (driver killed): nav_state stays 14, no failsafe, altitude constant, PX4 carries on with flow and wanders 0.7 m in 29 s.
- Camera frames now carry their capture time (`t_wall`) and the PX4 pose is interpolated at that time (it used the newest pose, a 0.07 m / several degree error while moving or turning).
- Planner: `margin` 0.10 -> 0.16, waypoint `arrive_radius` 0.2 -> 0.1 (with accurate localisation the drone still clipped door frames at 0.34-0.38 m clearance against the 0.39 m body radius: corners were cut by up to the arrive radius and PX4 overshoots 0.15 m on turns). Door frames at 1.1 m leave only +-0.16 m of total tolerance for an x500.

Results with mocap, held-out seeds 1000000-1000004, 360 s budget (NOT exit results):

| seed | coverage | collisions | C6 | estimate err mean / max (m) |
|---|---|---|---|---|
| 1000000 | 0.83 | 2 | 1.0 | 0.025 / 0.125 |
| 1000001 | 0.60 | 1 | 1.0 | 0.030 / 0.104 |
| 1000002 | 0.98 | 0 (pass) | 1.0 | 0.032 / 0.149 |
| 1000003 | 0.81 | 20 | 1.0 | 0.079 / 0.971 |
| 1000004 | 0.82 | 5 | 1.0 | 0.044 / 0.426 |

goto_room (explore first, then "kitchen"): 0/5 (explore crashes or does not find a route: 4-55 collisions). Localisation meets its target here (error <= 0.15 m before any crash); the remaining collisions are in planning and flight (door frames and corners, tracking overshoot), not localisation.

## Small drone (default, `MAVERICK_DRONE=small`)

Owner decision 2026-11-11: the simulated drone is a small indoor quad, because the x500 (0.78 m across) cannot pass real interior doors (0.7-0.9 m). Files: `sim/models/maverick_small/` (standalone SDF, camera mount in `camera_mount.json`), `sim/px4/airframes/4501_gz_maverick_small` (sources 4500, so failsafes, limits, EKF2 and external-vision settings are identical; adds the frame-specific values). Gazebo model instance `maverick_small_0`. Same sensors and Gazebo topics as the `maverick` model (RGB/depth/semantic camera, optical flow, downward rangefinder, IMU, baro, mag, navsat).

Target: 3.5 inch ducted ("cinewhoop") class quad with prop guards, circumscribed radius 0.15 m (0.30 m across), about 0.5 kg all-up, thrust-to-weight 3.5, speed limits unchanged (1 m/s indoor).

Derivation (design values [E]; checked values [F] in the next list):

| Quantity | Value | Basis |
|---|---|---|
| Motor positions | (+-0.067, +-0.067) m, 0.095 m from the centre | duct outer radius 0.055 m, 0.095 + 0.055 = 0.15 m circumscribed [E] |
| Collision | one cylinder, radius 0.15 m, 0.09 m high, around body and guards | the planner/referee model the body as a 0.15 m disc; the x500 collision geometry is boxes [E] |
| Mass | base 0.38 + 4 x 0.03 (motor + prop + duct share) + camera 0.005 + flow 0.005 + range 0.001 = 0.511 kg | typical 3.5 inch cinewhoop with 4S battery, camera, depth sensor, bridge board [E] |
| Inertia | base 0.38 kg as a cylinder r 0.06 m, h 0.07 m: Ixx = Iyy = 5.0e-4, Izz = 6.8e-4; the rotor links add m y^2 per motor in Gazebo: total about Ixx 1.0e-3, Izz 1.8e-3 kg m^2 | solid-body formulas [E] |
| Thrust | hover 0.511 x 9.81 / 4 = 1.25 N per motor; T/W 3.5 gives 4.4 N max; `motorConstant` k = 4.41e-6 N/(rad/s)^2 with `maxRotVelocity` 1000 rad/s (4.41 N) | PX4's gz output is ω = 150 + 850 u rad/s (SIM_GZ_EC_MIN 150, MAX 1000); hover at u = 0.45 means ω = 532 rad/s, k = 1.25 / 532^2 [E] |
| `momentConstant` | 0.01 m (x500: 0.016) | smaller props, torque/thrust ratio scales with diameter [E]; `CA_ROTOR*_KM` = +-0.01 is the same number (x500 uses 0.05 against 0.016 in the model) |
| `rotorDragCoefficient`, `rollingMomentCoefficient` | 4.2e-5, 5e-7 | x500 values scaled by k (x500 ratio 9.4 x k) [E] |
| Motor time constants | 0.008 s up, 0.016 s down (x500 0.0125 / 0.025) | small motors spin up faster [E] |
| Belly sensors | flow camera near clip 0.01 m, rangefinder min 0.01 m, both 0.03 m under the centre; base 0.05 m above the floor at rest | x500's 0.1 m limits would blind them; `EKF2_MIN_RNG` 0.03 = rangefinder reading at rest [E] |
| Camera mount | x 0.07, z 0.04 m from `base_link` (x500: 0.12, 0.10), same optics | in front of and above the duct tops so the ducts are not in the image; scaled with the frame [E] |
| Allocation | `CA_ROTOR0..3_PX/PY` = (+-0.067, +-0.067) in the 4001 channel order | must equal the model motor positions [F by construction] |
| `MPC_THR_HOVER` | 0.45 | from the thrust derivation [E], measured below [F] |
| Rate gains | roll/pitch P 0.02, I 0.03, D 0.0004; yaw P 0.06, I 0.03 | PX4 defaults (0.15 / 0.2 / 0.003, yaw 0.2 / 0.1) scaled by about 1/8 to 1/3: effective authority per unit controller output is thrust over inertia, which is about 15x the x500's [E] |

Hover results with this airframe (20 trials, each on a fresh stack, `just hover-trials 20`, through the bridge, take-off to 2 m, 10 s hover, land) [F]:
- 20/20 pass; maximum altitude error 0.062 m (limit 0.15), maximum horizontal drift 0.15 m (limit 0.2).
- Hover thrust estimate of PX4's estimator: 0.437-0.446 (design 0.45): hover sits at 44 % throttle. Mean motor commands 0.445-0.454.
- Rate loop (PX4 log, airborne window about 14 s per trial, `scripts/ulog_rates.py`): angular-rate RMS about the mean roll 0.0118 / pitch 0.0095 / yaw 0.0087 rad/s (mean over trials; worst trial 0.021 / 0.014 / 0.018 rad/s, i.e. 0.5-1.2 deg/s). The spectrum above 8 Hz has at most 0.0015 rad/s RMS, and the dominant components are below 1 Hz at under 0.004 rad/s: no sustained rate oscillation. The numbers are gyro noise plus the position loop's small corrections; no gain retuning was needed beyond the initial scaling [F, hover only; manoeuvres are covered by the S3 runs].
- S1 failsafe suite on the small drone (`just test-failsafe --runs-per-case 3`): F1, F2, F3, F4, F6 all 3/3 [F] (F4: failsafe 0.43-0.48 s after the kill, lands and disarms 8.5 s later; F3: lands and disarms 9.1 s after the host kill).
- S2, hover in 10 generated worlds (small profile, tuning seeds 10-19, flow only, `just test-s3 --scenario hover --seeds 10-19`): 8/10 pass at the 0.3 m drift limit [F]. 3D drift per seed 0.29 / 0.10 / 0.08 / 0.15 / 0.27 / 0.14 / **0.30** / **0.37** / 0.12 / 0.22 m (seeds 16 and 17 fail by 0.002 and 0.07 m). The x500 on its own generated worlds for the same seeds (a different flat per profile): 10/10 pass, 0.04-0.12 m except one 0.22 m. In the textured test world the small drone's estimate-frame hover is as steady as the x500's (position std 0.02-0.03 m after the 2-3 s settle). Not diagnosed: flow quality in these sparsely textured floors is mediocre for both drones (quality 32-128 on most packets, 40 ms integration instead of 20 ms), so the likely cause is flow-only dead reckoning error scaling with the quicker lateral response of the light frame [H, NV]. Mocap and VO runs are not affected.
- S3 with mocap on the small drone (not exit results): held-out explore 4/5 pass, 0 collisions in 5 runs, coverage 0.89-0.985; goto_room 1/5 (every run arrives, the goal ends too close to the room edge). Details and the planner values: docs/s3-plan.md "Small drone". The hover/S1/S2 numbers above were measured with the final model and airframe.

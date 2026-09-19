# Maverick PX4 airframe and drone model

Files:
- `sim/px4/airframes/4500_gz_maverick`: airframe (sources `4021_gz_x500_flow`, then `param set-default` for Maverick's parameters, each with its measured reason).
- `sim/models/maverick/{model.config,model.sdf}`: Gazebo model `maverick`, a plain include of PX4's `x500_flow` (x500 + optical flow + LW20 downward rangefinder, no GPS). Plus the S3 forward camera (RGB, depth, semantic; mount in `camera_mount.json`).

## Installing into the sim container (bind mounts)

PX4 SITL starts with `PX4_SIM_MODEL=gz_maverick`. `rcS` finds the airframe by file name (`<id>_gz_maverick`) and `px4-rc.gzsim` spawns `file://$PX4_GZ_MODELS/maverick/model.sdf`, so mount:

```
/abs/sim/px4/airframes/4500_gz_maverick -> /opt/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/4500_gz_maverick  (ro)
/abs/sim/models/maverick               -> /opt/PX4-Autopilot/Tools/simulation/gz/models/maverick                                  (ro)
env: PX4_SIM_MODEL=gz_maverick
```

The airframe sets no `PX4_PARAM_*` workaround any more; `NAV_DLL_ACT 0` is in the file. The Gazebo model instance is named `maverick_0`.

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

Data path: `sim/camera_driver` (sim container, started by `sim-entrypoint.sh` once PX4 is ready) pairs the three images by sim time, downsamples depth (nearest valid return of each 4x4 block) and semantic (block-centre sample) to 160x120, JPEG-encodes RGB (quality 80) and streams `maverick_bridge.camera_protocol` frames over TCP to the bridge (10.231.1.3:5600, drone network only). `maverick_bridge camera_relay` attaches the PX4 pose and publishes `/maverick/camera/{rgb/compressed,depth,semantic,info,pose}` with one stamp; Zenoh forwards them best-effort. The map frame origin is the PX4 local origin (the spawn point), not the Gazebo world origin. Disable the driver with `CAMERA_DRIVER=0`.

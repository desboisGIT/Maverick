# Maverick PX4 airframe and drone model

Files:
- `sim/px4/airframes/4500_gz_maverick`: airframe (sources `4021_gz_x500_flow`, then `param set-default` for Maverick's parameters, each with its measured reason).
- `sim/models/maverick/{model.config,model.sdf}`: Gazebo model `maverick`, a plain include of PX4's `x500_flow` (x500 + optical flow + LW20 downward rangefinder, no GPS). Nothing new yet; the camera arrives in S3.

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

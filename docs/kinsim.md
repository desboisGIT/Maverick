# kinsim: a fast kinematic simulator for tuning

Status: built 2026-11-12 (`sim/kinsim/`). Tags as in the study: [F] verified (measured, by running), [E] estimate, [H] hypothesis, [NV] not verified.

## What it is for

A Gazebo run of S3 explore takes 3 to 4 minutes of wall time (real time, one stack per run). That makes tuning slow and
leaves one run per seed, with +-15 points of run-to-run noise (docs/s3-plan.md, "Run-to-run noise"). kinsim runs the
**real drone-stack modules** on simulated time, with a ray-cast camera and a vehicle model instead of Gazebo and PX4, so
agents can look at 60 apartments in about five minutes on one machine and use Gazebo only to confirm.

It replaces `explore_offline.py` as the tuning tool. That harness ran its own simplified loop (97 % pass offline against
about 60 % in Gazebo [F, docs/s3-plan.md]); kinsim runs the code that flies (skills, guard, mapping, memory, flight
mission), models the things that made the old harness too kind (tracking error, estimate error, contact, map noise), and
reports how well it agrees with Gazebo on runs it was not fitted to.

**Not an S3 or S4 result.** Numbers from kinsim are never reported as exit results; see "When to trust kinsim".

## Design

```
                    sim/kinsim/kinsim/stack.py  (lockstep, dt = 25 ms, simulated time)
 worldgen SDF --> KScene --raycast--> DepthCamera --depth (mm, 320x240 pixels the nodes read)--+
   (map frame)      |  contact                              semantic ids + depth (160x120)    |
                    v                                                                         v
              Vehicle (PX4 position   <-- setpoints (20 Hz) -- FlightMission + SafetyGate   VoxelMap.integrate_depth   (real)
              control, estimate       --> estimate + camera pose (10 Hz, latency)  ---> mapping 5 Hz, clearance, grid 1 Hz
              error, contact, crash)                                                     |
                    ^                                                                    v
                    +------ goals (goto, speed, yaw) ---- SkillRunner = skill_node glue + Executive + skills + guard (10 Hz)
                                                           MemoryShim = memory_node glue + SemanticDetector + SceneGraph (goto, search)
 scoring: maverick_mapping.metrics (coverage, surface error, C6) + referee.s3_metrics (collisions) + referee.s3_verdicts / s4_verdicts
```

| Part | Real (imported unchanged) | Modelled by kinsim |
|---|---|---|
| Mapping | `VoxelMap` (integration, projection), `clearance`, `frontiers` | depth images by ray casting; 5 Hz limiter with arrival jitter; 0.1 s transport latency |
| Skills | `Executive`, `ExploreSkill`, `GotoRoomSkill`, `search`, `come_to`, `guard`, planner, A* | the node's scheduling glue is copied (`SkillRunner`), pinned by `tests/test_parity.py` |
| Memory | `SemanticDetector` (noise model), `lift_detection`, `SceneGraph`, `answer_query` | the segmentation image by ray casting; message plumbing replaced by calls |
| Flight client | `FlightMission` (goal handling, setpoint stepping), `SafetyGate` (speed, jump, geofence) | started in HOVER at 2.0 m (take-off not simulated) |
| Vehicle | none | PX4 offboard position control as a cascade with limits (`vehicle.py`), estimate noise, contact, rare crash |
| Camera | `camera_mount.json` (mount, FOV, clip, size) | exact slab ray casting on the worldgen collision boxes (+ visual-only rugs, bananas, bread); pixels the nodes read (`[::2]` for mapping, `[1::2]` for the detector); mm quantisation |
| Scoring | `GroundTruth`, `c6_area_recall`, `collisions`, `verdict_explore`, `verdict_goto`, `summarise` | the same functions on kinsim's own map and ground-truth track |

Deterministic per (seed, run index): all noise comes from `numpy.random.SeedSequence([seed, run, ...])` streams;
two processes give identical rows [F, `runs/kinsim-v3` and `kinsim-v4` agree row for row].

## Usage

```
just kinsim --scenario explore --seeds 20-79 --jobs 12 --out runs/kinsim-explore
just kinsim --scenario goto_room --seeds 20-29 --jobs 12
just kinsim --scenario search --seeds 10-19 --object banana        # not calibrated
python3 -m kinsim --scenario explore --seeds 20-23                  # inside the sim container, PYTHONPATH=sim/kinsim
```

`results.jsonl` rows have the referee's format (`verdict`, `reason`, `metrics`, `goto`, ...) plus `sim: "kinsim"`, `sim_s`
(simulated seconds), `cpu_s` and `kin` (guard stats, contacts, crashes, event counts, track features); `summary.json` adds
`x_real_time_per_core` and `runs_per_minute`. `--runs-per-seed N` repeats seeds with different run indices (different noise).
`--map-stride 2|4` selects the mapping node's depth sampling (below). Held-out seeds (>= 1 000 000) are refused unless
`--held-out-ok` is given: kinsim is for tuning seeds (docs/seed-ledger.md).

## Speed [F]

Measured on the 60-run calibration batch with 12 worker processes on a 16-core machine that was also running other agents'
simulator stacks (load average 30 to 55), so wall times are pessimistic; the per-core figure uses each run's CPU time.

| Setting | CPU time per run | Simulated time per run | x real time per core | Runs per minute, 12 busy workers (measured) | Runs per minute, 12 idle cores [E] |
|---|---|---|---|---|---|
| explore, `--map-stride 4` (default, 60 runs) | 30.5 s | 212 s | **6.9** | 13.1 (load 40-55) | about 23 |
| explore, `--map-stride 2` (real launch file, 60 runs) | 100 s | 214 s | **2.1** | 2.6 (load 40-55) | about 7 |
| goto_room, stride 4 (10 runs, adds memory node) | 107 s | 257 s | 2.4 | 1.4 | about 7 |

The "idle" column is 12 x 60 / CPU seconds, not measured on an idle machine.

The 50x per core target is **not reached**. Where the time goes (explore, stride 4): `VoxelMap.integrate_depth` 70 %
(28 ms per frame at 5 frames per simulated second: it is the real mapping code and the stack's own cost), ray casting 15 %
(6 ms per frame, down from 210 ms for the offline harness' brute force), the rest is skills, planner and vehicle (planner
steps are cheap, 0.2 s per run). Going faster means integrating fewer or smaller depth images, which changes the map the
skills see; `--map-stride 4` (80x60 mapping pixels instead of 160x120) is the one speed/fidelity knob, and its effect is
measured below. The throughput that matters for tuning is runs per minute: kinsim does 12 to 13 explore runs per minute
here against about one run per minute and stack for Gazebo (3 to 4 minutes per run, one stack each).

## Calibration

### What was fitted and on what

Only vehicle, estimator, contact and sensor-path parameters were tuned (`VehicleParams` in `vehicle.py`, stored in
`kinsim/calibrated.json`). **No drone-stack parameter was touched.** Data: existing Gazebo runs on tuning seeds only,
all with the current explore / guard / EV-gate code (guard v4, `498f2b3`, results of 2026-11-12, mocap, small drone):

| Source | Runs | Seeds | Used for |
|---|---|---|---|
| `runs/g4-explore` | 10 | 20-29 | all calibration |
| `runs/g4b-explore` | 50 | 30-79 | all calibration |
| `runs/g4-goto` | 10 | 20-29 | goto_room agreement |
| `runs/noise-explore-r1..r3` | 3 x 10 | 20-29 | Gazebo's own repeatability (earlier code, before the guard) |

The 60 explore runs are split by seed parity and never re-drawn: **even seeds = fit half (30 runs), odd seeds = validation
half (30 runs)** (`agreement.split_of`). A parameter counts as "fitted on the fit half" when only fit-half data entered it.

Reference data is committed so the analysis can be regenerated without the `runs/` directory: `sim/kinsim/data/gazebo_explore.json`
(per-run verdicts, metrics, skill log counts, track features), `gazebo_goto.json`, `gazebo_noise_r*.json`, and `replay.json.gz`
(10 Hz ground-truth tracks, PX4 estimates and the flight client's goal log of the 60 runs; 2 MB). They are produced by
`python3 -m kinsim.reference` / `python3 -m kinsim.replay build` from the run directories.

### Method, step by step

1. **Estimator** (`est_*`) [F]. PX4's own estimate (`raw.json: px4_pos`, 92 000 samples) against the ground-truth track:
   error mean 0.033 m, p95 0.087 m, max 0.26 m; per axis std 0.031 m; correlation time about 1 s (autocorrelation 0.67 at
   0.6 s, 0.19 at 1.2 s); **no lag**: along-track mean error -0.002 m at speeds above 0.2 m/s (implied delay 2 ms), so the
   estimate is the truth plus slow noise. The two halves agree (fit 0.0334 / 0.087, validation 0.0331 / 0.0866). Model:
   truth plus an Ornstein-Uhlenbeck noise, sigma 0.026 m, tau 1.0 s, no delay. The initial guess of a 40 ms delay
   (`EKF2_EV_DELAY`) was wrong and was dropped.
2. **Controller** (`kp_xy`, `kv_xy`, `a_max`, `tau_a`, `v_max`) by **replay** [F]. The flight client logs every accepted goal with
   a wall-clock time (`stack-host.log`). Replaying those goals through the real `FlightMission` / `SafetyGate` gives the
   exact setpoint stream; the vehicle model is started from the recorded track and run for 8 s windows (3 s warm-up)
   away from collisions; the loss is the Huber loss (0.15 m) of the horizontal position error. Fitted on the fit half with a (1+1) evolution
   strategy (150 iterations): kp 0.925 (PX4 default 0.95), kv 1.26 (default 1.8), tau_a 0.10 s, a_max 2.7, v_max 0.85.
   Replay error [window RMSE / median / p90]: **fit half 0.092 / 0.055 / 0.137 m, validation half 0.089 / 0.055 / 0.136 m**
   (uncalibrated PX4-default parameters: 0.097 / 0.062 / 0.143 fit, 0.093 / 0.060 / 0.144 validation). The floor is the estimate
   noise of step 1 (the real vehicle follows the *estimate*, which the replay does not know).
3. **Lateral wander** (`dist_sigma_acc`, `dist_tau_s`) [F]. The cross-track error of the ground-truth track against the legs between
   consecutive goals is much larger than the ideal model gives: Gazebo p50 / p90 / p99 = 0.031 / 0.106 / 0.45 m against
   0.017 / 0.051 / 0.40 m (replay with the estimate noise). An Ornstein-Uhlenbeck disturbance on the true horizontal
   acceleration (sigma 0.09 m/s^2, tau 0.5 s) closes it: fit half grid search (sigma 0.04-0.8, tau 0.1-1.0), validation
   half 0.037 / 0.097 / 0.28 against Gazebo 0.031 / 0.106 / 0.38 m. p99 stays low: Gazebo's tail (overshoot events, 0.4 m) is not modelled.
4. **Yaw loop** (`kp_yaw`, `yaw_rate_max`, `tau_w`) [E]. Yaw is not in the ground-truth track. The 360 degree scan at the start of every
   explore takes 6.0 s in Gazebo (first plan at 6.3 s median, goals every 2 ticks) and 6.0 s in kinsim; the values are PX4's
   defaults (kp 2.8, 120 deg/s) and were not fitted [E].
5. **Contact** (`contact_radius`, `contact_jitter`) [F]. All 8 Gazebo collision runs of the 60 are against **wall pieces beside doors** at
   0.10-0.35 m/s, mostly right after a guard stop or nudge; furniture never. The minimum clearance of runs
   without a hit sits at exactly the body radius (0.150-0.153 m in 20 runs): the drone touches and slides. Model: a vertical
   cylinder of radius 0.1502 m (+- 1.2 mm per contact), pushed out of the surface, normal velocity reflected with restitution 0.2,
   tangent damped. About 40 % of touches then fall under the referee's 0.15 m threshold.
6. **Crash** (`crash_prob`, `crash_bias_m`) [E]. In 4 of 60 Gazebo runs a contact ended in a crash (minimum clearance 0.036-0.039 m, the
   estimate meaningless afterwards, coverage 0.67-0.89 in three of them). 4 of the 19 Gazebo collisions: each contact
   crashes with probability 0.15, modelled as a persistent estimate bias of 0.3-0.9 m. Too few events to split between halves.
7. **Map noise** (`pose_lead_s`) [F]. The bridge stamps frames at receipt and pairs them with the newest PX4 pose, so the pose is newer than the
   image. A lead of 25 ms makes the surface error match (fit-half seeds, 12 runs: Gazebo mean 0.040 m / p95 0.073 m; lead 0: 0.033 / 0.050;
   0.04: 0.043 / 0.120; 0.08: 0.085 / 0.329). Set from fit-half seeds only.
8. **Rate and latency** (`frame_jitter_s`, `map_latency_s`) [E]. The nodes' 5 Hz rate limiters take about 4.6 of 10 camera frames per
   second in Gazebo (808 map frames per 176 s run); frame arrival jitter of 20 ms reproduces it (815 in kinsim). The 0.1 s
   latency is a guess.

### Agreement with Gazebo

Explore, 60 Gazebo runs (`g4-explore` + `g4b-explore`, current code, seeds 20-79) against one kinsim run per seed (run index 0), `--map-stride 4` (default). Fit half = even seeds, validation half = odd seeds.

| | fit (even seeds) | validation (odd seeds) | all |
|---|---|---|---|
| runs (Gazebo / kinsim pairs) | 30 | 30 | 60 |
| pass rate Gazebo / kinsim | 0.633 / 0.800 | 0.867 / 0.833 | 0.750 / 0.817 |
| per-seed pass/fail agreement | 0.567 | 0.767 | 0.667 |
| coverage mean Gazebo / kinsim | 0.960 / 0.974 | 0.968 / 0.972 | 0.964 / 0.973 |
| coverage correlation (per seed) | 0.02 | 0.13 | 0.08 |
| collisions per run Gazebo / kinsim | 0.50 / 0.23 | 0.13 / 0.03 | 0.32 / 0.13 |
| runs with a collision Gazebo / kinsim | 5 / 4 | 3 / 1 | 8 / 5 |
| agreement on "had a collision" | 0.767 | 0.867 | 0.817 |
| duration kinsim / Gazebo (sum, median of seeds) | 1.13, 1.12 | 1.11, 1.12 | 1.12, 1.12 |
| duration correlation (per seed) | 0.75 | 0.80 | 0.81 |
| median moving speed Gazebo / kinsim (m/s) | 0.219 / 0.199 | 0.219 / 0.203 | 0.219 / 0.201 |
| cross-track error p50 / p90 Gazebo (m) | 0.040 / 0.131 | 0.038 / 0.129 | 0.039 / 0.130 |
| cross-track error p50 / p90 kinsim (m) | 0.042 / 0.109 | 0.041 / 0.122 | 0.042 / 0.115 |

Same with the exact mapping (`--map-stride 2`, 60 runs): pass rate 0.850 (Gazebo 0.750), pass/fail agreement 0.633 fit / 0.767 validation / 0.700 all, collision runs 4 (Gazebo 8), duration ratio 1.13 (median 1.06), duration correlation 0.73, speed 0.207 / 0.219 m/s. Stride 4 is within noise of stride 2 on every row except duration/plans (about 5 percentage points) [F].

**Ceilings** (what any simulator can reach) [F]: two Gazebo runs of the same seed agree on pass/fail in **73 %** of pairs (noise runs r1-r3, seeds 20-29, 30 pairs, earlier code); two kinsim runs of the same seed (run index 0 against 1, 60 seeds) agree in **72 %**; kinsim run 1 against Gazebo agrees 75 % (run 0: 67 %). So the per-seed agreement of kinsim with Gazebo (67-75 %) is at the level of Gazebo's own repeatability; seed-to-seed duration correlation (0.81) is close to Gazebo against itself (0.84-0.90). Kinsim pass rate over the two runs 82 % / 73 % against Gazebo 75 %: the pass rate is compatible within the +-15 points of run noise (docs/s3-plan.md).

goto_room, seeds 20-29, `g4-goto` against kinsim (stride 4, 10 pairs; each half would be 5, so only the total is reported): C2 pass 8/10 Gazebo, 9/10 kinsim; **verdict agreement 9/10** (both fail seed 20, ended 0.28 / 0.51 m inside against 0.87 m; Gazebo also fails seed 25, kinsim passes it); old rule 6 / 7; skill succeeded 10 / 10; mean distance from the room's edge 0.88 m / 0.97 m (correlation 0.64); explore-phase duration correlation 0.64. Too few seeds for more [NV].

## Known fidelity gaps

Measured differences, largest first (stride 4, 60 runs; "validation" numbers where the fit half could have been tuned on):

1. **Duration +12 %** [F] (kinsim 197 s against 176 s mean, 192 against 153 s median; per seed correlation 0.81). More replanning: 13.7 plans per run against 12.2, 8.6 `look_failed` against 7.1, path length 37.7 m against 35.0 m. A 12-seed check at stride 2 gives 224 s against Gazebo's 217 s (plans 14.1 against 14.3): about half of the gap is the stride-4 depth sampling leaving more thin unknown slivers behind after a look [E]; the rest is not understood [NV]. 23 % of seeds are more than 30 % longer in kinsim, 3 % shorter: kinsim has a slow tail (5 runs reach the 360 s budget, Gazebo 3).
2. **Per-seed verdicts agree only 67 %** (kappa 0.02), as two Gazebo runs of one seed agree 73 % (noise runs, 3 x 10 seeds, earlier code) [F]. Coverage correlates 0.08 per seed (Gazebo against itself 0.39-0.51): coverage is 0.96-0.99 in almost every run and varies with rare events (crashes) that are random in both simulators. Use rates over many seeds, not single verdicts.
3. **Collisions: 5 runs against 8** in Gazebo, 0.13 collisions per run against 0.32 [F, small counts]. The distribution of minimum clearance matches (p25 / median / p75: kinsim 0.152 / 0.196 / 0.239 m, Gazebo 0.151 / 0.197 / 0.242 m) and the contexts are the same kind (low speed 0.1-0.5 m/s, after a guard stop or nudge, mostly wall pieces beside doors; 2 of 5 kinsim runs hit a shower glass and a wardrobe, Gazebo's 8 were all wall pieces). **Crashes are crude**: Gazebo's 4 crash runs (clearance 0.036-0.039 m, coverage 0.67-0.89) and the multi-collision cascades after them (up to 7 collisions) come from an estimate bias jump in kinsim, not from physics. Coverage failures: 2 of 60 against 5 [F].
4. **C6 failures 5 against 6** (counts of runs; the C6 rule is a count over few regions and flips with the run, docs/s3-plan.md). Surface error mean 0.040 m against 0.044 m but **p95 0.060 m against 0.091 m** [F]: kinsim's map has fewer outliers (no tilt, no depth-edge artefacts of Gazebo's renderer).
5. **First plan 0.4 s later** [F] (6.8 s against 6.4 s): in about half of the runs a size-4 frontier at the spawn point is chosen first (the planner's start-up choice differs on small map differences), costing a 1.5 s look.
6. **Cross-track error tail**: p90 0.115 m against 0.130 m, p99 0.28 against 0.38 m in the replay [F]; the 0.4 m overshoot events of Gazebo (a drone that carries 0.4 m/s into a waypoint) are rarer.
7. **Speed profile**: median moving speed 0.20 m/s against 0.22 m/s, p90 0.41 against 0.42 m/s, moving 83 % of the time against 81 % [F].

Not modelled at all, so not measured:

- **Attitude**: the camera stays level; Gazebo's tilts with acceleration (the pose uses PX4's attitude, so the map is consistent, but near-obstacle tilt of the guard ring and the camera view are not) [NV].
- **PX4 estimator behaviour**: EKF resets, vision-fusion start (the EV gate waits for it), the 0.9-1.4 m single-sample spikes of earlier runs, flow-only localisation (everything here is mocap) [NV].
- **Take-off and landing** (kinsim starts in hover at 2.0 m after 10 s of hover), the **return-to-land** phase, battery, wifi link effects [NV].
- **Real-time-factor dips and CPU contention** (Gazebo 0.9-1.0 during these runs, node latencies vary with load; kinsim's 0.1 s map latency is fixed) [NV].
- **Depth sensor artefacts**: kinsim's depth is exact to the mm; Gazebo's renderer has edge pixels and quantisation that this does not reproduce (surface error p95 is the visible symptom) [E].
- **Stride 4 mapping** (default) is an approximation of the real `stride: 2`; stride 2 is exact for the sampled pixels (see the table).
- The **skill node glue is a copy** (pinned by hash, see "Keeping kinsim in sync"); the contact model is a cylinder against collision boxes: no prop strike, no sliding along rough corners, no door leaf (worldgen doors are openings) [E].
- **goto_room** and **search**: goto_room was compared on 10 seeds (below), search on none (no Gazebo search run on the current code exists): its numbers have no measured fidelity [NV]. The 5 Hz detector and memory run on 160x120 frames with the same noise model but a different random stream than Gazebo's.

## When to trust kinsim

**Use it for** tuning and for screening: many tuning-seed apartments per minute to find which seeds fail, why (event log,
collision contexts, coverage curves), and whether a planner / skill / guard change moves the failure rates it is meant to move
(compare two code versions on the same seeds and `--runs-per-seed`; the code change is the only difference). Rates are meaningful at
the level the validation table shows: pass rate, collision-run rate, coverage and duration over 30+ seeds, speed profile and
tracking error. Individual seeds flip between pass and fail in Gazebo itself (73 % pass/fail agreement between two Gazebo runs of the same seed), so no
single-seed verdict of kinsim, and no difference below about 15 points between two code versions, means anything.

**Do not use it for** exit results or any number in a report: kinsim results are tuning / screening numbers, never S3 or S4 results, never
held-out seeds (the CLI refuses them), never an argument that a requirement is met. Do not tune the simulator's parameters to a drone-stack change (calibration
is vehicle and sensor only, from existing Gazebo runs). Treat these as outside its reach until Gazebo says otherwise: crash
and post-contact behaviour, estimator spikes and EKF resets, the 0.4 m overshoot tail, anything that depends on yaw or tilt dynamics,
real-time-factor effects, perception beyond the oracle detector (it is the same oracle as in Gazebo).

**Every change is still confirmed in Gazebo** on a small set (the tuning seeds used for the comparison, 10 to 20 runs) before it is
accepted, and Gazebo is where held-out numbers come from.

## Keeping kinsim in sync

kinsim imports the drone stack's pure modules from the checked-out sources, so skills, guard, planner, mapping, memory and the
flight mission are always the code under test. What it cannot import is the nodes' own glue (it needs ROS): `SkillRunner` in `stack.py` repeats `skill_node`'s
`_tick`, `_guard_tick`, `_emit`, `_on_request`, `_on_pose`, `_on_rooms`, `_on_objects`; `KinStack` repeats `mapping_node._integrate`, `_publish_clearance`, `_publish_grid`, `flight_node._on_goal` and the 20 Hz
tick; `memory.py` repeats `memory_node._process`, `_update_rooms`, `_on_query`; the S3 host launch file's parameters (mapping stride 2,
altitude, flight client altitude 2.0, geofence) are copied as constants. `kinsim/parity.py` stores a hash of the AST of each of those (`data/parity.json`) and
`tests/test_parity.py` fails when one changes: read the change, copy it into kinsim, run `python3 -m kinsim.parity --update`. A change to the
referee's scoring needs nothing (kinsim calls it). Re-run the calibration (`python3 -m kinsim.agreement`, below) after such a sync and after any change to
the airframe parameters (`sim/px4/airframes/`), the camera mount or the drone model.

## Regenerating the numbers

```
python3 -m kinsim.reference --runs runs/g4-explore runs/g4b-explore --out sim/kinsim/data/gazebo_explore.json
python3 -m kinsim.replay build --runs runs/g4-explore runs/g4b-explore --out sim/kinsim/data/replay.json.gz
python3 -m kinsim.replay score --data sim/kinsim/data/replay.json.gz --params sim/kinsim/kinsim/calibrated.json   # step 2 errors
python3 -m kinsim.replay fit --data sim/kinsim/data/replay.json.gz --out vehicle_fit.json                          # step 2 refit (fit half)
python3 -m kinsim --scenario explore --seeds 20-79 --jobs 12 --out runs/kinsim-explore
python3 -m kinsim.agreement --kin runs/kinsim-explore/results.jsonl                                              # the table above
```

`tests/test_run.py::test_agreement_report_can_be_regenerated` regenerates the report from the committed reference and fresh kinsim
runs on three seeds (CI-sized). Seed ledger: seeds 20-79 were already tuning seeds in use (docs/seed-ledger.md); kinsim never touches held-out seeds.

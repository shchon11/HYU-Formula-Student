# hyu_control_harness

Headless closed-loop controller tuning harness. Runs the real planning/control
code against the real simulator plant, without Gazebo or a ROS graph, at ~45x
realtime single-threaded — fast enough for grid sweeps.

## What is in the loop

```
ground-truth cone map (eufs_tracks csv)
  -> ego-frame transform with the MEASURED pose      (= slam_map source mode)
  -> local_planner::buildLocalPath                    (real lap-1 planner, allow_partial_boundary)
  -> map-frame path -> pure pursuit (MAP / geometric) (real controller + steering LUT)
  -> command queue, 0.2 s control delay               (eufs_plugins controlDelay)
  -> steering-rate limit, 1.04 rad/s                  (steeringLockTime 1 s)
  -> eufs::models::DynamicBicycle                     (the exact Gazebo plant, 1 kHz)
```

The measured pose models the localization chain: sampled at `odom_rate_hz`
(50 Hz), delayed by `odom_latency_s` (0.05 s), optional gaussian noise. Keep it
non-ideal: MAP's lookup table inverts this plant by construction, so a zero-
latency noiseless loop is too optimistic to tune against.

## Known gaps vs the full sim

- No perception/SLAM error: the cone map is ground truth and complete from t=0
  (lap-2+ conditions). Association regressions are invisible here — final
  validation still needs a real-perception fullstack run.
- No mission state machine, no TF, single-threaded deterministic timing.
- Off-track detection is centre-of-car vs the cone line (+0.3 m slack), laps
  are counted on centerline arc-length progress, target 10 laps (trackdrive).

## Usage

```bash
./install/hyu_control_harness/lib/hyu_control_harness/map_harness \
  track=src/sim/eufs_sim/eufs_tracks/csv/trackdrive_kase2026.csv \
  plant_yaml=src/sim/eufs_sim/eufs_racecar/robots/eufs/configDry.yaml \
  map_lookahead_max_m=5.0 two_sided_speed_mps=4.5 \
  traj_csv=/tmp/traj.csv        # optional 100 Hz dump for plotting
```

Prints one JSON object (laps, lap times, CTE RMSE/max, cone-line violation
fraction, steering-rate stats, planner-invalid fraction, DNF reason). Exit code
1 on DNF. `map_harness --help` lists every key; defaults mirror the shipped
trackdrive configs (`pure_pursuit_controller.yaml`, `local_planner.yaml`).

Grid sweeps (cartesian product, parallel, ranked summary + CSV):

```bash
python3 src/control/hyu_control_harness/scripts/sweep_map.py \
  --bin install/hyu_control_harness/lib/hyu_control_harness/map_harness \
  --track src/sim/eufs_sim/eufs_tracks/csv/trackdrive_kase2026.csv \
  --plant-yaml src/sim/eufs_sim/eufs_racecar/robots/eufs/configDry.yaml \
  --grid map_lookahead_max_m=3.0,4.0,5.0,6.0 \
  --grid map_lookahead_slope_s=0.4,0.55,0.7 \
  --fixed two_sided_speed_mps=4.5 \
  --out sweep.csv
```

## Pitfalls

- `allow_partial_boundary` must stay true (the slam_map node default): with a
  full map, strict mode rejects any frame where one ROI-edge cone lacks its
  cross-pair, which kills the path on straights.
- Lap times saturate at the local planner speed cap (`two_sided_speed_mps`,
  curvature-limited by `max_lateral_accel_mps2`) long before the controller
  limits — raise the cap to stress the steering law.

## TODO

- TMPC leg: same plant + `tmpc_trajectory_builder` + `mvdc_mpc_step()` once the
  `P_VDC_*` instance parameters are exposed for tuning.

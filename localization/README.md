# EUFS Graph SLAM

`hyu_localization` is a ROS 2 Humble package that builds a 2D g2o graph from EUFS simulator car state and cone observations.

The node subscribes to:

- `/odometry_integration/car_state` (`hyu_msgs/msg/CarState`) for SE2 keyframe motion
- `/cones` (`hyu_msgs/msg/ConeArrayWithCovariance`) for local cone observations in `base_footprint`

It publishes:

- `/localization/cone_map` (`hyu_msgs/msg/ConeArrayWithCovariance`)
- `/localization/ego_odom` (`nav_msgs/msg/Odometry`)
- `/graph_slam/path` (`nav_msgs/msg/Path`)
- `/graph_slam/markers` (`visualization_msgs/msg/MarkerArray`)
- `status_topic`, default `~/status` (`std_msgs/msg/String`)
- `map_converged_topic`, default `~/map_converged` (`std_msgs/msg/Bool`)

When TF publishing is enabled, the node owns `map -> odom` and
`odom -> base_footprint`. The simulator ground-truth TF publisher must stay
disabled so `base_footprint` has one parent.

## Planner-facing contract

Graph SLAM owns the localization outputs consumed by the planning integration:

- `/localization/cone_map` is a reliable transient-local
  `hyu_msgs/msg/ConeArrayWithCovariance` map snapshot.
- `/localization/ego_odom` is the live `nav_msgs/msg/Odometry` ego pose stream.
- `/graph_slam/status` remains Graph-SLAM-owned lifecycle state with values
  `mapping`, `mapping_converged`, and `localization`.
- `/graph_slam/map_converged` remains a latched map convergence signal.

The planning stack only allows global waypoint use when `/graph_slam/status` is
`localization`. Planner liveness is not inferred from the status topic; it comes
from the selected global waypoint writer's reliable volatile
`/planning/global_path_valid` heartbeat.

Graph SLAM does not publish `/global_waypoints` or
`/planning/global_path_valid`. Those topics must have one writer in any launch:
the SLAM `planner_node` or the CSV global planner, never both on the default
topics.

The phase-1 planner consumes the existing `ConeArrayWithCovariance` map. A
planner-friendly `SlamConeMap.msg` with landmark IDs/versioning is deferred to a
later compatible schema phase.

## Build

From the EUFS workspace root:

```bash
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-up-to hyu_localization
source install/setup.zsh
```

`--packages-up-to` builds workspace dependencies such as `hyu_msgs` before building `hyu_localization`.

The package first tries `find_package(g2o CONFIG)`. If that is not available, it vendors a sibling g2o source tree at `../g2o` relative to `eufs_simulator`. For a different location, pass:

```bash
colcon build --symlink-install --packages-up-to hyu_localization \
  --cmake-args -DG2O_VENDOR_SOURCE_DIR=/path/to/g2o
```

## Run

Start the simulator with simulated perception, then run:

```bash
ros2 launch hyu_localization graph_slam.launch.py
```

Useful services:

```bash
ros2 service call /graph_slam/reset std_srvs/srv/Trigger "{}"
ros2 service call /graph_slam/save_graph std_srvs/srv/Trigger "{}"   # raw g2o graph
ros2 service call /graph_slam/save_map std_srvs/srv/Trigger "{}"     # cone map CSV
```

## Saving and loading cone maps

`~/save_map` writes the current landmark map to
`map_save_dir/map_<timestamp>.csv` using the same columns as `eufs_tracks`
track CSVs (`tag,x,y,direction,x_variance,y_variance,xy_covariance`, plus a
`car_start` origin row), so saved maps are interchangeable with track files.
Launched via `graph_slam.launch.py`, `map_save_dir` defaults to the package's
`map/` directory.

Set `localization_mode:=true load_map_path:=<csv>` to localize against a saved
map instead of building one. The loaded cones become **fixed** landmarks
(`setFixed(true)`); mapping, deletion, and merging are disabled, and the
optimizer moves only the pose to fit the fixed map — so drift is corrected
against a known map. The loaded map is published once on the configured
`map_topic` (latched) for preview; the default is the planner-facing
`/localization/cone_map` topic.

```bash
ros2 launch hyu_localization graph_slam.launch.py \
  localization_mode:=true \
  load_map_path:=<workspace>/hyu_localization/map/small_track_slam.csv
```

If localization is lost, use RViz's **2D Pose Estimate** tool (fixed frame
`map`): it publishes `/initialpose`, and the node drops its pose trajectory,
re-anchors at the clicked pose (keeping the fixed map), and re-localizes from
there.

## Trackdrive lifecycle (automatic mapping -> localization)

With `auto_localization_after_lap` (default on) the node runs the full
trackdrive lifecycle without operator input:

1. **Mapping**: lap 1 builds the map as usual. The lap origin is captured
   `lap_origin_capture_distance` (15 m) into the drive so it sits on the
   racing line rather than the spawn pose.
2. **Lap completion**: once the map has converged (loop closures reconciled)
   and the car returns within `lap_return_radius` / `lap_return_yaw` of the lap
   origin, the node freezes every landmark, auto-saves the map CSV to
   `map_save_dir`, and switches to localization.
3. **Localization**: only the most recent `localization_window_poses` pose
   vertices are kept (the oldest is fixed as the anchor), so the graph stays
   bounded for arbitrarily many laps — full-batch optimization never outgrows
   real time.

The lifecycle is published (latched) on:

- `status_topic`, default `~/status` (`std_msgs/String`): `mapping`,
  `mapping_converged`, or `localization` — RViz HUD via
  `/graph_slam/status_overlay`.
- `map_converged_topic`, default `~/map_converged` (`std_msgs/Bool`):
  planning can switch from local to global planning on this flag. It also
  publishes true in localization mode after a fixed map has been loaded.

The map and odometry output topics are launch parameters. For an older tool
that still expects the legacy Graph SLAM names, start with:

```bash
ros2 launch hyu_localization graph_slam.launch.py \
  map_topic:=/graph_slam/map \
  slam_odom_topic:=/graph_slam/odom
```

## Wheel-encoder odometry

`ros2 run hyu_localization wheel_odometry` integrates rear wheel speeds (RPM,
`/ros_can/wheel_speeds`) with an IMU yaw rate (`/imu/data`; bicycle-model
steering fallback) into `/wheel_odometry/car_state` — a GNSS-independent
odometry source for `car_state_topic`, keeping the GNSS prior as the only
absolute channel. On the real car only the two input topics change.

## GUI

`graph_slam.launch.py` starts a control GUI (`gui:=true`, default) alongside
the node: switch between mapping and localization, pick a saved map from the
map directory with a live cone preview, and save the current map. It drives the
node's `~/save_map`, `~/load_map`, and `~/start_mapping` services.

Landmark deletion is handled separately from reset. When `delete_stale_landmarks`
is enabled, landmarks that should be visible but are missed for
`landmark_missed_observations_to_delete` deletion updates are removed from the
g2o graph with their connected observation edges. The default visibility gate
matches the simulator perception window: 180 degree FOV, 30 m range, and
20 m absolute x/y bounds in `base_footprint`. Landmarks that accumulated at
least `landmark_confirm_observations` keyframe observations are confirmed and
get a 10x miss budget: occlusions or perception dropouts cannot erase their
loop-closure constraints, while drift-era ghost duplicates that are never
observed again still age out.

Existing landmark positions are also updated between graph optimizations. When
`update_existing_landmarks` is enabled, each associated cone observation is
transformed through the latest live pose estimate and fused into the landmark
vertex with a covariance-aware update. Keyframe observations still add g2o
edges; duplicate in-between observations only update the vertex estimate and
published covariance.

## Estimation pipeline notes

- The node runs on a two-thread `MultiThreadedExecutor` with two mutually
  exclusive callback groups: car state in one, cones + a 250 ms optimization
  timer in the other. The state callback only *tries* to take the graph lock;
  when optimization holds it, live odometry is dead-reckoned from the last
  keyframe snapshot instead of blocking, so `/localization/ego_odom` and TF keep
  the input rate.
- The optimizer uses g2o's sparse `LinearSolverEigen`, so the whole session
  (`max_optimization_poses`) stays inside periodic Levenberg-Marquardt runs
  every `optimize_every_n_keyframes` keyframes.
- Cone messages are re-anchored in time: raw odometry is interpolated at the
  cone stamp and each measurement is re-expressed in the keyframe base frame
  before an `EdgeSE2PointXY` is added. Without this the measurement can be off
  by up to `keyframe_distance` at speed.
- Data association is a Mahalanobis nearest-neighbour gate
  (`association_gate_chi2`) over landmark + observation covariance. The gate
  is inflated by `association_inflation_per_keyframe` for every keyframe a
  landmark went unseen (capped at `association_max_inflation`), which is what
  lets lap-closure re-associations succeed despite accumulated drift. Nearly
  tied candidates (`association_ambiguity_ratio`) are skipped instead of
  guessed.
- Re-associating a landmark unseen for `loop_gap_keyframes` keyframes is
  treated as a loop closure and forces an immediate graph optimization.
- After each optimization, landmarks of compatible colours closer than
  `landmark_merge_distance` are merged; the surviving vertex inherits the
  other's observation edges. Big orange cones are exempt: start-line pairs
  legitimately stand ~0.4 m apart.
- Landmark colours are decided by majority vote over associated observations,
  so a single mislabelled detection cannot lock in a wrong colour.

Known limitations:

- Landmarks whose observations mostly fall outside the simulated camera FOV
  (120 deg, 15 m) keep colour `unknown` — the vote never receives a coloured
  sample. This is a perception characteristic, not a SLAM association error.
- When the state callback falls back to snapshot dead-reckoning (optimization
  in progress), the published pose lags any correction from that very
  optimization by one cycle; the error is bounded by one keyframe of drift.
- Loop closure relies on gated nearest-neighbour re-association. It recovers
  multi-metre drift (validated to ~5 m RMSE input error on small_track), but
  has no place-recognition fallback for drift far beyond the inflated gate.
- The estimator is 2D (x, y, yaw); slopes and banking are not modelled.
- Big orange start-line pairs (~0.4 m apart) are closer than the association
  gate, so each pair typically collapses into a single landmark at creation.
  Separating them needs joint per-frame assignment (e.g. Hungarian), which is
  not implemented.

Parameters live in `config/graph_slam.yaml`.

## Experiment harness

`scripts/` contains a self-contained evaluation loop used to tune the node:

- `run_experiment.sh OUT.json [DURATION] [DRIFT] [-p param:=value ...]` —
  headless Gazebo (small_track) + graph SLAM + pure-pursuit driver +
  evaluator; tears everything down afterwards and writes a JSON report.
- `drive_track.py` — sets the TRACK_DRIVE mission and follows the track
  centreline (from the track CSV) with ground truth, so driving quality does
  not depend on SLAM output.
- `evaluate_slam.py` — reports trajectory ATE for SLAM vs the raw odometry
  input, and map quality (matches, RMSE, duplicates, false positives, colour
  accuracy) against the track CSV. It listens to `/localization/ego_odom` and
  `/localization/cone_map` by default; use `--slam-odom /graph_slam/odom` and
  `--map-topic /graph_slam/map` for legacy runs.

The drifting odometry the node consumes is produced by the simulator itself:
the race-car plugin publishes ground truth on `/ground_truth/state` and a
drift-integrated pose on `/odometry_integration/car_state` (the node's default
`car_state_topic`). Drift is enabled by `driftOdometry` in
`eufs_plugins.gazebo.xacro` and tuned by `driftVelocityBias`,
`driftYawRateBias`, `driftVelocityNoise`, and `driftYawRateNoise`. The legacy
standalone `drift_odom.py` node is superseded by this and no longer used by the
harness.

Example:

```bash
./hyu_localization/scripts/run_experiment.sh /tmp/slam_report.json 120 1
```

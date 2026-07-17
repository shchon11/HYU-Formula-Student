# hyu_global_planner

`hyu_global_planner` provides two mutually exclusive global waypoint producers and
the local waypoint window publisher used by the planning stack.

- `planner_node` consumes Graph SLAM localization outputs and generates a
  conservative centerline/global waypoint path from blue/yellow cone
  boundaries. It is a phase-1 integration generator, not a production
  racing-line optimizer.
- `hyu_global_planner_trajectory_publisher_node` preserves the existing offline CSV
  workflow for debug and replay launches.
- `wpnt_publisher_node` republishes the next global window on its configured
  output topic only while the global path validity heartbeat is fresh. The
  standalone default is `/planning/path`; integrated bringup remaps this
  output to `/planning/global_path_waypoints` so the selector owns the final
  `/planning/path` topic.

## SLAM Planning Contract

Default topics:

| Topic | Type | QoS | Owner |
| --- | --- | --- | --- |
| `/localization/cone_map` | `hyu_msgs/msg/ConeArrayWithCovariance` | reliable transient-local | `graph_slam` |
| `/localization/ego_odom` | `nav_msgs/msg/Odometry` | reliable volatile | `graph_slam` |
| `/localization/status` | `std_msgs/msg/String` | reliable transient-local | `graph_slam` |
| `/global_waypoints` | `hyu_msgs/msg/WaypointArrayStamped` | reliable transient-local | selected global waypoint writer |
| `/planning/global_path_valid` | `std_msgs/msg/Bool` | reliable volatile | selected global waypoint writer |
| `/planning/path` | `hyu_msgs/msg/WaypointArrayStamped` | reliable volatile | `hyu_path_selector_node` in integrated bringup |

Only one node may write `/global_waypoints` and `/planning/global_path_valid`
in a launch. Use `slam_hyu_global_planner.launch.py planner_source:=slam` for the
Graph SLAM path generator, or `planner_source:=csv` for the CSV publisher. Do
not run both writers on the default topics; remap both output topics if a
comparison launch needs both producers.

In the integrated bringup, `hyu_path_selector_node` is the sole
`/planning/path` writer and the global window is consumed through
`/planning/global_path_waypoints`.

`/planning/global_path_valid` is a heartbeat, not a latched state. It is
published as reliable volatile `std_msgs/Bool`: `false` on startup, while Graph
SLAM is not in `localization`, on stale inputs, or before the first path can be
generated. By default `planner_node` holds the last valid SLAM-derived
`/global_waypoints` snapshot if a later refresh fails, so transient
`/localization/cone_map` geometry failures do not invalidate the accepted
global path. When a later refresh succeeds, `planner_node` publishes the new
snapshot and consumers switch to it. Consumers still clear cached state on
`false` or timeout if the selected writer emits one.

The phase-1 SLAM planner consumes the existing
`ConeArrayWithCovariance` map. A planner-specific `SlamConeMap.msg` with
landmark IDs, versions, and richer lifecycle metadata is intentionally deferred
to a later compatible schema phase.

## SLAM Integration Launch

```bash
cd /home/shchon11/fsk
export EUFS_MASTER=$PWD
source /opt/ros/humble/setup.zsh
source install/setup.zsh

ros2 launch hyu_global_planner slam_hyu_global_planner.launch.py planner_source:=slam
```

This launch starts `planner_node`, `frenet_odom_node`, and
`wpnt_publisher_node`. It wires `frenet_odom_node` to
`/localization/ego_odom`, not `/ground_truth/odom`, and uses the selected writer
as the sole owner of `/global_waypoints` and `/planning/global_path_valid`.

The standalone rolling-window and time defaults are preserved:

| Launch argument | Default | Description |
| --- | --- | --- |
| `path_waypoints_topic` | `/planning/path` | Rolling global waypoint window published by `wpnt_publisher_node`; the integrated launch overrides it to `/planning/global_path_waypoints`. |
| `path_topic` | `/planning/debug/path` | RViz path matching the rolling global waypoint window. |
| `use_sim_time` | `false` | Use ROS simulated time for all nodes started by this launch. |

Override the rolling window only from an integrated launch, for example:

```bash
ros2 launch hyu_global_planner slam_hyu_global_planner.launch.py \
  path_waypoints_topic:=/planning/global_path_waypoints \
  path_topic:=/planning/global_path_waypoints/path \
  use_sim_time:=true
```

The integrated handoff gate requires a non-empty global snapshot, a fresh true
`/planning/global_path_valid` heartbeat, fresh Frenet odometry, and a continuous
`/planning/global_handoff_ready` dwell before the state machine selects
`GLOBAL_FULL`. With the default SLAM planner, later SLAM map geometry failures
hold the last valid global snapshot instead of forcing a local-mode demotion;
the next successful refresh publishes a replacement snapshot. A false or stale
validity heartbeat from any selected writer still invalidates the accepted
snapshot; a new snapshot is required for recovery.

For CSV replay/debug with the same consumer gating:

```bash
ros2 launch hyu_global_planner slam_hyu_global_planner.launch.py planner_source:=csv
```

The older CSV launch remains available and keeps its conservative
`/ground_truth/odom` default.

## Formula TMPC trajectory output

`wpnt_publisher_node` keeps the legacy Frenet-callback rolling outputs above
and independently builds a fixed-size Formula TMPC trajectory pair at 20 Hz.
It publishes only while all of the following are true:

- `/planning/global_path_valid` is a fresh `true` heartbeat and the accepted
  `/global_waypoints` snapshot is valid;
- `/planning/path_source` is a fresh `GLOBAL_FULL` or `GLOBAL_FINAL_STOP`;
- `/planning/frenet_odom`, `/localization/ego_odom`, and
  `/planning/lap_count` are fresh and finite.

The input timeout defaults to 0.5 s. A static global waypoint snapshot does
not age out by itself: a false/stale validity heartbeat invalidates it, and a
new snapshot must be accepted before publication resumes. On any invalid
input the node publishes nothing; it never sends a zero-filled trajectory as
though it were valid.

| Topic | Type | QoS |
| --- | --- | --- |
| `/planning/trajectory_performance` | `hyu_tmpc_msgs/msg/TumTrajectory` | reliable, volatile, KeepLast 10 |
| `/planning/trajectory_emergency` | `hyu_tmpc_msgs/msg/TumTrajectory` | reliable, volatile, KeepLast 10 |

Each output contains exactly 50 uniformly spaced global-arc-length samples.
The interval starts 0.5 m behind the current Frenet `s`, extends forward by a
speed-dependent horizon, and interpolates across the closed-loop seam. The
performance and emergency messages are validated together and use the same
nonzero `traj_cnt` and clamped nonnegative `lap_cnt`. The emergency message
keeps the same geometry but applies a nonnegative, monotonically non-increasing
speed profile ending at or below 0.5 m/s. If the nominal horizon cannot brake
safely, the builder retries the complete pair with the configured maximum
horizon; if that still cannot stop, neither message is published.

The launch selects the input heading convention from `planner_source`:

- `slam`: waypoints contain ROS yaw (east-zero, counter-clockwise positive),
  so `psi_rad` is converted with `wrap(yaw_ros - pi/2)`;
- `csv`: the offline trajectory already contains Formula north-zero heading,
  so only angle normalization is applied.

The default GGV table comes from the installed copy of
`trajectory_generator/inputs/veh_dyn_info/ggv.csv`. Set
`tmpc_ggv_csv_path` to an empty string to use the equal-length fallback arrays
in `hyu_global_planner.yaml`. A malformed file, non-increasing speed axis, or
nonpositive limit is a startup error. Speed queries beyond the table use the
nearest endpoint.

This adapter does not consume TMPC control output, write `/vehicle/cmd`, disable Pure
Pursuit, or arbitrate command ownership. Those are separate integration steps.

## RViz debug visualization


Terminal 1: run the simulation and RViz.

```bash
cd ~/HYU-Formula-Student
export EUFS_MASTER=$PWD
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch eufs_launcher simulation.launch.py \
  track:=peanut \
  gazebo_gui:=true \
  rviz:=true \
  launch_group:=no_perception \
  commandMode:=acceleration
```

Terminal 2: run the global planner in the same localhost-only ROS graph as
RViz.

```bash
cd ~/HYU-Formula-Student
export EUFS_MASTER=$PWD
source /opt/ros/humble/setup.zsh
source install/setup.zsh
export ROS_LOCALHOST_ONLY=1
ros2 launch hyu_global_planner hyu_global_planner_trajectory_publisher_debug.launch.py
```

RViz should show `/hyu_global_planner/debug/markers` as a
`visualization_msgs/msg/MarkerArray` topic. Use a `MarkerArray` display, not a
`Path` display.

## Generate Centerline

Run this first to build the track mask from the EUFS cone CSV and generate
`outputs/peanut/centerline.csv`.

```bash
cd ~/HYU-Formula-Student/planning/trajectory_generator
python3 csv_to_track_mask.py --cone-csv ../../sim/eufs_sim/eufs_tracks/csv/peanut.csv
python3 lane_generator.py --headless
```

## Generate Offline Minimum Curvature CSV

Run this after `outputs/peanut/centerline.csv` has been generated.

```bash
cd ~/HYU-Formula-Student/planning/trajectory_generator
python3 main_globaltraj.py --headless
```

This updates:

```text
outputs/peanut/traj_race_cl.csv
```

The generated CSV can be replayed by `hyu_global_planner_trajectory_publisher_node`.
That offline workflow is separate from the runtime SLAM `planner_node`.

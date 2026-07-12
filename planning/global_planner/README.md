# global_planner

`global_planner` provides two mutually exclusive global waypoint producers and
the local waypoint window publisher used by the planning stack.

- `planner_node` consumes Graph SLAM localization outputs and generates a
  conservative centerline/global waypoint path from blue/yellow cone
  boundaries. It is a phase-1 integration generator, not a production
  racing-line optimizer.
- `global_planner_trajectory_publisher_node` preserves the existing offline CSV
  workflow for debug and replay launches.
- `wpnt_publisher_node` republishes the next global window on its configured
  output topic only while the global path validity heartbeat is fresh. The
  standalone default is `/path_waypoints`; integrated bringup remaps this
  output to `/planning/global_path_waypoints` so the selector owns the final
  `/path_waypoints` topic.

## SLAM Planning Contract

Default topics:

| Topic | Type | QoS | Owner |
| --- | --- | --- | --- |
| `/localization/cone_map` | `eufs_msgs/msg/ConeArrayWithCovariance` | reliable transient-local | `graph_slam` |
| `/localization/ego_odom` | `nav_msgs/msg/Odometry` | reliable volatile | `graph_slam` |
| `/graph_slam/status` | `std_msgs/msg/String` | reliable transient-local | `graph_slam` |
| `/global_waypoints` | `eufs_msgs/msg/WaypointArrayStamped` | reliable transient-local | selected global waypoint writer |
| `/planning/global_path_valid` | `std_msgs/msg/Bool` | reliable volatile | selected global waypoint writer |
| `/path_waypoints` | `eufs_msgs/msg/WaypointArrayStamped` | reliable volatile | `path_selector_node` in integrated bringup |

Only one node may write `/global_waypoints` and `/planning/global_path_valid`
in a launch. Use `slam_global_planner.launch.py planner_source:=slam` for the
Graph SLAM path generator, or `planner_source:=csv` for the CSV publisher. Do
not run both writers on the default topics; remap both output topics if a
comparison launch needs both producers.

In the integrated bringup, `path_selector_node` is the sole
`/path_waypoints` writer and the global window is consumed through
`/planning/global_path_waypoints`.

`/planning/global_path_valid` is a heartbeat, not a latched state. It is
published as reliable volatile `std_msgs/Bool`: `false` on startup, while Graph
SLAM is not in `localization`, on stale inputs, or after path generation fails;
`true` repeats only while the currently published `/global_waypoints` snapshot
is still valid. Consumers clear cached state on `false` or timeout and require a
new `/global_waypoints` snapshot after invalidation before accepting recovered
true heartbeats.

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

ros2 launch global_planner slam_global_planner.launch.py planner_source:=slam
```

This launch starts `planner_node`, `frenet_odom_node`, and
`wpnt_publisher_node`. It wires `frenet_odom_node` to
`/localization/ego_odom`, not `/ground_truth/odom`, and uses the selected writer
as the sole owner of `/global_waypoints` and `/planning/global_path_valid`.

The standalone rolling-window and time defaults are preserved:

| Launch argument | Default | Description |
| --- | --- | --- |
| `path_waypoints_topic` | `/path_waypoints` | Rolling global waypoint window published by `wpnt_publisher_node`; the integrated launch overrides it to `/planning/global_path_waypoints`. |
| `path_topic` | `/path_waypoints/path` | RViz path matching the rolling global waypoint window. |
| `use_sim_time` | `false` | Use ROS simulated time for all nodes started by this launch. |

Override the rolling window only from an integrated launch, for example:

```bash
ros2 launch global_planner slam_global_planner.launch.py \
  path_waypoints_topic:=/planning/global_path_waypoints \
  path_topic:=/planning/global_path_waypoints/path \
  use_sim_time:=true
```

The integrated handoff gate requires a non-empty global snapshot, a fresh true
`/planning/global_path_valid` heartbeat, fresh Frenet odometry, and a continuous
`/planning/global_handoff_ready` dwell before the state machine selects
`GLOBAL_FULL`. A false or stale validity heartbeat invalidates the accepted
snapshot; a new snapshot is required for recovery.

For CSV replay/debug with the same consumer gating:

```bash
ros2 launch global_planner slam_global_planner.launch.py planner_source:=csv
```

The older CSV launch remains available and keeps its conservative
`/ground_truth/odom` default.

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
ros2 launch global_planner global_planner_trajectory_publisher_debug.launch.py
```

RViz should show `/global_planner/debug/markers` as a
`visualization_msgs/msg/MarkerArray` topic. Use a `MarkerArray` display, not a
`Path` display.

## Generate Centerline

Run this first to build the track mask from the EUFS cone CSV and generate
`outputs/peanut/centerline.csv`.

```bash
cd ~/HYU-Formula-Student/planning/trajectory_generator
python3 csv_to_track_mask.py --cone-csv ../../eufs_sim/eufs_tracks/csv/peanut.csv
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

The generated CSV can be replayed by `global_planner_trajectory_publisher_node`.
That offline workflow is separate from the runtime SLAM `planner_node`.

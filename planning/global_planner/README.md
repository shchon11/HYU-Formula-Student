# global_planner

Publishes the offline peanut raceline as `/global_waypoints` and RViz debug
markers as `/global_planner/debug/markers`.

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

## Generate Minimum Curvature Global Path

Run this after `outputs/peanut/centerline.csv` has been generated.

```bash
cd ~/HYU-Formula-Student/planning/trajectory_generator
python3 main_globaltraj.py --headless
```

This updates:

```text
outputs/peanut/traj_race_cl.csv
```

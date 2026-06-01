# EUFS Graph SLAM

`eufs_graph_slam` is a ROS 2 Humble package that builds a 2D g2o graph from EUFS simulator odometry and cone observations.

The node subscribes to:

- `/ground_truth/odom` (`nav_msgs/msg/Odometry`) for SE2 keyframe odometry
- `/cones` (`eufs_msgs/msg/ConeArrayWithCovariance`) for local cone observations in `base_footprint`

It publishes:

- `/graph_slam/map` (`eufs_msgs/msg/ConeArrayWithCovariance`)
- `/graph_slam/odom` (`nav_msgs/msg/Odometry`)
- `/graph_slam/path` (`nav_msgs/msg/Path`)
- `/graph_slam/markers` (`visualization_msgs/msg/MarkerArray`)

## Build

From the EUFS workspace root:

```bash
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-select eufs_graph_slam
source install/setup.zsh
```

The package first tries `find_package(g2o CONFIG)`. If that is not available, it vendors a sibling g2o source tree at `../g2o` relative to `eufs_simulator`, which matches this workspace layout. For a different location, pass:

```bash
colcon build --symlink-install --packages-select eufs_graph_slam \
  --cmake-args -DG2O_VENDOR_SOURCE_DIR=/path/to/g2o
```

## Run

Start the simulator with simulated perception, then run:

```bash
ros2 launch eufs_graph_slam graph_slam.launch.py
```

Useful services:

```bash
ros2 service call /graph_slam/reset std_srvs/srv/Trigger "{}"
ros2 service call /graph_slam/save_graph std_srvs/srv/Trigger "{}"
```

Parameters live in `config/graph_slam.yaml`.

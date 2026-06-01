# EUFS Graph SLAM

`eufs_graph_slam` is a ROS 2 Humble package that builds a 2D g2o graph from EUFS simulator car state and cone observations.

The node subscribes to:

- `/odometry_integration/car_state` (`eufs_msgs/msg/CarState`) for SE2 keyframe motion
- `/cones` (`eufs_msgs/msg/ConeArrayWithCovariance`) for local cone observations in `base_footprint`

It publishes:

- `/graph_slam/map` (`eufs_msgs/msg/ConeArrayWithCovariance`)
- `/graph_slam/odom` (`nav_msgs/msg/Odometry`)
- `/graph_slam/path` (`nav_msgs/msg/Path`)
- `/graph_slam/markers` (`visualization_msgs/msg/MarkerArray`)

When TF publishing is enabled, the node owns `map -> odom` and
`odom -> base_footprint`. The simulator ground-truth TF publisher must stay
disabled so `base_footprint` has one parent.

## Build

From the EUFS workspace root:

```bash
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-up-to eufs_graph_slam
source install/setup.zsh
```

`--packages-up-to` builds workspace dependencies such as `eufs_msgs` before building `eufs_graph_slam`.

The package first tries `find_package(g2o CONFIG)`. If that is not available, it vendors a sibling g2o source tree at `../g2o` relative to `eufs_simulator`. For a different location, pass:

```bash
colcon build --symlink-install --packages-up-to eufs_graph_slam \
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

Landmark deletion is handled separately from reset. When `delete_stale_landmarks`
is enabled, landmarks that should be visible but are missed for
`landmark_missed_observations_to_delete` deletion updates are removed from the
g2o graph with their connected observation edges. The default visibility gate
matches the simulator perception window: 180 degree FOV, 30 m range, and
20 m absolute x/y bounds in `base_footprint`.

Existing landmark positions are also updated between graph optimizations. When
`update_existing_landmarks` is enabled, each associated cone observation is
transformed through the latest live pose estimate and fused into the landmark
vertex with a covariance-aware update. Keyframe observations still add g2o
edges; duplicate in-between observations only update the vertex estimate and
published covariance.

Parameters live in `config/graph_slam.yaml`.

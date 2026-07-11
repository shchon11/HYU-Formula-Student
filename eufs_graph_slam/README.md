# EUFS Graph SLAM

`eufs_graph_slam` is a ROS 2 Humble package that builds a 2D g2o graph from EUFS simulator car state and cone observations.

The node subscribes to:

- `/odometry_integration/car_state` (`eufs_msgs/msg/CarState`) for SE2 keyframe motion
- `/cones` (`eufs_msgs/msg/ConeArrayWithCovariance`) for acquisition-stamped local cone observations in `base_footprint`

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
transformed through its acquisition-time pose and fused into the landmark
vertex with a covariance-aware update. Keyframe observations still add g2o
edges; duplicate in-between observations only update the vertex estimate and
published covariance.

## Observation-time alignment

Graph SLAM retains every `CarState` sample in a bounded history and linearly
interpolates x/y at each cone array timestamp. Yaw uses the shortest angular
path, including across the `-pi`/`pi` boundary; timestamps are never
extrapolated. The node selects the latest graph keyframe at or before the
observation and computes

```text
delta = inverse(raw_keyframe_pose) * raw_observation_pose
```

The cone point and its 2x2 covariance are transformed by `delta` before the g2o
edge is attached to that keyframe. The corresponding observation-time map pose
is used consistently for data association, landmark fusion, map covariance,
and stale-landmark visibility. This removes the position bias that otherwise
appears when an acquisition-stamped detector result arrives after the vehicle
has moved (for example, a 100 ms delay is 1 m at 10 m/s).

Cone arrays must carry a non-zero timestamp and use the configured
`slam_base_frame`; invalid frames are dropped rather than assigned a fabricated
time or transform. A cone array slightly ahead of the newest `CarState` is held
in a bounded, timestamp-ordered queue until a successor state arrives. Frames
older than the retained history are dropped with a throttled diagnostic. When
the queue is full, the farthest-future frame is discarded so nearer frames can
still become processable. Out-of-order `CarState` samples are inserted into the
retained history and never reset the map. A ROS clock source transition or an
authoritative backward jump at least `clock_rollback_threshold` wide starts a
new epoch and resets the graph, history, pending observations, and observation
watermarks together.
Normal forward `/clock` ticks do not reset the graph. A reset also publishes
empty transient-local map and path messages plus a `DELETEALL` marker so late
subscribers cannot retain visuals from the previous epoch.
Duplicate or out-of-order cone arrays are dropped after their timestamp has
already been processed. `CarState` and cone samples too far ahead of ROS time
are rejected so delayed messages from a previous epoch cannot poison the new
history; cone samples are also bounded relative to the newest `CarState`.
`max_future_stamp_lead` must be smaller than `clock_rollback_threshold`, which
keeps every reset-triggering rollback wider than the accepted future window.
During replay of a rolled-back interval, the same bounded future tolerance is
kept so a valid input published a few milliseconds ahead of the `/clock`
callback is not lost. Inputs beyond that tolerance or older than the epoch
start are rejected. Until replay reaches the pre-reset high watermark, stamps
at or beyond that watermark are also rejected as an exclusive upper fence. An
input callback clears each reached fence as soon as the clock catches up. If a
second rollback occurs during replay, both inner and outer fences are retained
until their respective watermarks are reached, so the outer epoch cannot be
released early. Consequently,
later sub-threshold clock jitter cannot reactivate a stale replay guard. The
same callback path detects system-clock rollbacks that do not produce an rcl
ROS-time jump callback. Diagnostics use a steady clock so throttling continues
to work immediately after a rewind. The node disables Galactic's dedicated
`/clock` thread and handles clock updates on its single-thread executor, so a
clock reset cannot mutate the optimizer concurrently with state or cone
callbacks.
Because no input TF conversion is performed, `car_state_frame` must equal
`map_frame` and `car_state_child_frame` must equal `slam_base_frame`; the node
fails at startup when either invariant is violated. `map_frame`, `odom_frame`,
and `slam_base_frame` must also be non-empty and pairwise distinct.

Message headers do not carry an epoch identifier, so a stale DDS sample whose
timestamp is indistinguishable from the replayed current epoch cannot be proven
stale. The epoch floor, bounded replay-window future gate, monotonic cone gate,
and shallow sensor QoS make that residual narrow, but upstream publishers
should still discard their own queued work on clock rewind.

Time-alignment parameters:

| Parameter | Default | Purpose |
| --- | ---: | --- |
| `pose_history_duration` | `3.0` s | Time horizon for interpolation (keep at least 2 s for the detector pipeline) |
| `pose_history_max_samples` | `1024` | Hard memory bound for retained `CarState` samples |
| `clock_rollback_threshold` | `0.1` s | Backward ROS clock jump that resets the graph epoch |
| `max_future_stamp_lead` | `0.09` s | Maximum input lead over ROS time, including rollback replay, and cone lead over newest `CarState`; covers measured callback skew while remaining below the rollback threshold |
| `max_pending_cone_messages` | `32` | Hard bound for future cone arrays waiting for a state bracket |

All five are launch arguments as well as entries in
`config/graph_slam.yaml`.

Parameters live in `config/graph_slam.yaml`.

## Optimizer workload bound

The default `max_optimization_poses: 300` keeps the newest 300 pose vertices
variable during each optimization and fixes every older pose at its latest
estimate. Pose zero always remains fixed as the graph gauge. Set the parameter
to `0` only when explicit full-batch optimization is required. The Eigen sparse
linear solver is used so solve cost follows the active sparse graph rather than
forming a dense pose system. Keep the active window at least twice the
`optimize_every_n_keyframes` interval unless a measured workload justifies a
smaller horizon.

This active-pose window bounds the Hessian dimension, but it does not prune
historical vertices or edges. Edge storage and linearization work therefore
still grow with route duration; the current design is appropriate for bounded
Formula Student runs, not an indefinitely running mapping daemon.

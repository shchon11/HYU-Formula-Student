#!/usr/bin/env bash
# Copyright 2026 shchon11
#
# Runs a full headless graph SLAM experiment:
#   Gazebo (small_track) + drift odometry + graph SLAM + pure-pursuit driver
#   + evaluator, then tears everything down and leaves a JSON report.
#
# Usage:
#   run_experiment.sh OUTPUT_JSON [DURATION] [DRIFT] [EXTRA_SLAM_PARAMS...]
#     OUTPUT_JSON        report path
#     DURATION           sim-time seconds to record (default 120)
#     DRIFT              1 = drifting odometry input (default), 0 = sim odometry
#     EXTRA_SLAM_PARAMS  forwarded verbatim, e.g. -p optimize_every_n_keyframes:=15

# ROS Humble needs the system python; drop any conda entries from PATH.
PATH="$(echo "$PATH" | tr ':' '\n' | grep -v conda | paste -sd:)"
export PATH
unset PYTHONHOME

WS="$(cd "$(dirname "$0")/../.." && pwd)"
OUT_JSON="${1:?output json path required}"
DURATION="${2:-120}"
DRIFT="${3:-1}"
shift $(( $# > 3 ? 3 : $# ))
EXTRA_PARAMS=("$@")

TRACK=small_track
CSV="$WS/eufs_sim/eufs_tracks/csv/$TRACK.csv"
SCRIPTS="$WS/eufs_graph_slam/scripts"
LOG_DIR="$(dirname "$OUT_JSON")"
mkdir -p "$LOG_DIR"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export EUFS_MASTER="$WS"
export ROS_LOCALHOST_ONLY=1

kill_harness() {
    # ros2 run/launch wrappers do not reliably forward signals to their
    # children, so kill the underlying processes by pattern as well.
    pkill -9 -f "install/eufs_graph_slam/lib/eufs_graph_slam/graph_slam_node" 2>/dev/null
    pkill -9 -f "scripts/drift_odom.py" 2>/dev/null
    pkill -9 -f "scripts/evaluate_slam.py" 2>/dev/null
    pkill -9 -f "scripts/drive_track.py" 2>/dev/null
    pkill -9 -x gzserver 2>/dev/null
    pkill -9 -f spawner.py 2>/dev/null
    # Leftover sim launch trees keep latching stale /robot_description,
    # which confuses RViz in later sessions.
    pkill -9 -f "launch eufs_launcher simulation.launch.py" 2>/dev/null
    pkill -9 -x robot_state_publisher 2>/dev/null
}

# Clear leftovers from any earlier aborted run.
kill_harness
sleep 1

PIDS=()
cleanup() {
    for pid in "${PIDS[@]}"; do
        kill -TERM "$pid" 2>/dev/null
    done
    sleep 2
    for pid in "${PIDS[@]}"; do
        kill -KILL "$pid" 2>/dev/null
    done
    kill_harness
}
trap cleanup EXIT INT TERM

echo "== launching simulator (headless, $TRACK) =="
# launch_group no_perception enables the simulated-perception /cones topic.
ros2 launch eufs_launcher simulation.launch.py \
    track:=$TRACK gazebo_gui:=false rviz:=false show_rqt_gui:=false \
    publish_gt_tf:=false pub_ground_truth:=true launch_group:=no_perception \
    > "$LOG_DIR/sim.log" 2>&1 &
PIDS+=($!)

echo "== waiting for /ground_truth/state =="
for i in $(seq 1 60); do
    if timeout 3 ros2 topic echo /ground_truth/state --once > /dev/null 2>&1; then
        break
    fi
    if [ "$i" = 60 ]; then
        echo "ERROR: simulator did not come up" >&2
        exit 1
    fi
done
echo "simulator up"

# The simulator now publishes drifting odometry natively on the localisation
# car state topic (driftOdometry in eufs_plugins.gazebo.xacro); ground truth
# stays on /ground_truth/state. The legacy drift_odom.py node is no longer used.
SLAM_INPUT=/odometry_integration/car_state
RAW_TOPIC=/odometry_integration/car_state

echo "== starting graph SLAM (input: $SLAM_INPUT) =="
ros2 run eufs_graph_slam graph_slam_node --ros-args \
    -r __node:=graph_slam \
    --params-file "$WS/eufs_graph_slam/config/graph_slam.yaml" \
    -p use_sim_time:=true \
    -p car_state_topic:=$SLAM_INPUT \
    "${EXTRA_PARAMS[@]}" \
    > "$LOG_DIR/slam.log" 2>&1 &
PIDS+=($!)

echo "== starting evaluator (${DURATION}s sim time) =="
python3 "$SCRIPTS/evaluate_slam.py" \
    --csv "$CSV" --duration "$DURATION" --output "$OUT_JSON" \
    --raw-odom "$RAW_TOPIC" \
    > "$LOG_DIR/eval.log" 2>&1 &
EVAL_PID=$!
PIDS+=($EVAL_PID)

echo "== starting driver =="
python3 "$SCRIPTS/drive_track.py" \
    --csv "$CSV" --duration $(( ${DURATION%.*} + 30 )) \
    > "$LOG_DIR/drive.log" 2>&1 &
PIDS+=($!)

wait $EVAL_PID
EVAL_RC=$?
echo "== evaluator finished (rc=$EVAL_RC) =="
tail -6 "$LOG_DIR/eval.log"

echo "== saving map (~/save_map) =="
timeout 10 ros2 service call /graph_slam/save_map std_srvs/srv/Trigger "{}" 2>&1 | tail -3

exit $EVAL_RC

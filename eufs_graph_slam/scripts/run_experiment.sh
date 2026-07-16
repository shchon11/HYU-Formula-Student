#!/usr/bin/env bash
# Copyright 2026 shchon11
#
# Runs a full headless graph SLAM experiment on the real sensor chain:
#   Gazebo (small_track) + sim Ellipse-D INS + SBG bridge (/gnss/odom anchor)
#   + wheel odometry (SLAM motion input) + graph SLAM + scripted driver
#   + evaluator, then tears everything down and leaves a JSON report.
#
# The plugin's synthetic /odometry_integration/car_state no longer exists
# (publishLocalisationCarState=false); the only state sources are the ones
# the real car publishes.
#
# Usage:
#   run_experiment.sh OUTPUT_JSON [DURATION] [GNSS_MODE] [EXTRA_SLAM_PARAMS...]
#     OUTPUT_JSON        report path
#     DURATION           sim-time seconds to record (default 120)
#     GNSS_MODE          rtk    = RTK fixed throughout (default)
#                        none   = bridge runs, SLAM gnss_prior_enable=false
#                        outage = RTK -> single-point at t=40s, back at t=80s
#     EXTRA_SLAM_PARAMS  forwarded verbatim, e.g. -p optimize_every_n_keyframes:=15

# ROS Humble needs the system python; drop any conda entries from PATH.
PATH="$(echo "$PATH" | tr ':' '\n' | grep -v conda | paste -sd:)"
export PATH
unset PYTHONHOME

# The repo IS the workspace's src folder: sources live under $SRC, but the
# live colcon install is at the workspace root above it ($SRC/install is a
# stale partial remnant — see the dual-install trap).
SRC="$(cd "$(dirname "$0")/../.." && pwd)"
WS="$(cd "$SRC/.." && pwd)"
if [ ! -f "$WS/install/setup.bash" ] && [ -f "$SRC/install/setup.bash" ]; then
    WS="$SRC"
fi
OUT_JSON="${1:?output json path required}"
DURATION="${2:-120}"
GNSS_MODE="${3:-rtk}"
shift $(( $# > 3 ? 3 : $# ))
EXTRA_PARAMS=("$@")

TRACK=small_track
CSV="$SRC/eufs_sim/eufs_tracks/csv/$TRACK.csv"
SCRIPTS="$SRC/eufs_graph_slam/scripts"
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
    pkill -9 -f "scripts/evaluate_slam.py" 2>/dev/null
    pkill -9 -f "scripts/drive_track.py" 2>/dev/null
    pkill -9 -f "sim_ellipse_d" 2>/dev/null
    pkill -9 -f "sbg_odometry_bridge" 2>/dev/null
    pkill -9 -f "eufs_graph_slam/wheel_odometry" 2>/dev/null
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

echo "== waiting for simulator =="
# Log-marker based wait: `ros2 topic echo` can flake on fresh DDS discovery
# even when the simulator is fully up, so trust the plugin-load log instead.
for i in $(seq 1 60); do
    if grep -q "RaceCarModelPlugin Loaded" "$LOG_DIR/sim.log" 2>/dev/null; then
        break
    fi
    if [ "$i" = 60 ]; then
        echo "ERROR: simulator did not come up" >&2
        tail -5 "$LOG_DIR/sim.log" >&2
        exit 1
    fi
    sleep 2
done
sleep 3
echo "simulator up"

# Real sensor chain: sim INS + SBG bridge (anchor) + wheel odometry (motion).
SLAM_INPUT=/wheel_odometry/car_state
RAW_TOPIC=/wheel_odometry/car_state

# ros2 launch rejects name:= with an empty value as malformed, so the
# schedule argument is only passed when it has content.
INS_ARGS=(slam:=false use_sim_time:=true)
if [ "$GNSS_MODE" = "outage" ]; then
    INS_ARGS+=(correction_schedule:="40:single,80:rtk_fixed")
fi

echo "== starting INS pipeline (sim Ellipse-D + SBG bridge, gnss=$GNSS_MODE) =="
ros2 launch eufs_graph_slam ins_pipeline.launch.py \
    "${INS_ARGS[@]}" \
    > "$LOG_DIR/ins.log" 2>&1 &
PIDS+=($!)

echo "== starting wheel odometry =="
ros2 run eufs_graph_slam wheel_odometry --ros-args \
    -p use_sim_time:=true \
    > "$LOG_DIR/wheel_odom.log" 2>&1 &
PIDS+=($!)

GNSS_PARAMS=()
if [ "$GNSS_MODE" = "none" ]; then
    GNSS_PARAMS+=(-p gnss_prior_enable:=false)
fi

echo "== starting graph SLAM (input: $SLAM_INPUT) =="
ros2 run eufs_graph_slam graph_slam_node --ros-args \
    -r __node:=graph_slam \
    --params-file "$SRC/eufs_graph_slam/config/graph_slam.yaml" \
    -p use_sim_time:=true \
    -p car_state_topic:=$SLAM_INPUT \
    "${GNSS_PARAMS[@]}" \
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

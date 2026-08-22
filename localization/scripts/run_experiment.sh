#!/usr/bin/env bash
# Copyright 2026 shchon11
#
# Runs a full headless graph SLAM experiment on the real sensor chain:
#   Gazebo (small_track) + sim Ellipse-D INS + sbg_raw_ekf (/localization/gnss_odom anchor)
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
#
#   PERCEPTION=sim overrides the default REAL perception (YOLO+LiDAR fusion).
#   Real perception is the default on purpose: ground-truth cones have no
#   position noise, so association-gate regressions sail through them — the
#   2026-07-17 map-pollution regression was invisible on GT cones.

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

# TRACK env overrides for long-map campaigns (autocross/trackdrive_kase2026).
TRACK="${TRACK:-small_track}"
CSV="$SRC/eufs_sim/eufs_tracks/csv/$TRACK.csv"
SCRIPTS="$SRC/hyu_localization/scripts"
# Per-run log dir: parallel/sequential runs into the same output folder must
# not overwrite each other's slam/sim logs.
LOG_DIR="${OUT_JSON%.json}.logs"
mkdir -p "$LOG_DIR"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export EUFS_MASTER="$WS"
export ROS_LOCALHOST_ONLY=1

kill_harness() {
    # ros2 run/launch wrappers do not reliably forward signals to their
    # children, so kill the underlying processes by pattern as well.
    pkill -9 -f "install/hyu_localization/lib/hyu_localization/graph_slam_node" 2>/dev/null
    pkill -9 -f "scripts/evaluate_slam.py" 2>/dev/null
    pkill -9 -f "scripts/drive_track.py" 2>/dev/null
    pkill -9 -f "sim_ellipse_d" 2>/dev/null
    pkill -9 -f "sbg_raw_ekf" 2>/dev/null
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

PERCEPTION="${PERCEPTION:-real}"
if [ "$PERCEPTION" = "real" ]; then
    PERC_ARGS=(perception_mode:=real perception_motion_compensation_frame:=odom)
else
    PERC_ARGS=(launch_group:=no_perception)
fi

echo "== launching simulator (headless, $TRACK, perception=$PERCEPTION) =="
ros2 launch eufs_launcher simulation.launch.py \
    track:=$TRACK gazebo_gui:=false rviz:=false show_rqt_gui:=false \
    publish_gt_tf:=false pub_ground_truth:=true "${PERC_ARGS[@]}" \
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

# Real sensor chain: sim INS + sbg_raw_ekf (fused odometry = SLAM motion input).
SLAM_INPUT=/localization/ins_odom
RAW_TOPIC=/localization/ins_odom

# ros2 launch rejects name:= with an empty value as malformed, so the
# schedule argument is only passed when it has content.
INS_ARGS=(slam:=false use_sim_time:=true)
if [ "$GNSS_MODE" = "outage" ]; then
    INS_ARGS+=(correction_schedule:="40:single,80:rtk_fixed")
fi

echo "== starting INS pipeline (sim Ellipse-D + SBG bridge, gnss=$GNSS_MODE) =="
ros2 launch hyu_localization ins_pipeline.launch.py \
    "${INS_ARGS[@]}" \
    > "$LOG_DIR/ins.log" 2>&1 &
PIDS+=($!)

GNSS_PARAMS=()
if [ "$GNSS_MODE" = "none" ]; then
    GNSS_PARAMS+=(-p gnss_prior_enable:=false)
fi

echo "== starting graph SLAM (input: $SLAM_INPUT) =="
ros2 run hyu_localization graph_slam_node --ros-args \
    -r __node:=graph_slam \
    --params-file "$SRC/hyu_localization/config/graph_slam.yaml" \
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
    --csv "$CSV" --duration $(( ${DURATION%.*} + 30 )) --speed "${DRIVE_SPEED:-4.5}" \
    > "$LOG_DIR/drive.log" 2>&1 &
PIDS+=($!)

# Fail fast instead of hanging when the simulator dies mid-run: the
# evaluator counts SIM time and waits forever once the clock stops. The
# patched gazebo segfaults roughly 1 run in 10 under GPU contention, and an
# unattended batch must not wedge on it.
(
    while kill -0 "$EVAL_PID" 2>/dev/null; do
        if ! pgrep -x gzserver >/dev/null 2>&1; then
            echo "gzserver died mid-run — aborting this run" >&2
            kill "$EVAL_PID" 2>/dev/null
            break
        fi
        sleep 5
    done
) &

wait $EVAL_PID
EVAL_RC=$?
echo "== evaluator finished (rc=$EVAL_RC) =="
tail -6 "$LOG_DIR/eval.log"

echo "== one-lap freeze latency =="
RET=$(grep -m1 "lap_return_criteria_satisfied=true" "$LOG_DIR/slam.log" | grep -oE "\[[0-9]+\.[0-9]+\]" | head -1 | tr -d '[]')
FRZ=$(grep -m1 "Mapping lap complete" "$LOG_DIR/slam.log" | grep -oE "\[[0-9]+\.[0-9]+\]" | head -1 | tr -d '[]')
if [ -n "$RET" ] && [ -n "$FRZ" ]; then
    python3 -c "print(f'lap-return -> map-freeze: {$FRZ - $RET:.1f} s')"
else
    echo "map freeze did not happen (RET=$RET FRZ=$FRZ)"
fi

echo "== saving map (~/save_map) =="
timeout 10 ros2 service call /graph_slam/save_map std_srvs/srv/Trigger "{}" 2>&1 | tail -3

exit $EVAL_RC

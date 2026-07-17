#!/usr/bin/env bash
# Copyright 2026 shchon11
#
# Acceptance trial for the FULL race composition (race.sh: sim + real
# perception + planning + PP controller + INS pipeline). Unlike
# run_experiment.sh — whose scripted driver follows the track CSV and
# therefore misses the live feedback loop (SLAM map -> local path -> PP
# driving -> observation quality -> map) — this runs the actual stack and
# scores the mission-level outcome the stack exists for:
#
#   * did mapping freeze and hand off to GLOBAL, and when
#   * final cone-map size (pollution shows up as >> GT count)
#   * laps completed within the window
#
# Usage:  race_trial.sh RESULT_JSON [TRACK] [TIMEOUT_SEC]
#   RESULT_JSON   where to write the outcome record
#   TRACK         default small_track
#   TIMEOUT_SEC   give up waiting for GLOBAL after this long (default 300)
#
# The race tmux session is torn down afterwards either way. Run repeatedly:
# the failure mode this exists to catch is PROBABILISTIC (association
# splits under real-perception noise), so a single pass proves nothing.

PATH="$(echo "$PATH" | tr ':' '\n' | grep -v conda | paste -sd:)"
export PATH
unset PYTHONHOME

SRC="$(cd "$(dirname "$0")/../.." && pwd)"
WS="$(cd "$SRC/.." && pwd)"
OUT_JSON="${1:?result json path required}"
TRACK="${2:-small_track}"
TIMEOUT_SEC="${3:-300}"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export EUFS_MASTER="$WS"
export ROS_LOCALHOST_ONLY=1

tmux kill-session -t race 2>/dev/null
sleep 2

"$SRC/scripts/race.sh" "$TRACK" real </dev/null >/dev/null 2>&1 || true
sleep 5
if ! tmux has-session -t race 2>/dev/null; then
    echo '{"error": "race session failed to start"}' > "$OUT_JSON"
    exit 1
fi

START_WALL=$(date +%s)
STATE="?"
GLOBAL_AT=""
MAP_COUNT="?"
LAPS="?"
FROZE_AT=""
while [ $(( $(date +%s) - START_WALL )) -lt "$TIMEOUT_SEC" ]; do
    sleep 10
    STATE=$(timeout 5 ros2 topic echo --once /planning/state 2>/dev/null \
        | grep -oE "LOCAL|GLOBAL|STOP" | head -1)
    LAPS=$(timeout 5 ros2 topic echo --once /planning/lap_count 2>/dev/null \
        | grep -oE "[0-9]+" | head -1)
    if [ "$STATE" = "GLOBAL" ]; then
        GLOBAL_AT=$(( $(date +%s) - START_WALL ))
        break
    fi
    # Early verdict: once SLAM has frozen its map, the planner either
    # accepts it within a minute or never will — waiting out the full
    # timeout only slows the A/B down.
    if [ -z "$FROZE_AT" ]; then
        SLAM_MODE=$(timeout 5 ros2 topic echo --once /localization/status 2>/dev/null \
            | grep -oE "localization" | head -1)
        [ -n "$SLAM_MODE" ] && FROZE_AT=$(( $(date +%s) - START_WALL ))
    elif [ $(( $(date +%s) - START_WALL - FROZE_AT )) -gt 60 ]; then
        break
    fi
done

MAP_COUNT=$(timeout 8 python3 - <<'PYEOF' 2>/dev/null
import rclpy, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from hyu_msgs.msg import ConeArrayWithCovariance
rclpy.init()
n = Node("race_trial_probe")
got = []
n.create_subscription(
    ConeArrayWithCovariance, "/localization/cone_map",
    lambda m: got.append(sum(len(getattr(m, c)) for c in (
        "blue_cones", "yellow_cones", "big_orange_cones",
        "unknown_color_cones"))),
    QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
t = time.time()
while time.time() - t < 6 and not got:
    rclpy.spin_once(n, timeout_sec=0.2)
print(got[-1] if got else "None")
PYEOF
)

# SLAM frame-latency snapshot (~/timing JSON: frame_ms p50/p99) for the
# backend A/B; taken before teardown while the node is still publishing.
TIMING=$(timeout 6 ros2 topic echo --once /graph_slam/timing 2>/dev/null \
    | grep -oE '\{.*\}' | head -1)

# Preserve the stack's node output BEFORE teardown: with output="screen" it
# lives only in the tmux panes (launch.log is a 4 KB skeleton on this
# system), so snapshot every pane's joined scrollback.
LOG_SNAP="${OUT_JSON%.json}.logs"
mkdir -p "$LOG_SNAP"
i=0
for p in $(tmux list-panes -t race -F '#{pane_id}' 2>/dev/null); do
    tmux capture-pane -t "$p" -p -J -S -20000 > "$LOG_SNAP/pane_$i.log" 2>/dev/null
    i=$((i + 1))
done

# tmux kill-session does not reap the detached ros2 launch trees: the
# perception pair (YOLO at >100% CPU each) survived it and contaminated
# every later trial through GPU contention. Kill the whole stack by name.
tmux kill-session -t race 2>/dev/null
sleep 2
for pat in gzserver gzclient graph_slam_node sim_ellipse_d sbg_odometry_bridge \
    wheel_odometry lidar_realism yolov8_bbox_node perception_node \
    planner_node wpnt_publisher_node frenet_odom_node hyu_local_planner_node \
    hyu_pure_pursuit_node ate_monitor planning_hyu_state_machine \
    hyu_path_selector robot_state_publisher; do
    pkill -9 -f "$pat" 2>/dev/null
done
pkill -9 -f "ros2 launch" 2>/dev/null
sleep 1

python3 - "$OUT_JSON" <<PYEOF
import json, sys
json.dump({
    "track": "$TRACK",
    "global_handoff": ${GLOBAL_AT:-None} is not None,
    "global_at_wall_sec": ${GLOBAL_AT:-None},
    "final_state": "${STATE:-unknown}",
    "laps": ${LAPS:-None},
    "map_count": ${MAP_COUNT:-None},
    "timing": ${TIMING:-None},
    "timeout_sec": $TIMEOUT_SEC,
}, open(sys.argv[1], "w"), indent=1)
PYEOF
echo "trial: state=${STATE} global_at=${GLOBAL_AT:-none}s map=${MAP_COUNT} laps=${LAPS}"

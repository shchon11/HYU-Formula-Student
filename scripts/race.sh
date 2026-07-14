#!/usr/bin/env bash
# race.sh — full autonomous stack in one tmux session.
#
#   race [track] [sim|real] [extra sim args...]   # start (default: small_track real)
#   race perception [track] [extra args]   # sim+perception+SLAM+teleop only,
#                                          # for per-tier evaluation vs GT cones
#   race stop                              # tear down
#   race attach                            # re-attach
#
# Brings up sim+perception, the whole planning_bringup graph (graph_slam +
# global/local planner + state machine + path_selector + pure-pursuit
# controller), then arms the mission so the CONTROLLER drives the car itself —
# no teleop. Panes self-sequence; a monitor pane shows the live path source /
# state / lap / cross-track error.

set -o pipefail

SESSION="race"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DEFAULT_EUFS_MASTER="$(cd "$SRC_DIR/.." && pwd)"
if [ ! -f "$DEFAULT_EUFS_MASTER/install/setup.zsh" ] && [ -f "$SRC_DIR/install/setup.zsh" ]; then
  DEFAULT_EUFS_MASTER="$SRC_DIR"
fi
EUFS_MASTER="${EUFS_MASTER:-$DEFAULT_EUFS_MASTER}"
if [ ! -f "$EUFS_MASTER/install/setup.zsh" ] && [ -f "$DEFAULT_EUFS_MASTER/install/setup.zsh" ]; then
  EUFS_MASTER="$DEFAULT_EUFS_MASTER"
fi
WS_SETUP="$EUFS_MASTER/install/setup.zsh"       # tmux panes run zsh
ROS_SETUP="/opt/ros/humble/setup.zsh"

case "${1:-start}" in
  stop|kill|down)
    tmux kill-session -t "$SESSION" 2>/dev/null && echo "race: stopped." || echo "race: no session."
    exit 0 ;;
  attach|a)
    exec tmux attach -t "$SESSION" ;;
esac

# Perception selector: default real = YOLO+LiDAR fusion. A bare 'sim' token
# switches to lightweight Gazebo cones on /cones without YOLO.
TRACK="small_track"
TRACK_SET=0
PMODE="real"
FILTERED=""
HAS_YOLO_MODEL_ARG=0
HAS_YOLO_DEVICE_ARG=0
HAS_MOTION_COMP_ARG=0
HAS_MONOCULAR_FALLBACK_ARG=0
HAS_STEREO_FALLBACK_ARG=0
EVAL_MODE=0
for tok in "$@"; do
  case "$tok" in
    perception) EVAL_MODE=1 ;;
    sim|real) PMODE="$tok" ;;
    *)
      case "$tok" in
        yolo_model_path:=*) HAS_YOLO_MODEL_ARG=1 ;;
        yolo_device:=*) HAS_YOLO_DEVICE_ARG=1 ;;
        perception_motion_compensation_frame:=*) HAS_MOTION_COMP_ARG=1 ;;
        motion_compensation_frame:=*) HAS_MOTION_COMP_ARG=1 ;;
        perception_monocular_fallback_enabled:=*) HAS_MONOCULAR_FALLBACK_ARG=1 ;;
        monocular_fallback_enabled:=*) HAS_MONOCULAR_FALLBACK_ARG=1 ;;
        perception_stereo_fallback_enabled:=*) HAS_STEREO_FALLBACK_ARG=1 ;;
        stereo_fallback_enabled:=*) HAS_STEREO_FALLBACK_ARG=1 ;;
      esac
      if [ "$TRACK_SET" -eq 0 ] && [[ "$tok" != *":="* ]]; then
        TRACK="$tok"
        TRACK_SET=1
      else
        FILTERED="$FILTERED $tok"
      fi
      ;;
  esac
done

# Tier evaluation needs the package's own (pose) weight and ALL tiers on, so the
# legacy-model pin and the tier disables below are skipped in perception mode.
LOCAL_YOLO_MODEL="$SRC_DIR/eufs_perception_baseline/models/fsoco_yolov8n/weights/best.pt"
if [ "$EVAL_MODE" -eq 0 ] && [ "$PMODE" = "real" ] && [ "$HAS_YOLO_MODEL_ARG" -eq 0 ] && [ -f "$LOCAL_YOLO_MODEL" ]; then
  FILTERED="$FILTERED yolo_model_path:=$LOCAL_YOLO_MODEL"
fi
if [ "$PMODE" = "real" ] && [ "$HAS_YOLO_DEVICE_ARG" -eq 0 ]; then
  FILTERED="$FILTERED yolo_device:=cuda"
fi
if [ "$PMODE" = "real" ] && [ "$HAS_MOTION_COMP_ARG" -eq 0 ]; then
  FILTERED="$FILTERED perception_motion_compensation_frame:=odom"
fi
if [ "$EVAL_MODE" -eq 0 ] && [ "$PMODE" = "real" ] && [ "$HAS_MONOCULAR_FALLBACK_ARG" -eq 0 ]; then
  FILTERED="$FILTERED perception_monocular_fallback_enabled:=false"
fi
if [ "$EVAL_MODE" -eq 0 ] && [ "$PMODE" = "real" ] && [ "$HAS_STEREO_FALLBACK_ARG" -eq 0 ]; then
  FILTERED="$FILTERED perception_stereo_fallback_enabled:=false"
fi
if [ "$EVAL_MODE" -eq 1 ]; then
  # cone_tiers labels every published cone with its tier; GT cones are the ruler.
  FILTERED="$FILTERED perception_publish_fusion_debug:=true pub_ground_truth:=true"
fi
EXTRA="perception_mode:=$PMODE$FILTERED"

# Skidpad tracks run the skidpad mission profile: local-planner-only planning
# behind the phase-gating director, no global planner, AMI_SKIDPAD mission.
PLAN_EXTRA=""
AMI_STATE=14   # AMI_TRACK_DRIVE
MISSION_NOTE="Lap 1 = local path, then handoff to global."
MONITOR_EXTRA=""
case "$TRACK" in
  skidpad*)
    PLAN_EXTRA=" skidpad:=true"
    AMI_STATE=12   # AMI_SKIDPAD
    MISSION_NOTE="skidpad: entry → right x2 → left x2 → exit (laps via skidpad_right/left_laps)."
    MONITOR_EXTRA="echo -n 'skidpad:     '; timeout 1 ros2 topic echo --once /skidpad/phase 2>/dev/null | grep -o 'data:.*'; "
    ;;
  accel|acceleration*)
    # Straight-line sprint on the 'acceleration' track (accept 'accel' shorthand).
    # Local-only planning, no global planner; the controller brakes itself when
    # the corridor cones end past the finish (local path goes invalid).
    TRACK="acceleration"
    PLAN_EXTRA=" acceleration:=true"
    AMI_STATE=11   # AMI_ACCELERATION
    MISSION_NOTE="acceleration: local-only straight sprint; controller brakes when the corridor cones end (finish → braking zone)."
    ;;
esac

if [ ! -f "$WS_SETUP" ]; then
  echo "race: workspace not built ($WS_SETUP missing). Run 'fsb' first." >&2
  exit 1
fi
tmux has-session -t "$SESSION" 2>/dev/null && { echo "race: already running — 'race stop' first, or 'race attach'."; exit 1; }

SRC="source $ROS_SETUP; source $WS_SETUP; export EUFS_MASTER=$EUFS_MASTER ROS_LOCALHOST_ONLY=1;"
WAIT_CAR="until ros2 node list 2>/dev/null | grep -q race_car; do sleep 2; done"
WAIT_GT="until ros2 topic list 2>/dev/null | grep -q /ground_truth/state; do sleep 2; done"
WAIT_STATE="until ros2 topic list 2>/dev/null | grep -q /planning/state; do sleep 2; done"

# Perception evaluation: sim + real 3-tier perception + SLAM + teleop. No
# planner, no controller — you drive, and the evaluator scores each tier's range
# error against the simulator's ground-truth cones.
if [ "$EVAL_MODE" -eq 1 ]; then
  echo "race: launching PERCEPTION+SLAM (teleop, no planner/controller) on track '$TRACK'…"
  tmux new-session -d -s "$SESSION" -n FSK

  P_SIM=$(tmux list-panes -t "$SESSION" -F '#{pane_id}' | head -1)
  tmux send-keys -t "$P_SIM" \
    "$SRC echo '[① SIM + PERCEPTION (real 3-tier, fusion debug on)]'; ros2 launch eufs_launcher simulation.launch.py track:=$TRACK gazebo_gui:=false rviz:=true show_rqt_gui:=false $EXTRA" C-m

  P_SLAM=$(tmux split-window -h -t "$P_SIM" -P -F '#{pane_id}')
  tmux send-keys -t "$P_SLAM" \
    "$SRC echo '[② GRAPH SLAM only] waiting for car…'; $WAIT_CAR; ros2 launch eufs_graph_slam graph_slam.launch.py" C-m

  # teleop arms AMI_MANUAL itself, so no separate mission pane is needed.
  P_TELE=$(tmux split-window -v -t "$P_SIM" -P -F '#{pane_id}')
  tmux send-keys -t "$P_TELE" \
    "$SRC echo '[③ TELEOP — drive a lap. arms AMI_MANUAL itself]'; $WAIT_CAR; sleep 3; ros2 run eufs_teleop teleop" C-m

  P_EVAL=$(tmux split-window -v -t "$P_TELE" -P -F '#{pane_id}')
  tmux send-keys -t "$P_EVAL" \
    "$SRC echo '[④ EVALUATOR] drive a lap first, then press Enter to score 60 s:'; echo '  ros2 run eufs_perception_baseline evaluate_perception_tiers.py --duration 60'" C-m

  P_MON=$(tmux split-window -v -t "$P_SLAM" -P -F '#{pane_id}')
  tmux send-keys -t "$P_MON" \
    "$SRC echo '[⑤ MONITOR] waiting for cones…'; until ros2 topic list 2>/dev/null | grep -q /cones; do sleep 2; done; while true; do printf '\\n== %s ==\\n' \"\$(date +%H:%M:%S)\"; for t in /yolo_bounding_boxes /yolo_cone_keypoints /cones /fusion/debug/cone_tiers /ground_truth/cones /graph_slam/map; do printf '%-28s ' \"\$t\"; timeout 2 ros2 topic hz \"\$t\" 2>/dev/null | grep -m1 -o 'average rate: [0-9.]*' || echo '(silent)'; done; sleep 3; done" C-m

  tmux select-layout -t "$SESSION" tiled
  tmux select-pane   -t "$P_TELE"
  tmux set-option    -t "$SESSION" mouse on

  cat <<EOF
race: perception+SLAM up on '$TRACK'.  attach → 'race attach'   |   stop → 'race stop'
  panes: ①sim+perception(3-tier, debug on)  ②graph_slam  ③teleop  ④evaluator  ⑤monitor
  NO planner/controller — drive with teleop (pane ③), then run the evaluator (pane ④).
  Per-tier range error vs /ground_truth/cones comes from /fusion/debug/cone_tiers.
EOF
  exit 0
fi

echo "race: launching AUTONOMOUS stack on track '$TRACK'…"
tmux new-session -d -s "$SESSION" -n FSK

# 0 · Simulator + perception ────────────────────────────────────────────────
P_SIM=$(tmux list-panes -t "$SESSION" -F '#{pane_id}' | head -1)
tmux send-keys -t "$P_SIM" \
  "$SRC echo \"[① SIM + PERCEPTION ($PMODE)]\"; ros2 launch eufs_launcher simulation.launch.py track:=$TRACK gazebo_gui:=false rviz:=true show_rqt_gui:=false $EXTRA" C-m

# 1 · Full planning graph (starts its OWN graph_slam) ────────────────────────
P_PLAN=$(tmux split-window -h -t "$P_SIM" -P -F '#{pane_id}')
tmux send-keys -t "$P_PLAN" \
  "$SRC echo '[② PLANNING: slam+global+local+SM+selector+controller] waiting for car…'; $WAIT_CAR; ros2 launch planning_bringup local_global_planning.launch.py graph_slam_ate_monitor:=true local_max_stamp_skew_sec:=2.0 local_max_input_age_sec:=3.0 local_max_start_distance_m:=8.0$PLAN_EXTRA" C-m

# 2 · Arm the mission — then the controller drives autonomously ──────────────
P_DRIVE=$(tmux split-window -v -t "$P_SIM" -P -F '#{pane_id}')
tmux send-keys -t "$P_DRIVE" \
  "$SRC echo '[③ MISSION] waiting for car…'; $WAIT_CAR; sleep 5; ros2 service call /ros_can/set_mission eufs_msgs/srv/SetCanState '{ami_state: $AMI_STATE}'; echo 'mission armed → controller now drives (no teleop). /cmd:'; ros2 topic hz /cmd" C-m

P_GNSS=$(tmux split-window -v -t "$P_DRIVE" -P -F '#{pane_id}')
tmux send-keys -t "$P_GNSS" \
  "$SRC echo '[④ GNSS HUD] waiting for ground truth…'; $WAIT_GT; ros2 launch eufs_graph_slam ins_pipeline.launch.py slam:=false use_sim_time:=true car_state_topic:=/ins_odom/car_state" C-m

P_MON=$(tmux split-window -v -t "$P_PLAN" -P -F '#{pane_id}')
tmux send-keys -t "$P_MON" \
  "$SRC echo '[⑤ MONITOR] waiting for planning…'; $WAIT_STATE; while true; do printf '\\n== %s ==\\n' \"\$(date +%H:%M:%S)\"; echo -n 'path_source: '; ros2 topic echo --once /planning/path_source 2>/dev/null | grep -o 'data:.*'; echo -n 'state:       '; ros2 topic echo --once /planning/state 2>/dev/null | grep -o 'data:.*'; echo -n 'lap:         '; ros2 topic echo --once /planning/lap_count 2>/dev/null | grep -o 'data:.*'; echo -n 'CTE d(m):    '; ros2 topic echo --once /planning/cte 2>/dev/null | grep -o 'data:.*'; ${MONITOR_EXTRA}sleep 3; done" C-m

tmux select-layout -t "$SESSION" tiled
tmux select-pane   -t "$P_MON"
tmux set-option    -t "$SESSION" mouse on

cat <<EOF
race: up.  attach → 'race attach'   |   stop everything → 'race stop'
  panes: ①sim+perception  ②planning(slam+global+local+SM+selector+controller)  ③mission  ④gnss-hud  ⑤monitor
  the CONTROLLER drives the car — no teleop. $MISSION_NOTE
EOF
exec tmux attach -t "$SESSION"

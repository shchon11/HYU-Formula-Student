#!/usr/bin/env bash
# sim.sh — STEP 1 (sim): simulator + perception, in a tmux session.
# (Was race.sh; 'race' is now the one-shot VEHICLE pipeline, see race.sh.)
#
#   sim [track] [sim|real] [bg] [norviz] [use_sim_time:=true|false] [extra sim args...]
#                                          # start (default: small_track real)
#   sim perception [track] [extra args]    # sim+perception+SLAM+teleop only,
#                                          # for per-tier evaluation vs GT cones
#   sim stop                               # tear down the WHOLE session (all steps)
#   sim attach                             # re-attach
#
# The run flow is three explicit steps — nothing drives until step 3:
#
#   step 1   sim small_track sim      simulator + perception   (this script)
#            fsk                      vehicle sensors + perception (fsk.sh)
#            race <mission>           vehicle: all three steps in one (race.sh)
#   step 2   stack                    INS + SLAM + planning + control (standby)
#   step 3   mission trackdrive 10    arm a mission — ONLY now can the car move
#
# The car physically cannot move before step 3: the sim race-car plugin
# ignores commands until /vehicle/set_mission puts it in AS_DRIVING, and
# mission.sh is the only thing that calls that service.
#
# Options:
#   sim | real   perception mode. 'real' (default) = YOLO+LiDAR fusion on the
#                simulated sensors; 'sim' = lightweight Gazebo ground-truth
#                cones straight onto /perception/cones, no YOLO.
#   bg           do not attach the tmux session (background/headless run).
#   norviz       start without RViz.

set -o pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/fsk-session.sh"

case "${1:-start}" in
  stop|kill|down)
    tmux kill-session -t "$SESSION" 2>/dev/null && echo "sim: stopped." || echo "sim: no session."
    exit 0 ;;
  attach|a)
    exec tmux attach -t "$SESSION" ;;
esac

# Perception selector: default real = YOLO+LiDAR fusion. A bare 'sim' token
# switches to lightweight Gazebo cones on /perception/cones without YOLO.
TRACK="small_track"
TRACK_SET=0
PMODE="real"
FILTERED=""
HAS_MOTION_COMP_ARG=0
EVAL_MODE=0
USE_SIM_TIME="true"
BG=0
RVIZ="true"
for tok in "$@"; do
  case "$tok" in
    perception) EVAL_MODE=1 ;;
    bg|headless) BG=1 ;;
    norviz) RVIZ="false" ;;
    rviz:=*) RVIZ="${tok#rviz:=}" ;;
    use_sim_time:=*) USE_SIM_TIME="${tok#use_sim_time:=}" ;;
    sim|real) PMODE="$tok" ;;
    tmpc)
      echo "sim: the TMPC hybrid stack is retired from this flow — the MAP controller drives. Ignoring 'tmpc'." >&2 ;;
    *)
      case "$tok" in
        perception_motion_compensation_frame:=*) HAS_MOTION_COMP_ARG=1 ;;
        motion_compensation_frame:=*) HAS_MOTION_COMP_ARG=1 ;;
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
# Accept the mission-name shorthands people type here out of habit: the track
# for accel is called 'acceleration'. (The mission itself is step 3's job.)
case "$TRACK" in
  accel) TRACK="acceleration" ;;
esac

# Only what simulation.launch.py actually declares may be passed. ros2 launch
# does NOT reject an undeclared argument -- it accepts it silently and the
# value goes nowhere -- so a name that has gone away reads as "configured"
# forever. (History of names removed for exactly that reason: yolo_model_path,
# yolo_device, perception_*_fallback_enabled — see git log.)
if [ "$PMODE" = "real" ] && [ "$HAS_MOTION_COMP_ARG" -eq 0 ]; then
  FILTERED="$FILTERED perception_motion_compensation_frame:=odom"
fi
if [ "$EVAL_MODE" -eq 1 ]; then
  # cone_provenance labels every published cone with what produced it; the
  # full GT track is the ruler. The real flag is perception_publish_debug.
  FILTERED="$FILTERED perception_publish_debug:=true pub_ground_truth:=true"
fi
EXTRA="perception_mode:=$PMODE$FILTERED"

fsk_require_built
fsk_session_exists && { echo "sim: already running — 'sim stop' first, or 'sim attach'."; exit 1; }

# Perception evaluation: sim + real perception + SLAM + teleop. No planner, no
# controller — you drive, and the evaluator scores each PROVENANCE against the
# full ground-truth track. Self-contained; steps 2/3 do not apply.
if [ "$EVAL_MODE" -eq 1 ]; then
  echo "sim: launching PERCEPTION+SLAM (teleop, no planner/controller) on track '$TRACK'…"
  tmux new-session -d -s "$SESSION" -n FSK

  P_SIM=$(tmux list-panes -t "$SESSION" -F '#{pane_id}' | head -1)
  tmux send-keys -t "$P_SIM" \
    "$SRC echo '[① SIM + PERCEPTION (LiDAR backbone + camera, provenance markers on)]'; ros2 launch eufs_launcher simulation.launch.py track:=$TRACK gazebo_gui:=false rviz:=$RVIZ show_rqt_gui:=false $EXTRA" C-m

  P_SLAM=$(tmux split-window -h -t "$P_SIM" -P -F '#{pane_id}')
  tmux send-keys -t "$P_SLAM" \
    "$SRC echo '[② INS + GRAPH SLAM] waiting for car…'; $WAIT_CAR; ros2 launch hyu_localization ins_pipeline.launch.py ${INS_MODE_SCHED:+mode_schedule:=$INS_MODE_SCHED} ${INS_CORR_SCHED:+correction_schedule:=$INS_CORR_SCHED}" C-m

  # teleop arms AMI_MANUAL itself, so no separate mission pane is needed.
  P_TELE=$(tmux split-window -v -t "$P_SIM" -P -F '#{pane_id}')
  tmux send-keys -t "$P_TELE" \
    "$SRC echo '[③ TELEOP — drive a lap. arms AMI_MANUAL itself]'; $WAIT_CAR; sleep 3; ros2 run hyu_teleop teleop" C-m

  # --duration 0 collects until Ctrl-C: start now, stop when the lap is done.
  P_EVAL=$(tmux split-window -v -t "$P_TELE" -P -F '#{pane_id}')
  tmux send-keys -t "$P_EVAL" \
    "$SRC echo '[④ EVALUATOR] collecting… drive a lap in pane ③, then Ctrl-C HERE for the per-tier report.'; $WAIT_CONES; ros2 run hyu_perception evaluate_perception_tiers.py --duration 0" C-m

  P_MON=$(tmux split-window -v -t "$P_SLAM" -P -F '#{pane_id}')
  tmux send-keys -t "$P_MON" \
    "$SRC echo '[⑤ MONITOR] rates are WALL clock: ros2 topic hz cannot read sim time.'; echo '   At RTF ~0.35 a 10 Hz sim topic reads ~3.5 here. That is CORRECT, not slow.'; echo '   Divide by RTF, or use: ros2 run hyu_perception measure_sim_rates.py 25'; $WAIT_CONES; while true; do printf '\\n== %s (wall Hz) ==\\n' \"\$(date +%H:%M:%S)\"; for t in /perception/bounding_boxes /perception/debug/cone_keypoints /perception/cones /perception/debug/cone_provenance /ground_truth/cones /ground_truth/track /localization/map; do printf '%-32s ' \"\$t\"; r=\$(timeout 6 env PYTHONUNBUFFERED=1 ros2 topic hz \"\$t\" 2>/dev/null | grep -m1 -o 'average rate: [0-9.]*'); if [ -n \"\$r\" ]; then echo \"\$r\"; elif timeout 3 ros2 topic echo --once \"\$t\" >/dev/null 2>&1; then echo 'alive (slow)'; else echo '(silent)'; fi; done; sleep 3; done" C-m

  tmux select-layout -t "$SESSION" tiled
  tmux select-pane   -t "$P_TELE"
  tmux set-option    -t "$SESSION" mouse on

  cat <<EOF
sim: perception+SLAM up on '$TRACK'.  attach → 'sim attach'   |   stop → 'sim stop'
  panes: ①sim+perception(provenance markers on)  ②ins+graph_slam  ③teleop  ④evaluator  ⑤monitor
  NO planner/controller — drive with teleop (pane ③), then run the evaluator (pane ④).
  Errors are measured against /ground_truth/track — the FULL track, not the
  FOV/range-filtered /ground_truth/cones.
EOF
  [ "$BG" -eq 1 ] || { [ -t 1 ] && exec tmux attach -t "$SESSION"; }
  exit 0
fi

echo "sim: STEP 1 — simulator + perception ($PMODE) on track '$TRACK'…"
tmux new-session -d -s "$SESSION" -n FSK

P_SIM=$(tmux list-panes -t "$SESSION" -F '#{pane_id}' | head -1)
tmux send-keys -t "$P_SIM" \
  "$SRC echo \"[① SIM + PERCEPTION ($PMODE)] step 2: 'stack'  step 3: 'mission <name> [laps]'\"; ros2 launch eufs_launcher simulation.launch.py track:=$TRACK gazebo_gui:=false rviz:=$RVIZ show_rqt_gui:=false $EXTRA" C-m

tmux set-option -t "$SESSION" mouse on
fsk_setenv FSK_ENV sim
fsk_setenv FSK_TRACK "$TRACK"
fsk_setenv FSK_USE_SIM_TIME "$USE_SIM_TIME"
fsk_setenv FSK_RVIZ "$RVIZ"

cat <<EOF
sim: step 1 up (track '$TRACK', perception '$PMODE', rviz $RVIZ).
  next:  step 2 → 'stack'                 (INS + SLAM + planning + control, standby)
         step 3 → 'mission trackdrive 10' | 'mission autocross' | 'mission skidpad 2' | 'mission acceleration'
  The car does NOT move until a mission is armed in step 3.
  attach → 'sim attach'   |   stop everything → 'sim stop'
EOF
[ "$BG" -eq 1 ] && { fsk_attach_or_report bg; exit 0; }
fsk_attach_or_report

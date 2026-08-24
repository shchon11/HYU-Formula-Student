#!/usr/bin/env bash
# sim.sh — STEP 1 (sim): simulator + perception, in a tmux session.
# (Was race.sh; 'race' is now the one-shot VEHICLE pipeline, see race.sh.)
#
#   sim [track] [sim|real] [bg] [norviz] [use_sim_time:=true|false] [extra sim args...]
#                                          # start (default: small_track real)
#   sim [track] lite [bg] [rviz] [clutter:=N] [seed:=N] [clutter_file:=..]
#                    [ecu:=udp|ros] [button:=auto|manual] [fix:=..] [extra launch args]
#                                          # the Gazebo-FREE simulator (hyu_lite_sim):
#                                          # bicycle car + emulated ECU/SBG/perception,
#                                          # off-track clutter as unknown cones. This is
#                                          # what runs on the Jetson (no arm64 Gazebo);
#                                          # chosen automatically when eufs is not built.
#   sim perception [track] [extra args]    # sim+perception+SLAM+teleop only,
#                                          # for per-tier evaluation vs GT cones (Gazebo)
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
#   lite|gazebo  backend. 'gazebo' = eufs (x86 workstation); 'lite' =
#                hyu_lite_sim, no Gazebo, no perception/YOLO -- cones are
#                emulated from the track csv plus CLUTTER (poles, bushes,
#                fences... that show up as unknown-colour cones, as on the
#                real course). Default: gazebo if eufs_launcher is built, else lite.
#   bg           do not attach the tmux session (background/headless run).
#   norviz/rviz  RViz off/on (gazebo default on, lite default off).
#
# lite backend knobs (all optional):
#   clutter:=N        off-track objects (default 60)      seed:=N   their placement seed
#   clutter_file:=f   load a clutter yaml (ros2 run hyu_lite_sim clutter_tool ...)
#   ecu:=udp|ros      udp (default) = through the REAL drive_udp_bridge on loopback
#                     ports, RPM feedback -> /vehicle/wheel_speeds; ros = /vehicle/cmd direct
#   button:=auto|manual  auto (default) latches the AS button ON: 'mission <name>'
#                     drives at once; manual = arm, then 'mission go' as on the car
#   fix:=SCHED        receiver fix schedule, e.g. fix:=90:rtk_float,100:rtk_fixed,150:outage,153:rtk_fixed
#   datum_lat:= datum_lon:= antenna_x:= antenna_y:=   world datum / antenna position (also
#                     handed to sbg_raw_ekf in step 2 so ground truth == odom frame)
#   anything else name:=value goes to lite_sim.launch.py as is.

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
RVIZ=""
BACKEND="auto"
LITE_ARGS=""
LITE_DATUM_LAT=""; LITE_DATUM_LON=""; LITE_ANT_X=""; LITE_ANT_Y=""; LITE_BUTTON="auto"; LITE_ECU="udp"
for tok in "$@"; do
  case "$tok" in
    perception) EVAL_MODE=1 ;;
    bg|headless) BG=1 ;;
    norviz) RVIZ="false" ;;
    rviz) RVIZ="true" ;;
    rviz:=*) RVIZ="${tok#rviz:=}" ;;
    use_sim_time:=*) USE_SIM_TIME="${tok#use_sim_time:=}" ;;
    lite|gazebo) BACKEND="$tok" ;;
    # lite-backend knobs (harmless to parse in gazebo mode; ignored there)
    clutter:=*)      LITE_ARGS="$LITE_ARGS clutter_count:=${tok#clutter:=}" ;;
    seed:=*)         LITE_ARGS="$LITE_ARGS clutter_seed:=${tok#seed:=}" ;;
    clutter_file:=*) LITE_ARGS="$LITE_ARGS clutter_file:=${tok#clutter_file:=}" ;;
    ecu:=*)          LITE_ECU="${tok#ecu:=}" ;;
    button:=*)       LITE_BUTTON="${tok#button:=}" ;;
    fix:=*)          LITE_ARGS="$LITE_ARGS fix_schedule:=${tok#fix:=}" ;;
    datum_lat:=*)    LITE_DATUM_LAT="${tok#datum_lat:=}" ;;
    datum_lon:=*)    LITE_DATUM_LON="${tok#datum_lon:=}" ;;
    antenna_x:=*)    LITE_ANT_X="${tok#antenna_x:=}" ;;
    antenna_y:=*)    LITE_ANT_Y="${tok#antenna_y:=}" ;;
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

# Backend: Gazebo (eufs) where it is built, else the Gazebo-free lite sim.
# eufs cannot be built on the Jetson (no arm64 gazebo), so there this is
# always 'lite' unless forced.
if [ "$BACKEND" = "auto" ]; then
  if [ -d "$EUFS_MASTER/install/eufs_launcher" ]; then BACKEND="gazebo"; else BACKEND="lite"; fi
fi
if [ "$BACKEND" = "lite" ]; then
  [ -d "$EUFS_MASTER/install/hyu_lite_sim" ] || {
    echo "sim: lite backend requested but hyu_lite_sim is not built (colcon build --symlink-install --base-paths src --packages-select hyu_lite_sim)." >&2; exit 1; }
  fsk_require_built
  fsk_session_exists && { echo "sim: already running — 'sim stop' first, or 'sim attach'."; exit 1; }
  if [ "$EVAL_MODE" -eq 1 ]; then
    echo "sim: 'sim perception' (per-tier perception evaluation) needs the Gazebo backend — the lite sim has no camera/LiDAR, its cones are emulated." >&2
    exit 1
  fi
  [ "$PMODE" = "sim" ] && echo "sim: note — the lite backend always emulates perception ('sim'/'real' only select YOLO in the Gazebo flow)."
  RVIZ="${RVIZ:-false}"
  case "$LITE_BUTTON" in
    auto|on|true)    AUTO_BUTTON=true ;;
    manual|off|false) AUTO_BUTTON=false ;;
    *) echo "sim: button:= must be auto or manual (got '$LITE_BUTTON')" >&2; exit 1 ;;
  esac
  # Everything that is NOT a lite knob is passed through to the launch.
  for tok in $FILTERED; do
    case "$tok" in
      perception_*) ;;   # gazebo perception args have no meaning here
      *:=*) LITE_ARGS="$LITE_ARGS $tok" ;;
    esac
  done
  LAUNCH="ros2 launch hyu_lite_sim lite_sim.launch.py track:=$TRACK rviz:=$RVIZ ecu:=$LITE_ECU auto_button:=$AUTO_BUTTON"
  LAUNCH="$LAUNCH${LITE_DATUM_LAT:+ datum_latitude:=$LITE_DATUM_LAT}${LITE_DATUM_LON:+ datum_longitude:=$LITE_DATUM_LON}"
  LAUNCH="$LAUNCH${LITE_ANT_X:+ antenna_offset_x:=$LITE_ANT_X}${LITE_ANT_Y:+ antenna_offset_y:=$LITE_ANT_Y}$LITE_ARGS"

  echo "sim: STEP 1 — LITE simulator (no Gazebo) on track '$TRACK', ecu $LITE_ECU, AS button $LITE_BUTTON…"
  tmux new-session -d -s "$SESSION" -n FSK
  P_SIM=$(tmux list-panes -t "$SESSION" -F '#{pane_id}' | head -1)
  tmux send-keys -t "$P_SIM" \
    "$SRC echo \"[① LITE SIM: car + ECU($LITE_ECU) + SBG + perception emulation | vehicle_state | bridge(loopback)] step 2: 'stack'  step 3: 'mission <name> [laps]'\"; $LAUNCH" C-m
  # A one-line-per-second status of the simulated car for the pane layout.
  P_ST=$(tmux split-window -v -t "$P_SIM" -P -F '#{pane_id}')
  tmux send-keys -t "$P_ST" \
    "$SRC echo '[② SIM STATUS] ecu rx/enable, command, speed, pose, fix'; until ros2 topic list --no-daemon 2>/dev/null | grep -q /sim/status; do sleep 2; done; ros2 topic echo /sim/status | grep --line-buffered '^data:' | sed -u 's/^data: //'" C-m
  tmux select-layout -t "$SESSION" tiled
  tmux select-pane   -t "$P_SIM"
  tmux set-option    -t "$SESSION" mouse on
  fsk_setenv FSK_ENV litesim
  fsk_setenv FSK_TRACK "$TRACK"
  fsk_setenv FSK_USE_SIM_TIME false
  fsk_setenv FSK_RVIZ "$RVIZ"
  fsk_setenv FSK_DATUM_LAT "$LITE_DATUM_LAT"
  fsk_setenv FSK_DATUM_LON "$LITE_DATUM_LON"
  fsk_setenv FSK_ANT_X "$LITE_ANT_X"
  fsk_setenv FSK_ANT_Y "$LITE_ANT_Y"
  cat <<EOF
sim: step 1 up (LITE backend, track '$TRACK', ecu $LITE_ECU, rviz $RVIZ).
  next:  step 2 → 'stack'                 (sbg_raw_ekf + SLAM + planning + control, standby)
         step 3 → 'mission trackdrive 10' | 'mission autocross' | 'mission skidpad 2' | 'mission acceleration'
  $( [ "$AUTO_BUTTON" = true ] && echo "AS button is latched ON: arming a mission drives at once (the bridge resets the map first)." \
                                 || echo "AS button OFF: after arming, 'mission go' releases the car (as on the vehicle)." )
  ground truth: /ground_truth/{state,track,clutter,cones}  markers: /sim/debug/{world,car,live_cones}  (frame odom == sim world)
  attach → 'sim attach'   |   stop everything → 'sim stop'
EOF
  [ "$BG" -eq 1 ] && { fsk_attach_or_report bg; exit 0; }
  fsk_attach_or_report
  exit 0
fi
RVIZ="${RVIZ:-true}"

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

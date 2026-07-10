#!/usr/bin/env bash
# race.sh — full autonomous stack in one tmux session.
#
#   race [track] [extra simfull args...]   # start (default track: small_track)
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
EUFS_MASTER="${EUFS_MASTER:-$HOME/fsk}"
WS_SETUP="$EUFS_MASTER/install/setup.zsh"       # tmux panes run zsh
ROS_SETUP="/opt/ros/humble/setup.zsh"

case "${1:-start}" in
  stop|kill|down)
    tmux kill-session -t "$SESSION" 2>/dev/null && echo "race: stopped." || echo "race: no session."
    exit 0 ;;
  attach|a)
    exec tmux attach -t "$SESSION" ;;
esac

TRACK="${1:-small_track}"; [ $# -gt 0 ] && shift || true
EXTRA="$*"

if [ ! -f "$WS_SETUP" ]; then
  echo "race: workspace not built ($WS_SETUP missing). Run 'fsb' first." >&2
  exit 1
fi
tmux has-session -t "$SESSION" 2>/dev/null && { echo "race: already running — 'race stop' first, or 'race attach'."; exit 1; }

SRC="source $ROS_SETUP; source $WS_SETUP; export EUFS_MASTER=$EUFS_MASTER ROS_LOCALHOST_ONLY=1;"
WAIT_CAR="until ros2 node list 2>/dev/null | grep -q race_car; do sleep 2; done"
WAIT_STATE="until ros2 topic list 2>/dev/null | grep -q /planning/state; do sleep 2; done"

echo "race: launching AUTONOMOUS stack on track '$TRACK'…"
tmux new-session -d -s "$SESSION" -n FSK

# 0 · Simulator + perception ────────────────────────────────────────────────
P_SIM=$(tmux list-panes -t "$SESSION" -F '#{pane_id}' | head -1)
tmux send-keys -t "$P_SIM" \
  "$SRC echo '[① SIM + PERCEPTION]'; ros2 launch eufs_launcher simulation.launch.py track:=$TRACK gazebo_gui:=true rviz:=true perception:=true $EXTRA" C-m

# 1 · Full planning graph (starts its OWN graph_slam) ────────────────────────
P_PLAN=$(tmux split-window -h -t "$P_SIM" -P -F '#{pane_id}')
tmux send-keys -t "$P_PLAN" \
  "$SRC echo '[② PLANNING: slam+global+local+SM+selector+controller] waiting for car…'; $WAIT_CAR; ros2 launch planning_bringup local_global_planning.launch.py" C-m

# 2 · Arm the mission — then the controller drives autonomously ──────────────
P_DRIVE=$(tmux split-window -v -t "$P_SIM" -P -F '#{pane_id}')
tmux send-keys -t "$P_DRIVE" \
  "$SRC echo '[③ MISSION] waiting for car…'; $WAIT_CAR; sleep 5; ros2 service call /ros_can/set_mission eufs_msgs/srv/SetCanState '{ami_state: 14}'; echo 'mission armed → controller now drives (no teleop). /cmd:'; ros2 topic hz /cmd" C-m

# 3 · Live monitor: path source / state / lap / cross-track error ────────────
P_MON=$(tmux split-window -v -t "$P_PLAN" -P -F '#{pane_id}')
tmux send-keys -t "$P_MON" \
  "$SRC echo '[④ MONITOR] waiting for planning…'; $WAIT_STATE; while true; do printf '\\n== %s ==\\n' \"\$(date +%H:%M:%S)\"; echo -n 'path_source: '; ros2 topic echo --once /planning/path_source 2>/dev/null | grep -o 'data:.*'; echo -n 'state:       '; ros2 topic echo --once /planning/state 2>/dev/null | grep -o 'data:.*'; echo -n 'lap:         '; ros2 topic echo --once /planning/lap_count 2>/dev/null | grep -o 'data:.*'; echo -n 'CTE d(m):    '; ros2 topic echo --once /planning/cte 2>/dev/null | grep -o 'data:.*'; sleep 3; done" C-m

tmux select-layout -t "$SESSION" tiled
tmux select-pane   -t "$P_MON"
tmux set-option    -t "$SESSION" mouse on

cat <<EOF
race: up.  attach → 'race attach'   |   stop everything → 'race stop'
  panes: ①sim+perception  ②planning(slam+global+local+SM+selector+controller)  ③mission  ④monitor
  the CONTROLLER drives the car — no teleop. Lap 1 = local path, then handoff to global.
EOF
exec tmux attach -t "$SESSION"

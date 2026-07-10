#!/usr/bin/env bash
# race.sh — bring up the whole HYU FS stack in one tmux session.
#
#   race [track] [extra simfull args...]   # start   (default track: small_track)
#   race stop                              # tear down the session
#   race attach                            # re-attach if detached
#
# Panes self-sequence: SLAM waits for the car, planning waits for SLAM odom,
# and the mission is armed automatically. Each stage keeps its own pane/log.

set -euo pipefail

SESSION="race"
EUFS_MASTER="${EUFS_MASTER:-$HOME/fsk}"
WS_SETUP="$EUFS_MASTER/install/setup.bash"
ROS_SETUP="/opt/ros/humble/setup.bash"

# ── sub-commands ──────────────────────────────────────────────────────────
case "${1:-start}" in
  stop|kill|down)
    tmux kill-session -t "$SESSION" 2>/dev/null && echo "race: stopped." || echo "race: no session."
    exit 0 ;;
  attach|a)
    exec tmux attach -t "$SESSION" ;;
esac

TRACK="${1:-small_track}"; [ $# -gt 0 ] && shift || true
EXTRA="$*"   # anything after the track is forwarded to simfull

if [ ! -f "$WS_SETUP" ]; then
  echo "race: workspace not built ($WS_SETUP missing). Run 'fsb' first." >&2
  exit 1
fi
tmux has-session -t "$SESSION" 2>/dev/null && { echo "race: already running — 'race stop' first, or 'race attach'."; exit 1; }

# Prefix every pane command with a fresh ROS + workspace source so panes work
# regardless of the user's shell rc.
SRC="source $ROS_SETUP; source $WS_SETUP; export EUFS_MASTER=$EUFS_MASTER ROS_LOCALHOST_ONLY=1;"

# Readiness guards (poll the ROS graph instead of blind sleeps).
WAIT_CAR="until ros2 node list 2>/dev/null | grep -q race_car; do sleep 2; done"
WAIT_ODOM="until ros2 topic list 2>/dev/null | grep -q /localization/ego_odom; do sleep 2; done"
WAIT_WPNT="until ros2 topic list 2>/dev/null | grep -q /global_waypoints; do sleep 2; done"

echo "race: launching stack on track '$TRACK'…"
tmux new-session -d -s "$SESSION" -n FSK

# Target panes by their stable pane-id (%N), not window.pane indices — the
# latter break under a non-zero base-index tmux config.

# 0 · Simulator + perception ────────────────────────────────────────────────
P_SIM=$(tmux list-panes -t "$SESSION" -F '#{pane_id}' | head -1)
tmux send-keys -t "$P_SIM" \
  "$SRC echo '[① SIM + PERCEPTION]'; simfull track:=$TRACK gazebo_gui:=true rviz:=true $EXTRA" C-m

# 1 · SLAM (waits for the spawned car) ───────────────────────────────────────
P_SLAM=$(tmux split-window -h -t "$P_SIM" -P -F '#{pane_id}')
tmux send-keys -t "$P_SLAM" \
  "$SRC echo '[② SLAM] waiting for car…'; $WAIT_CAR; slam gnss_prior_enable:=false" C-m

# 2 · Planning (waits for SLAM ego odom) ─────────────────────────────────────
P_PLAN=$(tmux split-window -v -t "$P_SIM" -P -F '#{pane_id}')
tmux send-keys -t "$P_PLAN" \
  "$SRC echo '[③ PLANNING] waiting for SLAM…'; $WAIT_ODOM; plan" C-m

# 3 · State machine (waits for global waypoints) ─────────────────────────────
P_SM=$(tmux split-window -v -t "$P_SLAM" -P -F '#{pane_id}')
tmux send-keys -t "$P_SM" \
  "$SRC echo '[④ STATE MACHINE] waiting for /global_waypoints…'; $WAIT_WPNT; smachine" C-m

# 4 · Drive: arm mission automatically, then hand the pane to teleop ──────────
P_DRIVE=$(tmux split-window -v -t "$P_PLAN" -P -F '#{pane_id}')
tmux send-keys -t "$P_DRIVE" \
  "$SRC echo '[⑤ DRIVE] waiting for car…'; $WAIT_CAR; sleep 3; mission; echo 'mission armed → AS_DRIVING in ~5s'; sleep 6; teleop" C-m

tmux select-layout -t "$SESSION" tiled
tmux select-pane   -t "$P_DRIVE"       # focus the teleop pane so you can drive
tmux set-option    -t "$SESSION" mouse on

cat <<EOF
race: up.  attach → 'race attach'   |   stop everything → 'race stop'
  panes: ①sim+perception  ②slam  ③planning  ④state-machine  ⑤drive(teleop)
  drive pane is focused; mission arms itself once the car is spawned.
EOF
exec tmux attach -t "$SESSION"

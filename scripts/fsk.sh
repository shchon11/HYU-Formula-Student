#!/usr/bin/env bash
# fsk.sh — STEP 1 (vehicle): real sensor drivers + perception, in tmux.
#
#   fsk [bg] [rviz] [extra perception args...]
#   fsk stop | attach        # same session as the sim flow
#
# Vehicle counterpart of 'race <track> sim' — same session name, so steps 2/3
# ('stack', 'mission …') work identically on top of either. RViz is OFF by
# default here (pit laptop opt-in: 'fsk rviz'); background with 'bg'.
#
# SENSOR DRIVERS ARE NOT IMPLEMENTED YET (2026-07: no LiDAR/ZED/SBG bringup in
# tree — see the real-vehicle readiness audit). Pane ① is the placeholder
# where that bringup will live; until it exists this pane just states what is
# missing so a vehicle run fails loudly at step 1, not silently at step 2.

set -o pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/fsk-session.sh"

case "${1:-start}" in
  stop|kill|down)
    tmux kill-session -t "$SESSION" 2>/dev/null && echo "fsk: stopped." || echo "fsk: no session."
    exit 0 ;;
  attach|a)
    exec tmux attach -t "$SESSION" ;;
esac

BG=0
RVIZ="false"
EXTRA=""
for tok in "$@"; do
  case "$tok" in
    bg|headless) BG=1 ;;
    rviz) RVIZ="true" ;;
    *) EXTRA="$EXTRA $tok" ;;
  esac
done

fsk_require_built
fsk_session_exists && { echo "fsk: already running — 'fsk stop' first, or 'fsk attach'."; exit 1; }

echo "fsk: STEP 1 — vehicle sensors + perception…"
tmux new-session -d -s "$SESSION" -n FSK

# ① Sensor drivers — placeholder until the real bringup lands. Replace the
# heredoc with `ros2 launch <vehicle_bringup> sensors.launch.py` when it does.
P_SENS=$(tmux list-panes -t "$SESSION" -F '#{pane_id}' | head -1)
tmux send-keys -t "$P_SENS" \
  "$SRC clear; cat <<'TODO'
[① VEHICLE SENSORS — NOT IMPLEMENTED]
  Needed here (in launch order):
    - LiDAR driver           → /velodyne_points (or remap perception input)
    - ZED camera driver      → left image + camera_info (watch topic names/QoS:
                               ZED publishes sensor-data QoS, perception expects it)
    - SBG Ellipse-D driver   → sbg_ros2_driver is in-tree but needs a NON-stock
                               config (stock config != our INS chain expectations)
  Until these exist the vehicle flow stops here by design.
TODO" C-m

# ② Perception — the same pipeline the sim 'real' mode runs, on real topics.
# Topic remaps for the actual sensors belong HERE next to the drivers.
P_PERC=$(tmux split-window -h -t "$P_SENS" -P -F '#{pane_id}')
tmux send-keys -t "$P_PERC" \
  "$SRC echo '[② PERCEPTION (YOLO + LiDAR fusion)]'; ros2 launch hyu_perception perception.launch.py use_sim_time:=false$EXTRA" C-m

if [ "$RVIZ" = "true" ]; then
  P_RV=$(tmux split-window -v -t "$P_PERC" -P -F '#{pane_id}')
  tmux send-keys -t "$P_RV" \
    "$SRC echo '[③ RVIZ]'; rviz2 -d \"$EUFS_MASTER/install/eufs_launcher/share/eufs_launcher/config/default.rviz\"" C-m
fi

tmux select-layout -t "$SESSION" tiled
tmux set-option    -t "$SESSION" mouse on
fsk_setenv FSK_ENV vehicle
fsk_setenv FSK_USE_SIM_TIME false
fsk_setenv FSK_RVIZ "$RVIZ"

cat <<EOF
fsk: step 1 up (vehicle mode, rviz $RVIZ). Sensor drivers are still a stub — see pane ①.
  next:  step 2 → 'stack'                 (INS + SLAM + planning + control, standby)
         step 3 → 'mission trackdrive 10' | 'mission autocross' | 'mission skidpad 2' | 'mission acceleration'
  The car does NOT move until a mission is armed in step 3.
  attach → 'fsk attach'   |   stop everything → 'fsk stop'
EOF
[ "$BG" -eq 1 ] && { fsk_attach_or_report bg; exit 0; }
fsk_attach_or_report

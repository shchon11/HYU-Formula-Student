#!/usr/bin/env bash
# fsk.sh — STEP 1 (vehicle): real sensor drivers + perception, in tmux.
#
#   fsk [bg] [rviz] [sensor args...] [extra perception args...]
#   fsk stop | attach        # same session as the sim flow
#
# Vehicle counterpart of 'race <track> sim' — same session name, so steps 2/3
# ('stack', 'mission …') work identically on top of either. RViz is OFF by
# default here (pit laptop opt-in: 'fsk rviz'); background with 'bg'.
#
# Pane ① is hyu_sensor_bringup: RS-16 + ZED + SBG published under /sensors,
# and the TF chain. base_footprint is the ground point below the ZED's stereo
# centre (+x = camera forward); the camera pose comes from the `ground` block
# of the active extrinsic (calib.sh writes it), and base_footprint -> rslidar
# is composed from that plus the extrinsic. Args of the form lidar:=,
# camera:=, gnss:=, tf:=, mount:=, extrinsic:=, camera_frame:=, ntrip:= (and
# ntrip_*:=) are routed there; anything else goes to perception in pane ②.
# Examples:
#   fsk extrinsic:=~/fsk/extrinsics/2026-07-26_0223.yaml
#   fsk gnss:=off camera:=off          # lidar-only smoke test
#   fsk ntrip:=true                    # + RTK corrections (NGII defaults)
#
# Pane ① prints the composed base_footprint -> rslidar pose on startup: the
# LiDAR sits on the nose ~1.7 m ahead of the hoop camera at x = 0, ~0.54 m up,
# yaw ~ -83 deg (twisted mount) -- eyeball that after any re-mount or
# re-calibration. If it says "FALLBACK", the extrinsic has no ground block.

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
SENSOR_EXTRA=""
for tok in "$@"; do
  case "$tok" in
    bg|headless) BG=1 ;;
    rviz) RVIZ="true" ;;
    # Sensor-side knobs go to pane ①; everything else is a perception arg.
    lidar:=*|camera:=*|camera_model:=*|gnss:=*|tf:=*|mount:=*|extrinsic:=*|camera_frame:=*)
      SENSOR_EXTRA="$SENSOR_EXTRA $tok" ;;
    # ntrip:= and every ntrip_* credential/override belong to the SBG driver.
    ntrip:=*|ntrip_*:=*)
      SENSOR_EXTRA="$SENSOR_EXTRA $tok" ;;
    *) EXTRA="$EXTRA $tok" ;;
  esac
done

fsk_require_built
fsk_session_exists && { echo "fsk: already running — 'fsk stop' first, or 'fsk attach'."; exit 1; }

echo "fsk: STEP 1 — vehicle sensors + perception…"
tmux new-session -d -s "$SESSION" -n FSK

# ① Sensor drivers — RS-16 + ZED + SBG under /sensors, plus the TF chain
# (base_footprint -> rslidar from the mount yaml, -> camera composed with the
# active extrinsic). Topic remaps for the real sensors live in that launch, not
# here: perception already defaults to the /sensors names it publishes.
P_SENS=$(tmux list-panes -t "$SESSION" -F '#{pane_id}' | head -1)
tmux send-keys -t "$P_SENS" \
  "$SRC echo '[① VEHICLE SENSORS (RS-16 + ZED + SBG) + TF]'; ros2 launch hyu_sensor_bringup sensors.launch.py$SENSOR_EXTRA" C-m

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

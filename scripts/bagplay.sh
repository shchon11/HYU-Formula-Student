#!/usr/bin/env bash
# bagplay.sh — STEP 1 (bag): replay recorded sensors, run perception live.
#
#   bagplay [bag] [bg] [rate:=1.0] [extra perception args...]
#   bagplay stop | attach          # same session as the sim/vehicle flow
#
# Drop-in replacement for step 1 when the car is not available: 'fsk' but fed
# from a bag. Steps 2 and 3 ('stack', 'mission ...') work identically on top,
# so this is how you check that control commands actually come out end to end
# without a car, a wheel encoder, or an SBG on the desk.
#
# loop:=false (or 'once') plays the bag ONE time. Looping keeps the pipeline
# fed, but every wrap rewinds /clock by the bag's length, and TF cannot resolve
# across a backward jump while the buffer still holds post-jump transforms --
# in RViz the map frame blinks out for a few seconds at each wrap. Use once
# when you need a steady view, loop when you need the stream to never stop.
#
# What is REPLAYED (looping by default, so the run never ends):
#     /sensors/lidar/points                 the LiDAR backbone
#     /sensors/camera/{left,right}/*        decompressed to the raw topics
#                                           perception subscribes
#
# What is FAKED (both stationary, which is what these bags actually are):
#     /vehicle/wheel_speeds                 zeros. No encoder is fitted.
#     /sbg/ekf_nav, ekf_euler,              a NAV_POSITION fix at a fixed
#     ekf_rot_accel_body                    anchor. The recorded SBG is NOT
#                                           replayed: it is VERTICAL_GYRO for
#                                           all 3643 messages, never once an
#                                           absolute fix, so the bridge refuses
#                                           it and SLAM gets no motion input.
#
# What is NOT replayed: the bag's own /perception/* topics. Perception runs
# live here -- that is the point -- and two publishers on /perception/cones
# would interleave two different answers on one topic.
#
# The bag's TF is skipped too. It carries the ZED's internal chain rooted at
# zed_camera_link, and this flow publishes base_footprint -> camera directly
# from the mount and the calibration; replaying both would give one frame two
# parents, which TF resolves silently and wrongly.
#
# Frames: bags recorded before the 2026-07 merge name the camera optical frame
# zed_left_camera_frame_optical (word order differs from today's
# zed_left_camera_optical_frame). This script reads the name out of the bag
# and tells both the TF publisher and perception to use it, so old and new
# bags both work.
set -o pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/fsk-session.sh"

case "${1:-start}" in
  stop|kill|down)
    tmux kill-session -t "$SESSION" 2>/dev/null && echo "bagplay: stopped." || echo "bagplay: no session."
    exit 0 ;;
  attach|a)
    exec tmux attach -t "$SESSION" ;;
esac

BAG=""; BG=0; RATE=1.0; LOOP=1; EXTRA=""
for tok in "$@"; do
  case "$tok" in
    bg|headless) BG=1 ;;
    rate:=*)     RATE="${tok#rate:=}" ;;
    loop:=false|once) LOOP=0 ;;
    *:=*)        EXTRA="$EXTRA $tok" ;;
    *)           [ -z "$BAG" ] && BAG="$tok" || EXTRA="$EXTRA $tok" ;;
  esac
done

# Default to the newest bag under $EUFS_MASTER/bag.
if [ -z "$BAG" ]; then
  BAG="$(find "$EUFS_MASTER/bag" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' \
         2>/dev/null | sort -rn | head -1 | cut -d' ' -f2-)"
  [ -n "$BAG" ] || { echo "bagplay: no bag in $EUFS_MASTER/bag — pass one explicitly." >&2; exit 1; }
  echo "bagplay: no bag given, using the newest: $(basename "$BAG")"
fi
[ -d "$BAG" ] || { echo "bagplay: not a bag directory: $BAG" >&2; exit 1; }
BAG="$(cd "$BAG" && pwd)"

fsk_require_built
fsk_session_exists && { echo "bagplay: already running — 'bagplay stop' first, or 'bagplay attach'."; exit 1; }

# --- what is actually in this bag --------------------------------------------
# Topic names moved around over the project's life, so resolve them from the
# bag rather than assuming. A bag missing the camera still replays the LiDAR.
# shellcheck disable=SC1090
source "/opt/ros/humble/setup.bash"; source "$WS_SETUP_BASH"
BAG_TOPICS="$(ros2 bag info "$BAG" 2>/dev/null | sed -n 's/.*Topic: \([^ |]*\).*/\1/p')"
has() { echo "$BAG_TOPICS" | grep -qx "$1"; }

CAM_COMPRESSED=""; CAM_INFO=""
for c in /sensors/camera/left/compressed /sensors/zed/left/color/rect/image/compressed; do
  has "$c" && { CAM_COMPRESSED="$c"; break; }
done
for c in /sensors/camera/left/info /sensors/zed/left/color/rect/camera_info; do
  has "$c" && { CAM_INFO="$c"; break; }
done

PLAY_TOPICS=(); REMAPS=()
has /sensors/lidar/points && PLAY_TOPICS+=(/sensors/lidar/points)
if [ -n "$CAM_COMPRESSED" ]; then
  PLAY_TOPICS+=("$CAM_COMPRESSED")
  [ -n "$CAM_INFO" ] && { PLAY_TOPICS+=("$CAM_INFO")
    REMAPS+=("$CAM_INFO:=/sensors/zed/left/color/rect/camera_info"); }
fi
# The INS is SYNTHESISED, not replayed -- see pane 3. Recorded SBG is skipped
# even when the bag has plenty of it, because what these bags have is
# solution_mode 1 (VERTICAL_GYRO) from end to end: no absolute fix, which the
# odometry bridge correctly refuses, so nothing downstream ever moves.
[ ${#PLAY_TOPICS[@]} -gt 0 ] || { echo "bagplay: no usable sensor topics in $BAG" >&2; exit 1; }

# Camera optical frame, straight from the recorded image header.
CAM_FRAME="zed_left_camera_optical_frame"
if [ -n "$CAM_COMPRESSED" ]; then
  f="$(timeout 60 python3 - "$BAG" "$CAM_COMPRESSED" <<'PY' 2>/dev/null
import sys, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id="sqlite3"),
       rosbag2_py.ConverterOptions("cdr", "cdr"))
while r.has_next():
    topic, data, _ = r.read_next()
    if topic == sys.argv[2]:
        print(deserialize_message(data, CompressedImage).header.frame_id); break
PY
)"
  [ -n "$f" ] && CAM_FRAME="$f"
fi

LOOP_FLAG=""; [ "$LOOP" = 1 ] && LOOP_FLAG=" --loop"
echo "bagplay: STEP 1 — $(basename "$BAG")  (rate $RATE, $([ "$LOOP" = 1 ] && echo looping || echo 'single pass'))"
echo "  topics: ${PLAY_TOPICS[*]}"
echo "  camera optical frame: $CAM_FRAME"

tmux new-session -d -s "$SESSION" -n FSK

# (1) The bag, on loop. -l restarts at the end, so the pipeline never starves.
# --clock, and every node below on use_sim_time, because the bag republishes
# messages with their ORIGINAL header stamps. Run the synthetic INS off the
# wall clock instead and its stamps sit days away from the cones': GraphSLAM
# drops every cone frame as "outside odometry buffer span" and the map stays
# empty while every topic looks alive. One clock domain, sourced from the bag.
P_BAG=$(tmux list-panes -t "$SESSION" -F '#{pane_id}' | head -1)
tmux send-keys -t "$P_BAG" \
  "$SRC echo '[① BAG: $(basename "$BAG")]'; ros2 bag play '$BAG'$LOOP_FLAG --clock 200 --rate $RATE --topics ${PLAY_TOPICS[*]}${REMAPS:+ --remap ${REMAPS[*]}}" C-m

# (2) Perception wants raw Image; bags carry the compressed stream because that
# is what fits on disk at 30 Hz. republish is the decoder.
if [ -n "$CAM_COMPRESSED" ]; then
  P_IMG=$(tmux split-window -h -t "$P_BAG" -P -F '#{pane_id}')
  tmux send-keys -t "$P_IMG" \
    "$SRC echo '[② DECOMPRESS $CAM_COMPRESSED -> raw]'; ros2 run image_transport republish compressed raw --ros-args -r in/compressed:=$CAM_COMPRESSED -r out:=/sensors/zed/left/color/rect/image" C-m
fi

# (3) TF from the mount and the active extrinsic, plus the stopped-car encoder.
# Drivers off — the bag is the sensor.
P_TF=$(tmux split-window -v -t "$P_BAG" -P -F '#{pane_id}')
tmux send-keys -t "$P_TF" \
  "$SRC echo '[③ TF + stopped INS/encoder]'; ros2 launch hyu_sensor_bringup sensors.launch.py lidar:=off camera:=off gnss:=off use_sim_time:=true camera_frame:=$CAM_FRAME & sleep 3; ros2 run hyu_sensor_bringup stationary_ins.py --ros-args -p use_sim_time:=true & ros2 run hyu_sensor_bringup stationary_wheels.py --ros-args -p use_sim_time:=true" C-m

# (4) Perception, live, on the replayed sensors.
# motion_compensation_frame:=base_footprint, not the default 'map'. map only
# exists once GraphSLAM is up in step 2, and until then EVERY cloud->camera
# lookup fails with "map ... does not exist" and the cones come out uncoloured.
# On a car standing still a frame rigidly attached to it is the correct fixed
# frame anyway: the cloud-stamp -> bbox-stamp transform is exactly identity.
P_PERC=$(tmux split-window -v -t "${P_IMG:-$P_BAG}" -P -F '#{pane_id}')
tmux send-keys -t "$P_PERC" \
  "$SRC echo '[④ PERCEPTION (YOLO + LiDAR fusion)]'; ros2 launch hyu_perception perception.launch.py use_sim_time:=true camera_frame:=$CAM_FRAME motion_compensation_frame:=base_footprint$EXTRA" C-m

tmux select-layout -t "$SESSION" tiled
tmux set-option    -t "$SESSION" mouse on
# vehicle, not sim: wall clock, and step 2 must not wait for gazebo ground truth.
fsk_setenv FSK_ENV vehicle
fsk_setenv FSK_USE_SIM_TIME true
fsk_setenv FSK_RVIZ false

cat <<EOF
bagplay: step 1 up (bag mode, looping). Sensors are recorded, perception is live.
  next:  step 2 → 'stack'
         step 3 → 'mission trackdrive 10' | 'mission autocross' | ...
  Control output is on /vehicle/cmd — 'rth /vehicle/cmd' once a mission is armed.
  attach → 'bagplay attach'   |   stop everything → 'bagplay stop'
EOF
[ "$BG" -eq 1 ] && { fsk_attach_or_report bg; exit 0; }
fsk_attach_or_report

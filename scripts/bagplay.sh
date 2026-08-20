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
# The INS is either REPLAYED or FAKED, decided per bag (override with ins:=).
#
#   ins:=real        replay the bag's own /sbg/* -- the car moves exactly as it
#                    did on the day, which is the only way step 2 and step 3 get
#                    a meaningful workout. Needs a receiver fix in the
#                    recording: gps_pos status SOL_COMPUTED, type >= SINGLE
#                    (what sbg_raw_ekf initialises on). The 0801 17:xx bags are
#                    gps_pos type 7 (RTK fixed) throughout.
#   ins:=stationary  synthesise a stationary RTK fix (raw gps_pos/vel/hdt +
#                    imu_data, plus the EKF topics) at a fixed anchor and zero
#                    wheel speeds. For bags whose receiver never got a fix --
#                    sbg_raw_ekf correctly waits on those, so replaying them
#                    leaves SLAM with no motion input and the map never builds.
#   ins:=auto        (default) sample the bag's gps_pos (ekf_nav on older bags
#                    without it) and pick.
#
# gnss:=off below turns off the sbg_driver only. sbg_raw_ekf and wheel_odometry
# come up with the 'odometry' arg regardless, so replayed /sbg/* is consumed
# and /localization/ins_odom appears exactly as on the car.
#
# No encoder is fitted, so /vehicle/wheel_speeds is synthesised either way:
# zeros in stationary mode, and in real mode back-computed from the replayed
# INS by sbg_wheels.py (wheels:=off to leave the topic silent). Zeros on a
# moving car are the worst of the three -- perception's LiDAR deskew reads
# /localization/wheel_odom and would smear every sweep taken in a corner.
#
# What real-mode wheels cannot do: wheel_odometry fuses them with the same INS
# they were derived from, so /localization/wheel_odom is no longer independent
# of the INS. Good enough for deskew, useless for judging the dead-reckoning
# fallback -- when the INS drops out here, its fallback drops out with it.
#
# What is NOT replayed: the bag's own /perception/* topics. Perception runs
# live here -- that is the point -- and two publishers on /perception/cones
# would interleave two different answers on one topic.
#
# The bag's TF is skipped too. It carries the ZED's internal chain rooted at
# zed_camera_link, and this flow publishes base_footprint -> camera directly
# from the calibration (its ground block); replaying both would give one frame two
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

BAG=""; BG=0; RATE=1.0; LOOP=1; LOOP_SET=0; EXTRA=""; INS_MODE=auto; WHEELS=on
for tok in "$@"; do
  case "$tok" in
    bg|headless) BG=1 ;;
    rate:=*)     RATE="${tok#rate:=}" ;;
    loop:=false|once) LOOP=0; LOOP_SET=1 ;;
    loop:=true)  LOOP=1; LOOP_SET=1 ;;
    ins:=*)      INS_MODE="${tok#ins:=}" ;;
    wheels:=*)   WHEELS="${tok#wheels:=}" ;;
    *:=*)        EXTRA="$EXTRA $tok" ;;
    *)           [ -z "$BAG" ] && BAG="$tok" || EXTRA="$EXTRA $tok" ;;
  esac
done
case "$INS_MODE" in
  auto|real|stationary) ;;
  *) echo "bagplay: ins:= must be auto, real or stationary (got '$INS_MODE')" >&2; exit 1 ;;
esac

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
[ ${#PLAY_TOPICS[@]} -gt 0 ] || { echo "bagplay: no usable sensor topics in $BAG" >&2; exit 1; }

# --- INS: replay the recording, or synthesise a stationary one ----------------
# Only the raw driver topics. /sbg_bridge/status and /localization/* are the
# bridge's OWN output and are regenerated live; replaying them would put two
# publishers on one topic and hide whether the bridge actually still works.
SBG_ALL=(/sbg/ekf_nav /sbg/ekf_euler /sbg/ekf_rot_accel_body /sbg/imu_data \
         /sbg/gps_pos /sbg/gps_vel /sbg/gps_hdt /sbg/status /sbg/utc_time)
SBG_TOPICS=()
for t in "${SBG_ALL[@]}"; do has "$t" && SBG_TOPICS+=("$t"); done

if [ "$INS_MODE" = auto ]; then
  INS_MODE=stationary
  if has /sbg/gps_pos || has /sbg/ekf_nav; then
    # A usable recording has a receiver fix: gps_pos SOL_COMPUTED with type >=
    # SINGLE, which is what sbg_raw_ekf initialises on. Older bags without
    # gps_pos fall back to the EKF's solution_mode >= 3. Sample rather than
    # scan: a 25 GB bag takes minutes to walk, and the fix comes early and
    # stays.
    fix="$(timeout 180 python3 - "$BAG" <<'PY' 2>/dev/null
import sys, rosbag2_py
from rclpy.serialization import deserialize_message
from sbg_driver.msg import SbgEkfNav, SbgGpsPos
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id="sqlite3"),
       rosbag2_py.ConverterOptions("cdr", "cdr"))
topics = {t.name for t in r.get_all_topics_and_types()}
want = "/sbg/gps_pos" if "/sbg/gps_pos" in topics else "/sbg/ekf_nav"
r.set_filter(rosbag2_py.StorageFilter(topics=[want]))
ok = n = 0
while r.has_next() and n < 4000:
    _, data, _ = r.read_next(); n += 1
    if want == "/sbg/gps_pos":
        m = deserialize_message(data, SbgGpsPos)
        good = (m.status.status == 0 and m.status.type >= 2
                and not (m.latitude == 0 and m.longitude == 0))
    else:
        good = deserialize_message(data, SbgEkfNav).status.solution_mode >= 3
    if good:
        ok = 1; break
print(f"{want} {ok}")
PY
)"
    if [ "${fix##* }" = 1 ] 2>/dev/null; then INS_MODE=real; fi
    echo "bagplay: ${fix% *} fix $([ "${fix##* }" = 1 ] && echo found || echo 'not found') -> ins:=$INS_MODE"
  fi
fi

if [ "$INS_MODE" = real ]; then
  [ ${#SBG_TOPICS[@]} -gt 0 ] || {
    echo "bagplay: ins:=real but this bag has no /sbg/* topics" >&2; exit 1; }
  PLAY_TOPICS+=("${SBG_TOPICS[@]}")
  # Looping a replayed INS teleports the car back to the start line every wrap.
  # A stationary INS does not care, a real one puts SLAM through a step change
  # it has no way to model, so single-pass unless the caller insists.
  if [ "$LOOP_SET" = 0 ] && [ "$LOOP" = 1 ]; then
    LOOP=0
    echo "bagplay: ins:=real -> single pass (loop:=true to override)"
  fi
fi

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
if [ "$INS_MODE" = real ]; then
  echo "  INS: real (recorded /sbg/*, sbg_raw_ekf live)"
  echo "  wheels: $([ "$WHEELS" = off ] && echo 'off (topic silent)' || echo 'synthesised from the INS -- wheel_odom is not independent of it')"
else
  echo "  INS: stationary (synthesised fix, zero wheels -- car stays parked)"
fi

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

# (3) TF from the active extrinsic (camera over ground + lidar), plus the stopped-car encoder.
# Drivers off — the bag is the sensor.
P_TF=$(tmux split-window -v -t "$P_BAG" -P -F '#{pane_id}')
if [ "$INS_MODE" = stationary ]; then
  TF_LABEL='[③ TF + stopped INS/encoder]'
  TF_FAKES=" & sleep 3; ros2 run hyu_sensor_bringup stationary_ins.py --ros-args -p use_sim_time:=true & ros2 run hyu_sensor_bringup stationary_wheels.py --ros-args -p use_sim_time:=true"
elif [ "$WHEELS" = off ]; then
  TF_LABEL='[③ TF + recorded INS, no encoder]'
  TF_FAKES=""
else
  TF_LABEL='[③ TF + recorded INS + encoder from INS]'
  TF_FAKES=" & sleep 3; ros2 run hyu_sensor_bringup sbg_wheels.py --ros-args -p use_sim_time:=true"
fi
tmux send-keys -t "$P_TF" \
  "$SRC echo '$TF_LABEL'; ros2 launch hyu_sensor_bringup sensors.launch.py lidar:=off camera:=off gnss:=off use_sim_time:=true camera_frame:=$CAM_FRAME$TF_FAKES" C-m

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

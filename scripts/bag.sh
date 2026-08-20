#!/usr/bin/env bash
# bag.sh — record everything worth keeping, minus the streams that cost the
# camera its frame rate.
#
#   bag                     record into bag/<timestamp>
#   bag drive3              record into bag/drive3
#   bag ... -d 60           anything else is passed to `ros2 bag record`
#
# WHY THE EXCLUSIONS
#
# image_transport encodes LAZILY -- a compressed/theora publisher does nothing
# until something subscribes. rosbag2 subscribing is that something. Recording
# the theora topics is therefore not a passive readout, it switches on a theora
# encoder per camera stream, and the raw topics push a full 3.69 MB BGRA frame
# (1280x720) per camera per frame through DDS with IPC off.
#
# Measured on the 0801 17:xx bags, from the wrapper's own /diagnostics:
#
#     Camera Grab rate:   30 Hz
#     Data Capture:       30.0 Hz (100%), frame drop 0.17%   <- grabbing is fine
#     Video/Depth:        20.5 -> 12.0 -> 9.0 Hz             <- publishing collapses
#     Video/Depth time:   27.7 -> 53.4 -> 83.2 ms  (budget 33 ms)
#
# The recordings averaged 3.3 Hz on every camera topic while pub_frame_rate was
# set to 30. Grab was never the problem; the publish path was, and most of that
# load exists only because it was being recorded.
#
# What goes, and what survives:
#   dropped   */image          the raw BGRA frames -- 3.69 MB each
#   dropped   */theora         and with them the theora encoders
#   dropped   /perception/debug/cloud_*   the RViz kept/removed LiDAR split:
#                             built only while subscribed, and the recorder
#                             IS a subscriber -- ~10 MB/s of a cloud that is
#                             a pure function of /sensors/lidar/points anyway
#   dropped   /sensors/zed/**  everything else under the ZED namespace, see below
#   kept      /sensors/zed/{left,right}/color/rect/image/compressed
#   kept      /sensors/zed/{left,right}/color/rect/camera_info
#   kept      everything else      lidar, sbg, perception, planning, control
#
# THE ZED WHITELIST
#
# The wrapper publishes far more than the four topics anything here consumes
# (perception.yaml, bagplay.sh, sync_capture.py all read left/right
# color/rect/image[/compressed] and color/rect/camera_info):
#
#   /sensors/zed/rgb/**              a copy of left -- verified byte-identical on
#                                    38/38 stamp-matched frames of
#                                    rosbag2_2026_08_01-17_22_34 -- so recording
#                                    it is one more JPEG encoder for a picture
#                                    already in the bag
#   */image/camera_info              image_transport's CameraPublisher twin of
#                                    */camera_info, same content, second topic
#   /sensors/zed/depth/**            depth is off; only camera_info survives
#   /sensors/zed/status/*, /sensors/zed_description
#
# So the ZED part of the filter is a whitelist written as a blacklist: exclude
# ^/sensors/zed unless the rest of the name is one of the four. rosbag2 feeds
# -x to std::regex (ECMAScript), which supports the (?!...) lookahead.
#
# QOS
#
# The recorder subscribes reliable + keep_all to the four ZED topics and the
# LiDAR, via --qos-profile-overrides-path (bag/.qos_overrides.yaml, rewritten
# on every run). Left to itself rosbag2 picks the subscription QoS from what
# the publishers offer; pinning it here makes the transport side of the
# recording explicit so a lost frame can only be the publisher's doing.
#
# Bags land in $EUFS_MASTER/bag, same as rbr().
set -o pipefail

WS="${EUFS_MASTER:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)}"

ZED_KEEP='(left|right)/color/rect/(image/compressed|camera_info)'
EXCLUDE="(/image\$)|(/theora\$)|(^/perception/debug/cloud_)|(^/sensors/zed(?!/$ZED_KEEP\$))"
NAME=""; PASS=()
for tok in "$@"; do
  case "$tok" in
    slim)  echo "bag: 'slim' is now the default (rgb dropped); ignoring" ;;
    -*)    PASS+=("$tok") ;;
    *)     if [ -z "$NAME" ] && [ ${#PASS[@]} -eq 0 ]; then NAME="$tok"
           else PASS+=("$tok"); fi ;;
  esac
done

mkdir -p "$WS/bag" || exit 1
cd "$WS/bag" || exit 1
[ -n "$NAME" ] && PASS+=(-o "$NAME")

QOS="$WS/bag/.qos_overrides.yaml"
{
  for t in /sensors/lidar/points \
           /sensors/zed/left/color/rect/image/compressed \
           /sensors/zed/right/color/rect/image/compressed \
           /sensors/zed/left/color/rect/camera_info \
           /sensors/zed/right/color/rect/camera_info; do
    printf '%s:\n  history: keep_all\n  reliability: reliable\n  durability: volatile\n' "$t"
  done
} > "$QOS"

echo "bag: recording into $WS/bag${NAME:+/$NAME}"
echo "  excluding: $EXCLUDE"
echo "  zed: only {left,right}/color/rect/{image/compressed,camera_info}"
echo "  qos: reliable/keep_all on zed + lidar ($QOS)"
echo "  Ctrl+C to stop."
exec ros2 bag record -a -x "$EXCLUDE" --qos-profile-overrides-path "$QOS" "${PASS[@]}"

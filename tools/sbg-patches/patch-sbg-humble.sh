#!/usr/bin/env bash
# Make the vendored sbg_ros2_driver (pinned to 3.3.2 in external.repos) build
# on ROS 2 Humble. Idempotent — safe to re-run after every `vcs import`.
#
#   bash tools/sbg-patches/patch-sbg-humble.sh
#
# Env override: SBG_SRC (default: the sbg_ros2_driver next to this repo's src).
#
# Why: sbg_driver 3.3.1 switched two tf2_ros includes from the C `.h` header to
# the `.hpp` variant ("removed usage of deprecated tf2 C header"). Those `.hpp`
# shims only exist on Iron+; Humble ships `tf2_ros/transform_broadcaster.h` and
# `tf2_ros/static_transform_broadcaster.h` only, so 3.3.2 fails to compile with
# "fatal error: tf2_ros/transform_broadcaster.hpp: No such file or directory".
#
# We touch ONLY the two tf2_ros includes. tf2/LinearMath/Quaternion.hpp and
# tf2_geometry_msgs/tf2_geometry_msgs.hpp DO exist on Humble, so they stay as-is.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="${SBG_SRC:-$(cd "$HERE/../.." && pwd)/sbg_ros2_driver}"

if [ ! -d "$SRC" ]; then
  echo "sbg_ros2_driver not found at: $SRC" >&2
  echo "Import it first:  vcs import src < src/external.repos" >&2
  exit 1
fi

changed=0
for rel in include/sbg_driver/message_wrapper.h src/message_wrapper.cpp; do
  f="$SRC/$rel"
  [ -f "$f" ] || { echo "skip (missing): $rel" >&2; continue; }
  before="$(cat "$f")"
  sed -i \
    -e 's#tf2_ros/transform_broadcaster\.hpp#tf2_ros/transform_broadcaster.h#g' \
    -e 's#tf2_ros/static_transform_broadcaster\.hpp#tf2_ros/static_transform_broadcaster.h#g' \
    "$f"
  if [ "$before" != "$(cat "$f")" ]; then
    echo "patched: $rel"
    changed=1
  fi
done

# Publish the EKF body rot/accel (bias-corrected yaw rate + gravity-free accel).
# wheel_odometry.py consumes /sbg/ekf_rot_accel_body for GNSS-free dead reckoning;
# the stock config ships it disabled (log_ekf_rot_accel_body: 0). The DEVICE must
# also output ekfRotAccelBody (comA) for the topic to carry data — that lives in
# the unit's flash, not here.
for rel in config/sbg_device_uart_default.yaml config/sbg_device_udp_default.yaml; do
  f="$SRC/$rel"
  [ -f "$f" ] || continue
  before="$(cat "$f")"
  sed -i 's/^\(\s*log_ekf_rot_accel_body:\s*\)0\b/\18/' "$f"
  if [ "$before" != "$(cat "$f")" ]; then echo "patched: $rel (log_ekf_rot_accel_body -> 8)"; changed=1; fi
  # Subscribe to the NTRIP client's RTCM so the driver forwards carrier-phase
  # corrections to the Ellipse-D for RTK (only the rtcm.subscribe flag; the
  # feed is hyu_localization ntrip_client on ntrip_client/rtcm). Scoped to the
  # rtcm: block so the nmea: block's own subscribe/publish is untouched.
  before="$(cat "$f")"
  sed -i '/^\s*rtcm:/,/^\s*nmea:/ s/^\(\s*subscribe:\s*\)false\b/\1true/' "$f"
  if [ "$before" != "$(cat "$f")" ]; then echo "patched: $rel (rtcm.subscribe -> true)"; changed=1; fi
done

if [ "$changed" -eq 0 ]; then
  echo "sbg_ros2_driver already Humble-compatible (no changes)."
else
  echo "Done. Rebuild:  colcon build --packages-select sbg_driver --symlink-install"
fi

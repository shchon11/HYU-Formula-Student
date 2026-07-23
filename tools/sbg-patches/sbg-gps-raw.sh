#!/usr/bin/env bash
# Toggle raw GNSS output (gps1Pos/gps1Vel/gps1Hdt) on the SBG Ellipse-D, for
# diagnosing GNSS reception (num_sv, fix type SINGLE/RTK, position accuracy).
#
#   sbg-gps-raw.sh on     # enable raw GNSS logs  -> /sbg/gps_pos, /sbg/gps_hdt
#   sbg-gps-raw.sh off    # ROLLBACK to committed backup state (all three off)
#
# Why a save is needed: the REST API POST only STAGES a setting; it does not
# affect the live output until the device reboots with the staged config
# applied. So each toggle = POST (stage) + SETTINGS_ACTION save (flash+reboot).
#
# ROLLBACK: this only ever touches the three gps1* message triggers. `off`
# returns them to the exact state in localization/docs/sbg_ellipse_d_settings_
# backup.json (gps1Pos/Vel/Hdt = off). Lever arms, headingMode, baudrate, and
# the ekfNav/ekfEuler/imuData/status/utcTime outputs are never modified.
#
# Env: SBG_PORT (default /dev/ttyUSB0), SBG_BAUD (default 115200).
set -euo pipefail

MODE="${1:-}"
case "$MODE" in
  on)  TRIG='"onChange"' ; LABEL="ENABLE raw GNSS" ;;
  off) TRIG='"off"'      ; LABEL="DISABLE raw GNSS (rollback)" ;;
  *)   echo "usage: $0 on|off" >&2 ; exit 2 ;;
esac

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
API="$HERE/bin/sbgEComApi"
ACT="$HERE/bin/sbg_settings_action"
PORT="${SBG_PORT:-/dev/ttyUSB0}"
BAUD="${SBG_BAUD:-115200}"

echo "== $LABEL  ($PORT @ $BAUD) =="

echo "1) freeing serial port (stopping any running sbg driver)..."
if command -v fuser >/dev/null && fuser "$PORT" >/dev/null 2>&1; then
  fuser -k -TERM "$PORT" 2>/dev/null || true
  for _ in $(seq 1 12); do fuser "$PORT" >/dev/null 2>&1 || break; sleep 0.5; done
  fuser "$PORT" >/dev/null 2>&1 && { fuser -k "$PORT" 2>/dev/null || true; sleep 1; } || true
fi
fuser "$PORT" >/dev/null 2>&1 && { echo "   ERROR: port still busy" >&2; exit 1; } || echo "   port free."

echo "2) staging gps1Pos / gps1Vel / gps1Hdt = $TRIG ..."
for m in gps1Pos gps1Vel gps1Hdt; do
  "$API" -s "$PORT" -r "$BAUD" -t 3 -n 5 -p -b "$TRIG" \
      "api/v1/settings/output/comA/messages/$m" >/dev/null
  echo "   staged $m"
done

echo "3) SAVE_SETTINGS (flash + reboot to apply) ..."
"$ACT" "$PORT" "$BAUD" save

echo "4) waiting for device reboot (~10 s) ..."
sleep 10

echo "5) verify:"
for m in gps1Pos gps1Vel gps1Hdt; do
  printf "   %-8s= " "$m"
  "$API" -s "$PORT" -r "$BAUD" -t 3 -n 8 "api/v1/settings/output/comA/messages/$m" || echo "(read failed)"
  echo
done

echo "done. Launch the driver to see topics:"
echo "   ros2 launch sbg_driver sbg_device_launch.py"

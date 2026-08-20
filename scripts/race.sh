#!/usr/bin/env bash
# race.sh — the whole VEHICLE pipeline in one go, mission armed.
#
#   race <mission> [laps] [step-1 args...]
#         sensors + perception  ->  vehicle state + ECU bridge  ->
#         INS + SLAM + planning + control  ->  mission ARMED
#   race <mission> [laps] bag:=<dir>       same, but step 1 is a bag replay
#                                          (bagplay.sh) -- full rehearsal on the bench
#   race stop | attach | status
#
# Missions (arming is mission.sh's): trackdrive [laps=10] | autocross [laps=1]
#   | skidpad [laps-per-circle=2] | acceleration | dlc | inspection
# Step-1 args are fsk's: bg, rviz, lidar:= camera:= gnss:= extrinsic:= ntrip:= ...
# (or bagplay's with bag:=: rate:= loop:= ins:= wheels:=)
#
# Afterwards the stack is live and /vehicle/cmd carries real driving commands.
# Whether the car MOVES is the ECU's decision from the autonomous-enable byte
# drive_udp_bridge sends: 1 only while /vehicle/as_state is AS_DRIVING, i.e.
# the AS button is ON (the physical toggle; 'mission go' / 'mission halt'
# stand in for it until its driver is wired, via /vehicle/set_as_button). On
# every OFF->ON edge the bridge first resets the SLAM map (/graph_slam/reset)
# and raises the byte only after that succeeded, so each run starts on a clean
# map the instant the button goes on; ON->OFF drops it immediately.
#
# The three steps stay usable by hand ('fsk', 'stack', 'mission …'); this
# script only sequences them. The simulator flow is 'sim <track> …' (sim.sh).

set -o pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/fsk-session.sh"

usage() { sed -n '2,22p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

case "${1:-}" in
  ""|help|-h|--help) usage; exit 0 ;;
  stop|kill|down)
    tmux kill-session -t "$SESSION" 2>/dev/null && echo "race: stopped." || echo "race: no session."
    exit 0 ;;
  attach|a) exec tmux attach -t "$SESSION" ;;
  status|st)
    fsk_require_session
    exec "$SCRIPT_DIR/mission.sh" status ;;
esac

MISSION="$1"; shift
case "$MISSION" in
  trackdrive|td|autocross|ax|skidpad|sp|acceleration|accel|dlc|inspection|insp) ;;
  *) echo "race: unknown mission '$MISSION'." >&2; usage; exit 1 ;;
esac
LAPS=""; STEP1=""; BAG=""
for tok in "$@"; do
  case "$tok" in
    bag:=*) BAG="${tok#bag:=}" ;;                # bench rehearsal: bag replay as step 1
    ''|*[!0-9]*) STEP1="$STEP1 $tok" ;;          # fsk / bagplay / sensor / perception args
    *) if [ -z "$LAPS" ]; then LAPS="$tok"; else STEP1="$STEP1 $tok"; fi ;;
  esac
done

fsk_require_built
fsk_session_exists && { echo "race: already running — 'race stop' first, or 'race attach'."; exit 1; }

wait_for() {  # wait_for <label> <seconds> <shell-test>   (never fatal: report and go on)
  local label="$1" limit="$2" test="$3" waited=0
  echo -n "race: waiting for $label"
  until eval "$test"; do
    sleep 2; waited=$((waited + 2)); echo -n "."
    if [ "$waited" -ge "$limit" ]; then
      echo " not up after ${limit}s — continuing; check the panes ('race attach')."; return 1
    fi
  done
  echo " up."
}

if [ -n "$BAG" ]; then
  echo "race: ① bag replay + perception ($BAG) — bench rehearsal, no real sensors"
  "$SCRIPT_DIR/bagplay.sh" "$BAG" $STEP1 || exit 1
else
  echo "race: ① vehicle sensors + perception"
  "$SCRIPT_DIR/fsk.sh" bg $STEP1 || exit 1
fi
fsk_source_env

# Vehicle state (AS/AMI: the /vehicle/* contract the sim plugin provides in
# sim) and the ECU bridge, each in its own pane so their logs stay readable.
P_FIRST=$(tmux list-panes -t "$SESSION" -F '#{pane_id}' | head -1)
P_VS=$(tmux split-window -v -t "$P_FIRST" -P -F '#{pane_id}')
tmux send-keys -t "$P_VS" \
  "$SRC echo '[③ VEHICLE STATE — /vehicle/as_state from the AS button; /vehicle/set_mission, reset, ebs]'; ros2 run hyu_planning_bringup vehicle_state.py" C-m
P_BR=$(tmux split-window -v -t "$P_VS" -P -F '#{pane_id}')
tmux send-keys -t "$P_BR" \
  "$SRC echo '[④ ECU BRIDGE — /vehicle/cmd -> Speedgoat UDP; autonomous byte = AS_DRIVING after a map reset]'; ros2 launch drive_udp_bridge drive_udp_bridge.launch.py" C-m
tmux select-layout -t "$SESSION" tiled
fsk_setenv FSK_VEHICLE_STATE_PANE "$P_VS"
fsk_setenv FSK_BRIDGE_PANE "$P_BR"

wait_for "vehicle state (/vehicle/set_mission)" 30 \
  "ros2 service list --no-daemon 2>/dev/null | grep -q /vehicle/set_mission"
wait_for "perception (/perception/cones)" 120 \
  "ros2 topic list --no-daemon 2>/dev/null | grep -q /perception/cones"

echo "race: ② stack (INS + SLAM + planning + control, standby)"
"$SCRIPT_DIR/stack.sh" || exit 1
wait_for "planning (/planning/state)" 120 \
  "ros2 topic list --no-daemon 2>/dev/null | grep -q /planning/state"
sleep 2

echo "race: ③ arming $MISSION${LAPS:+ $LAPS}"
if ! "$SCRIPT_DIR/mission.sh" "$MISSION" $LAPS; then
  echo "race: arming failed — fix the cause and run 'mission $MISSION${LAPS:+ $LAPS}' by hand." >&2
  exit 1
fi

cat <<SUMMARY
race: pipeline up, $MISSION ARMED. /vehicle/cmd is live; the ECU drives only while the AS button is ON.
  AS button stand-in: 'mission go'  (ON: SLAM map reset, then autonomous-enable 1)
                      'mission halt' (OFF: autonomous-enable 0 at once)
  'mission status' | 'race attach' | 'race stop' (everything)
SUMMARY

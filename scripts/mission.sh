#!/usr/bin/env bash
# mission.sh — STEP 3: arm a mission. ONLY this makes the car move.
#
#   mission trackdrive [laps]     # default 10 laps (KASE trackdrive)
#   mission autocross  [laps]     # default 1 lap, stops itself at the target
#   mission skidpad    [laps]     # default 2 — laps PER CIRCLE (right N, left N)
#   mission acceleration          # straight corridor sprint (acceleration
#                                 # track), brakes when the cones end
#   mission dlc                   # open-corridor demo (dlc_track): default
#                                 # planner tuning, rolls to a stop at the end
#   mission inspection            # 제22조 검차: axle spin + steering sine on
#                                 # jacks; brake test via /inspection/brake_test
#
#   mission go | halt             # AS button stand-in (until the physical
#                                 # toggle is wired): go = ON -> the ECU bridge
#                                 # resets the SLAM map, then enables; halt =
#                                 # OFF -> enable drops at once
#   mission stop                  # EBS — emergency-brake the run
#   mission reset                 # back to standby: clear mission, reset car
#                                 # pose, respawn the planning graph fresh
#   mission status                # one-shot state/lap/AS/DSSI readout
#
# Until this script arms a mission the vehicle CANNOT move: the sim race-car
# plugin (and, on the vehicle, the actuation bridge honoring the same
# /vehicle/set_mission contract) ignores commands outside AS_DRIVING.
#
# How arming works: the planning graph launched by step 2 runs a mission
# PROFILE (launch args: lap targets, skidpad/acceleration mode). If the
# requested mission needs a different profile than the pane currently runs —
# or a mission was already armed, so SLAM/planning state is dirty — the
# planning pane is respawned in-place with the right profile first, then
# /vehicle/set_mission arms the AMI. Standby equals the default-trackdrive
# profile, so the common 'mission trackdrive' arms without a respawn.

set -o pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/fsk-session.sh"

usage() { sed -n '2,24p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

CMD="${1:-}"
ARG="${2:-}"

[ -z "$CMD" ] && { usage; exit 0; }
fsk_require_session
fsk_source_env

PLAN_PANE="$(fsk_getenv FSK_PLAN_PANE)"

require_stack() {
  if [ -z "$PLAN_PANE" ]; then
    echo "mission: step 2 is not up — run 'stack' first." >&2
    exit 1
  fi
}

positive_int() {
  case "$1" in ''|*[!0-9]*|0) return 1 ;; *) return 0 ;; esac
}

svc() {  # svc <service> <type> [yaml]
  # `ros2 service call` can hang forever when its request beats the server's
  # discovery of the client (the response is then never delivered -- seen
  # 2026-08-21 on /graph_slam/reset: the server answered an rclpy client in
  # 2 ms while the CLI sat for minutes). Bound every call and retry once.
  local out
  out="$(timeout 8 ros2 service call "$1" "$2" ${3:+"$3"} 2>&1)"
  if ! echo "$out" | grep -q "response:"; then
    out="$(timeout 8 ros2 service call "$1" "$2" ${3:+"$3"} 2>&1)"
  fi
  echo "$out"
}

topic_once() {  # topic_once <topic> [timeout]
  timeout "${2:-2}" ros2 topic echo --once "$1" 2>/dev/null | sed -n 's/^data: //p' | head -1
}

# Respawn the planning pane with a new profile: Ctrl-C the current launch,
# wait for its nodes to actually leave the graph, relaunch. The pane's zsh
# already sourced the ROS env (step 2's $SRC ran in it), so the bare launch
# line is enough. The special profile "inspection" swaps the WHOLE planning
# graph for the 제22조 inspection node — no planner, no SLAM.
respawn_planning() {
  local profile="$1"
  local cmd wait_node
  if [ "$profile" = "inspection" ]; then
    cmd="ros2 launch hyu_planning_bringup inspection.launch.py use_sim_time:=$(fsk_getenv FSK_USE_SIM_TIME)"
    wait_node="inspection_mission"
  else
    cmd="$(fsk_plan_cmd "$profile")"
    wait_node="planning_hyu_state_machine"
  fi
  echo "mission: reconfiguring planning pane → $profile"
  tmux send-keys -t "$PLAN_PANE" C-c
  local waited=0
  while ros2 node list --no-daemon 2>/dev/null | grep -qE "planning_hyu_state_machine|inspection_mission"; do
    sleep 2
    waited=$((waited + 2))
    if [ $((waited % 10)) -eq 0 ]; then tmux send-keys -t "$PLAN_PANE" C-c; fi
    if [ "$waited" -ge 40 ]; then
      echo "mission: planning pane refuses to die — check pane $PLAN_PANE ('race attach')." >&2
      exit 1
    fi
  done
  sleep 1
  tmux send-keys -t "$PLAN_PANE" "$cmd" C-m
  echo -n "mission: waiting for $wait_node"
  waited=0
  until ros2 node list --no-daemon 2>/dev/null | grep -q "$wait_node"; do
    sleep 2; echo -n "."
    waited=$((waited + 2))
    if [ "$waited" -ge 90 ]; then
      echo; echo "mission: $wait_node did not come up — check pane $PLAN_PANE ('race attach')." >&2
      exit 1
    fi
  done
  echo " up."
}

# Restart the INS pipeline pane. Needed whenever the car is TELEPORTED
# (mission reset): the INS chain integrates straight through a teleport, and
# graph SLAM re-seeded from that stale odometry has been observed 20+ m off
# the real car — which then fakes gate crossings while standing still.
restart_ins() {
  local ins_pane; ins_pane="$(fsk_getenv FSK_INS_PANE)"
  if [ -z "$ins_pane" ]; then
    echo "mission: WARNING — no INS pane recorded; skipping INS restart." >&2
    return 0
  fi
  echo "mission: restarting INS pipeline (a teleport invalidates its integration)."
  tmux send-keys -t "$ins_pane" C-c
  local waited=0
  # Process-based check: `ros2 topic info` answers from the (possibly stale)
  # daemon cache and kept reporting a live publisher long after the nodes died.
  while pgrep -f "ins_pipeline.launch.py" >/dev/null 2>&1; do
    sleep 2
    waited=$((waited + 2))
    if [ $((waited % 10)) -eq 0 ]; then tmux send-keys -t "$ins_pane" C-c; fi
    if [ "$waited" -ge 30 ]; then
      echo "mission: WARNING — INS refuses to die; relaunching anyway (check pane $ins_pane)." >&2
      break
    fi
  done
  sleep 1
  tmux send-keys -t "$ins_pane" "$(fsk_ins_cmd)" C-m
}

warn_track_mismatch() {  # warn_track_mismatch <mission>
  local track; track="$(fsk_getenv FSK_TRACK)"
  [ -z "$track" ] && return 0   # vehicle mode: no track notion
  case "$1" in
    inspection)   ;;  # jack-stand mission, any (or no) track
    skidpad)      case "$track" in skidpad*) ;; *) echo "mission: WARNING — skidpad mission on track '$track'." >&2 ;; esac ;;
    # The acceleration profile fits a STRAIGHT line through the corridor cones
    # (extend_straight_to_horizon) — unsafe on the curved dlc_track; the dlc
    # mission (default curved-planner tuning + open-end stop) drives that one.
    acceleration) case "$track" in acceleration*) ;; *) echo "mission: WARNING — acceleration mission on track '$track'." >&2 ;; esac ;;
    dlc)          case "$track" in dlc*|acceleration*) ;; *) echo "mission: WARNING — dlc mission on track '$track'." >&2 ;; esac ;;
    *)            case "$track" in skidpad*|acceleration*|dlc*) echo "mission: WARNING — $1 mission on track '$track'." >&2 ;; esac ;;
  esac
}

arm() {  # arm <profile> <ami_code> <mission-name> <note>
  local profile="$1" ami="$2" name="$3" note="$4"
  require_stack
  warn_track_mismatch "$name"

  local armed cur
  armed="$(fsk_getenv FSK_ARMED)"
  cur="$(fsk_getenv FSK_PROFILE)"

  if [ "$armed" = "1" ]; then
    # A mission was already armed: the sim state machine only accepts ONE
    # set_mission, and SLAM/planning carry the old run's state. Clear the
    # vehicle state and always start the graph fresh. (Car pose is NOT
    # touched — 'mission reset' does that.)
    echo "mission: previous mission armed — clearing vehicle state and restarting planning fresh."
    svc /vehicle/reset std_srvs/srv/Trigger >/dev/null
    respawn_planning "$profile"
  elif [ "$cur" != "$profile" ]; then
    respawn_planning "$profile"
  else
    echo "mission: planning graph already runs this profile — arming directly."
  fi

  # Start the SLAM map fresh at every arm. The mapping lap should begin from
  # a clean slate exactly as the car starts to move (set_mission below is what
  # releases it), so the previous run's cone map can never bleed into it. This
  # matters most in the "arm directly" branch above, which does NOT respawn the
  # graph; harmless elsewhere (a just-respawned graph is already empty). The
  # service is graph SLAM's private ~/reset -> /graph_slam/reset.
  svc /graph_slam/reset std_srvs/srv/Trigger >/dev/null 2>&1 || \
    echo "mission: note — graph SLAM /graph_slam/reset not answered (map not cleared)." >&2

  local out
  out="$(svc /vehicle/set_mission hyu_msgs/srv/SetCanState "{ami_state: $ami}")"
  if ! echo "$out" | grep -q "success=True"; then
    echo "mission: ARMING FAILED — $out" >&2
    echo "mission: if a mission is stuck, 'mission reset' clears it." >&2
    exit 1
  fi
  fsk_setenv FSK_PROFILE "$profile"
  fsk_setenv FSK_ARMED 1
  echo "mission: $name ARMED. $note"
  echo "  watch → 'race attach'  |  'mission status'  |  EBS → 'mission stop'"
}

case "$CMD" in
  trackdrive|td)
    LAPS="${ARG:-$FSK_DEFAULT_TRACKDRIVE_LAPS}"
    positive_int "$LAPS" || { echo "mission: laps must be a positive integer, got '$ARG'." >&2; exit 1; }
    arm "target_lap_count:=$LAPS" 14 trackdrive \
      "$LAPS laps: lap 1 maps on LOCAL, then GLOBAL raceline; stops after lap $LAPS."
    ;;
  autocross|ax)
    LAPS="${ARG:-1}"
    positive_int "$LAPS" || { echo "mission: laps must be a positive integer, got '$ARG'." >&2; exit 1; }
    arm "target_lap_count:=$LAPS stop_on_target_laps:=true" 13 autocross \
      "$LAPS lap(s), stops at the lap target from any state (stop_on_target_laps)."
    ;;
  skidpad|sp)
    LAPS="${ARG:-2}"
    positive_int "$LAPS" || { echo "mission: laps must be a positive integer, got '$ARG'." >&2; exit 1; }
    arm "skidpad:=true skidpad_right_laps:=$LAPS skidpad_left_laps:=$LAPS finish_on_rest:=true" 12 skidpad \
      "entry → right x$LAPS → left x$LAPS → exit (phase-gated local-only planning)."
    ;;
  acceleration|accel)
    [ -n "$ARG" ] && echo "mission: acceleration takes no lap argument — ignoring '$ARG'." >&2
    arm "acceleration:=true finish_on_rest:=true" 11 acceleration \
      "local-only straight-corridor sprint; brakes when the cones end, then reports Finished."
    ;;
  dlc)
    [ -n "$ARG" ] && echo "mission: dlc takes no lap argument — ignoring '$ARG'." >&2
    # AMI 15 = AUTONOMOUS_DEMO: the dlc_track run is a demo/video mission, not
    # a KASE scoring event. Local-only with the DEFAULT planner/controller
    # tunings (the corridor curves — no straight-line fit); the planner tapers
    # the path speed to zero at the open corridor end (stop_at_path_end), so
    # the car rolls to a stop where the cones run out and reports Finished.
    arm "dlc:=true finish_on_rest:=true" 15 dlc \
      "open-corridor demo (default local planner); rolls to a stop at the corridor end, then reports Finished."
    ;;
  inspection|insp)
    arm "inspection" 16 inspection \
      "제22조: axle slow-spin + steering sine 25-30 s on jack stands. DSB check: 'ros2 service call /inspection/brake_test std_srvs/srv/Trigger'. Afterwards: 'mission reset'."
    ;;
  go|on)
    out="$(svc /vehicle/set_as_button std_srvs/srv/SetBool "{data: true}")"
    echo "$out" | grep -q "success=True" && echo "mission: AS button ON — bridge resets the map, then enables the ECU." \
      || { echo "mission: could not set the AS button — is vehicle_state up? ($out)" >&2; exit 1; }
    ;;
  halt|off)
    out="$(svc /vehicle/set_as_button std_srvs/srv/SetBool "{data: false}")"
    echo "$out" | grep -q "success=True" && echo "mission: AS button OFF — ECU autonomous enable dropped." \
      || { echo "mission: could not clear the AS button — is vehicle_state up? ($out)" >&2; exit 1; }
    ;;
  stop|ebs)
    require_stack
    echo "mission: EBS requested."
    svc /vehicle/ebs std_srvs/srv/Trigger
    ;;
  reset)
    require_stack
    echo "mission: resetting — vehicle state + pose, INS + planning back to standby."
    # Stop planning FIRST so no stale SLAM keeps consuming the dying INS.
    tmux send-keys -t "$PLAN_PANE" C-c
    svc /vehicle/reset std_srvs/srv/Trigger >/dev/null
    svc /vehicle/reset_vehicle_pos std_srvs/srv/Trigger >/dev/null
    restart_ins
    respawn_planning "$FSK_STANDBY_PROFILE"
    fsk_setenv FSK_PROFILE "$FSK_STANDBY_PROFILE"
    fsk_setenv FSK_ARMED 0
    echo "mission: standby. Arm the next run with 'mission <name> [laps]'."
    ;;
  status|st)
    echo "profile:  $(fsk_getenv FSK_PROFILE)  (armed: $(fsk_getenv FSK_ARMED))"
    echo "state:    $(topic_once /planning/state)"
    echo "lap:      $(topic_once /planning/lap_count)"
    echo "AS:       $(topic_once /vehicle/as_state_str)"
    echo "DSSI:     $(topic_once /vehicle/dssi)"
    ;;
  help|-h|--help)
    usage
    ;;
  *)
    echo "mission: unknown mission '$CMD'." >&2
    usage
    exit 1
    ;;
esac

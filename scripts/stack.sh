#!/usr/bin/env bash
# stack.sh — STEP 2: INS + SLAM + planning + control, in STANDBY.
#
#   stack [use_sim_time:=true|false] [extra planning launch args...]
#
# Adds onto the step-1 tmux session ('race <track> sim' or 'fsk'):
#   ② INS pipeline (sim Ellipse-D + SBG bridge / vehicle INS chain)
#   ③ planning graph: graph_slam + global/local planner + state machine +
#     path selector + Pure Pursuit (MAP steering) + DSSI + HUD
#   ④ monitor (path source / state / lap / CTE / DSSI)
#
# STANDBY means: every node is up and inspectable, the controller already
# publishes /vehicle/cmd — but the vehicle ignores commands until a mission
# is armed (sim plugin gates on AS_DRIVING). Nothing moves until step 3:
#   mission trackdrive 10 | autocross | skidpad 2 | acceleration
#
# The planning pane is respawned by mission.sh whenever the chosen mission
# needs different launch args (skidpad/acceleration profiles, lap targets),
# so extra args given here are re-applied on every respawn (kept in
# FSK_STACK_EXTRA).

set -o pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/fsk-session.sh"

fsk_require_built
fsk_require_session

if [ -n "$(fsk_getenv FSK_PLAN_PANE)" ]; then
  echo "stack: already up (planning pane $(fsk_getenv FSK_PLAN_PANE)) — 'race stop' to tear down everything." >&2
  exit 1
fi

ENV_KIND="$(fsk_getenv FSK_ENV)"
UST="$(fsk_getenv FSK_USE_SIM_TIME)"
UST="${UST:-true}"
EXTRA=""
for tok in "$@"; do
  case "$tok" in
    tmpc)
      echo "stack: the TMPC hybrid stack is retired from this flow — the MAP controller drives. Ignoring 'tmpc'." >&2 ;;
    use_sim_time:=*) UST="${tok#use_sim_time:=}" ;;
    *) EXTRA="$EXTRA $tok" ;;
  esac
done
EXTRA="${EXTRA# }"
fsk_setenv FSK_USE_SIM_TIME "$UST"
fsk_setenv FSK_STACK_EXTRA "$EXTRA"

FIRST_PANE=$(tmux list-panes -t "$SESSION" -F '#{pane_id}' | head -1)

# ② INS — sim waits for ground truth (Ellipse-D sim feeds off it); the
# vehicle chain starts immediately and waits on its own driver topics.
P_INS=$(tmux split-window -v -t "$FIRST_PANE" -P -F '#{pane_id}')
if [ "$ENV_KIND" = "sim" ]; then
  tmux send-keys -t "$P_INS" \
    "$SRC echo '[② INS: sim Ellipse-D + SBG bridge (GNSS anchor + HUD)] waiting for ground truth…'; $WAIT_GT; $(fsk_ins_cmd)" C-m
else
  tmux send-keys -t "$P_INS" \
    "$SRC echo '[② INS pipeline (vehicle)]'; $(fsk_ins_cmd)" C-m
fi
fsk_setenv FSK_INS_PANE "$P_INS"

# ③ Planning graph in the STANDBY profile. mission.sh finds this pane via
# FSK_PLAN_PANE and respawns it in-place for mission-specific profiles.
P_PLAN=$(tmux split-window -h -t "$FIRST_PANE" -P -F '#{pane_id}')
if [ "$ENV_KIND" = "sim" ]; then PLAN_WAIT="$WAIT_CAR"; else PLAN_WAIT="$WAIT_CONES"; fi
tmux send-keys -t "$P_PLAN" \
  "$SRC echo '[③ PLANNING: slam+global+local+SM+selector+MAP-PP+DSSI — STANDBY, waiting for mission] waiting for inputs…'; $PLAN_WAIT; $(fsk_plan_cmd "$FSK_STANDBY_PROFILE")" C-m

# ④ Monitor.
P_MON=$(tmux split-window -v -t "$P_PLAN" -P -F '#{pane_id}')
tmux send-keys -t "$P_MON" \
  "$SRC echo '[④ MONITOR] waiting for planning…'; $WAIT_STATE; while true; do printf '\\n== %s ==\\n' \"\$(date +%H:%M:%S)\"; echo -n 'path_source: '; ros2 topic echo --once /planning/path_source 2>/dev/null | grep -o 'data:.*'; echo -n 'state:       '; ros2 topic echo --once /planning/state 2>/dev/null | grep -o 'data:.*'; echo -n 'lap:         '; ros2 topic echo --once /planning/lap_count 2>/dev/null | grep -o 'data:.*'; echo -n 'CTE d(m):    '; ros2 topic echo --once /planning/cte 2>/dev/null | grep -o 'data:.*'; echo -n 'dssi:        '; timeout 1 ros2 topic echo --once /vehicle/dssi 2>/dev/null | grep -o 'data:.*'; echo -n 'skidpad:     '; timeout 1 ros2 topic echo --once /planning/skidpad/phase 2>/dev/null | grep -o 'data:.*'; sleep 3; done" C-m

tmux select-layout -t "$SESSION" tiled
tmux select-pane   -t "$P_MON"

fsk_setenv FSK_PLAN_PANE "$P_PLAN"
fsk_setenv FSK_PROFILE "$FSK_STANDBY_PROFILE"
fsk_setenv FSK_ARMED 0

cat <<EOF
stack: step 2 up (standby, use_sim_time=$UST${EXTRA:+, extra: $EXTRA}).
  The controller is live but the vehicle IGNORES commands until step 3 arms a mission.
  next:  mission trackdrive [laps=$FSK_DEFAULT_TRACKDRIVE_LAPS] | mission autocross | mission skidpad [laps=2] | mission acceleration
  view → 'race attach'   |   stop everything → 'race stop'
EOF

#!/usr/bin/env bash
# lite_regression.sh LABEL LAPS [extra planning launch args...]
#   e.g. lite_regression.sh axle091 3            (current yaml: rear axle 0.91)
#        lite_regression.sh axle0 3 controller_rear_axle_from_base_m:=0.0
#   Output dir: $FSK_REG_OUT/<label> (default ~/fsk/runs/lite/<label>): bag/, frozen_map.csv,
#   graph_slam.log, pure_pursuit.log, run.log. Analyse with analyze_run.py <dir>.
# Headless lite-sim run through the real 3-step flow (sim -> stack -> mission),
# recording a bag + the frozen map + the graph_slam log into the scratchpad.
set +u
LABEL=$1; LAPS=$2; shift 2; EXTRA="$*"
WS=/home/race/fsk; S=$WS/src/scripts
OUT=${FSK_REG_OUT:-$WS/runs/lite}/$LABEL
rm -rf "$OUT"; mkdir -p "$OUT"
# shellcheck disable=SC1090
source "$S/fsk-session.sh"
fsk_source_env
log(){ echo "[$(date +%T)] $*" | tee -a "$OUT/run.log"; }
tmux kill-session -t "$SESSION" 2>/dev/null || true
sleep 2
log "step1: sim ${FSK_REG_TRACK:-small_track} lite bg norviz ${FSK_REG_SIM_ARGS:-}"
# shellcheck disable=SC2086
"$S/sim.sh" ${FSK_REG_TRACK:-small_track} lite bg norviz ${FSK_REG_SIM_ARGS:-} >>"$OUT/run.log" 2>&1 || { log "sim failed"; exit 1; }
for _ in $(seq 1 60); do ros2 node list --no-daemon 2>/dev/null | grep -q race_car && break; sleep 2; done
log "race_car up; step2: stack $EXTRA"
# shellcheck disable=SC2086
"$S/stack.sh" $EXTRA >>"$OUT/run.log" 2>&1 || { log "stack failed"; exit 1; }
for _ in $(seq 1 90); do ros2 topic list --no-daemon 2>/dev/null | grep -q /planning/state && break; sleep 2; done
log "planning up; settling"
sleep 12
# CPU sampler: %CPU of the heavy nodes every 0.5 s (utime+stime jiffies deltas).
(
  echo "t,name,cpu_pct" > "$OUT/cpu.csv"
  declare -A prev
  HZ=$(getconf CLK_TCK)
  while :; do
    ts=$(date +%s.%N)
    for name in graph_slam_node lite_sim hyu_pure_pursuit planner_node hyu_local_planner sbg_raw_ekf; do
      pid=$(pgrep -f "$name" | head -1); [ -z "$pid" ] && continue
      stat=$(cat /proc/$pid/stat 2>/dev/null) || continue
      set -- $stat; ut=${14}; st=${15}; tot=$((ut+st))
      if [ -n "${prev[$name]:-}" ]; then
        d=$((tot - prev[$name])); echo "$ts,$name,$(echo "scale=1; $d*100/($HZ*0.5)" | bc)" >> "$OUT/cpu.csv"
      fi
      prev[$name]=$tot
    done
    sleep 0.5
  done
) &
CPUPID=$!
log "bag record start"
ros2 bag record -o "$OUT/bag" /ground_truth/state /localization/ego_odom /planning/cte /planning/cte_rmse \
  /vehicle/cmd /localization/status /planning/lap_count /planning/state /localization/cone_map \
  /planning/path_source /planning/path /planning/selected_path_valid /planning/local_path_valid \
  /planning/local_path_reason >"$OUT/bag.log" 2>&1 &
BAGPID=$!
sleep 5
log "step3: mission trackdrive $LAPS"
"$S/mission.sh" trackdrive "$LAPS" >>"$OUT/run.log" 2>&1 || log "mission returned non-zero"
T0=$(date +%s)
while :; do
  lap=$(timeout 3 ros2 topic echo --once /planning/lap_count 2>/dev/null | sed -n 's/^data: //p' | head -1)
  st=$(timeout 3 ros2 topic echo --once /planning/state 2>/dev/null | sed -n 's/^data: //p' | head -1)
  el=$(( $(date +%s) - T0 ))
  log "t=${el}s lap=${lap:-?} state=${st:-?}"
  if [ -n "${lap:-}" ] && [ "$lap" -ge "$LAPS" ] 2>/dev/null; then sleep 8; break; fi
  case "${st:-}" in *FINISH*|*Finish*|*finish*) sleep 5; break;; esac
  # A mission can report FINISHED with the lap counter short of the target
  # (e.g. a lap-late map freeze): sustained STOP means it is over either way.
  case "${st:-}" in *STOP*) STOPN=$((${STOPN:-0}+1));; *) STOPN=0;; esac
  [ "${STOPN:-0}" -ge 4 ] && { log "state STOP sustained - ending"; sleep 3; break; }
  [ "$el" -ge 330 ] && { log "timeout"; break; }
  sleep 5
done
kill -INT "$BAGPID" 2>/dev/null; wait "$BAGPID" 2>/dev/null
kill "$CPUPID" 2>/dev/null
MAPCSV=$(ls -t "$WS"/src/localization/map/map_*.csv 2>/dev/null | head -1)
[ -n "$MAPCSV" ] && cp "$MAPCSV" "$OUT/frozen_map.csv" && log "frozen map: $MAPCSV"
GSLOG=$(ls -t ~/.ros/log/graph_slam*.log 2>/dev/null | head -1)
[ -n "$GSLOG" ] && cp "$GSLOG" "$OUT/graph_slam.log"
PPLOG=$(ls -t ~/.ros/log/hyu_pure_pursuit*.log 2>/dev/null | head -1)
[ -n "$PPLOG" ] && cp "$PPLOG" "$OUT/pure_pursuit.log"
log "teardown"
"$S/sim.sh" stop >>"$OUT/run.log" 2>&1
sleep 3
pkill -f lite_sim_node 2>/dev/null; pkill -f local_global_planning 2>/dev/null
log "done"

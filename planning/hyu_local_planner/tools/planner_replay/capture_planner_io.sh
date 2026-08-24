#!/usr/bin/env bash
# capture.sh <bag_dir> <out_name> -- bagplay (sensors+perception+INS live) + graph_slam + local planner, record planner I/O.
set +u
BAG="$1"; OUT="$2"
SCR="${PLANNER_REPLAY_OUT:-$HOME/planner_replay}"; mkdir -p "$SCR"
LOG=$SCR/capture_${OUT}.log
exec > "$LOG" 2>&1
cd /home/race/fsk
if ip link show lo 2>/dev/null | grep -q MULTICAST; then RLO=1; else RLO=0; fi
export PATH="$(echo "$PATH" | tr ":" "\n" | grep -vE "conda|/\.venv" | paste -sd:)"
source /opt/ros/humble/setup.bash; source /home/race/fsk/install/setup.bash
export EUFS_MASTER=/home/race/fsk ROS_LOCALHOST_ONLY=$RLO
echo "[capture] $(date) bag=$BAG out=$OUT RLO=$RLO"
bash src/scripts/bagplay.sh "$BAG" bg < /dev/null || { echo "[capture] bagplay failed"; exit 1; }
sleep 8
ros2 launch hyu_localization graph_slam.launch.py use_sim_time:=true gui:=false ate_monitor:=false > $SCR/slam_${OUT}.log 2>&1 &
SLAM_PID=$!
sleep 3
ros2 launch hyu_local_planner hyu_local_planner.launch.py use_sim_time:=true > $SCR/planner_${OUT}.log 2>&1 &
PLAN_PID=$!
sleep 2
rm -rf "$SCR/bag_$OUT"
ros2 bag record -o "$SCR/bag_$OUT" /localization/cone_map /localization/ego_odom /perception/cones /localization/status /planning/local_waypoints/path /planning/local_path_reason /planning/local_path_valid /clock /localization/ins_odom /localization/debug/markers /perception/debug/cones_viz > $SCR/record_${OUT}.log 2>&1 &
REC_PID=$!
echo "[capture] pids slam=$SLAM_PID plan=$PLAN_PID rec=$REC_PID"
# wait for the bag player to appear, then to finish (single pass with ins:=real)
# The bag player runs in pane 0 of the fsk_race tmux session; it is done when that pane is back at a shell.
for i in $(seq 1 60); do [ "$(tmux display-message -p -t fsk_race:0.0 '#{pane_current_command}' 2>/dev/null)" != "bash" ] && [ "$(tmux display-message -p -t fsk_race:0.0 '#{pane_current_command}' 2>/dev/null)" != "zsh" ] && break; sleep 2; done
echo "[capture] $(date) bag play running ($(tmux display-message -p -t fsk_race:0.0 '#{pane_current_command}'))"
while true; do c="$(tmux display-message -p -t fsk_race:0.0 '#{pane_current_command}' 2>/dev/null)"; [ "$c" = "bash" ] || [ "$c" = "zsh" ] || [ -z "$c" ] && break; sleep 5; done
echo "[capture] $(date) bag play finished; stopping"
sleep 3
kill -INT $REC_PID 2>/dev/null; sleep 4
kill -INT $PLAN_PID $SLAM_PID 2>/dev/null; sleep 4
kill -TERM $REC_PID $PLAN_PID $SLAM_PID 2>/dev/null
bash src/scripts/bagplay.sh stop
sleep 2
pkill -f "hyu_local_planner_node" ; pkill -f "graph_slam" ; pkill -f "ros2 bag record"
echo "[capture] $(date) done"

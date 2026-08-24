#!/usr/bin/env bash
# fsk-session.sh — shared tmux-session plumbing for the 3-step run flow.
#
# Sourced by sim.sh (step 1, sim), fsk.sh (step 1, vehicle), stack.sh
# (step 2), mission.sh (step 3) and race.sh (vehicle, all three). All drive ONE tmux session
# ($SESSION); this file owns everything they must agree on: the session
# name, the environment line every pane starts with, the wait snippets,
# and the tmux-environment variables that carry state between the steps:
#
#   FSK_ENV       sim | litesim | vehicle  (who ran step 1: Gazebo/eufs, the
#                                          Gazebo-free hyu_lite_sim, the car/bag)
#   FSK_TRACK     track name             (sim/litesim only; mission sanity check)
#   FSK_DATUM_LAT/LON, FSK_ANT_X/Y  litesim: what sbg_raw_ekf must be given so its
#                                  frame coincides with the simulator's world
#   FSK_USE_SIM_TIME true|false          (single clock-domain switch)
#   FSK_PLAN_PANE tmux pane id of the planning launch (step 2 sets it)
#   FSK_STACK_EXTRA user extra planning args from step 2
#   FSK_PROFILE   mission launch-arg profile currently running in that pane
#   FSK_ARMED     1 once a mission has been armed via /vehicle/set_mission

SESSION="fsk_race"

FSK_SESSION_LIB_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(cd "$FSK_SESSION_LIB_DIR/.." && pwd)"
DEFAULT_EUFS_MASTER="$(cd "$SRC_DIR/.." && pwd)"
if [ ! -f "$DEFAULT_EUFS_MASTER/install/setup.bash" ] && [ -f "$SRC_DIR/install/setup.bash" ]; then
  DEFAULT_EUFS_MASTER="$SRC_DIR"
fi
EUFS_MASTER="${EUFS_MASTER:-$DEFAULT_EUFS_MASTER}"
if [ ! -f "$EUFS_MASTER/install/setup.bash" ] && [ -f "$DEFAULT_EUFS_MASTER/install/setup.bash" ]; then
  EUFS_MASTER="$DEFAULT_EUFS_MASTER"
fi
# Which setup file the PANES source depends on the shell tmux actually starts,
# not on what we assume. Sourcing setup.zsh under bash floods every pane with
# "bad substitution" and leaves the workspace only half-sourced -- nodes then
# fail far from the cause. Ask tmux; fall back to $SHELL, then zsh.
_PANE_SHELL="$(tmux show-options -gv default-shell 2>/dev/null)"
_PANE_SHELL="${_PANE_SHELL:-${SHELL:-/bin/zsh}}"
case "$_PANE_SHELL" in
  *bash) PANE_EXT=bash ;;
  *)     PANE_EXT=zsh  ;;
esac
WS_SETUP="$EUFS_MASTER/install/setup.$PANE_EXT"
WS_SETUP_BASH="$EUFS_MASTER/install/setup.bash" # these scripts run bash
ROS_SETUP="/opt/ros/humble/setup.$PANE_EXT"

# ROS's own tools resolve their interpreter through `#!/usr/bin/env python3`,
# so whatever python3 sits earliest on PATH BECOMES ROS's python. A conda env
# on PATH replaces python3.10 with an interpreter that has none of ROS's deps,
# and the failure is nowhere near the cause: spawn_entity.py dies on lxml, no
# car spawns, and every pane waits forever. Strip it in every pane.
SANE_PATH='export PATH="$(echo "$PATH" | tr ":" "\n" | grep -vE "conda|/\.venv" | paste -sd:)";'
# ROS_LOCALHOST_ONLY=1 (loopback-only) DDS discovery needs the MULTICAST flag
# on `lo`. Detect HERE — not via an inherited env var, which a fresh tmux pane
# shell can drop — so the value baked into every pane is right for this box.
if ip link show lo 2>/dev/null | grep -q "MULTICAST"; then RLO=1; else RLO=0; fi
SRC="$SANE_PATH source $ROS_SETUP; source $WS_SETUP; export EUFS_MASTER=$EUFS_MASTER ROS_LOCALHOST_ONLY=$RLO;"

# --no-daemon everywhere: the ros2 daemon can serve a stale graph cache in
# which race_car never appears, wedging every pane at "waiting…" forever.
WAIT_CAR="until ros2 node list --no-daemon 2>/dev/null | grep -q race_car; do sleep 2; done"
WAIT_GT="until ros2 topic list --no-daemon 2>/dev/null | grep -q /ground_truth/state; do sleep 2; done"
WAIT_STATE="until ros2 topic list --no-daemon 2>/dev/null | grep -q /planning/state; do sleep 2; done"
WAIT_CONES="until ros2 topic list --no-daemon 2>/dev/null | grep -q /perception/cones; do sleep 2; done"

# Standby = the profile step 2 launches before any mission is chosen. It IS
# the default-trackdrive profile so that a plain 'mission trackdrive' arms
# instantly instead of respawning an identical planning graph.
FSK_DEFAULT_TRACKDRIVE_LAPS=10
FSK_STANDBY_PROFILE="target_lap_count:=$FSK_DEFAULT_TRACKDRIVE_LAPS"

fsk_require_built() {
  if [ ! -f "$WS_SETUP" ]; then
    echo "fsk: workspace not built ($WS_SETUP missing). Run 'fsb' first." >&2
    exit 1
  fi
}

fsk_session_exists() { tmux has-session -t "$SESSION" 2>/dev/null; }

fsk_require_session() {
  if ! fsk_session_exists; then
    echo "fsk: no '$SESSION' session — run step 1 first: 'sim <track>' (sim; 'sim <track> lite' = no Gazebo), 'fsk' (vehicle), or 'race <mission>' for the whole vehicle pipeline." >&2
    exit 1
  fi
}

# Source ROS into THIS bash process (service calls, topic queries). Panes get
# $SRC instead; keep the two aligned.
fsk_source_env() {
  export PATH="$(echo "$PATH" | tr ":" "\n" | grep -vE "conda|/\.venv" | paste -sd:)"
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1090
  [ -f "$WS_SETUP_BASH" ] && source "$WS_SETUP_BASH"
  export EUFS_MASTER ROS_LOCALHOST_ONLY=$RLO
}

fsk_getenv() {  # fsk_getenv NAME -> value (empty if unset)
  tmux show-environment -t "$SESSION" "$1" 2>/dev/null | sed -n "s/^$1=//p"
}

fsk_setenv() {  # fsk_setenv NAME VALUE
  tmux set-environment -t "$SESSION" "$1" "$2"
}

# The planning launch line for a given mission profile. stack.sh (standby) and
# mission.sh (respawn) MUST build the same command or the profile comparison
# in mission.sh lies; that is why it lives here.
#   $1 = profile args ("target_lap_count:=10", "skidpad:=true …", …)
fsk_plan_cmd() {
  local profile="$1"
  local env use_sim_time extra gates=""
  env="$(fsk_getenv FSK_ENV)"
  use_sim_time="$(fsk_getenv FSK_USE_SIM_TIME)"
  extra="$(fsk_getenv FSK_STACK_EXTRA)"
  if [ "$env" = "sim" ]; then
    # Sim-load-tolerant input gates (sim time stalls under load spikes); the
    # vehicle keeps the tighter yaml defaults.
    gates=" local_max_stamp_skew_sec:=2.0 local_max_input_age_sec:=3.0 local_max_start_distance_m:=8.0"
    # EUFS publishes base_footprint ~at the CoG, not 0.91 m behind the rear
    # axle like the car/lite sim, and its actuator locks at 0.52 rad (the car:
    # 0.335, wheel +-90 deg): keep Pure Pursuit on the pose as-is and the
    # EUFS lock instead of vehicle_mount.yaml's car geometry.
    gates="$gates controller_rear_axle_from_base_m:=0.0 controller_max_steering_rad:=0.52"
  fi
  echo "ros2 launch hyu_planning_bringup local_global_planning.launch.py use_sim_time:=${use_sim_time:-true} graph_slam_ate_monitor:=true${gates}${SLAM_MOTION_TOPIC:+ car_state_topic:=$SLAM_MOTION_TOPIC}${GRAPH_SLAM_PARAMS_FILE:+ graph_slam_params_file:=$GRAPH_SLAM_PARAMS_FILE}${extra:+ $extra} $profile"
}

# The INS pipeline launch line. stack.sh (bringup) and mission.sh ('mission
# reset' restarts INS after teleporting the car: the INS chain integrates
# straight through a pose teleport, and SLAM re-seeded from that stale odom
# put the ego estimate 20+ m off the real car — fake gate crossings included)
# must agree on it, so it lives here.
fsk_ins_cmd() {
  local use_sim_time
  use_sim_time="$(fsk_getenv FSK_USE_SIM_TIME)"
  # sim_ins follows the ENVIRONMENT, not the clock. Bag replay runs on
  # use_sim_time to share the bag's clock domain, but it is still a real INS
  # chain -- and eufs_sensors (the simulated Ellipse-D) does not exist on the
  # vehicle compute, so keying off the clock would kill the whole pipeline.
  # odometry conditioning (sbg_raw_ekf) belongs to whoever owns the sensors.
  # On the car and on a bag, step 1's sensor bringup runs it -- perception's
  # LiDAR deskew reads /localization/ins_odom and would otherwise sit
  # uncorrected for the whole of step 1. The simulator has no sensor bringup,
  # so there it stays here alongside sim_ellipse_d.
  # litesim (hyu_lite_sim, no Gazebo): the simulator emits the raw SBG topics
  # itself, so only sbg_raw_ekf is needed here -- and it IS started here
  # rather than in step 1 so that 'mission reset' (teleport) restarts it. It
  # gets the simulator's datum and antenna position so its frame is the
  # simulator's world frame (ground truth == odom).
  local sim_ins=false odometry=false extra=""
  case "$(fsk_getenv FSK_ENV)" in
    sim) sim_ins=true; odometry=true ;;
    litesim)
      odometry=true; use_sim_time=false
      local lat lon ax ay
      lat="$(fsk_getenv FSK_DATUM_LAT)"; lon="$(fsk_getenv FSK_DATUM_LON)"
      ax="$(fsk_getenv FSK_ANT_X)"; ay="$(fsk_getenv FSK_ANT_Y)"
      extra="${lat:+ datum_latitude:=$lat}${lon:+ datum_longitude:=$lon}${ax:+ antenna_offset_x:=$ax}${ay:+ antenna_offset_y:=$ay} allow_gnss_denied_init:=true" ;;
  esac
  echo "ros2 launch hyu_localization ins_pipeline.launch.py slam:=false sim_ins:=$sim_ins odometry:=$odometry use_sim_time:=${use_sim_time:-true}${extra}${INS_MODE_SCHED:+ mode_schedule:=$INS_MODE_SCHED}${INS_CORR_SCHED:+ correction_schedule:=$INS_CORR_SCHED}"
}

# Attach if stdout is a real terminal, else leave the session detached
# (nohup/CI: tmux would die on "open terminal failed: not a terminal").
fsk_attach_or_report() {
  if [ "${1:-attach}" = "bg" ]; then
    echo "fsk: running detached — 'race attach' to view."
    return 0
  fi
  if [ -t 1 ]; then
    exec tmux attach -t "$SESSION"
  fi
  echo "fsk: headless (no TTY) — session '$SESSION' running detached; 'race attach' to view."
}

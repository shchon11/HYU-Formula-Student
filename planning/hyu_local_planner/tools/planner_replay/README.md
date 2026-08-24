# planner_replay — offline replay of the slam_map local planner over a bag

Reproduces `hyu_local_planner_node` (slam_map source mode, trackdrive yaml
values) frame by frame from a bag that carries `/localization/cone_map`,
`/localization/ego_odom`, `/perception/cones` (and `/localization/status`),
without ROS nodes: the same `slamConeSet` → `extendConeSetWithLiveCones` →
`planWithLiveExtension` pipeline, one JSON line per odometry frame (ego pose,
validity/kind/reason, waypoints in the ego frame, optional cone dump).

Standalone CMake project (not part of the colcon package) — it links the
planner sources directly so it always reflects the working tree:

```bash
source /opt/ros/humble/setup.bash && source ~/fsk/install/setup.bash
cmake -S tools/planner_replay -B /tmp/planner_replay -DCMAKE_BUILD_TYPE=Release && make -C /tmp/planner_replay
/tmp/planner_replay/planner_replay <bag_dir> out.jsonl --reconcile --dump-cones   # current node behaviour
/tmp/planner_replay/planner_replay <bag_dir> old.jsonl --unk-range 1e9            # pre-2026-08-22 behaviour (no reconcile, unlimited ego split)
/tmp/planner_replay/planner_replay <bag_dir> /dev/null --reconcile --explain 300.0 # dump classified cones + midpoint chain at t=300 s
python3 tools/planner_replay/analyze_paths.py out.jsonl old.jsonl   # kinks (>80 deg within 1 m) / reversals (>120 deg within 2 m), episodes
python3 tools/planner_replay/plot_frame.py out.jsonl 253.0 300.0    # PNG of cones + path at those times (needs --dump-cones)
```

Options: `--no-live` (map only), `--as-unknown`, `--merge R`, `--max-age S`,
`--unk-range R` (unknown_geom_max_range_m), `--max-dev D` / `--max-turn T`
(live-extension reconciliation), `--reconcile` (use planWithLiveExtension,
i.e. what the node does), `--dump-cones`, `--explain T`.

`capture_planner_io.sh <sensor_bag_dir> <name>` records such a bag from the
CURRENT perception + graph SLAM + local planner over a replayed sensor bag
(`bagplay` + `graph_slam.launch.py` + `hyu_local_planner.launch.py` +
`ros2 bag record`) into `$PLANNER_REPLAY_OUT/bag_<name>`; see the 2026-08-22
investigation of the frontier hooks (0801 bags) for how it was used.

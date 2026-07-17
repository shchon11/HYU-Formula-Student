# Planning Bringup

`local_global_planning.launch.py` composes the local planner, graph-SLAM-aware
global planner, Frenet conversion, state machine, selector, and optional Pure
Pursuit controller. It is the integrated simulator entry point; the package
launches remain available for standalone use.

## Run

```bash
cd /home/shchon11/fsk
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch hyu_planning_bringup local_global_planning.launch.py
```

GLOBAL 구간에서 Formula TMPC를 사용하고 LOCAL 구간에서는 Pure Pursuit를
유지하려면 hybrid launch를 사용합니다.

```zsh
ros2 launch hyu_planning_bringup tmpc_trackdrive.launch.py
```

이 구성에서는 Pure Pursuit가 `/cmd/pure_pursuit`, TMPC bridge가 `/cmd/tmpc`를
발행하며 `hyu_cmd_selector`만 최종 `/cmd`를 발행합니다. GLOBAL 진입 후 TMPC
명령과 validity가 0.1초 연속 정상일 때 takeover하고, takeover 이후 fault가
나면 Pure Pursuit로 폴백한 뒤 TMPC가 다시 0.1초 연속 정상이어야 재인계합니다.
STOP에서는 경로를 따라 제동하는 Pure Pursuit 명령을 우선 포워딩하고, 그것도
불가할 때만 직진 안전 제동을 냅니다.

기존 `local_global_planning.launch.py`나 별도 Pure Pursuit 노드를 먼저 종료한 뒤
실행해야 합니다. 두 launch를 동시에 실행하면 기존 Pure Pursuit가 `/cmd`에 남아
최종 명령 publisher가 중복됩니다. 실행 후 아래 결과가 `Publisher count: 1`,
`Node name: hyu_cmd_selector`인지 확인합니다.

```zsh
ros2 topic info /cmd --verbose
ros2 topic echo /tmpc/cmd_selector/status
```

TMPC bridge는 `drive.acceleration`을 사용하므로 EUFS 차량은
`commandMode:=acceleration`으로 실행해야 합니다.

The launch defaults to `use_sim_time:=true`. Synthetic callers should set
`use_sim_time:=false`. Graph SLAM is enabled by default, with its GUI disabled
and ATE monitor enabled. Set `start_graph_slam:=false` when an external localization
stack already owns the graph-SLAM outputs.

## Ownership

| Topic | Sole default writer |
| --- | --- |
| `/global_waypoints` | Selected `planner_source` writer |
| `/planning/global_path_valid` | Selected `planner_source` writer |
| `/planning/global_path_waypoints` | `wpnt_publisher` |
| `/path_waypoints` | `hyu_path_selector_node` |
| `/planning/selected_path_valid` | `hyu_path_selector_node` |
| `/cmd` | Standard launch: `hyu_pure_pursuit_node`; TMPC launch: `hyu_cmd_selector` |

`planner_source` is either `slam` or `csv`, so the two global writers are never
started together. The global rolling window is remapped to
`/planning/global_path_waypoints`; `/path_waypoints` is reserved for the
selector's chosen local or global candidate. This launch never publishes
`/cones`.

## Key Arguments

| Argument | Default | Purpose |
| --- | --- | --- |
| `planner_source` | `slam` | Select the exclusive global waypoint writer. |
| `local_source_mode` | `slam_map` | Build local paths from the latched SLAM cone map and ego pose. `live_cones` is an explicit diagnostic override. |
| `enable_controller` | `true` | Start the Pure Pursuit controller. |
| `controller_cmd_topic` | `/cmd` | Pure Pursuit command output. Hybrid TMPC launch sets `/cmd/pure_pursuit`. |
| `cmd_topic` | `/cmd` | Command topic monitored by the HUD. |
| `start_graph_slam` | `true` | Start graph SLAM with the integrated graph. |
| `graph_slam_localization_mode` | `false` | Enable saved-map localization. |
| `graph_slam_load_map_path` | empty | Saved map used during localization. |
| `use_sim_time` | `true` | Forward the requested clock setting to every node. |

The launch exposes topic arguments for every graph edge, including graph-SLAM
inputs and outputs, global and local candidates, state outputs, selector
outputs, stop-zone inputs, and controller input and command endpoints. It also
exposes a parameter-file argument for each composed package.

## Handoff QA

The approved headless physical handoff driver is
`.omo/evidence/task-9-local-planning-handoff/simulator_drive.py`. It starts the
simulator and this launch in separate owned process groups, requests mission
`ami_state=14` and requires `as_state=2`, then observes the vehicle on
`/ground_truth/state` only for displacement. It does not publish a path or
control command.

The `full-handoff` scenario requires valid `LOCAL` selection and at least 10 m
of motion while Graph SLAM reports `mapping` or `mapping_converged`, calls
`/graph_slam/load_map`, and requires `localization` plus `GLOBAL_FULL` and a
further 10 m. The `global-to-local-and-brake` scenario calls
`/graph_slam/start_mapping`, verifies the `GLOBAL` to `LOCAL` fallback, then
signals exactly one owned `/hyu_local_planner_node`; stale local validity must make
the selector invalid and the controller must publish speed `0` with
acceleration `-5` within one second.

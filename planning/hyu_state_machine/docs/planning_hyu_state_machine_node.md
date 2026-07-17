# planning_hyu_state_machine_node

## 목적

`planning_hyu_state_machine_node`는 Formula Student Korea 자율주행 planning 흐름에서 현재
planning 상태를 결정하고, `hyu_path_selector_node`가 사용할 `path_source`를 publish한다.
이 노드는 path를 생성하지 않으며, local/global 후보의 선택과 최종
`/path_waypoints` publish는 selector가 소유한다. Pure Pursuit controller는
selector의 유효성 heartbeat와 선택 path만 소비한다.

## 동작 원리

노드는 timer callback에서 주기적으로 다음 순서로 동작한다.

1. 입력 topic callback에서 저장된 최신 상태를 확인한다.
2. `updateLapCount()`를 호출한다.
3. 현재 state에 따라 전이 조건을 검사한다.
4. `/planning/state`, `/planning/path_source`, `/planning/lap_count`, `/planning/stop_request`, `/planning/debug`를 publish한다.

현재 state는 세 가지이다.

```cpp
enum class PlanningState
{
  LOCAL,
  GLOBAL,
  STOP
};
```

## SLAM/global path contract

`planning_hyu_state_machine_node`는 global waypoint writer가 아니라 consumer이다.
`/global_waypoints`와 `/planning/global_path_valid`는 launch 안에서 정확히
하나의 writer만 가져야 한다. SLAM integration에서는
`hyu_global_planner/slam_hyu_global_planner.launch.py planner_source:=slam`이
`planner_node`를 writer로 선택하고, CSV replay/debug에서는
`planner_source:=csv`가 기존 CSV publisher를 선택한다.

`/graph_slam/status`는 latched lifecycle state로 취급한다. 값이
`localization`일 때만 GLOBAL을 허용하고, status message age만으로는
demote하지 않는다. Planner liveness는 `/planning/global_path_valid`에서
판단한다.

`/planning/global_path_valid`는 reliable volatile `std_msgs/msg/Bool`
heartbeat이다. latched true 상태로 쓰지 않는다. `false` 또는
`global_path_valid_timeout_sec` 초과 timeout이면 state machine은 accepted
global waypoint snapshot을 버리고 GLOBAL에서 LOCAL로 demote한다. 복구 시
fresh true heartbeat만으로는 충분하지 않으며, invalidation 이후에 새
non-empty `/global_waypoints` snapshot이 들어와야 한다.

SLAM `planner_node` writer의 기본 동작은 후속 refresh가 실패하면 마지막 valid
global waypoint snapshot을 유지하는 것이다. 따라서 한 번 accepted global path가
만들어진 뒤에는 후속 `/localization/cone_map` geometry jitter가 새 invalid
heartbeat로 이어지지 않는다. 이후 refresh가 다시 성공하면 새 snapshot을
publish하고 consumer가 그 path로 전환한다. 이 문서의 invalidation 규칙은
selected writer가 실제로 `false`를 발행하거나 heartbeat가 stale해진 경우에
적용된다.

Wave 1에서는 Graph SLAM map을 기존 `ConeArrayWithCovariance`로 소비한다.
landmark ID/version을 담는 planner-friendly `SlamConeMap.msg`는 호환 schema
phase로 defer되어 있다.

## Subscribe Topic

| Topic | Type | 설명 |
| --- | --- | --- |
| `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | Frenet odometry. `pose.pose.position.x`를 `current_s`, `pose.pose.position.y`를 `current_d`로 저장한다. |
| `/global_waypoints` | `eufs_msgs/msg/WaypointArrayStamped` | reliable transient-local QoS로 받는 latched global waypoint snapshot. non-empty snapshot만 accept한다. |
| `/graph_slam/status` | `std_msgs/msg/String` | reliable transient-local QoS로 받는 Graph SLAM lifecycle state. 최신 latched 값이 `localization`일 때만 global path를 사용할 수 있다. status message age만으로 demote하지 않는다. |
| `/planning/global_path_valid` | `std_msgs/msg/Bool` | reliable volatile QoS로 받는 global path validity heartbeat. `false` 또는 timeout이면 기존 waypoint snapshot을 invalidation하고 새 snapshot을 기다린다. |
| `/planning/local_path_valid` | `std_msgs/msg/Bool` | reliable volatile QoS로 받는 local planner validity heartbeat. 값, 수신 여부, 수신 시각, freshness를 debug에 기록하며 state/STOP 전이에는 사용하지 않는다. |
| `/planning/global_handoff_ready` | `std_msgs/msg/Bool` | reliable volatile QoS로 받는 selector continuity heartbeat. fresh true가 연속 dwell을 만족할 때만 LOCAL에서 GLOBAL로 진입한다. |
| `/cones` | `eufs_msgs/msg/ConeArrayWithCovariance` | `base_footprint` 기준 local perception cone observation. 색상별 cone 개수와 freshness를 저장한다. |
| `/stop_zone_s_start` | `std_msgs/msg/Float64` | stop zone 시작 지점의 global path projection `s` |
| `/stop_zone_s_end` | `std_msgs/msg/Float64` | stop zone 끝 지점의 global path projection `s` |
| `/stop_zone_valid` | `std_msgs/msg/Bool` | stop zone detector 결과 유효 여부 |

## Publish Topic

| Topic | Type | 값 |
| --- | --- | --- |
| `/planning/state` | `std_msgs/msg/String` | `LOCAL`, `GLOBAL`, `STOP` |
| `/planning/path_source` | `std_msgs/msg/String` | `LOCAL`, `GLOBAL_FULL`, `GLOBAL_FINAL_STOP`, `STOP` |
| `/planning/lap_count` | `std_msgs/msg/Int32` | 현재 lap count |
| `/planning/stop_request` | `std_msgs/msg/Bool` | STOP 상태이면 `true` |
| `/planning/debug` | `std_msgs/msg/String` | state, path_source, lap_count, current_s, current_d, freshness 등 |

## 주요 파라미터

파라미터 파일 위치:

```bash
planning/hyu_state_machine/config/planning_hyu_state_machine.yaml
```

| Parameter | Default | 설명 |
| --- | --- | --- |
| `frenet_odom_topic` | `/car_state/frenet/odom` | Frenet odometry 입력 topic |
| `global_waypoints_topic` | `/global_waypoints` | latched global waypoint 입력 topic |
| `graph_slam_status_topic` | `/graph_slam/status` | Graph SLAM lifecycle status 입력 topic |
| `global_path_valid_topic` | `/planning/global_path_valid` | global path validity heartbeat 입력 topic |
| `local_path_valid_topic` | `/planning/local_path_valid` | local path validity heartbeat 입력 topic |
| `global_handoff_ready_topic` | `/planning/global_handoff_ready` | local/global continuity handoff heartbeat 입력 topic |
| `cone_map_topic` | `/cones` | local perception cone 입력 topic. 기존 파라미터 이름은 호환을 위해 유지한다. |
| `stop_zone_s_start_topic` | `/stop_zone_s_start` | stop zone 시작 `s` 입력 topic |
| `stop_zone_s_end_topic` | `/stop_zone_s_end` | stop zone 끝 `s` 입력 topic |
| `stop_zone_valid_topic` | `/stop_zone_valid` | stop zone 유효 여부 입력 topic |
| `target_lap_count` | `4` | STOP 후보가 되는 목표 lap count |
| `initial_lap_count` | `0` | 시작 lap count |
| `final_lap_start_count` | `3` | final stop path source로 넘어가는 lap count |
| `frenet_odom_timeout_sec` | `0.5` | Frenet odometry freshness timeout |
| `global_path_valid_timeout_sec` | `0.5` | `/planning/global_path_valid` true heartbeat freshness timeout |
| `global_handoff_timeout_sec` | `0.5` | `/planning/global_handoff_ready` heartbeat freshness timeout |
| `global_entry_dwell_sec` | `0.5` | GLOBAL 진입 전 fresh true handoff가 연속 유지되어야 하는 시간 |
| `cone_map_timeout_sec` | `1.0` | Cone map freshness timeout |
| `stop_zone_timeout_sec` | `1.0` | stop zone detector freshness timeout |
| `lap_path_closure_tolerance_m` | `1.0` | lap tracking에서 closed global path로 인정하는 최대 first/last XY 거리 |
| `lap_closing_duplicate_tolerance_m` | `0.05` | closing duplicate 인코딩으로 판단해 `last.s_m`을 그대로 length로 쓰는 first/last XY 거리 |
| `final_path_end_threshold` | `2.0` | final path end 판단 거리 threshold |
| `stop_zone_s_margin` | `0.0` | stop zone `s` 범위 판정 margin |
| `max_abs_d_for_global` | `2.0` | GLOBAL 전이 허용 lateral error |
| `state_timer_period_ms` | `50` | state update 주기 |
| `enable_manual_lap_override` | `false` | 추후 수동 lap override용 예약 파라미터 |

## State Transition

초기 상태는 `LOCAL`이다.

`LOCAL -> GLOBAL` 조건:

- global path가 ready: non-empty `/global_waypoints` snapshot, fresh true `/planning/global_path_valid`, and Graph SLAM status `localization`
- Frenet odometry가 fresh
- `abs(current_d) <= max_abs_d_for_global`
- `/planning/global_handoff_ready`가 `global_handoff_timeout_sec` 이내의 fresh true이며 `global_entry_dwell_sec` 동안 연속 유지됨

Handoff는 GLOBAL 진입 gate에만 사용한다. false 또는 stale이면 dwell 시작 시각을
reset하지만, 이미 GLOBAL인 state를 handoff만으로 demote하거나 STOP으로 바꾸지 않는다.
Local/global planner heartbeat loss도 mission STOP으로 변환하지 않는다. 기존 global path
validity/status/invalidation 조건만 GLOBAL에서 LOCAL로 demote한다.

`planner_node`가 만드는 runtime global path는 blue/yellow cone boundaries의
conservative centerline/global waypoint generator 결과이다. Production
racing-line optimizer가 아니며, offline minimum-curvature CSV workflow와
분리되어 있다. 기본 설정에서는 refresh 실패 시 마지막 valid 결과를 유지하고,
refresh 성공 시 새 global path를 publish한다.

`/global_waypoints`는 latched snapshot으로 처리하므로 wall-clock freshness timeout을 적용하지 않는다. 대신 `/planning/global_path_valid`가 `false`이거나 stale이면 state machine은 invalidation generation을 기록하고 현재 accepted waypoint snapshot을 버린다. 그 뒤에는 true heartbeat만으로 복구하지 않고, invalidation 이후에 새 non-empty waypoint snapshot을 받은 뒤에만 global path ready가 될 수 있다. 기본 SLAM `planner_node`는 refresh 실패 시 기존 valid snapshot을 유지해 이 invalidation 경로를 피한다.

`GLOBAL -> LOCAL` demotion 조건:

- Graph SLAM status가 `localization`이 아닌 값으로 바뀜
- `/planning/global_path_valid`가 `false` 또는 stale
- invalidation 이후 새 accepted waypoint snapshot이 아직 없음

단, STOP 조건이 먼저 만족되면 STOP 전이가 우선한다.

`GLOBAL` 유지 시 path source:

- `lap_count < final_lap_start_count`: `GLOBAL_FULL`
- `lap_count >= final_lap_start_count`: `GLOBAL_FINAL_STOP`

`GLOBAL -> STOP` 후보 조건:

- `lap_count >= target_lap_count`
- 그리고 다음 중 하나를 만족:
- `isFinalPathEndReached() == true`
- `isStoplineDetected() == true`

단, Frenet odometry가 stale이면 `GLOBAL -> STOP` 전이는 막는다.

`isStoplineDetected()`는 `stop_zone_valid == true`, stop zone 입력이 fresh, 그리고 `current_s`가
`[stop_zone_s_start - stop_zone_s_margin, stop_zone_s_end + stop_zone_s_margin]` 범위 안에 있을 때 true이다.
`stop_zone_s_end < stop_zone_s_start`인 경우 closed-loop wrap-around 구간으로 보고 `global_path_length > 0`일 때만 판정한다.

`STOP` 상태:

- `path_source = STOP`
- `stop_request = true`
- state는 계속 `STOP` 유지

## Lap Counting

이 process가 `/graph_slam/status`에서 `mapping` 또는 `mapping_converged` 직후
`localization`을 직접 관측하면 discovery lap을 한 번만 반영해
`lap_count = max(lap_count, 1)`로 만든다. 시작 시 첫 status가 `localization`인
loaded-map 실행은 lap을 만들지 않는다.

Frenet wrap lap은 frame이 정확히 `map`이고 waypoint가 3개 이상인 global path만
사용한다. 모든 XY와 `s_m`은 finite여야 하고 `s_m`은 strictly increasing이어야 하며,
first/last XY 거리는 `lap_path_closure_tolerance_m` 이하여야 한다. Closure가
`lap_closing_duplicate_tolerance_m` 이하이면 normalized length는 `last.s_m`, 그보다
크면 `last.s_m + closure`이다. Length가 20 m 미만이면 lap tracking을 disable한다.

`zone = min(5.0, 0.1 * length)`이다. 두 개의 연속 fresh Frenet sample이 모두
`zone < s < length-zone`에 있고 positive `s` progress를 보인 뒤에만 arm한다. Armed
상태에서 previous `s >= length-zone`, next `s <= zone`인 forward seam wrap을 관측하면
lap을 한 번 증가시키고 disarm한다. 다시 middle progress를 관측해야 re-arm하며 count
cooldown은 2초이다. Accepted path generation 변경, stale/non-finite sample, seam이 아닌
backward jump는 count 없이 disarm한다. Closing duplicate와 non-duplicate 인코딩은 같은
normalized length 기준을 사용한다.

## TODO

- start/finish gate detection 구현
- stop zone detector 입력을 만드는 `stop_zone_detector_node` 구현
- closed-loop wrap-around를 고려한 final path end 판정 개선

## 실행 방법

빌드:

```bash
colcon build --packages-select hyu_state_machine
```

실행:

```bash
source install/setup.zsh
ros2 launch hyu_state_machine planning_hyu_state_machine.launch.py
```

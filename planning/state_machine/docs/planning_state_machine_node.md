# planning_state_machine_node

## 목적

`planning_state_machine_node`는 Formula Student Korea 자율주행 planning 흐름에서 현재 planning 상태를 결정하고, downstream selector가 사용할 `path_source`를 publish하는 뼈대 노드이다.

이 노드는 path를 생성하지 않고, waypoint도 publish하지 않는다. 실제 waypoint 선택과 publish는 추후 `wpnt_selector_publisher_node`가 `/planning/path_source`를 구독해서 처리한다.

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

`planning_state_machine_node`는 global waypoint writer가 아니라 consumer이다.
`/global_waypoints`와 `/planning/global_path_valid`는 launch 안에서 정확히
하나의 writer만 가져야 한다. SLAM integration에서는
`global_planner/slam_global_planner.launch.py planner_source:=slam`이
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
planning/state_machine/config/planning_state_machine.yaml
```

| Parameter | Default | 설명 |
| --- | --- | --- |
| `frenet_odom_topic` | `/car_state/frenet/odom` | Frenet odometry 입력 topic |
| `global_waypoints_topic` | `/global_waypoints` | latched global waypoint 입력 topic |
| `graph_slam_status_topic` | `/graph_slam/status` | Graph SLAM lifecycle status 입력 topic |
| `global_path_valid_topic` | `/planning/global_path_valid` | global path validity heartbeat 입력 topic |
| `cone_map_topic` | `/cones` | local perception cone 입력 topic. 기존 파라미터 이름은 호환을 위해 유지한다. |
| `stop_zone_s_start_topic` | `/stop_zone_s_start` | stop zone 시작 `s` 입력 topic |
| `stop_zone_s_end_topic` | `/stop_zone_s_end` | stop zone 끝 `s` 입력 topic |
| `stop_zone_valid_topic` | `/stop_zone_valid` | stop zone 유효 여부 입력 topic |
| `target_lap_count` | `4` | STOP 후보가 되는 목표 lap count |
| `initial_lap_count` | `0` | 시작 lap count |
| `final_lap_start_count` | `3` | final stop path source로 넘어가는 lap count |
| `frenet_odom_timeout_sec` | `0.5` | Frenet odometry freshness timeout |
| `global_path_valid_timeout_sec` | `0.5` | `/planning/global_path_valid` true heartbeat freshness timeout |
| `cone_map_timeout_sec` | `1.0` | Cone map freshness timeout |
| `stop_zone_timeout_sec` | `1.0` | stop zone detector freshness timeout |
| `final_path_end_threshold` | `2.0` | final path end 판단 거리 threshold |
| `stop_zone_s_margin` | `0.0` | stop zone `s` 범위 판정 margin |
| `max_abs_d_for_global` | `2.0` | GLOBAL 전이 허용 lateral error |
| `state_timer_period_ms` | `50` | state update 주기 |
| `enable_manual_lap_override` | `false` | 추후 수동 lap override용 예약 파라미터 |

## State Transition

초기 상태는 `LOCAL`이다.

`LOCAL -> GLOBAL` 조건:

- `lap_count >= 1`
- global path가 ready: non-empty `/global_waypoints` snapshot, fresh true `/planning/global_path_valid`, and Graph SLAM status `localization`
- Frenet odometry가 fresh
- `abs(current_d) <= max_abs_d_for_global`

`planner_node`가 만드는 runtime global path는 blue/yellow cone boundaries의
conservative centerline/global waypoint generator 결과이다. Production
racing-line optimizer가 아니며, offline minimum-curvature CSV workflow와
분리되어 있다.

`/global_waypoints`는 latched snapshot으로 처리하므로 wall-clock freshness timeout을 적용하지 않는다. 대신 `/planning/global_path_valid`가 `false`이거나 stale이면 state machine은 invalidation generation을 기록하고 현재 accepted waypoint snapshot을 버린다. 그 뒤에는 true heartbeat만으로 복구하지 않고, invalidation 이후에 새 non-empty waypoint snapshot을 받은 뒤에만 global path ready가 될 수 있다.

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

## TODO

- start/finish gate detection 구현
- lap count 자동 증가 조건 구현
- stop zone detector 입력을 만드는 `stop_zone_detector_node` 구현
- closed-loop wrap-around를 고려한 final path end 판정 개선

## 실행 방법

빌드:

```bash
colcon build --packages-select state_machine
```

실행:

```bash
source install/setup.zsh
ros2 launch state_machine planning_state_machine.launch.py
```

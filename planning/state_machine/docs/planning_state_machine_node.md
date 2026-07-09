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

## Subscribe Topic

| Topic | Type | 설명 |
| --- | --- | --- |
| `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | Frenet odometry. `pose.pose.position.x`를 `current_s`, `pose.pose.position.y`를 `current_d`로 저장한다. |
| `/global_waypoints` | TODO | waypoint message type이 확정되면 subscriber를 추가한다. 현재는 빌드가 깨지지 않도록 TODO만 남긴다. |
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
| `global_waypoints_topic` | `/global_waypoints` | 추후 global waypoint 입력 topic |
| `cone_map_topic` | `/cones` | local perception cone 입력 topic. 기존 파라미터 이름은 호환을 위해 유지한다. |
| `stop_zone_s_start_topic` | `/stop_zone_s_start` | stop zone 시작 `s` 입력 topic |
| `stop_zone_s_end_topic` | `/stop_zone_s_end` | stop zone 끝 `s` 입력 topic |
| `stop_zone_valid_topic` | `/stop_zone_valid` | stop zone 유효 여부 입력 topic |
| `target_lap_count` | `4` | STOP 후보가 되는 목표 lap count |
| `initial_lap_count` | `0` | 시작 lap count |
| `final_lap_start_count` | `3` | final stop path source로 넘어가는 lap count |
| `frenet_odom_timeout_sec` | `0.5` | Frenet odometry freshness timeout |
| `global_waypoints_timeout_sec` | `2.0` | Global waypoint freshness timeout |
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
- global path가 ready
- Frenet odometry가 fresh
- `abs(current_d) <= max_abs_d_for_global`

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

- `/global_waypoints` message type 확정 후 subscriber 추가
- `/global_waypoints`에서 `global_path_ready`, `global_path_length` 계산 구현
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

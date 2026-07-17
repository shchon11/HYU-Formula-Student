# HYU driverless topic contract

단일 출처(single source of truth). 토픽을 추가/변경할 때 이 표를 함께 갱신한다.
원칙: 토픽은 **발행 서브시스템의 네임스페이스** 아래 둔다. 드라이버 원시 토픽은
upstream 기본값을 유지한다. 디버그/시각화는 `<subsystem>/debug/*`.

이 문서는 hyu_bringup 패키지가 생기면 그 안으로 이동한다.

## 드라이버 원시 토픽 (upstream 기본값, 개명 금지)

| 토픽 | 타입 | 발행자 | QoS |
|---|---|---|---|
| `/velodyne_points` | sensor_msgs/PointCloud2 | velodyne_pointcloud (sim: lidar_realism) | SensorData |
| `/zed/zed_node/left/image_rect_color` 등 | sensor_msgs/Image, CameraInfo | zed-ros2-wrapper (sim: gazebo camera) | SensorData |
| `/sbg/ekf_nav`, `/sbg/ekf_euler`, `/sbg/ekf_rot_accel_body` 등 | sbg_driver/* | sbg_ros2_driver (sim: sim_ellipse_d) | driver 기본 |

sim은 위 토픽명을 그대로 흉내내어 발행한다(실차/sim 동일 계약).

## Perception (`/perception`)

| 토픽 | 타입 | 발행자 | 구독자 | QoS |
|---|---|---|---|---|
| `/perception/cones` | hyu_msgs/ConeArrayWithCovariance (frame: base_footprint) | hyu_perception | hyu_localization, planning | Reliable d10 → SensorData 구독 |
| `/perception/bounding_boxes` | hyu_msgs/BoundingBoxes | yolov8_bbox_node | hyu_perception | Reliable d10 |
| `/perception/debug/cones_viz` | MarkerArray | hyu_perception | RViz | 디버그 |
| `/perception/debug/cone_provenance` | MarkerArray | hyu_perception | RViz | 디버그 |

## Localization (`/localization`)

| 토픽 | 타입 | 발행자 | 구독자 | QoS |
|---|---|---|---|---|
| `/localization/ego_odom` | nav_msgs/Odometry | hyu_localization | planning, control | Reliable d10 |
| `/localization/cone_map` | hyu_msgs/ConeArrayWithCovariance | hyu_localization | planning | Reliable |
| `/localization/map` | hyu_msgs/ConeArrayWithCovariance | hyu_localization | planning, 도구 | Reliable(TransientLocal 후보) |
| `/localization/map_converged` | std_msgs/Bool | hyu_localization | planning | Reliable |
| `/localization/status` | (기존 graph_slam/status 타입) | hyu_localization | planning, HUD | Reliable |
| `/localization/gnss_odom` | nav_msgs/Odometry | sbg_odometry_bridge | hyu_localization | Reliable |
| `/localization/wheel_odom` | hyu_msgs/CarState | wheel_odometry | hyu_localization, perception(deskew), tmpc_state_bridge | Reliable |
| `/initialpose` | PoseWithCovarianceStamped | RViz(수동 재국지화) | hyu_localization | 표준 |
| `/localization/ins_odom` | hyu_msgs/CarState | sbg_odometry_bridge | (구성에 따라) SLAM 모션 입력 | Reliable |
| `/localization/drift_odom` | hyu_msgs/CarState | drift_odom(평가 도구) | evaluate_slam | 도구 |
| `/localization/debug/{markers,path,status_overlay,gnss_markers,gnss_overlay}` | Marker/Path/Overlay | hyu_localization·sbg_bridge | RViz/HUD | 디버그 |

TF: `map→odom`, `odom→base_footprint`는 hyu_localization이 발행.
센서 마운트 TF는 robot_state_publisher(URDF).

## Planning (`/planning`)

| 토픽 | 타입 | 발행자 | 구독자 |
|---|---|---|---|
| `/planning/path` | (기존 path_waypoints 타입) | hyu_path_selector | hyu_pure_pursuit |
| `/planning/state` | std_msgs/String (LOCAL/GLOBAL/STOP) | hyu_state_machine | hyu_cmd_selector 등 |
| `/planning/frenet_odom` | nav_msgs/Odometry | frenet_odom_node | planning 내부 |
| `/planning/trajectory_performance` / `_emergency` | hyu_tmpc_msgs 궤적 | planning(TMPC 경로) | hyu_tmpc |
| `/planning/stop_zone/{valid,s_start,s_end}` | Bool/Float | planning | planning/control |
| `/planning/stop_request`, `/planning/lap_count`, `/planning/*_path_valid`, `/planning/path_source`, `/planning/global_handoff_ready` 등 | 기존 유지 | planning | selector/HUD |
| `/planning/global_waypoints` (+`/path` 파생) | 글로벌 웨이포인트 (latched) | hyu_global_planner | selector/TMPC 경로 |
| `/planning/skidpad/{cone_map,phase}` | ConeArray/phase | skidpad_director | hyu_local_planner |
| `/planning/debug/path`, `/planning/debug/frenet`, `/planning/debug`, `/planning/debug/global_planner_markers` | 디버그 | planning | RViz |

## Control (`/control`)

| 토픽 | 타입 | 발행자 | 구독자 |
|---|---|---|---|
| `/control/pp/cmd` | ackermann_msgs/AckermannDriveStamped | hyu_pure_pursuit | hyu_cmd_selector |
| `/control/tmpc/cmd` | AckermannDriveStamped | hyu_tmpc_output_bridge | hyu_cmd_selector |
| `/control/tmpc/cmd_shadow` | AckermannDriveStamped | hyu_tmpc_output_bridge(내부) | (런치 리맵 전 단계) |
| `/control/tmpc/valid` | std_msgs/Bool (하트비트) | hyu_tmpc_output_bridge | hyu_cmd_selector |
| `/control/tmpc/vehicle_state` | hyu_tmpc_msgs/TumVehicleState | hyu_tmpc_state_bridge | hyu_tmpc |
| `/control/selector/status` | (기존 셀렉터 상태 타입) | hyu_cmd_selector | HUD/로그 |

## Vehicle (`/vehicle`) — 실차에서는 hyu_can_bridge 소유, sim에서는 gazebo 플러그인

| 토픽 | 타입 | 방향 | 비고 |
|---|---|---|---|
| `/vehicle/cmd` | AckermannDriveStamped | stack → vehicle | 최종 명령 (hyu_cmd_selector 발행) |
| `/vehicle/wheel_speeds` | hyu_msgs/WheelSpeedsStamped | vehicle → stack | 휠속 + 조향각 피드백 |
| `/vehicle/as_state`, `/vehicle/as_state_str` | hyu_msgs/CanState, String | vehicle → stack | AS/미션 상태 |
| `/vehicle/set_mission` | hyu_msgs/srv/SetCanState | 서비스 | 미션 지정 |
| `/vehicle/mission_completed`, `/vehicle/ebs` | 서비스/토픽 | stack → vehicle | 미션 종료·EBS 요청 |
| `/vehicle/reset`, `/vehicle/reset_cone_pos`, `/vehicle/reset_vehicle_pos` | 서비스 | 도구 → sim | sim 전용 리셋 |

## sim 전용 (개명하지 않음)

`/ground_truth/*` (GT odom/state/cones — 평가 전용, 스택 소비 금지),
`/velodyne_points_ideal`, gazebo 내부 토픽.

## QoS 정책

- 센서 스트림 구독: SensorDataQoS(BestEffort). 발행자 QoS를 먼저 확인.
- 명령·하트비트·상태: Reliable + KeepLast(1~10), 셀렉터 신선도 게이트와 조합.
- 저빈도 상태물(맵, 글로벌 경로): Reliable + TransientLocal 후보.
- `/perception/cones`: 발행 Reliable, SLAM 구독 SensorData(BestEffort) — DDS 합법 조합.

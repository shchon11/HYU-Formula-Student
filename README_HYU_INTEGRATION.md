# HYU Formula Student SLAM Integration 정리

작성일: 2026-06-02

이 문서는 지금까지 만든 HYU simulator + graph SLAM + perception baseline
통합 구조를 설명한다.

핵심 목표는 다음이다.

```text
simulator cone observation
  -> perception baseline output contract
  -> graph SLAM
```

즉 지금 단계에서는 실제 perception detector를 완성하는 것이 아니라,
SLAM이 받을 `/cones` 출력 형식을 먼저 고정하고, 그 형식으로 graph SLAM이
실제로 돌아가는지 확인하는 것이 목적이다.

## 1. Workspace 구조

현재 workspace는 두 개로 나뉜다.

### 1.1 `/home/ircv/eufs_ws`

기존 EUFS simulator 설치 workspace이다.

역할:

- fallback workspace
- 원래 설치한 simulator가 망가지지 않게 보존
- 처음 만든 `eufs_perception_baseline`도 여기 있음
- 전체 작업 맥락 문서가 여기 있음

중요 파일:

- `/home/ircv/eufs_ws/EUFS_SLAM_PERCEPTION_CHAT_CONTEXT.md`
- `/home/ircv/eufs_ws/eufs_perception_baseline`

### 1.2 `/path/to/HYU-FS-Sim`

HYU repo 기반 SLAM integration workspace이다.

역할:

- HYU teammate repo 기반 simulator/SLAM 통합 실험
- `eufs_graph_slam` build
- `g2o` dependency 해결
- perception baseline adapter와 graph SLAM 연결 테스트

중요 폴더:

```text
/path/to/HYU-FS-Sim
  src/
    HYU-Formula-Student/
      eufs_sim/
      eufs_msgs/
      eufs_graph_slam/
    g2o/
    eufs_perception_baseline/
  scripts/
    hyu-docker
  README_HYU_INTEGRATION.md
```

## 2. 각 package가 하는 일

### 2.1 `eufs_sim`

HYU repo 안의 simulator package이다.

역할:

- Gazebo world 실행
- racecar spawn
- LiDAR, camera, IMU 등 sensor topic publish
- cone ground truth / simulated cone observation publish
- car state publish

중요 topic:

```text
/velodyne_points
/camera_0/cones
/camera_1/cones
/ground_truth/cones
/ground_truth/track
/odometry_integration/car_state
/tf
/tf_static
```

현재 headless에서 확인된 것:

- `/velodyne_points`는 정상 publish됨
- `/camera_0/cones`는 정상 publish됨
- `/odometry_integration/car_state`는 정상 publish됨
- `/zed/left/image_rect_color`는 headless render 문제로 publisher가 없었음

### 2.2 `eufs_graph_slam`

HYU teammate가 넣어둔 graph SLAM package이다.

입력:

```text
/odometry_integration/car_state
/cones
```

출력:

```text
/graph_slam/map
/graph_slam/odom
/graph_slam/path
/graph_slam/markers
/tf
```

SLAM이 기대하는 perception 입력은 반드시 다음 형식이어야 한다.

```text
topic: /cones
type: eufs_msgs/msg/ConeArrayWithCovariance
frame: base_footprint
```

### 2.3 `g2o`

graph SLAM 최적화에 필요한 C++ graph optimization library이다.

HYU repo에는 g2o가 포함되어 있지 않다. 그래서 따로 clone했다.

위치:

```text
/path/to/HYU-FS-Sim/src/g2o
```

`COLCON_IGNORE`를 둔 이유:

- g2o는 ROS package가 아니다.
- colcon이 g2o를 독립 package처럼 build하면 안 된다.
- `eufs_graph_slam`의 CMake가 g2o를 `add_subdirectory()`로 직접 build한다.

### 2.4 `eufs_perception_baseline`

우리가 만든 perception baseline package이다.

역할:

- SLAM이 원하는 `/cones` output contract를 고정
- image + pointcloud 입력을 받을 node 구조 준비
- 실제 detector가 아직 없어도 simulator cone topic을 `/cones`로 변환해서 SLAM에 넣을 수 있음

현재 node:

```text
node: perception_baseline_node
input image: /zed/left/image_rect_color
input pointcloud: /velodyne_points
oracle input: /camera_0/cones
output: /cones
```

현재 구현 모드:

1. LiDAR-camera fusion baseline mode
   - `/noisy_bounding_boxes`, `/velodyne_points`, `/custom_camera_info`, `/tf`를 구독
   - bbox의 class/color와 LiDAR cluster의 metric position을 결합
   - SLAM 입력 topic인 `/cones`로 publish

2. oracle adapter mode
   - `/camera_0/cones` 같은 simulator cone topic을 받음
   - frame/covariance를 정리함
   - `/cones`로 다시 publish
   - graph SLAM integration test용

perception baseline의 자세한 실행법과 topic 확인법:

```text
eufs_perception_baseline/docs/perception_baseline_usage.md
```

## 3. 지금 통일한 것

SLAM과 perception 사이에서 가장 중요한 계약을 다음처럼 통일했다.

```text
topic: /cones
message: eufs_msgs/msg/ConeArrayWithCovariance
frame_id: base_footprint
time: simulator time
point.x: 차량 기준 앞쪽 거리
point.y: 차량 기준 왼쪽 거리
point.z: 0.0
covariance: [xx, xy, yx, yy]
```

cone color는 하나의 list에 섞지 않고 message field별로 나눈다.

```text
blue_cones
yellow_cones
orange_cones
big_orange_cones
unknown_color_cones
```

이게 중요한 이유:

- graph SLAM은 `/cones`를 직접 subscribe한다.
- graph SLAM은 `blue_cones`, `yellow_cones` 등 배열을 나눠 읽는다.
- graph SLAM은 `point.x`, `point.y`만 2D landmark observation으로 사용한다.
- covariance가 비정상이면 default covariance로 대체한다.

## 4. 전체 연결 구조

현재 검증된 연결은 다음이다.

```text
Gazebo / HYU simulator
  publishes /camera_0/cones
        |
        v
eufs_perception_baseline
  subscribes /camera_0/cones
  publishes /cones
        |
        v
eufs_graph_slam
  subscribes /cones
  subscribes /odometry_integration/car_state
  publishes /graph_slam/map
  publishes /graph_slam/odom
  publishes /graph_slam/path
  publishes /tf
```

조금 더 자세히 보면:

```text
/camera_0/cones
  type: eufs_msgs/msg/ConeArrayWithCovariance
  frame: base_footprint
  source: simulator camera cone plugin

/cones
  type: eufs_msgs/msg/ConeArrayWithCovariance
  frame: base_footprint
  source: perception_baseline_node
  target: graph_slam

/odometry_integration/car_state
  type: eufs_msgs/msg/CarState
  source: race_car Gazebo plugin
  target: graph_slam

/graph_slam/map
  type: eufs_msgs/msg/ConeArrayWithCovariance
  frame: map
  source: graph_slam
```

## 5. 왜 `/camera_0/cones`를 `/cones`로 바꾸는가

HYU simulator에는 여러 cone topic이 있다.

```text
/ground_truth/cones
/ground_truth/track
/camera_0/cones
/camera_1/cones
/cones
```

그런데 graph SLAM은 `/cones`만 본다.

또 simulator가 직접 `/cones`를 publish하게 두면, 나중에 perception node도
`/cones`를 publish할 때 publisher가 2개가 되어 충돌하거나 헷갈린다.

그래서 현재 테스트에서는:

```text
simulator launch_group:=default
```

를 사용한다.

이 상태에서는 simulator의 simulated `/cones`는 꺼져 있고,
`/camera_0/cones`는 살아 있다.

그래서 우리가 만든 adapter가:

```text
/camera_0/cones -> /cones
```

를 담당한다.

이렇게 하면 `/cones` publisher는 딱 하나가 된다.

```text
publisher: perception_baseline_node
subscriber: graph_slam
```

## 6. TF 구조

graph SLAM을 쓸 때는 simulator의 ground truth base TF를 꺼야 한다.

그래서 simulator 실행에 다음 option을 넣는다.

```text
publish_gt_tf:=false
```

현재 graph SLAM이 publish하는 TF 구조:

```text
map -> odom -> base_footprint
```

robot_state_publisher는 차량 내부 link TF를 publish한다.

예:

```text
chassis -> left_steering_hinge
left_steering_hinge -> left_front_wheel
```

따라서 전체적으로는:

```text
map
  odom
    base_footprint
      chassis
        wheels / sensors ...
```

같은 구조가 된다.

## 7. Docker 실행 구조

중요한 발견이 있었다.

서로 다른 Docker container에서 ROS 2 topic discovery는 되는데,
실제 data echo가 안 되는 현상이 있었다.

즉:

```text
container A: simulator
container B: ros2 topic echo
```

에서 topic 이름은 보이지만 message가 안 들어오는 문제가 있었다.

그래서 현재 안정적인 방식은:

```text
container A: simulator
container A 안에서 docker exec로 adapter 실행
container A 안에서 docker exec로 graph_slam 실행
```

이다.

이를 편하게 하려고 script를 만들었다.

```text
/path/to/HYU-FS-Sim/scripts/hyu-docker
```

## 8. 실행 방법

### 8.1 build

```bash
/path/to/HYU-FS-Sim/scripts/hyu-docker setup-g2o
/path/to/HYU-FS-Sim/scripts/hyu-docker build-ws
```

성공하면 전체 10개 package가 build된다.

`setup-g2o`는 graph SLAM이 사용하는 `g2o` source dependency를 repo root의
`g2o/`에 clone한다. 이 directory는 `.gitignore`에 포함되어 commit하지 않는다.

### 8.2 headless simulator 실행

Terminal 1:

```bash
/path/to/HYU-FS-Sim/scripts/hyu-docker sim-headless
```

이 명령은 Docker container `hyu_eufs_sim`을 만든다.

내부적으로 실행되는 핵심 launch option:

```text
rviz:=false
gazebo_gui:=false
show_rqt_gui:=false
publish_gt_tf:=false
launch_group:=default
pub_ground_truth:=true
vehicleModelConfig:=configDry.yaml
```

### 8.2.1 GUI simulator 실행

정지 상태라도 Gazebo/RViz 화면으로 직접 보고 싶으면 GUI mode를 사용한다.

Terminal 1에서 foreground로 실행:

```bash
/path/to/HYU-FS-Sim/scripts/hyu-docker sim-gui
```

또는 background로 실행:

```bash
/path/to/HYU-FS-Sim/scripts/hyu-docker sim-gui-bg
```

GUI mode는 다음을 켠다.

```text
rviz:=true
gazebo_gui:=true
show_rqt_gui:=false
publish_gt_tf:=false
launch_group:=default
```

2026-06-02에 GUI mode로 확인한 것:

- `gzserver` 실행
- `gzclient` 실행
- `rviz2` 실행
- racecar spawn
- cone plugins loaded
- `/camera_0/cones` publish
- `/velodyne_points` publish
- `/zed/left/image_rect_color` publisher 생성

perception baseline을 확인할 때의 권장 실행 순서는 다음이다.

```bash
/path/to/HYU-FS-Sim/scripts/hyu-docker setup-g2o
/path/to/HYU-FS-Sim/scripts/hyu-docker sim-gui-bg
/path/to/HYU-FS-Sim/scripts/hyu-docker fusion-bg
/path/to/HYU-FS-Sim/scripts/hyu-docker slam-bg
/path/to/HYU-FS-Sim/scripts/hyu-docker status
```

`adapter-bg`는 SLAM wiring만 확인할 때 사용하고, 실제 perception baseline
확인은 `fusion-bg`로 한다.

### 8.3 perception adapter 실행

Terminal 2:

```bash
/path/to/HYU-FS-Sim/scripts/hyu-docker adapter-bg
```

이 명령은 이미 떠 있는 `hyu_eufs_sim` container 안에서 adapter를 background로 실행한다.

실제로는 다음 연결을 만든다.

```text
/camera_0/cones -> /cones
```

### 8.4 LiDAR-camera fusion baseline 실행

논문식 baseline을 테스트하려면 oracle adapter 대신 fusion mode를 실행한다.

Terminal 2:

```bash
/path/to/HYU-FS-Sim/scripts/hyu-docker fusion-bg
```

이 명령은 다음 입력을 사용한다.

```text
/noisy_bounding_boxes
/velodyne_points
/custom_camera_info
/tf
```

그리고 다음 출력으로 publish한다.

```text
/cones
type: eufs_msgs/msg/ConeArrayWithCovariance
frame: base_footprint
```

주의:

- `adapter-bg`와 `fusion-bg`를 동시에 `/cones`로 실행하지 않는다.
- 둘 다 `/cones` publisher가 되기 때문이다.
- integration 확인용은 `adapter-bg`, 실제 perception baseline 확인용은 `fusion-bg`를 사용한다.

현재 fusion baseline 구현 위치:

```text
/path/to/HYU-FS-Sim/eufs_perception_baseline/eufs_perception_baseline/perception_baseline_node.py
```

핵심 함수:

```text
_run_lidar_camera_fusion()
_pointcloud_to_xyz()
_roi_mask()
_cluster_cone_candidates()
_project_points()
_associate_detections_to_clusters()
_cluster_to_cone()
```

2026-06-02 smoke test:

- GUI simulator가 떠 있는 상태에서 `/fusion/cones_test` 임시 topic으로 fusion node 실행
- fused cone publish 확인
- 예시 output: yellow cone 1개, frame `base_footprint`

### 8.5 graph SLAM 실행

Terminal 2:

```bash
/path/to/HYU-FS-Sim/scripts/hyu-docker slam-bg
```

이 명령은 같은 container 안에서 graph SLAM을 background로 실행한다.

graph SLAM은 다음을 구독한다.

```text
/cones
/odometry_integration/car_state
```

그리고 다음을 publish한다.

```text
/graph_slam/map
/graph_slam/odom
/graph_slam/path
/graph_slam/markers
/tf
```

### 8.6 상태 확인

Terminal 2:

```bash
/path/to/HYU-FS-Sim/scripts/hyu-docker status
```

기대 결과:

```text
/cones
  Publisher: perception_baseline_node
  Subscriber: graph_slam

/graph_slam/map
/graph_slam/markers
/graph_slam/odom
/graph_slam/path
```

### 8.7 log 확인

```bash
/path/to/HYU-FS-Sim/scripts/hyu-docker logs
```

### 8.8 종료

```bash
/path/to/HYU-FS-Sim/scripts/hyu-docker stop
```

## 9. 직접 ROS 명령으로 확인하고 싶을 때

simulator container가 떠 있는 상태에서:

```bash
/path/to/HYU-FS-Sim/scripts/hyu-docker shell
```

container 안에서:

```bash
source /workspace/install/setup.bash
ros2 topic list
ros2 topic echo /cones --once
ros2 topic echo /graph_slam/map --once
ros2 topic echo /graph_slam/odom --once
ros2 run tf2_ros tf2_echo map base_footprint
```

## 10. 현재 검증 결과

검증 완료:

- HYU workspace build 성공
- g2o source dependency 해결
- `eufs_graph_slam` build 성공
- simulator headless 실행 성공
- simulator GUI 실행 성공
- `/camera_0/cones` publish 확인
- `/odometry_integration/car_state` publish 확인
- `/velodyne_points` publish 확인
- GUI mode에서 `/zed/left/image_rect_color` publisher 확인
- LiDAR-camera fusion baseline build 성공
- fusion baseline smoke test에서 `/fusion/cones_test` publish 확인
- adapter가 `/cones` publish 확인
- graph SLAM이 `/cones` subscribe 확인
- graph SLAM output publish 확인
- TF `map -> odom -> base_footprint` 확인

중요한 해석:

이 검증은 단순히 ROS node만 띄워서 가짜 topic을 주고받은 것이 아니다.
Gazebo simulator를 실제로 headless로 실행했고, Gazebo plugin들이 만든 실제
simulation topic을 사용했다.

추가로 GUI mode도 실행해서 정지 상태 simulator를 화면으로 볼 수 있는 것까지
확인했다.

다만 아직 차를 움직이며 주행한 검증은 아니다.

현재 완료한 검증은 다음에 가깝다.

```text
실제 Gazebo world 실행
실제 racecar spawn
실제 Gazebo cone plugin output 사용
실제 racecar plugin car_state 사용
차량은 거의 정지 상태
graph SLAM input/output 연결 확인
```

아직 하지 않은 검증은 다음이다.

```text
차량에 /cmd 입력
차량 pose 변화 확인
/camera_0/cones 관측 변화 확인
graph_slam path 증가 확인
graph_slam map이 움직이는 차량 기준 observation으로 계속 갱신되는지 확인
```

즉 현재 integration skeleton은 성공했지만, dynamic driving validation은
다음 단계로 필요하다.

## 11. 실제 주행 검증 방법

차량 제어 topic:

```text
topic: /cmd
type: ackermann_msgs/msg/AckermannDriveStamped
```

주의:

race car plugin은 state machine이 drive 가능한 상태가 아니면 `/cmd`를
무시하고 강제 제동한다.

코드상 drive 가능한 조건:

```text
as_state == AS_DRIVING
또는
ami_state == AMI_MANUAL
```

가장 간단한 테스트는 manual mission으로 바꾸는 것이다.

### 11.1 simulator 실행

Terminal 1:

```bash
/path/to/HYU-FS-Sim/scripts/hyu-docker sim-headless
```

### 11.2 adapter와 graph SLAM 실행

Terminal 2:

```bash
/path/to/HYU-FS-Sim/scripts/hyu-docker adapter-bg
/path/to/HYU-FS-Sim/scripts/hyu-docker slam-bg
```

### 11.3 manual mission 설정

Terminal 2:

```bash
/path/to/HYU-FS-Sim/scripts/hyu-docker mission-manual
```

이 명령은 내부적으로 다음 service를 호출한다.

```bash
ros2 service call /ros_can/set_mission eufs_msgs/srv/SetCanState '{ami_state: 21, as_state: 0}'
```

`ami_state: 21`은 `AMI_MANUAL`이다.

### 11.4 차량 전진 명령

Terminal 2:

```bash
/path/to/HYU-FS-Sim/scripts/hyu-docker drive-forward
```

이 명령은 약 8초 동안 `/cmd`에 acceleration command를 publish한다.

내부 명령:

```bash
ros2 topic pub -r 20 /cmd ackermann_msgs/msg/AckermannDriveStamped \
  '{drive: {steering_angle: 0.0, speed: 0.0, acceleration: 1.0}}'
```

### 11.5 정지 명령

Terminal 2:

```bash
/path/to/HYU-FS-Sim/scripts/hyu-docker drive-stop
```

### 11.6 pose 확인

Terminal 2:

```bash
/path/to/HYU-FS-Sim/scripts/hyu-docker pose-once
```

확인할 것:

- `/odometry_integration/car_state.pose.pose.position.x`
- `/odometry_integration/car_state.pose.pose.position.y`
- `/graph_slam/path` poses 길이
- `/graph_slam/map` cone 위치 갱신

## 12. 아직 해결해야 할 문제

### 12.1 headless ZED image 문제

headless 실행에서는 Gazebo rendering이 꺼져서 camera sensor가 생성되지 않았다.

증상:

```text
/zed/left/image_rect_color publisher count: 0
```

반면:

```text
/velodyne_points publisher count: 1
```

즉 LiDAR PCD는 나오지만 image는 안 나온다.

GUI mode에서는 `/zed/left/image_rect_color` publisher가 생기는 것을 확인했다.

따라서 실제 image + PCD perception baseline을 만들려면 다음 중 하나가 필요하다.

1. `sim-gui`로 X11/GPU forwarding 사용
2. Xvfb/EGL 같은 virtual rendering 환경 추가
3. camera 없이 LiDAR-only baseline부터 먼저 구현

GUI 실행 명령:

```bash
/path/to/HYU-FS-Sim/scripts/hyu-docker sim-gui
```

background GUI 실행:

```bash
/path/to/HYU-FS-Sim/scripts/hyu-docker sim-gui-bg
```

### 12.2 magnetometer plugin missing

HYU simulator가 다음 log를 남긴다.

```text
Failed to load plugin libhector_gazebo_ros_magnetic.so
```

현재 graph SLAM cone integration에는 영향 없었다.

하지만 magnetometer data를 쓰려면 나중에 dependency를 해결해야 한다.

## 13. 다음 작업 추천

현재 SLAM integration skeleton은 성공했다.

다음 perception 작업은 두 갈래 중 하나로 가면 된다.

### 선택 A: image 문제 먼저 해결

목표:

- `/zed/left/image_rect_color`가 실제 publish되게 만들기
- image + PCD baseline 준비

필요:

- `sim-gui` 확인
- X11/GPU forwarding 문제 확인
- 필요하면 virtual display 추가

### 선택 B: LiDAR-only baseline 먼저 구현

목표:

- `/velodyne_points`만 이용해서 cone candidate clustering
- color는 `unknown_color_cones`로 publish
- SLAM에 먼저 넣어보기

장점:

- headless에서도 가능
- perception baseline을 바로 진전시킬 수 있음

단점:

- cone color를 알 수 없음
- Formula Student track에서 blue/yellow 구분 성능은 아직 없음

추천:

먼저 `sim-gui`로 image publisher가 살아나는지 확인하고, 안 되면 LiDAR-only baseline을 병렬로 시작하는 것이 좋다.

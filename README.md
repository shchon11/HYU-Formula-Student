# Formula Student Simulator


## 0. Quick Start: 의존성 설치

이 저장소는 두 가지 환경에서 사용할 수 있습니다.

| 환경 | 상태 | 비고 |
| --- | --- | --- |
| Ubuntu 22.04 + ROS 2 Humble + Docker | 문서상의 기본 경로 | `scripts/hyu-docker`는 `eufs-sim:humble` Docker 이미지가 이미 있거나 별도로 제공된다는 전제입니다. |
| Ubuntu 20.04 + ROS 2 Galactic + native build | 2026-06-03 검증 완료 | `/opt/ros/galactic`과 conda 환경 `eufs` 기준으로 전체 빌드/테스트가 통과했습니다. |

### Humble 기준 시스템 패키지

ROS 2 Humble이 이미 설치되어 있다는 전제에서, 시뮬레이터에 필요한 패키지는 아래처럼 설치합니다.

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pandas \
  gazebo \
  ros-humble-gazebo-dev \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-plugins
```

### Galactic/Focal 기준 시스템 패키지

Ubuntu 20.04 + ROS 2 Galactic 환경에서는 Humble 패키지 대신 Galactic 패키지를 사용합니다.

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pandas \
  gazebo11 \
  ros-galactic-gazebo-dev \
  ros-galactic-gazebo-ros \
  ros-galactic-gazebo-plugins \
  ros-galactic-tf2-geometry-msgs \
  ros-galactic-libg2o \
  docker.io
```

`rosdep`을 처음 쓰는 PC라면 한 번만 초기화합니다.

```bash
sudo rosdep init
rosdep update
```

워크스페이스 의존성 확인 및 설치:

```bash
cd /home/dohyun/FS/HYU-Formula-Student
source /opt/ros/galactic/setup.bash

rosdep check --from-paths . --ignore-src --skip-keys ament_python
rosdep install --from-paths . --ignore-src -r -y --skip-keys ament_python
```

`ament_python`은 ROS 패키지의 build type으로 쓰이며, Ubuntu 20.04/Galactic rosdep key로는 해석되지 않을 수 있습니다. `python3-rosdep`과 ROS Galactic의 ament 패키지가 설치되어 있으면 위처럼 skip해도 워크스페이스 빌드에는 문제가 없습니다.

설치 확인:

```bash
dpkg -l | grep -E "python3-pandas|gazebo|ros-galactic-gazebo|ros-galactic-libg2o|docker.io"
gazebo --version
sudo docker info
```

## 1. 기본 구조

현재 검증된 워크스페이스는 아래 경로에 있습니다.

```bash
/home/dohyun/FS/HYU-Formula-Student
├── eufs_sim
├── eufs_msgs
├── eufs_graph_slam
├── eufs_perception_baseline
├── build
├── install
└── log
```

주요 패키지 역할은 다음과 같습니다.

| 패키지 | 역할 |
| --- | --- |
| `eufs_launcher` | 시뮬레이터 실행 GUI 및 launch 진입점 |
| `eufs_tracks` | Gazebo world, 트랙 launch 파일 |
| `eufs_racecar` | 차량 URDF, mesh, 차량 spawn launch |
| `eufs_plugins` | Gazebo 차량 모델, 콘 ground truth, bounding box plugin |
| `eufs_rqt` | 미션 제어 GUI, 수동 조향 GUI |
| `eufs_models` | 차량 동역학 모델 |
| `eufs_msgs` | EUFS 전용 ROS 2 message/service |
| `eufs_graph_slam` | `/cones`와 car state를 사용하는 graph SLAM baseline |
| `eufs_perception_baseline` | LiDAR-camera fusion 기반 `/cones` perception baseline |

## Perception + SLAM Quick Start

Perception baseline과 graph SLAM 통합 실행은 Docker 이미지가 준비된 Humble 환경에서는 아래 흐름을 사용합니다.

```bash
cd /path/to/HYU-FS-Sim

./scripts/hyu-docker setup-g2o
./scripts/hyu-docker build-ws
./scripts/hyu-docker sim-gui-bg
./scripts/hyu-docker fusion-bg
./scripts/hyu-docker slam-bg
./scripts/hyu-docker status
```

주의: `scripts/hyu-docker`는 기본 이미지 이름을 `eufs-sim:humble`로 사용합니다. 이 저장소에는 해당 이미지를 빌드하는 Dockerfile이 포함되어 있지 않으므로, 로컬에 이미지가 없으면 Docker 실행 단계는 먼저 실패합니다. 이 경우 이미지 제공 위치를 확인하거나 아래 native build 경로를 사용합니다.

Galactic native 환경에서는 아래처럼 g2o를 준비하고 빌드합니다.

```bash
cd /home/dohyun/FS/HYU-Formula-Student

./scripts/hyu-docker setup-g2o

conda run -n eufs bash -lc '
  source /opt/ros/galactic/setup.bash &&
  export EUFS_MASTER=/home/dohyun/FS/HYU-Formula-Student &&
  colcon build --symlink-install --cmake-args \
    -DG2O_VENDOR_SOURCE_DIR=/home/dohyun/FS/HYU-Formula-Student/g2o \
    -DG2O_USE_LOGGING=OFF
'
```

`-DG2O_USE_LOGGING=OFF`는 Ubuntu 20.04의 `spdlog` 헤더 버전과 최신 g2o 소스의 호환성 문제를 피하기 위한 옵션입니다.

정상 상태에서는 `/cones`를 `perception_baseline_node`가 publish하고,
`graph_slam`이 subscribe합니다.

자세한 설명:

- `eufs_perception_baseline/README.md`
- `eufs_perception_baseline/docs/perception_baseline_usage.md`
- `eufs_graph_slam/README.md`

## 2. 시스템 패키지 설치 확인

설치 확인:

```bash
dpkg -l | grep -E "python3-pandas|gazebo|ros-galactic-gazebo|ros-galactic-libg2o|docker.io"
gazebo --version
sudo docker info
```

ROS 의존성 확인:

```bash
cd /home/dohyun/FS/HYU-Formula-Student
source /opt/ros/galactic/setup.bash
rosdep check --from-paths . --ignore-src --skip-keys ament_python
```

부족한 의존성이 있으면:

```bash
rosdep install --from-paths . --ignore-src -r -y --skip-keys ament_python
```

## 3. 빌드

`eufs_tracks` launch 파일에서 `EUFS_MASTER` 환경변수를 사용하므로 워크스페이스 루트를 지정해둡니다.

```bash
cd /home/dohyun/FS/HYU-Formula-Student
export EUFS_MASTER=$PWD

source /opt/ros/galactic/setup.bash
colcon build --symlink-install --cmake-args \
  -DG2O_VENDOR_SOURCE_DIR=$PWD/g2o \
  -DG2O_USE_LOGGING=OFF
source install/setup.bash
```

`eufs_plugins`만 다시 빌드하고 싶으면:

```bash
colcon build --symlink-install --packages-select eufs_plugins
source install/setup.bash
```

## 4. 실행 방법

### 런처 GUI로 실행

```bash
cd /home/dohyun/FS/HYU-Formula-Student
export EUFS_MASTER=$PWD
source /opt/ros/galactic/setup.bash
source install/setup.bash

ros2 launch eufs_launcher eufs_launcher.launch.py
```

런처에서 트랙, 차량 모델, command mode, RViz, Gazebo GUI, simulated perception 사용 여부를 선택한 뒤 `Launch!`를 누릅니다.

### 런처 없이 바로 실행

```bash
ros2 launch eufs_launcher simulation.launch.py \
  track:=small_track \
  gazebo_gui:=true \
  rviz:=true \
  launch_group:=no_perception \
  commandMode:=acceleration
```

자주 쓰는 트랙:

```text
small_track
acceleration
skidpad
rectangle
comp_2021
rand
empty
peanut
garden_light
boa_constrictor
its_a_mess
hairpins_increasing_difficulty
```

주요 launch 옵션:

| 옵션 | 예시 | 설명 |
| --- | --- | --- |
| `track` | `small_track` | 실행할 트랙 |
| `gazebo_gui` | `true` | Gazebo 화면 표시 |
| `rviz` | `true` | RViz 실행 |
| `launch_group` | `no_perception` | simulated perception 사용 |
| `commandMode` | `acceleration` 또는 `velocity` | `/cmd` 해석 방식 |
| `publish_gt_tf` | `true` | ground truth TF 발행 |
| `pub_ground_truth` | `true` | ground truth topic 발행 |
| `robot_name` | `eufs` 또는 `ads-dv` | 차량 URDF 선택 |
| `vehicleModel` | `DynamicBicycle` 또는 `PointMass` | 차량 모델 |
| `vehicleModelConfig` | `configDry.yaml` | 차량 파라미터 preset |

`eufs_graph_slam`을 localization TF로 쓸 때는 `publish_gt_tf:=false`로 두고
GraphSLAM의 `map -> odom -> base_footprint` TF를 사용합니다. 이 모드의 perception
cross-time 보상은 `motion_compensation_frame:=odom`을 사용해야 합니다. GT TF와
GraphSLAM TF를 동시에 켜면 같은 차량 frame을 두 노드가 발행할 수 있습니다.

## 5. 기본 상호작용

시뮬레이터가 켜지면 보통 아래 세 가지 창을 사용합니다.

| 도구 | 역할 |
| --- | --- |
| Gazebo | 실제 월드, 차량, 콘 확인 |
| RViz | 토픽, TF, 차량 상태, 콘 시각화 |
| RQT | 미션 선택, 수동 조작, 리셋, EBS |

### RQT로 수동 주행

1. Mission Control에서 `Manual Drive`를 누릅니다.
2. Robot Steering GUI의 topic이 `/cmd`인지 확인합니다.
3. 세로 슬라이더로 가속도 또는 속도를 줍니다.
4. 가로 슬라이더로 조향각을 줍니다.
5. `Stop` 버튼으로 명령을 0으로 되돌립니다.

`commandMode:=acceleration`이면 세로 입력은 `acceleration`으로 해석됩니다.
`commandMode:=velocity`이면 세로 입력은 `speed`로 해석됩니다.

## 6. 터미널에서 차량 명령 보내기

먼저 수동 미션으로 전환합니다.

```bash
ros2 service call /ros_can/set_mission eufs_msgs/srv/SetCanState \
  "{ami_state: 21, as_state: 0}"
```

`acceleration` mode 예시:

```bash
ros2 topic pub -r 20 /cmd ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {steering_angle: 0.0, acceleration: 0.5, speed: 0.0}}"
```

`velocity` mode 예시:

```bash
ros2 topic pub -r 20 /cmd ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {steering_angle: 0.1, speed: 2.0, acceleration: 0.0}}"
```

주의: `/cmd`는 지속적으로 publish해야 합니다. 마지막 명령 이후 1초 이상 새 명령이 없으면 차량 plugin이 감속 명령을 넣습니다.

## 7. 주요 토픽

상태 머신:

```bash
ros2 topic echo /ros_can/state_str
ros2 topic echo /ros_can/state
```

차량 상태:

```bash
ros2 topic echo /ground_truth/state
ros2 topic echo /odometry_integration/car_state
ros2 topic echo /ground_truth/odom
ros2 topic echo /ros_can/wheel_speeds
ros2 topic echo /ground_truth/wheel_speeds
```

콘과 perception:

```bash
ros2 topic echo /ground_truth/track
ros2 topic echo /ground_truth/cones
ros2 topic echo /cones
ros2 topic echo /camera_0/cones
ros2 topic echo /camera_1/cones
```

Bounding box:

```bash
ros2 topic echo /ground_truth/bounding_boxes
ros2 topic echo /noisy_bounding_boxes
ros2 topic echo /custom_camera_info
```

센서:

```bash
ros2 topic echo /velodyne_points
ros2 topic echo /gps
ros2 topic echo /imu/data
ros2 topic echo /camera/imu/data
ros2 topic echo /sbg/magnetic
```

카메라 관련 토픽은 실행 모드에 따라 다릅니다.

`launch_group:=no_perception` 또는 런처의 `Use Simulated Perception` 체크 상태에서는 raw ZED 이미지 대신 추상화된 perception 결과를 봅니다.

```bash
ros2 topic echo /camera_0/cones
ros2 topic echo /camera_1/cones
ros2 topic echo /ground_truth/bounding_boxes
ros2 topic echo /noisy_bounding_boxes
ros2 topic echo /custom_camera_info
```

raw ZED 이미지가 필요하면 `Use Simulated Perception`을 끄거나 `launch_group:=default`로 실행합니다.

```bash
ros2 launch eufs_launcher simulation.launch.py \
  track:=small_track \
  gazebo_gui:=true \
  rviz:=true \
  launch_group:=default \
  commandMode:=acceleration
```

이때 확인할 카메라 토픽:

```bash
ros2 topic echo /zed/left/camera_info
ros2 topic echo /zed/right/camera_info
ros2 topic hz /zed/left/image_rect_color
ros2 topic hz /zed/right/image_rect_color
ros2 topic hz /zed/depth/image_raw
ros2 topic hz /zed/points
```

이미지는 `echo`보다 `rqt_image_view`로 보는 것이 편합니다.

```bash
ros2 run rqt_image_view rqt_image_view /zed/left/image_rect_color
```

## 8. 센서 Configuration 수정

센서의 장착 위치는 차량 URDF xacro에서 바꿉니다.

```bash
eufs_sim/eufs_racecar/robots/eufs/robot.urdf.xacro
eufs_sim/eufs_racecar/robots/ads-dv/robot.urdf.xacro
```

예를 들어 `eufs` 차량의 ZED 위치/자세는 아래 줄의 `origin`을 수정합니다.

```xml
<xacro:zed_camera parent="chassis" prefix="zed" active="$(arg simulate_perception)">
  <origin xyz="-0.08 0.0 0.76" rpy="0 0 0"/>
</xacro:zed_camera>
```

센서 고유 파라미터는 각 센서 xacro macro에서 수정하거나, macro 호출부에 인자로 넘깁니다.

| 센서 | 설정 파일 | 주요 값 |
| --- | --- | --- |
| IMU | `eufs_sim/eufs_sensors/urdf/imu.urdf.xacro` | `noise`, `update_rate`, `topic_prefix` |
| GPS | `eufs_sim/eufs_sensors/urdf/gps.urdf.xacro` | `update_rate`, position/velocity noise |
| LiDAR VLP-16R | `eufs_sim/eufs_sensors/urdf/VLP-16R.urdf.xacro` | `hz`, `lasers`, `samples`, `min_range`, `max_range`, `noise`, `min_angle`, `max_angle`, `topic` |
| ZED camera | `eufs_sim/eufs_sensors/urdf/zed.urdf.xacro` | `update_rate`, `horizontal_fov`, image `width/height`, clipping range, image noise, topic remapping |
| Magnetometer | `eufs_sim/eufs_sensors/urdf/magnetometer.urdf.xacro` | topic, noise |

LiDAR 예시:

```xml
<xacro:VLP-16R
  parent="chassis"
  name="velodyne"
  topic="/velodyne_points"
  hz="10"
  lasers="40"
  samples="350"
  min_range="0.2"
  max_range="100.0"
  noise="0.008"
  active="$(arg simulate_perception)">
  <origin xyz="-0.15 0.0 0.79" rpy="0 ${1*M_PI/180.0} 0"/>
</xacro:VLP-16R>
```

차량 동역학 파라미터는 센서가 아니라 vehicle model preset입니다.

```bash
eufs_sim/eufs_racecar/robots/eufs/configDry.yaml
eufs_sim/eufs_racecar/robots/eufs/configWet.yaml
eufs_sim/eufs_racecar/robots/ads-dv/configDry.yaml
eufs_sim/eufs_racecar/robots/ads-dv/configWet.yaml
```

simulated perception과 ground truth plugin의 거리/FOV/noise는 아래 파일에서 수정합니다.

```bash
eufs_sim/eufs_plugins/urdf/eufs_plugins.gazebo.xacro
```

예: `/camera_0/cones`, `/camera_1/cones`의 FOV/거리/noise는 `gz_camera_0_cones`, `gz_camera_1_cones` plugin 블록에서 조정합니다.

수정 후에는 관련 패키지를 다시 빌드하고 시뮬레이션을 재시작해야 합니다.

```bash
colcon build --symlink-install --packages-select eufs_sensors eufs_racecar eufs_plugins
source install/setup.bash
```

## 9. 주요 서비스

상태 머신 리셋:

```bash
ros2 service call /ros_can/reset std_srvs/srv/Trigger "{}"
```

차량 위치 리셋:

```bash
ros2 service call /ros_can/reset_vehicle_pos std_srvs/srv/Trigger "{}"
```

콘 위치 리셋:

```bash
ros2 service call /ros_can/reset_cone_pos std_srvs/srv/Trigger "{}"
```

비상정지:

```bash
ros2 service call /ros_can/ebs std_srvs/srv/Trigger "{}"
```

현재 command mode 확인:

```bash
ros2 service call /race_car_model/command_mode std_srvs/srv/Trigger "{}"
```

## 10. 상태 머신 흐름

기본 상태는 다음처럼 움직입니다.

```text
AS_OFF
  -> 미션 선택
AS_READY
  -> 약 5초 뒤
AS_DRIVING
  -> /ros_can/mission_completed true
AS_FINISHED
```

`AMI_MANUAL`은 수동 조작용입니다. 이 경우 `AS_DRIVING` 상태가 아니어도 차량 plugin의 `canDrive()` 조건을 통과합니다.

미션 번호:

| 값 | 의미 |
| --- | --- |
| `10` | `AMI_NOT_SELECTED` |
| `11` | `AMI_ACCELERATION` |
| `12` | `AMI_SKIDPAD` |
| `13` | `AMI_AUTOCROSS` |
| `14` | `AMI_TRACK_DRIVE` |
| `15` | `AMI_AUTONOMOUS_DEMO` |
| `20` | `AMI_JOYSTICK` |
| `21` | `AMI_MANUAL` |

## 11. Troubleshooting

### Gazebo plugin을 못 찾는 경우

`EUFS_MASTER`와 setup sourcing을 확인합니다.

```bash
cd /home/dohyun/FS/HYU-Formula-Student
export EUFS_MASTER=$PWD
source /opt/ros/galactic/setup.bash
source install/setup.bash
```

plugin library가 있는지도 확인합니다.

```bash
ls install/eufs_plugins/lib
```

### `eufs_msgs`를 못 찾는 경우

워크스페이스 전체를 빌드하고 setup을 다시 source합니다.

```bash
colcon build --symlink-install --cmake-args \
  -DG2O_VENDOR_SOURCE_DIR=$PWD/g2o \
  -DG2O_USE_LOGGING=OFF
source install/setup.bash
ros2 interface show eufs_msgs/msg/CanState
```

### Protobuf 충돌이 나는 경우

이 환경에는 `/usr/local`에 오래된 Protobuf가 있을 수 있습니다. Gazebo Classic은 Ubuntu Jammy의 system Protobuf와 맞춰 빌드되어 있으므로, `eufs_plugins/CMakeLists.txt`에서 system Protobuf를 우선 사용하도록 처리했습니다.

확인 명령:

```bash
which protoc
protoc --version
pkg-config --modversion protobuf
```

장기적으로는 `/usr/local`에 수동 설치된 오래된 Protobuf를 제거하거나 비활성화하는 것이 가장 깔끔합니다.

### RQT GUI에서 `setGeometry`, `setMaximum`, `setMinimum` float 에러가 나는 경우

ROS 2 Humble/Jammy의 PyQt는 일부 Qt 함수에 `float` 인자를 허용하지 않습니다. 이 레포에서는 런처와 조향 GUI에서 해당 값을 `int`로 변환하도록 패치했습니다.

수정 후 아래 패키지를 다시 빌드합니다.

```bash
colcon build --symlink-install --packages-select eufs_launcher eufs_rqt
source install/setup.bash
```

### `/cmd`를 보내도 차가 안 움직이는 경우

아래를 확인합니다.

```bash
ros2 topic echo /ros_can/state_str
ros2 service call /race_car_model/command_mode std_srvs/srv/Trigger "{}"
```

수동 조작이면 먼저 `AMI_MANUAL`로 바꿉니다.

```bash
ros2 service call /ros_can/set_mission eufs_msgs/srv/SetCanState \
  "{ami_state: 21, as_state: 0}"
```

그리고 `/cmd`를 1회가 아니라 지속적으로 publish합니다.

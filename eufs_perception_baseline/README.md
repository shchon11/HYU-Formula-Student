# eufs_perception_baseline

HYU Formula Student simulator와 graph SLAM을 연결하기 위한 ROS 2 perception
baseline package이다.

이 package의 핵심 역할은 perception 결과를 SLAM이 바로 받을 수 있는
`/cones` contract로 publish하는 것이다.

```text
camera / YOLO bbox / simulator bbox / LiDAR / TF
        |
        v
eufs_perception_baseline
        |
        v
/cones
  type: eufs_msgs/msg/ConeArrayWithCovariance
  frame: base_footprint
```

자세한 실행법, topic 확인법, 문제 해결은 아래 문서에 정리되어 있다.

- [Perception Baseline Usage](docs/perception_baseline_usage.md)
- [IIT Bombay Inspired Baseline Design](docs/iit_bombay_baseline_design.md)

## Modes

### LiDAR-Camera Fusion Baseline

IIT Bombay Racing Driverless 논문의 perception 구조 중 LiDAR-camera fusion
부분을 baseline 수준으로 구현한 모드이다.

Simulator bbox 입력:

```text
/noisy_bounding_boxes
+ /velodyne_points
+ /custom_camera_info
+ /tf
    -> LiDAR-camera fusion
    -> /cones
```

`/noisy_bounding_boxes`는 simulator가 제공하는 bbox topic이며, 지금은 실제
YOLO detector의 임시 대체 입력으로 사용한다. LiDAR cluster가 metric cone
position을 제공하고, bbox가 cone color/class를 제공한다.

### YOLOv8 Camera BBox Detector

실제 camera perception path는 YOLOv8 detector node를 fusion 앞단에 둔다.

```text
/zed/left/image_rect_color
    -> yolov8_bbox_node
    -> /yolo_bounding_boxes
    -> perception_baseline_node + /velodyne_points + /zed/left/camera_info + /tf
    -> /cones
```

기본 모델은 FSOCO로 fine-tuning한 YOLOv8n weight이다.
기본 `yolo_model_path`는
`/home/dohyun/FS/artifacts/yolov8/fsoco_yolov8n/weights/best.pt`이고,
`yolo_class_map`은 FSOCO class 이름을 EUFS cone color contract로 매핑한다.
특히 `large_orange_cone`은 `big_orange`로 매핑해야 fusion 이후
`big_orange_cones`에 들어간다.

## End-to-End Runbook: ROS Simulator + YOLOv8 + Fusion

Host workspace에서 Gazebo, RViz, ZED image, YOLOv8 bbox detector, LiDAR-camera
fusion을 한 번에 띄우는 기준 절차이다. 추가 상세 설명은
[`docs/perception_baseline_usage.md`](docs/perception_baseline_usage.md)에 있다.

### 1. Build and Source

새 terminal에서 workspace를 build하고 source한다.

```bash
cd /home/dohyun/FS/HYU-Formula-Student
source /home/dohyun/anaconda3/etc/profile.d/conda.sh
conda activate eufs
source /opt/ros/galactic/setup.bash

colcon build --packages-select eufs_launcher eufs_perception_baseline --symlink-install
source install/setup.bash
```

### 2. Preferred Launch: Simulator + RViz + YOLOv8 + Fusion

아래 한 명령이 현재 권장 실행법이다. `perception:=true`가 빠지면 RViz의
`Fusion Cones` display는 `/cones/viz`를 구독만 하고, fusion publisher는 뜨지
않는다.

```bash
cd /home/dohyun/FS/HYU-Formula-Student
source /home/dohyun/anaconda3/etc/profile.d/conda.sh
conda activate eufs
source /opt/ros/galactic/setup.bash
source install/setup.bash

export ROS_LOG_DIR=/tmp/eufs_ros_logs
export GAZEBO_LOG_PATH=/tmp/eufs_gazebo_logs
export QT_X11_NO_MITSHM=1

LD_PRELOAD=/lib/x86_64-linux-gnu/libffi.so.7 \
  ros2 launch eufs_launcher simulation.launch.py \
  track:=small_track \
  gazebo_gui:=true \
  rviz:=true \
  show_rqt_gui:=false \
  ros_localhost_only:=0 \
  publish_gt_tf:=true \
  launch_group:=default \
  perception:=true \
  perception_bbox_source:=yolov8 \
  perception_python_executable:=/home/dohyun/anaconda3/envs/eufs/bin/python3 \
  perception_publish_fusion_debug:=true \
  perception_publish_yolo_debug_image:=true
```

이 launch에서 실행되는 주요 path:

```text
/zed/left/image_rect_color
    -> yolov8_bbox_node
    -> /yolo_bounding_boxes
    -> perception_baseline_node + /velodyne_points + /zed/left/camera_info + /tf
    -> /cones
    -> /cones/viz
```

### 3. Alternative: Simulator First, Perception Later

시뮬레이터와 RViz만 먼저 띄우고 싶으면 `perception:=true` 없이 실행한다.

```bash
ros2 launch eufs_launcher simulation.launch.py \
  track:=small_track \
  gazebo_gui:=true \
  rviz:=true \
  show_rqt_gui:=false \
  ros_localhost_only:=0 \
  publish_gt_tf:=true \
  launch_group:=default
```

다른 terminal에서 YOLOv8 perception/fusion만 붙인다.

```bash
cd /home/dohyun/FS/HYU-Formula-Student
source /home/dohyun/anaconda3/etc/profile.d/conda.sh
conda activate eufs
source /opt/ros/galactic/setup.bash
source install/setup.bash

LD_PRELOAD=/lib/x86_64-linux-gnu/libffi.so.7 \
  ros2 launch eufs_perception_baseline perception_baseline.launch.py \
  bbox_source:=yolov8 \
  use_sim_time:=true \
  python_executable:=/home/dohyun/anaconda3/envs/eufs/bin/python3 \
  publish_fusion_debug:=true \
  publish_yolo_debug_image:=true
```

### 4. Runtime Checks

YOLO와 fusion이 실제로 떠 있는지 확인한다.

```bash
ros2 node list | grep -E 'yolo|perception'
ros2 topic list | grep -E 'zed/left|yolo|cones|fusion/debug|velodyne'
ros2 topic info /yolo_bounding_boxes
ros2 topic info /cones
ros2 topic info /cones/viz
```

Topic별 의미:

```text
Raw ZED image     -> /zed/left/image_rect_color
YOLO bbox         -> /yolo_bounding_boxes
YOLO bbox debug   -> /yolo_bounding_boxes/debug_image
Raw LiDAR         -> /velodyne_points
Fusion output     -> /cones
Fusion RViz marker-> /cones/viz
Fusion debug      -> /fusion/debug/*
```

RViz에서 기본적으로 확인할 display:

```text
zed raw image       /zed/left/image_rect_color
yolo bbox debug     /yolo_bounding_boxes/debug_image
PointCloud2         /velodyne_points
Fusion Cones        /cones/viz
Fusion debug topics /fusion/debug/*
```

### 5. Common Failure Points

- `DistributionNotFound: ultralytics`: ROS console script가 `/usr/bin/python3`
  shebang으로 실행되어 conda package를 못 보는 상태이다.
  `perception_python_executable:=/home/dohyun/anaconda3/envs/eufs/bin/python3`
  또는 `python_executable:=...`를 명시한다.
- `undefined symbol: ffi_type_pointer`: host Galactic + conda `eufs` 조합에서
  `cv_bridge`와 `libffi`가 충돌한 상태이다. 위 실행 명령처럼
  `LD_PRELOAD=/lib/x86_64-linux-gnu/libffi.so.7`를 붙인다.
- YOLO bbox는 보이는데 `/cones/viz`가 비어 있음: 먼저
  `/fusion/debug/rejections`의 `raw`, `roi`, `cl` 값을 확인한다.
  `raw=0`이면 LiDAR-camera TF/projection/calibration 문제 가능성이 크다.
- RViz display만 있고 publisher가 없음: `perception:=true`가 빠졌거나
  별도 `perception_baseline.launch.py`가 죽은 상태이다.

### 6. Active YOLO Contract

현재 기본 설정은 FSOCO fine-tuned weight와 `yolo_class_map`을 사용한다. 이
실행 순서는 ZED raw image가 YOLO node로 들어가고, fine-tuned YOLO bbox가 fusion
node로 전달되는지 확인하는 용도이다.

When `bbox_source:=yolov8`, the launch file switches fusion to the ZED-left
raw-image contract:

```text
image_topic: /zed/left/image_rect_color
yolo_image_topic: /zed/left/image_rect_color
yolo_camera_info_topic: /zed/left/camera_info
yolo_camera_frame: zed_left_camera_optical_frame
yolo_projection_model: pinhole
```

기본 launch parameter는 설치된 `config/perception_baseline.yaml`에서 읽는다.
Simulator bbox mode의 bbox topic은 `simulated_bbox_topic`으로 지정하고, 예전
`bbox_topic` launch argument는 호환 alias로만 남아 있다.

Dependency boundary:

- ROS/rosdep 의존성은 `package.xml`에 둔다.
- Ultralytics는 Galactic rosdep key가 없으므로 `setup.py`의
  `install_requires`가 authoritative PyPI runtime dependency이다.
- 현재 `eufs` conda 환경에는 `ultralytics==8.4.60`이 설치되어 있다.

### Oracle Adapter

SLAM integration 확인용 모드이다. 실제 perception을 거치지 않고 simulator가
주는 cone topic을 SLAM contract에 맞춰 다시 publish한다.

```text
/camera_0/cones -> /cones
```

이 모드는 graph SLAM 연결 확인에는 유용하지만, perception baseline 성능을
검증하는 모드는 아니다.

## Main Files

```text
eufs_perception_baseline/
  eufs_perception_baseline/
    perception_baseline_node.py
    yolov8_bbox_node.py
    yolov8_bbox_utils.py
  launch/
    perception_baseline.launch.py
  config/
    perception_baseline.yaml
  docs/
    perception_baseline_usage.md
    iit_bombay_baseline_design.md
```

## Core Logic

Fusion 구현은 `eufs_perception_baseline/perception_baseline_node.py`에 있다.

- `_try_publish_fusion()`: bbox, point cloud, camera info sync 확인
- `_run_lidar_camera_fusion()`: fusion pipeline 전체 orchestration
- `_extract_detections()`: bbox message를 내부 detection 구조로 변환
- `_pointcloud_to_xyz()`: `PointCloud2`를 xyz numpy array로 변환
- `_roi_mask()`: 차량 앞쪽 ROI와 간단한 ground filtering 적용
- `_cluster_cone_candidates()`: cone 크기 조건 기반 LiDAR clustering
- `_project_points()`: LiDAR point를 camera image plane에 projection
- `_associate_detections_to_clusters()`: bbox와 LiDAR cluster matching
- `_cluster_to_cone()`: SLAM용 `ConeWithCovariance` 생성
- `_oracle_callback()`: simulator cone topic을 `/cones` contract로 변환

YOLOv8 bbox detector 구현은
`eufs_perception_baseline/yolov8_bbox_node.py`와
`eufs_perception_baseline/yolov8_bbox_utils.py`에 있다.

- `yolov8_bbox_node`: ROS Image를 OpenCV image로 변환하고 YOLOv8 inference 실행
- `detections_from_ultralytics_results()`: YOLO 결과를 pixel bbox detection으로 변환
- output: `eufs_msgs/msg/BoundingBoxes`
- timestamp policy: `header`와 `image_header` 모두 원본 `Image.header` 사용

## Quick Run

Workspace root 기준:

```bash
cd /path/to/HYU-FS-Sim

./scripts/hyu-docker setup-g2o
./scripts/hyu-docker build-ws
./scripts/hyu-docker sim-gui-bg
./scripts/hyu-docker fusion-bg
./scripts/hyu-docker slam-bg
./scripts/hyu-docker status
```

정상 기준:

```text
/cones publisher: perception_baseline_node
/cones subscriber: graph_slam
/graph_slam/map topic exists
```

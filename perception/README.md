# hyu_perception

HYU Formula Student simulator와 graph SLAM을 연결하기 위한 ROS 2 perception
baseline package이다.

이 package의 핵심 역할은 perception 결과를 SLAM이 바로 받을 수 있는
`/perception/cones` contract로 publish하는 것이다.

```text
camera / YOLO bbox / simulator bbox / LiDAR / TF
        |
        v
hyu_perception
        |
        v
/perception/cones
  type: hyu_msgs/msg/ConeArrayWithCovariance
  frame: base_footprint
```

자세한 실행법, topic 확인법, 문제 해결은 아래 문서에 정리되어 있다.

- [Perception Baseline Usage](docs/perception_baseline_usage.md)
- [IIT Bombay Inspired Baseline Design](docs/iit_bombay_baseline_design.md)
- [Current Perception Pipeline](docs/current_perception_pipeline.md)

## Modes

### IIT Bombay-Inspired Three-Tier Baseline

IIT Bombay Racing Driverless 논문의 우선순위 perception 구조를 다음과 같이
구현한다. 한 detection이 상위 tier에서 할당되면 하위 tier에서는 제외되므로
동일 bbox가 중복 cone을 만들지 않는다.

1. bbox를 지지하는 LiDAR point가 있으면 LiDAR-camera association을 사용한다.
2. LiDAR가 없고 upright/full-visible 조건을 만족하면 normalized bbox-height
   monocular depth를 사용한다.
3. LiDAR가 없고 clipped, wide/fallen 등 mono 조건을 만족하지 않으면 ReKTNet
   7-keypoint -> robust PnP -> right ROI projection 뒤 양안 slender crop에서
   SIFT disparity를 구한다.

Mono와 stereo 모두 유효한 calibration, timestamp-matched image, depth bound를
통과할 때만 cone을 생성한다. 하나의 standard-cone mono curve는
`blue/yellow/orange`에만 적용하고, 크기가 다른 `big_orange`와 `unknown`은
stereo로 보낸다. 실패한 추정치를 임의 거리로 대체하지 않는다.

3-tier 입력(`bbox_source:=yolov8` 권장):

```text
/noisy_bounding_boxes or /perception/bounding_boxes
+ /velodyne_points
+ /zed/left|right/image_rect_color
+ /zed/left|right/camera_info
+ timestamped /tf
    -> prioritized LiDAR -> mono -> stereo routing
    -> /perception/cones
```

Simulator bbox mode에서는 right-camera 기준 `/noisy_bounding_boxes`와
`/custom_camera_info`를 사용한다. 이 source에는 left bbox가 없으므로 stereo는
강제로 끄고 LiDAR/mono만 사용한다. YOLO mode는 FSOCO fine-tuned YOLOv8n이
left-image bbox와 color/class를 제공하고, 각 depth tier가 metric position을
제공한다.

Ground 제거는 수평 법선 제한이 있는 RANSAC을 사용한다. RANSAC이 평면을
신뢰할 수 없으면 node는 기존 `ground_min_z` fixed-height filter로 돌아간다.
그 뒤 cone geometry와 bbox association 조건이 계속 적용된다.

YOLO 추론과 fusion 계산은 각각 single-slot latest-only worker에서 실행한다.
ROS subscription, `/perception/cones`, bbox/debug publish는 `SingleThreadedExecutor`에 남고,
worker 완료 결과는 `GuardCondition`을 통해 executor로 되돌아온다. `/clock`
rollback 전에 generation을 증가시키므로 이전 rosbag epoch에서 끝난 추론은
새 epoch의 bbox, cone, debug topic을 publish하거나 새 버퍼를 소비할 수 없다.
또한 완료 결과는 acquisition timestamp가 현재 ROS clock보다
`output_commit_settle_sec` 이상 뒤처진 뒤에만 commit한다. 이 settle fence는
동시에 ready가 된 `/clock`과 worker completion의 executor 처리 순서가 바뀌어도
rollback 직전 결과가 먼저 publish되는 경쟁 조건을 막는다.

### YOLOv8 Camera BBox Detector

실제 camera perception path는 YOLOv8 detector node를 fusion 앞단에 둔다.

```text
/zed/left/image_rect_color
    -> yolov8_bbox_node
    -> /perception/bounding_boxes
    -> perception_baseline_node
       + /velodyne_points
       + /zed/left|right/image_rect_color
       + /zed/left|right/camera_info
       + timestamped /tf
    -> /perception/cones
```

기본 모델은 FSOCO로 fine-tuning한 YOLOv8n weight이다. 이는 사용자가 승인한
논문 detector 대체이며, 논문의 2D detector를 그대로 재현한 것은 아니다.
기본 `yolo_model_path`는
`/home/dohyun/FS/artifacts/yolov8/fsoco_yolov8n/weights/best.pt`이고,
`yolo_class_map`은 FSOCO class 이름을 EUFS cone color contract로 매핑한다.
특히 `large_orange_cone`은 `big_orange`로 매핑해야 fusion 이후
`big_orange_cones`에 들어간다.

## End-to-End Runbook: ROS Simulator + YOLOv8 + Fusion

Host workspace에서 Gazebo, RViz, ZED image, YOLOv8 bbox detector, three-tier
perception을 한 번에 띄우는 기준 절차이다. 추가 상세 설명은
[`docs/perception_baseline_usage.md`](docs/perception_baseline_usage.md)에 있다.

### 1. Build and Source

새 terminal에서 workspace를 build하고 source한다.

```bash
cd /home/dohyun/FS/HYU-Formula-Student
source /home/dohyun/anaconda3/etc/profile.d/conda.sh
conda activate eufs
source /opt/ros/galactic/setup.bash

colcon build --packages-up-to eufs_launcher hyu_perception --symlink-install
source install/setup.bash
```

### 2. Preferred Launch: Simulator + RViz + YOLOv8 + Fusion

아래 한 명령이 현재 권장 실행법이다. `perception:=true`가 빠지면 RViz의
`Fusion Cones` display는 `/perception/debug/cones_viz`를 구독만 하고, fusion publisher는 뜨지
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
    -> /perception/bounding_boxes
    -> perception_baseline_node
       + /velodyne_points
       + /zed/left|right/image_rect_color
       + /zed/left|right/camera_info
       + timestamped /tf
    -> /perception/cones
    -> /perception/debug/cones_viz
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

ros2 launch hyu_perception perception_baseline.launch.py \
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
ros2 topic list | grep -E 'zed/(left|right)|yolo|cones|fusion/debug|velodyne'
ros2 topic info /perception/bounding_boxes
ros2 topic info /perception/cones
ros2 topic info /perception/debug/cones_viz
```

Topic별 의미:

```text
Raw ZED image     -> /zed/left/image_rect_color
Right ZED image   -> /zed/right/image_rect_color
YOLO bbox         -> /perception/bounding_boxes
YOLO bbox debug   -> /yolo_bounding_boxes/debug_image
Raw LiDAR         -> /velodyne_points
Fusion output     -> /perception/cones
Fusion RViz marker-> /perception/debug/cones_viz
Fusion debug      -> /fusion/debug/*
```

RViz에서 기본적으로 확인할 display:

```text
zed raw image       /zed/left/image_rect_color
yolo bbox debug     /yolo_bounding_boxes/debug_image
PointCloud2         /velodyne_points
Fusion Cones        /perception/debug/cones_viz
Fusion debug topics /fusion/debug/*
```

### 5. Common Failure Points

- `DistributionNotFound: ultralytics`: ROS console script가 `/usr/bin/python3`
  shebang으로 실행되어 conda package를 못 보는 상태이다.
  `perception_python_executable:=/home/dohyun/anaconda3/envs/eufs/bin/python3`
  또는 `python_executable:=...`를 명시한다.
- YOLO bbox는 보이는데 `/perception/debug/cones_viz`가 비어 있음: 먼저
  `/fusion/debug/rejections`의 `raw`, `roi`, `cl` 값을 확인한다.
  `raw=0`이면 LiDAR-camera TF/projection/calibration 문제 가능성이 크다.
- LiDAR가 없는 detection에서 mono/stereo cone이 나오지 않음: 이는 fail-closed
  동작일 수 있다. left/right image timestamp, `CameraInfo`, rectification,
  baseline/TF, bbox visibility 조건, depth bound와 SIFT match 수를 확인한다.
- Stereo fallback이 항상 비어 있음: runtime OpenCV에 `SIFT_create`가 없거나
  robust match가 부족하면 이 tier는 ORB로 조용히 변경되지 않고 해당 추정을
  거부한다.
- RViz display만 있고 publisher가 없음: `perception:=true`가 빠졌거나
  별도 `perception_baseline.launch.py`가 죽은 상태이다.

### 6. Active YOLO Contract

현재 기본 설정은 FSOCO fine-tuned weight와 `yolo_class_map`을 사용한다. 이
실행 순서는 ZED raw image가 YOLO node로 들어가고, fine-tuned YOLO bbox가
three-tier node로 전달되는지 확인하는 용도이다.

When `bbox_source:=yolov8`, the launch file switches fusion to the ZED-left
raw-image contract:

```text
image_topic: /zed/left/image_rect_color
yolo_image_topic: /zed/left/image_rect_color
yolo_camera_info_topic: /zed/left/camera_info
yolo_camera_frame: zed_left_camera_optical_frame
yolo_projection_model: pinhole
right_image_topic: /zed/right/image_rect_color
right_camera_info_topic: /zed/right/camera_info
right_camera_frame: zed_right_camera_optical_frame
```

Mono depth는 논문의 curve를 다음처럼 사용한다.

```text
h_n = bbox_height_px / image_height_px
D = 0.498 * h_n^(-0.954)
```

여기서 `h`를 image-height normalized 값으로 해석한 것은 이 repository의
명시적 가정이다. Camera intrinsics, mounting height/pitch, crop/resize 또는 cone
geometry가 바뀌면 coefficient와 exponent를 실제 데이터로 다시 calibration해야
한다.

Stereo tier의 source는 `stereo_rektnet_pnp_sift`이다. MIT/Delft 공개 ReKTNet
구조와 checkpoint contract를 복원하고, PnP로 projection한 right ROI 안에서만
SIFT disparity를 계산한다. IIT의 추가 1,000-image weight와 정확한 7x3 template은
공개되지 않았으므로 compatible checkpoint와 실측 template/calibration은 별도
필수다. 누락/불일치 weight는 bbox-only SIFT로 우회하지 않고 startup error가 된다.

기본 launch parameter는 설치된 `config/perception_baseline.yaml`에서 읽는다.
Simulator bbox mode의 bbox topic은 `simulated_bbox_topic`으로 지정하고, 예전
`bbox_topic` launch argument는 호환 alias로만 남아 있다. 기본 primary image도
simulator bbox와 같은 `/zed/right/image_rect_color`이며, YOLO mode가 이를 left
image로 override한다.

Dependency boundary:

- ROS/rosdep 의존성은 `package.xml`에 둔다.
- Ultralytics는 Galactic rosdep key가 없으므로 `setup.py`의
  `install_requires`가 authoritative PyPI runtime dependency이다.
- 현재 `eufs` conda 환경에는 `ultralytics==8.4.60`이 설치되어 있다.

### Oracle Adapter

SLAM integration 확인용 모드이다. 실제 perception을 거치지 않고 simulator가
주는 cone topic을 SLAM contract에 맞춰 다시 publish한다.

```text
/camera_0/cones -> /perception/cones
```

이 모드는 graph SLAM 연결 확인에는 유용하지만, perception baseline 성능을
검증하는 모드는 아니다.

## Main Files

```text
hyu_perception/
  hyu_perception/
    perception_baseline_node.py
    fusion_core.py
    sync_buffer.py
    ros_image_utils.py
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

ROS orchestration은 `hyu_perception/perception_baseline_node.py`, 순수
geometry는 `hyu_perception/fusion_core.py`, ReKTNet/PnP/right-ROI
propagation은 `hyu_perception/rektnet.py`에 있다.

- `_try_publish_fusion()`: bbox/cloud queue front 중 늦은 timestamp를 anchor로
  삼고 반대 stream의 predecessor/successor 중 nearest를 one-to-one으로 매칭한
  뒤 bbox timestamp에 맞는 joint left/right stereo pair를 선택
- `_run_lidar_camera_fusion()`: LiDAR -> mono -> stereo priority orchestration
- `_extract_detections()`: bbox message를 내부 detection 구조로 변환
- `_pointcloud_to_xyz()`: `PointCloud2`를 xyz numpy array로 변환
- `_spatial_roi_mask()`: 차량 앞쪽 ROI와 self mask 적용
- `_non_ground_mask()`: tilt-constrained RANSAC ground filtering 적용
- `_cluster_cone_candidates()`: cone 크기 조건 기반 LiDAR clustering
- `_project_points()`: LiDAR point를 camera image plane에 projection
- `_associate_detections_to_clusters()`: bbox-supported LiDAR point matching
- `_associate_visual_detections()`: unmatched detection의 mono/ReKTNet-PnP-SIFT
  routing과 optional horizontal-border recovery
- `_cluster_to_cone()`: SLAM용 `ConeWithCovariance` 생성
- `_oracle_callback()`: simulator cone topic을 `/perception/cones` contract로 변환

`sync_buffer.py`는 integer-nanosecond ordered buffer를 제공한다. `/perception/cones`는 bbox
timestamp를 canonical time으로 사용한다. LiDAR point는 `map` TF history를 통해
cloud time에서 bbox-time output/camera frame으로 보상하고, visual TF도 bbox
timestamp로 조회한다. Fusion 성공 후에만 pair를 소비하므로
out-of-order input을 처리하고 top-level TF 또는 required left-calibration
failure를 재시도할 수 있다. 개별 mono/stereo estimate rejection은 해당
detection만 fail closed한다.

`map` 기본값은 simulator ground-truth TF 모드용이다. GraphSLAM이 localization
TF를 소유하는 실행에서는 SLAM correction jump와 순수 ego motion을 분리하도록
`motion_compensation_frame:=odom`을 사용한다. `scripts/hyu-docker fusion*` 명령은
이 override를 자동으로 전달한다.

Cloud/bbox queue와 image history는 별도 크기를 사용한다. 기본 64-frame image
history는 60 Hz에서 약 1.07초를 보존한다. 센서 timestamp는 stream별로 엄격히
증가해야 하며 duplicate/역순 메시지는 해당 메시지만 폐기한다. Image, cloud,
bbox acquisition stamp, oracle stamp는 nanosecond 정규화 전에 canonical nonzero
ROS `Time`(`sec >= 0`, `0 <= nanosec < 1e9`)인지 검사한다. 설정 threshold 이상의
ROS clock backward jump 또는 time source 변경은 모든 sync buffer를 비우고 TF listener를
재생성한다. 정상적인 listener 해제 경로에서는 기존 TF buffer를 clear 후 재사용해
one-shot publisher가 종료된 뒤에도 cached `/tf_static`을 보존하고, volatile `/tf`
subscription queue는 폐기한다. 정상적인 양의 `/clock` tick은 reset하지 않는다.
Rollback에서는 jump 직전 clock high watermark까지 replay guard를 둔다. 그 구간에도
DDS callback 순서 차이를 흡수하도록 설정된 `max_future_stamp_lead_sec`(기본 90 ms)
이내의 clock 선행 stamp를 허용하되, 기존 watermark 이상인 stamp는 배제한다. Clock이
기존 watermark에 도달하면 upper fence가 해제된다. Replay 도중 다시 rollback되면
inner/outer watermark를 모두 보존하고 각 지점에 도달할 때 순서대로 해제한다.
정상 epoch의 90~300 ms 선행 입력과 simulated time의 첫 nonzero `/clock` 이전 입력은
고정 크기 buffer에 보관하고 clock이 따라온 뒤 처리한다.
YOLO bbox보다 늦게 도착하는 right image는 `stereo_pair_wait_sec` 동안만 기다리며,
right stream이 target을 지나거나 timeout이면 기존 fail-closed 경로로 진행한다.

Source별 covariance base는 dense LiDAR, sparse LiDAR, monocular,
`stereo_rektnet_pnp_sift`에 대해 각각 `fused_*`, `sparse_*`, `monocular_*`, `stereo_*`
parameter를 사용한다. 여기에 range-dependent term과 `min_variance` clamp가
적용된다. 기본값은 초기값일 뿐이며 bag/vehicle 측정으로 재조정해야 한다.

YOLOv8 bbox detector 구현은
`hyu_perception/yolov8_bbox_node.py`와
`hyu_perception/yolov8_bbox_utils.py`에 있다.

- `yolov8_bbox_node`: ROS Image를 OpenCV image로 변환하고 YOLOv8 inference 실행
- `detections_from_ultralytics_results()`: YOLO 결과를 pixel bbox detection으로 변환
- output: `hyu_msgs/msg/BoundingBoxes`
- timestamp policy: `header`와 `image_header` 모두 원본 `Image.header` 사용

Image 변환은 `CvBridge` 대신 `ros_image_utils.py`의 명시적
`bgr8`/`rgb8`/`mono8` converter를 사용한다. 지원하지 않는 encoding이나 잘못된
stride/buffer는 조용히 잘못 해석하지 않고 해당 frame을 거부한다.

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
/perception/cones publisher: perception_baseline_node
/perception/cones subscriber: graph_slam
/localization/map topic exists
```

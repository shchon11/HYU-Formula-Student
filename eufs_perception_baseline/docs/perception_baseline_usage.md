# Perception Baseline Usage

이 문서는 HYU Formula Student workspace에서 `eufs_perception_baseline`을
실행하고, ROS topic으로 결과를 확인하는 방법을 정리한다.

기준 workspace:

```bash
/path/to/HYU-FS-Sim
```

## 1. Package Purpose

`eufs_perception_baseline`은 perception module의 baseline package이다.

현재 package는 SLAM 출력 형식을 고정하고, IIT Bombay 논문의 우선순위를 따른
three-tier baseline으로 실제 cone observation을 `/cones`에 publish한다. Tier 1은
LiDAR-camera association, Tier 2는 normalized bbox-height monocular depth, Tier 3는
paper-inspired SIFT stereo이다.

SLAM output contract:

```text
topic: /cones
type: eufs_msgs/msg/ConeArrayWithCovariance
frame_id: base_footprint
```

`/cones` message는 색상별 cone array를 따로 가진다.

```text
blue_cones
yellow_cones
orange_cones
big_orange_cones
unknown_color_cones
```

각 cone은 차량 기준 좌표계인 `base_footprint`에서 표현된다.

```text
point.x: car forward direction
point.y: car left direction
point.z: 0.0
covariance: [xx, xy, yx, yy]
```

## 2. Current Architecture

현재 package에는 세 가지 실행 모드가 있다.

### 2.1 Simulator BBox + Three-Tier Baseline

실제 perception baseline으로 사용할 모드이다.

```text
/noisy_bounding_boxes
+ /velodyne_points
+ /custom_camera_info
+ /zed/right/image_rect_color
+ /tf
        |
        v
LiDAR -> monocular fallback
(simulator bbox stereo is disabled)
        |
        v
/cones
```

역할:

- `/noisy_bounding_boxes`: simulator가 주는 bbox와 cone color/class
- `/velodyne_points`: cone 위치를 계산하기 위한 LiDAR point cloud
- `/custom_camera_info`: camera intrinsic
- `/zed/right/image_rect_color`: simulator bbox와 같은 right-camera image contract
- `/tf`: LiDAR, camera, `base_footprint` 사이의 extrinsic transform
- `/cones`: graph SLAM으로 넘길 최종 cone observation

주의:

- `/noisy_bounding_boxes`는 simulator bbox source이며, YOLO mode에서는
  `bbox_source:=yolov8`과 `/yolo_bounding_boxes`를 사용한다.
- YOLO weight나 camera model을 바꾸면 bbox topic, projection 관련 parameter,
  confidence threshold를 다시 맞춰야 한다.

### 2.2 YOLOv8 Camera BBox Detector + Three-Tier Fusion

실제 camera perception을 붙이는 모드이다. YOLOv8은 별도 detector node로
동작하고, 기존 fusion node는 bbox topic만 바꿔서 그대로 사용한다.

```text
/zed/left/image_rect_color
        |
        v
yolov8_bbox_node
        |
        v
/yolo_bounding_boxes
+ /velodyne_points
+ /zed/left/camera_info
+ /zed/right/image_rect_color
+ /zed/right/camera_info
+ /tf
        |
        v
perception_baseline_node
        |
        v
/cones
```

기본 모델:

```text
yolo_model_path: /home/dohyun/FS/artifacts/yolov8/fsoco_yolov8n/weights/best.pt
yolo_class_map: blue_cone:blue,yellow_cone:yellow,orange_cone:orange,large_orange_cone:big_orange,unknown_cone:unknown
```

현재 기본값은 FSOCO fine-tuned YOLOv8n weight이다. `large_orange_cone`은
자동 추론만 쓰면 `orange`로 정규화될 수 있으므로, `yolo_class_map`에서
명시적으로 `big_orange`로 매핑한다. Offline 실행에서도 첫 실행 download에
의존하지 않고 absolute path weight를 사용한다.

```bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libffi.so.7 \
  ros2 launch eufs_perception_baseline perception_baseline.launch.py \
  bbox_source:=yolov8 \
  yolo_image_topic:=/zed/left/image_rect_color \
  yolo_camera_info_topic:=/zed/left/camera_info \
  yolo_camera_frame:=zed_left_camera_optical_frame \
  yolo_projection_model:=pinhole
```

Dependency boundary:

- `package.xml`은 ROS/rosdep dependency와 `python3-pip` runtime을 명시한다.
- `ultralytics`는 Galactic rosdep key가 없으므로 `setup.py`의
  `install_requires=["ultralytics==8.4.60"]`가 authoritative PyPI
  dependency contract이다.
- conda/Docker runtime은 ROS workspace 실행 전에 이 PyPI dependency가 설치되어
  있어야 한다.

### 2.3 Oracle Adapter

SLAM 연결 확인용 모드이다.

```text
/camera_0/cones -> /cones
```

역할:

- simulator가 제공하는 cone observation을 그대로 SLAM contract에 맞춰 재발행
- graph SLAM이 `/cones`를 제대로 받는지 빠르게 확인

주의:

- perception baseline 성능 검증 모드가 아니다.
- `fusion-bg`와 `adapter-bg`를 동시에 실행하면 둘 다 `/cones`를 publish하므로
  결과가 섞인다.

## 3. Important Files

```text
eufs_perception_baseline/
  eufs_perception_baseline/
    perception_baseline_node.py
    yolov8_bbox_node.py
    yolov8_bbox_utils.py
  config/
    perception_baseline.yaml
  launch/
    perception_baseline.launch.py
  docs/
    perception_baseline_usage.md
    iit_bombay_baseline_design.md
```

파일 역할:

- `perception_baseline_node.py`: 실제 node 구현
- `yolov8_bbox_node.py`: YOLOv8 image detector node
- `yolov8_bbox_utils.py`: YOLO result -> EUFS bbox 변환 helper
- `perception_baseline.yaml`: 기본 parameter 정리
- `perception_baseline.launch.py`: ROS launch entrypoint
- `docs/iit_bombay_baseline_design.md`: IIT Bombay 논문 기반 설계 정리
- `docs/perception_baseline_usage.md`: 실행법과 topic 확인법

## 4. Main Logic Location

`perception_baseline_node.py`의 주요 함수:

- `_try_publish_fusion()`
  - bbox/cloud queue의 오래된 front 중 더 늦은 timestamp를 anchor로 삼는다.
  - 반대 stream이 anchor까지 도달하면 predecessor/successor 중 가까운 sample을
    one-shot으로 선택한다. 같은 timestamp는 즉시 처리하고 빠른 stream에서
    건너뛴 sample은 폐기한다.

- `_run_lidar_camera_fusion()`
  - three-tier pipeline 전체를 실행한다.
  - detection 추출, point cloud 변환, TF 변환, ROI filtering, clustering,
    bbox-cluster/sparse association, mono/stereo fallback, `/cones` message 생성을
    담당한다.

- `_extract_detections()`
  - `eufs_msgs/msg/BoundingBoxes`를 내부 `Detection` 구조로 바꾼다.
  - cone color도 여기서 normalize한다.

- `_pointcloud_to_xyz()`
  - `sensor_msgs/msg/PointCloud2`에서 `(x, y, z)` point들을 읽어 numpy array로
    바꾼다.

- `_spatial_roi_mask()` / `_non_ground_mask()`
  - `base_footprint` 기준 spatial ROI와 vehicle self-mask를 적용한다.
  - tilt-constrained RANSAC으로 ground를 제거하고, plane이 성립하지 않으면
    `ground_min_z` contingency를 적용한다.

- `_cluster_cone_candidates()`
  - LiDAR point들을 XY plane에서 clustering한다.
  - cone으로 볼 수 있는 height, width 조건을 만족하는 cluster만 남긴다.

- `_project_points()`
  - LiDAR cluster point를 camera image plane으로 projection한다.
  - 현재 HYU simulator bbox convention 때문에 기본값은
    `projection_model: eufs_bbox`이다.

- `_associate_detections_to_clusters()`
  - projection된 LiDAR point가 bbox 안에 들어가는지 보고 bbox와 cluster를
    matching한다.

- `_associate_sparse_detections()`
  - global cluster가 없는 detection에 bbox-guided sparse LiDAR support를 적용한다.

- `_associate_visual_detections()`
  - LiDAR support가 전혀 없는 detection만 good cone은 monocular, bad cone은
    SIFT stereo 한 경로로 보낸다. calibration/TF/match가 부족하면 fail-closed한다.

- `_cluster_to_cone()`
  - matching된 LiDAR cluster centroid를 `ConeWithCovariance`로 변환한다.

- `_append_cone_by_color()`
  - detection color에 따라 `blue_cones`, `yellow_cones`, `orange_cones`,
    `big_orange_cones`, `unknown_color_cones` 중 하나에 넣는다.

- `_oracle_callback()`
  - Oracle Adapter 모드에서 simulator cone message를 `/cones` contract로 정리한다.

YOLOv8 detector의 주요 정책:

- 입력 image topic 기본값: `/zed/left/image_rect_color`
- bbox output topic 기본값: `/yolo_bounding_boxes`
- model 기본값:
  `/home/dohyun/FS/artifacts/yolov8/fsoco_yolov8n/weights/best.pt`
- bbox coordinate type: `BoundingBox.PIXEL`
- `BoundingBoxes.header`와 `BoundingBoxes.image_header`는 둘 다 원본
  `Image.header`를 사용한다.
- `unknown_color_policy` 기본값은 `unknown`이다. 색상 class를 알 수 없는
  detection은 fusion 후 `unknown_color_cones`로 들어간다.

## 5. Default Topics

Fusion mode default:

```text
image_topic: /zed/right/image_rect_color
right_image_topic: /zed/right/image_rect_color
pointcloud_topic: /velodyne_points
bbox_topic: /noisy_bounding_boxes
camera_info_topic: /custom_camera_info
camera_frame: zed_right_camera_optical_frame
output_cones_topic: /cones
output_frame: base_footprint
```

YOLO mode default:

```text
bbox_source: yolov8
simulated_bbox_topic: /noisy_bounding_boxes
yolo_bbox_topic: /yolo_bounding_boxes
yolo_image_topic: /zed/left/image_rect_color
yolo_camera_info_topic: /zed/left/camera_info
yolo_camera_frame: zed_left_camera_optical_frame
yolo_projection_model: pinhole
yolo_model_path: /home/dohyun/FS/artifacts/yolov8/fsoco_yolov8n/weights/best.pt
yolo_confidence_threshold: 0.25
yolo_iou_threshold: 0.45
yolo_imgsz: 640
yolo_max_det: 100
python_executable: ""  # installed console-script shebang 사용
yolo_class_map: blue_cone:blue,yellow_cone:yellow,orange_cone:orange,large_orange_cone:big_orange,unknown_cone:unknown
yolo_unknown_color_policy: unknown
```

Graph SLAM input:

```text
/cones
/odometry_integration/car_state
```

Graph SLAM output:

```text
/graph_slam/map
/graph_slam/odom
/graph_slam/path
/graph_slam/markers
/tf
```

## 6. Build

Workspace root에서 실행한다.

```bash
cd /path/to/HYU-FS-Sim

./scripts/hyu-docker setup-g2o
./scripts/hyu-docker build-ws
```

정상 예시:

```text
Summary: 10 packages finished
```

`setup-g2o`는 graph SLAM dependency인 `g2o`를 repository root의 `g2o/`에
clone하고 `COLCON_IGNORE`를 생성한다. `g2o/`는 `.gitignore`에 포함되어
GitHub에는 올라가지 않는다.

## 7. Run With Simulator

GUI simulator를 권장한다. 현재 fusion baseline은 bbox와 camera 관련 topic을
사용하므로 headless보다 GUI mode가 확인하기 쉽다.

```bash
cd /path/to/HYU-FS-Sim

./scripts/hyu-docker setup-g2o
./scripts/hyu-docker sim-gui-bg
./scripts/hyu-docker fusion-bg
./scripts/hyu-docker slam-bg
./scripts/hyu-docker status
```

각 명령의 의미:

- `sim-gui-bg`: Gazebo GUI, RViz, simulator를 background container로 실행
- `fusion-bg`: 기존 simulator container 안에서 perception fusion node 실행
- `slam-bg`: 기존 simulator container 안에서 graph SLAM node 실행
- `status`: ROS node와 `/cones` 연결 상태 확인

정상 상태:

```text
/perception_baseline_node
/graph_slam

/cones
Publisher count: 1
Node name: perception_baseline_node

Subscription count: 1
Node name: graph_slam
```

## 8. Clean Restart

이미 `hyu_eufs_sim` container가 실행 중이면 `sim-gui-bg`가 다음처럼 실패할 수 있다.

```text
Conflict. The container name "/hyu_eufs_sim" is already in use
```

이 경우 새로 깨끗하게 시작하려면:

```bash
cd /path/to/HYU-FS-Sim

./scripts/hyu-docker stop
./scripts/hyu-docker sim-gui-bg
./scripts/hyu-docker fusion-bg
./scripts/hyu-docker slam-bg
./scripts/hyu-docker status
```

시뮬레이터는 유지하고 perception/SLAM만 다시 켜고 싶으면, 현재 실행 중인 node를
확인한 뒤 중복 실행을 피한다.

```bash
./scripts/hyu-docker status
```

`/cones` publisher가 2개 이상이면 `fusion-bg`와 `adapter-bg`가 동시에 떠 있거나,
이전 node가 남아있는 상태이다.

## 9. ROS Topic Check

Container shell에 들어간다.

```bash
cd /path/to/HYU-FS-Sim
./scripts/hyu-docker shell
```

Container 안에서 workspace를 source한다.

```bash
source /workspace/install/setup.bash
```

### 9.1 Topic List

```bash
ros2 topic list | sort
```

중요 topic:

```text
/noisy_bounding_boxes
/velodyne_points
/custom_camera_info
/tf
/cones
/graph_slam/map
```

### 9.2 `/cones` Connection

```bash
ros2 topic info -v /cones
```

정상 기준:

```text
Type: eufs_msgs/msg/ConeArrayWithCovariance
Publisher count: 1
Subscription count: 1
```

publisher는 `perception_baseline_node`, subscriber는 `graph_slam`이어야 한다.

### 9.3 Fusion Output Message

```bash
ros2 topic echo --once /cones
```

정상 message 예시:

```text
header:
  frame_id: base_footprint
blue_cones: []
yellow_cones:
- point:
    x: 6.05
    y: -1.84
    z: 0.0
  covariance:
  - 0.043
  - 0.0
  - 0.0
  - 0.043
orange_cones: []
big_orange_cones: []
unknown_color_cones: []
```

확인할 것:

- `frame_id`가 `base_footprint`인지
- cone array 중 하나 이상에 값이 들어오는지
- `point.x`, `point.y`가 차량 기준으로 말이 되는지
- covariance가 4개 값으로 들어오는지

### 9.4 Publish Rate

```bash
ros2 topic hz /cones
```

### 9.5 SLAM Map Output

```bash
ros2 topic echo --once /graph_slam/map
```

정상이라면 `/cones`로 들어간 cone이 `map` frame의 landmark로 변환되어 나온다.

```text
header:
  frame_id: map
yellow_cones:
- point:
    x: ...
    y: ...
```

즉 아래 연결이 살아있는 것이다.

```text
fusion baseline -> /cones -> graph_slam -> /graph_slam/map
```

### 9.6 Input Topic Check

BBox:

```bash
ros2 topic echo --once /noisy_bounding_boxes
```

LiDAR:

```bash
ros2 topic info /velodyne_points
ros2 topic hz /velodyne_points
```

Camera info:

```bash
ros2 topic echo --once /custom_camera_info
```

TF:

```bash
ros2 run tf2_ros tf2_echo base_footprint velodyne
ros2 run tf2_ros tf2_echo zed_right_camera_optical_frame velodyne
```

## 10. Logs

Host에서 확인:

```bash
cd /path/to/HYU-FS-Sim
./scripts/hyu-docker logs
```

Fusion mode가 정상으로 켜졌다면 `perception_fusion` log에 다음 문구가 나온다.

```text
LiDAR-camera fusion enabled. BBox class/color is fused with LiDAR clusters.
```

Oracle adapter mode는 다음 문구가 나온다.

```text
Oracle adapter enabled
```

## 11. Oracle Adapter Run

SLAM wiring만 빠르게 확인하고 싶을 때 사용한다.

```bash
cd /path/to/HYU-FS-Sim

./scripts/hyu-docker sim-gui-bg
./scripts/hyu-docker adapter-bg
./scripts/hyu-docker slam-bg
./scripts/hyu-docker status
```

주의:

```text
adapter-bg and fusion-bg must not run at the same time.
```

둘 다 `/cones`를 publish하기 때문이다.

## 12. Direct ROS Launch

`hyu-docker` helper 없이 container 안에서 직접 실행할 수도 있다.

아래 fusion 예시는 GraphSLAM이 `map -> odom -> base_footprint` TF를 소유하는
통합 모드이므로 `motion_compensation_frame:=odom`을 사용한다. Simulator의
ground-truth TF(`publish_gt_tf:=true`)만 사용할 때는 이를 `map`으로 바꾼다.
두 TF publisher를 동시에 켜면 안 된다.

```bash
source /workspace/install/setup.bash
```

Fusion:

```bash
ros2 launch eufs_perception_baseline perception_baseline.launch.py \
  output_cones_topic:=/cones \
  output_frame:=base_footprint \
  motion_compensation_frame:=odom \
  fusion_enabled:=true \
  publish_empty_on_sync:=false
```

YOLOv8 + fusion:

```bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libffi.so.7 \
  ros2 launch eufs_perception_baseline perception_baseline.launch.py \
  bbox_source:=yolov8 \
  use_sim_time:=true \
  python_executable:=python3 \
  output_cones_topic:=/cones \
  output_frame:=base_footprint \
  motion_compensation_frame:=odom \
  fusion_enabled:=true \
  publish_empty_on_sync:=false \
  publish_fusion_debug:=true \
  publish_yolo_debug_image:=true
```

Oracle adapter:

```bash
ros2 launch eufs_perception_baseline perception_baseline.launch.py \
  oracle_cones_topic:=/camera_0/cones \
  output_cones_topic:=/cones \
  output_frame:=base_footprint \
  fusion_enabled:=false \
  publish_empty_on_sync:=false
```

Graph SLAM:

```bash
ros2 launch eufs_graph_slam graph_slam.launch.py \
  publish_tf:=true \
  use_sim_time:=true
```

## 13. Important Parameters

기본 parameter는 `config/perception_baseline.yaml`에 정리되어 있고, launch 파일은
이 YAML을 기본값 source로 읽는다. Launch argument는 runtime override이며, 예전
`bbox_topic` launch argument는 `simulated_bbox_topic`의 호환 alias로만 남아 있다.

Input/output:

```yaml
image_topic: /zed/right/image_rect_color
right_image_topic: /zed/right/image_rect_color
right_camera_info_topic: /zed/right/camera_info
pointcloud_topic: /velodyne_points
bbox_topic: /noisy_bounding_boxes
camera_info_topic: /custom_camera_info
camera_frame: zed_right_camera_optical_frame
projection_model: eufs_bbox
output_cones_topic: /cones
output_frame: base_footprint
motion_compensation_frame: map
fusion_enabled: true
```

YOLO launch overrides used only when `bbox_source:=yolov8`:

```yaml
yolo_image_topic: /zed/left/image_rect_color
yolo_camera_info_topic: /zed/left/camera_info
yolo_camera_frame: zed_left_camera_optical_frame
yolo_projection_model: pinhole
yolo_sync_tolerance_sec: 0.15
python_executable: ""
```

Sync:

```yaml
sync_tolerance_sec: 0.15
image_sync_tolerance_sec: 0.05
sync_queue_size: 12
image_sync_queue_size: 64
timestamp_reset_threshold_sec: 0.1
max_future_stamp_lead_sec: 0.09
publish_empty_on_sync: false
```

ROI:

```yaml
roi_min_x: 0.5
roi_max_x: 30.0
roi_abs_y: 15.0
roi_min_z: -0.2
roi_max_z: 1.5
ground_min_z: 0.05
ground_ransac_enabled: true
ground_ransac_distance_threshold: 0.03
ground_ransac_max_tilt_degrees: 20.0
```

Clustering:

```yaml
cluster_eps: 0.35
cluster_min_points: 3
cluster_min_height: 0.02
cluster_max_height: 0.80
cluster_max_width: 0.90
```

Association:

```yaml
min_bbox_probability: 0.0
min_projected_points: 1
min_project_depth: 0.2
```

Visual fallback:

```yaml
monocular_fallback_enabled: true
stereo_fallback_enabled: true  # YOLO-left mode only; simulated mode forces false
stereo_min_matches: 3
```

Covariance:

```yaml
fused_variance_x: 0.04
fused_variance_y: 0.04
range_variance_scale: 0.0005
min_variance: 0.0001
```

## 14. Current Limitations

- `bbox_source:=simulated` 기본값은 simulator의 `/noisy_bounding_boxes`를 사용한다.
- `bbox_source:=yolov8`은 YOLOv8 detector node를 함께 실행하고
  `/yolo_bounding_boxes`를 fusion bbox 입력으로 사용한다.
- 기본 YOLO weight는 FSOCO fine-tuned model이고, `yolo_class_map`은 FSOCO class를
  EUFS cone color contract로 매핑한다. Offline target에서도 first-run download에
  의존하지 않고 absolute path weight를 사용한다.
- ROS image 변환은 `cv_bridge` 없이 지원 encoding(`bgr8`, `rgb8`, `mono8`)을
  직접 처리한다. Image 변환, inference 또는 result 변환 예외 frame은 bbox를
  publish하지 않고 drop한다. 정상 inference의 실제 0 detections만 stamped empty
  bbox를 publish하므로, detector 장애가 SLAM landmark miss로 오인되지 않는다.
- YOLO bbox는 inference 완료 시각이 아니라 원본 image timestamp를 유지한다.
  기본 `yolo_sync_tolerance_sec`는 `0.15`이며, bbox/cloud queue front 중 늦은
  timestamp를 anchor로 삼아 반대 stream의 predecessor/첫 이후 frame 중 가까운
  것을 one-to-one으로 선택한다.
- `/cones` header는 bbox acquisition timestamp를 사용한다. LiDAR point는
  `motion_compensation_frame`(기본 `map`)의 TF history로 cloud time에서 bbox-time
  `base_footprint`로 변환하므로 한 array 안의 LiDAR/vision cone이 같은 시각과
  좌표계 계약을 갖는다. 해당 동적 TF history가 없으면 pair를 소비하지 않고
  재시도한다. Simulator ground-truth TF 모드는 `map`을 사용하고, GraphSLAM이
  `map -> odom -> base_footprint`를 소유하는 통합 모드는 localization correction
  jump를 ego-motion 보상에 섞지 않도록 `motion_compensation_frame:=odom`을 쓴다.
- `image_sync_queue_size: 64`는 60 Hz 기준 약 1.07초의 좌/우 raw image 이력을
  유지해 측정된 약 0.53초 YOLO 지연을 흡수한다. 1280x720 BGR 두 stream이 모두
  가득 차면 payload만 약 340 MiB이므로, 실제 detector latency와 frame rate를
  측정한 뒤 필요한 margin을 유지하는 범위에서 조정한다.
- 같은 sensor stream에서 duplicate 또는 역순 도착한 timestamp는 해당 메시지만
  폐기하며, timestamp는 stream별로 엄격히 증가해야 한다. Image/right image,
  cloud, 선택된 bbox acquisition stamp, oracle stamp는 integer nanosecond로
  바꾸기 전에 canonical nonzero ROS `Time`(`sec >= 0`,
  `0 <= nanosec < 1e9`)인지 검사하며 malformed/zero stamp는 폐기한다.
  기본 0.1초 이상의 ROS clock backward jump 또는 ROS time source 변경만 모든
  sync buffer를 비우고 TF listener를 재생성해 simulation clock의 새 epoch를
  시작한다. 정상적인 unregister 경로에서는 기존 TF buffer를 clear 후 재사용하므로
  publisher가 이미 종료된 one-shot `/tf_static` cache는 유지되고, 이전 `/tf`
  subscription의 volatile queue는 폐기된다. Unregister 실패 시에는 아직 살아 있는
  이전 subscription의 재오염을 막기 위해 새 buffer로 격리한다.
  Galactic이 정상적인 양의 clock tick에도 jump callback을 호출할 수 있어 callback
  delta를 다시 검사하며, `max_future_stamp_lead_sec`는 rollback 기준보다 작아야 한다.
  Rollback 직전 clock high watermark를 overflow-safe하게 복원해 replay guard 끝으로
  저장한다. Clock이 그 지점에 다시 도달하기 전에도 DDS callback 순서 차이를 위해
  설정된 `max_future_stamp_lead_sec`(기본 90 ms)까지 선행 stamp를 허용하지만, 기존
  watermark 이상은 exclusive upper fence로 폐기한다. 새 epoch 시작보다 과거인 stamp도
  폐기한다. Replay 중 다시 rollback되면 active watermark들을 함께 보존하고 inner부터
  순서대로 해제한다. Timestamp만으로 허용된 replay window 안에서 두 epoch의 payload를 완전히
  구별할 수는 없지만, 이 lower/upper fence와 buffer/subscription reset이 이전 epoch의
  watermark 오염 범위를 제한한다.
- `projection_model: eufs_bbox`는 HYU simulator bbox plugin의 projection convention에
  맞춘 simulated bbox 설정이다. `bbox_source:=yolov8`에서는 실제 ZED image
  projection을 위해 launch가 `yolo_projection_model:=pinhole`을 fusion node에
  전달한다.
- `/custom_camera_info`의 `header.frame_id`가 비어있을 수 있어, 기본
  simulated bbox `camera_frame` parameter를 `zed_right_camera_optical_frame`으로
  둔다. YOLO mode에서는 `/zed/left/camera_info`와
  `zed_left_camera_optical_frame`을 사용한다.
- Simulator bbox는 right-camera 기준이므로 left-to-right SIFT search를 시작할
  left bbox가 없다. 이 모드의 stereo tier는 강제로 꺼지고, YOLO-left mode에서만
  rectified right image와 함께 활성화된다.
- 논문의 exact Tier 3인 RekTNet 7-keypoint/PnP는 weights와 3D keypoint template이
  repository에 없어 재현하지 못한다. 현재 `stereo_sift`는 명시적으로
  paper-inspired 구현이며 synthetic disparity 검증 범위까지만 보장한다.
- `stereo_fallback_enabled:=true`인데 현재 OpenCV에 `cv2.SIFT_create`가 없으면
  node는 해당 tier를 조용히 비활성화하지 않고 실행 시 actionable error로 종료한다.
  project conda 환경을 사용하거나 SIFT-capable OpenCV를 준비해야 한다.
- 실제 camera detector weight로 바꾸면 bbox clipping, confidence threshold,
  `yolo_class_map`을 다시 조정해야 한다.
- Headless simulator에서는 camera rendering topic이 충분히 나오지 않을 수 있으므로
  fusion 확인은 GUI mode를 권장한다.
- RViz에서 `eufs_rviz_plugins` display plugin 관련 error가 나올 수 있지만,
  `/cones`와 `/graph_slam/map` topic 자체가 publish되면 ROS pipeline은 동작 중이다.

## 15. Troubleshooting

### Container name conflict

증상:

```text
Conflict. The container name "/hyu_eufs_sim" is already in use
```

확인:

```bash
docker ps -a --filter name=hyu_eufs_sim
```

해결:

```bash
./scripts/hyu-docker stop
./scripts/hyu-docker sim-gui-bg
```

### `/cones` publisher가 2개 이상

증상:

```text
Publisher count: 2
```

원인:

- `fusion-bg`와 `adapter-bg`가 동시에 실행됨
- 이전 perception node가 남아있음

해결:

```bash
./scripts/hyu-docker stop
./scripts/hyu-docker sim-gui-bg
./scripts/hyu-docker fusion-bg
./scripts/hyu-docker slam-bg
```

### `/cones`가 비어 있음

확인할 topic:

```bash
ros2 topic echo --once /noisy_bounding_boxes
ros2 topic info /velodyne_points
ros2 topic hz /velodyne_points
ros2 topic echo --once /custom_camera_info
ros2 run tf2_ros tf2_echo zed_right_camera_optical_frame velodyne
```

YOLO mode에서는 아래도 같이 확인한다:

```bash
ros2 topic hz /zed/left/image_rect_color
ros2 topic echo --once /zed/left/camera_info
ros2 topic echo --once /yolo_bounding_boxes
ros2 run tf2_ros tf2_echo zed_left_camera_optical_frame velodyne
```

가능한 원인:

- bbox, LiDAR, camera info timestamp 차이가 너무 큼
- TF가 없음
- ROI나 clustering parameter가 너무 강함
- 현재 시야에 bbox와 LiDAR가 동시에 잡히는 cone이 없음
- mono/stereo fallback이면 bbox visibility, synchronized left/right image,
  CameraInfo, rectification, SIFT match 수와 depth bound가 유효하지 않음

### `/graph_slam/map`이 비어 있음

확인:

```bash
ros2 topic info -v /cones
ros2 topic echo --once /cones
ros2 topic echo --once /odometry_integration/car_state
ros2 topic echo --once /graph_slam/map
```

가능한 원인:

- graph SLAM이 실행되지 않음
- `/cones`에 cone이 없음
- `/odometry_integration/car_state`가 없음
- `/cones` publisher가 여러 개라 입력이 섞임

## 16. Recommended Development Flow

1. `adapter-bg`로 SLAM wiring 확인
2. `fusion-bg`로 three-tier perception output 확인
3. `/cones`와 `/graph_slam/map`을 동시에 확인
4. 차량을 천천히 움직이며 detection 개수와 map 안정성 확인
5. `bbox_source:=yolov8`으로 YOLO bbox path를 켜고 `/yolo_bounding_boxes` 확인
6. cone fine-tuned weight, `yolo_class_map`, projection, ROI, clustering,
   covariance parameter tuning

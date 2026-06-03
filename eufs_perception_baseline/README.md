# eufs_perception_baseline

HYU Formula Student simulator와 graph SLAM을 연결하기 위한 ROS 2 perception
baseline package이다.

이 package의 핵심 역할은 perception 결과를 SLAM이 바로 받을 수 있는
`/cones` contract로 publish하는 것이다.

```text
camera / bbox / LiDAR / TF
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

현재 입력:

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

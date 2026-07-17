# YOLO-LiDAR Fusion Debug Scenarios

이 문서는 YOLO bbox는 보이지만 fusion cone이 적게 나오는 상황을 하나씩 분리해서 확인하기 위한 절차이다.
코드를 수정하기 전에 아래 순서대로 관찰값을 모으면, ROI 문제인지, LiDAR-camera projection 문제인지,
cluster/sparse association 문제인지 나눌 수 있다.

## 0. Baseline 실행

### 전체 simulation + perception

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

### perception만 따로 실행

시뮬레이터가 이미 떠 있으면 다른 터미널에서 perception만 실행한다.

```bash
cd /home/dohyun/FS/HYU-Formula-Student
source /home/dohyun/anaconda3/etc/profile.d/conda.sh
conda activate eufs
source /opt/ros/galactic/setup.bash
source install/setup.bash

LD_PRELOAD=/lib/x86_64-linux-gnu/libffi.so.7 \
  ros2 launch hyu_perception perception_baseline.launch.py \
  bbox_source:=yolov8 \
  use_sim_time:=true \
  python_executable:=/home/dohyun/anaconda3/envs/eufs/bin/python3 \
  publish_fusion_debug:=true \
  publish_yolo_debug_image:=true
```

## 1. Scenario A: graph가 정상인지 확인

### 실행

```bash
ros2 node list | grep -E 'yolo|perception'
ros2 topic list | grep -E 'zed/left|yolo|cones|fusion/debug|velodyne'
ros2 topic info /perception/bounding_boxes
ros2 topic info /perception/cones
ros2 topic info /perception/debug/cones_viz
```

### PASS

- `yolov8_bbox_node`와 `perception_baseline_node`가 보인다.
- `/perception/bounding_boxes`, `/velodyne_points`, `/zed/left/camera_info`, `/perception/cones`, `/perception/debug/cones_viz`,
  `/fusion/debug/rejections`가 보인다.
- `/perception/bounding_boxes`와 `/perception/cones`에 publisher/subscriber가 있다.

### FAIL이면

- perception node가 없으면 launch 인자 `perception:=true` 또는 별도 perception launch를 먼저 고친다.
- `/fusion/debug/*`가 없으면 `publish_fusion_debug:=true`가 적용되지 않은 것이다.
- `/perception/bounding_boxes` publisher가 없으면 YOLO node 실행 또는 Python executable 문제가 먼저다.

## 2. Scenario B: YOLO bbox가 충분히 나오는지 확인

### 실행

```bash
ros2 topic hz /perception/bounding_boxes
ros2 topic echo --once /perception/bounding_boxes
```

RViz에서는 `/yolo_bounding_boxes/debug_image`를 image display로 확인한다.

### PASS

- bbox 메시지가 주기적으로 나온다.
- bbox 좌표가 0 크기가 아니고, probability가 YOLO threshold 이상이다.
- debug image에서 cone bbox가 실제 cone 위치와 대략 맞는다.

### FAIL이면

- fusion 튜닝 전에 YOLO 입력 이미지, model path, confidence threshold를 먼저 확인한다.
- bbox가 cone보다 계속 옆으로 밀리면 fusion 파라미터가 아니라 camera/image contract 문제다.

## 3. Scenario C: LiDAR raw support가 bbox 안에 들어오는지 확인

### 실행

```bash
ros2 topic echo /fusion/debug/rejections
```

RViz에서는 `/velodyne_points`와 `/fusion/debug/bbox_support`를 같이 켠다.

### 판정

`/fusion/debug/rejections` marker text에서 `raw=... roi=... cl=...`를 본다.

### PASS

- 거절된 bbox 중 상당수가 `raw>0`이다.

### FAIL 패턴

- `raw=0`이 대부분이면 cluster 문제가 아니다.
- LiDAR point를 camera frame으로 projection했을 때 bbox 안에 들어오는 점이 없다는 뜻이다.

### 다음 액션

- `camera_frame`이 YOLO 모드에서 `zed_left_camera_optical_frame`인지 확인한다.
- `projection_model`이 YOLO 모드에서 `pinhole`인지 확인한다.
- `/zed/left/camera_info`와 `/zed/left/image_rect_color`가 같은 camera contract인지 확인한다.
- 이 단계에서 `cluster_min_points`나 `sparse_*`를 바꿔도 효과가 작다.

## 4. Scenario D: ROI/self-mask/ground filter가 지우는지 확인

### 실행

```bash
ros2 topic echo /fusion/debug/rejections
```

RViz에서는 `/fusion/debug/roi_points`를 켜고, raw `/velodyne_points`와 비교한다.

### PASS

- `raw>0`인 bbox에서 `roi>0`도 같이 나온다.

### FAIL 패턴

- `raw>0, roi=0`이 많으면 bbox 안의 LiDAR support가 ROI, self-mask, ground filter에서 제거된다.

### 다음 액션

첫 번째로 확인할 값:

```text
ground_min_z: 0.05
roi_min_z: -0.2
roi_max_z: 1.5
self_mask_enabled: true
self_mask_min_x: -1.5
self_mask_max_x: 1.8
self_mask_abs_y: 0.8
```

추천 실험 순서:

1. `self_mask_enabled:=false`로 한 번만 실행해서 near cone이 살아나는지 본다.
2. 살아나면 self mask가 너무 넓은 것이다. self mask를 줄이는 쪽이 맞다.
3. 안 살아나면 `ground_min_z`를 낮춰서 낮은 cone support가 제거되는지 확인한다.

예시 실행:

```bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libffi.so.7 \
  ros2 launch hyu_perception perception_baseline.launch.py \
  bbox_source:=yolov8 \
  use_sim_time:=true \
  python_executable:=/home/dohyun/anaconda3/envs/eufs/bin/python3 \
  publish_fusion_debug:=true \
  publish_yolo_debug_image:=true \
  self_mask_enabled:=false
```

## 5. Scenario E: cluster가 실패하고 sparse가 받아야 하는 상황인지 확인

### 실행

```bash
ros2 topic echo /fusion/debug/rejections
ros2 topic echo --once /perception/cones
```

RViz에서는 아래를 같이 켠다.

```text
/fusion/debug/cluster_candidates
/fusion/debug/sparse_support_points
/perception/debug/cones_viz
```

### PASS

- `roi>0`인 bbox가 있고, `/fusion/debug/sparse_support_points` 또는 `/perception/debug/cones_viz`가 같이 증가한다.
- 이 경우 sparse association이 LiDAR sparse support를 받아내는 중이다.

### FAIL 패턴

- `raw>0, roi>0, cl=0`이 많은데 `/perception/debug/cones_viz`가 거의 늘지 않는다.
- 이 경우 ROI까지는 살아남았지만 cluster 후보가 실패했고, sparse 조건도 충분히 받아주지 못하는 상태다.

### 다음 액션

이 케이스가 지금 의심하는 핵심이다. cluster보다 sparse를 먼저 완화한다.

추천 실험 1:

```bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libffi.so.7 \
  ros2 launch hyu_perception perception_baseline.launch.py \
  bbox_source:=yolov8 \
  use_sim_time:=true \
  python_executable:=/home/dohyun/anaconda3/envs/eufs/bin/python3 \
  publish_fusion_debug:=true \
  publish_yolo_debug_image:=true \
  sparse_near_min_points:=3 \
  sparse_mid_min_points:=2 \
  sparse_far_min_points:=2
```

추천 실험 2:

```bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libffi.so.7 \
  ros2 launch hyu_perception perception_baseline.launch.py \
  bbox_source:=yolov8 \
  use_sim_time:=true \
  python_executable:=/home/dohyun/anaconda3/envs/eufs/bin/python3 \
  publish_fusion_debug:=true \
  publish_yolo_debug_image:=true \
  sparse_bbox_margin_px:=8.0 \
  sparse_bbox_margin_ratio:=0.25
```

추천 실험 3:

```bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libffi.so.7 \
  ros2 launch hyu_perception perception_baseline.launch.py \
  bbox_source:=yolov8 \
  use_sim_time:=true \
  python_executable:=/home/dohyun/anaconda3/envs/eufs/bin/python3 \
  publish_fusion_debug:=true \
  publish_yolo_debug_image:=true \
  sparse_near_min_points:=3 \
  sparse_mid_min_points:=2 \
  sparse_bbox_margin_px:=8.0 \
  sparse_bbox_margin_ratio:=0.25
```

## 6. Scenario F: cluster 파라미터 완화가 필요한지 확인

이 단계는 Scenario E를 먼저 해본 뒤에만 한다. `cluster_min_points`는 이미 3이라 낮은 편이고,
무작정 낮추면 false positive가 늘 수 있다.

### 실행

현재 `perception_baseline.launch.py`는 `cluster_eps`, `cluster_min_points`를 launch argument로
노출하지 않는다. 따라서 이 단계에서는 먼저 관찰만 한다.

```bash
ros2 topic echo /fusion/debug/rejections
ros2 topic echo --once /fusion/debug/cluster_candidates
```

RViz에서는 `/fusion/debug/cluster_candidates`, `/fusion/debug/sparse_support_points`,
`/perception/debug/cones_viz`를 같이 켠다.

### PASS

- `/fusion/debug/cluster_candidates`가 cone 위치에 늘어난다.
- `/perception/debug/cones_viz`가 늘고, 엉뚱한 위치의 cone이 크게 늘지 않는다.

### FAIL이면

- false positive가 늘거나 cone 위치가 튀면 cluster 완화는 되돌린다.
- sparse 쪽으로 해결하는 것이 더 안전하다.
- `raw>0, roi>0, cl=0`이 계속 반복되는데 sparse 완화로도 부족하면, 그때 `cluster_eps`와
  `cluster_min_points`를 launch에서 조정 가능하게 노출하는 코드 변경을 검토한다.

## 7. Scenario G: 최종 후보값 비교

각 실험은 같은 track, 같은 시작 위치, 비슷한 주행 속도로 비교한다.

기록할 값:

```text
run name:
launch override:
visible YOLO bbox count:
/perception/cones count:
rejections dominant pattern: raw=0 / roi=0 / cl=0 / assigned_sparse / assigned_cluster
RViz false positive:
RViz missed obvious cone:
notes:
```

최종 선택 기준:

1. YOLO bbox 대비 `/perception/debug/cones_viz`가 가장 많이 살아난다.
2. 엉뚱한 cone false positive가 늘지 않는다.
3. `assigned_sparse`가 늘어도 cone 위치가 안정적이다.
4. `cluster_min_points:=2` 없이 해결되면 그 설정을 우선한다.

## 추천 진행 순서 요약

```text
A graph 확인
-> B YOLO bbox 확인
-> C raw support 확인
-> D ROI/self-mask 확인
-> E sparse 완화 실험
-> F cluster 완화 실험
-> G 최종 후보값 비교
```

가장 먼저 볼 핵심 지표는 `/fusion/debug/rejections`의 `raw`, `roi`, `cl`이다.

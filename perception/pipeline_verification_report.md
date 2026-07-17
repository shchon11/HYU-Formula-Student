# hyu_perception Pipeline Verification Report

> **Historical report (superseded 2026-07-10).** 이 문서는 수정 전 상태의 실패
> 증거를 보존한다. 현재 구현은 `sensor_msgs_py`가 없을 때 package-local
> `point_cloud2_compat`를 사용하며, 최신 실행 계약과 검증 범위는
> `README.md`, `docs/iit_bombay_baseline_design.md`,
> `docs/perception_baseline_usage.md`를 따른다. 아래 결론을 현재 상태 판정으로
> 사용하지 않는다.

검증 일시: 2026-06-03 16:08 KST
재검증 일시: 2026-06-03 16:26 KST (`conda` environment: `eufs`)
대상 경로: `/home/dohyun/FS/HYU-Formula-Student/hyu_perception`

## 결론

`conda activate eufs` 기준으로 다시 확인해도, 현재 이 환경에서는 `hyu_perception` 파이프라인이 문제없이 동작한다고 볼 수 없다.

다만 이전 검증의 Python 환경 해석은 정정한다. `eufs` conda 환경은 Python 3.8.20이라 ROS Galactic의 `rclpy` import는 가능하다. 하지만 실제 노드 실행은 여전히 import 단계에서 실패한다. 남는 실패 지점은 `hyu_perception/perception_baseline_node.py`의 `sensor_msgs_py` import이다.

또한 루트 README와 `scripts/hyu-docker`는 ROS 2 Humble Docker 환경을 기준으로 하지만, 현재 호스트에서는 `docker` 명령이 없어 의도된 simulator + fusion + SLAM end-to-end 검증을 수행할 수 없었다. 현재 호스트에서 source 가능한 ROS는 Galactic이고, 이번 재검증은 `conda` environment `eufs` + `/opt/ros/galactic/setup.bash` 조합으로 수행했다.

## 검증 범위

확인한 파이프라인:

```text
/noisy_bounding_boxes
+ /velodyne_points
+ /custom_camera_info
+ /tf
    -> perception_baseline_node
    -> /cones
```

확인한 파일:

- `hyu_perception/perception_baseline_node.py`
- `launch/perception_baseline.launch.py`
- `config/perception_baseline.yaml`
- `package.xml`
- `docs/perception_baseline_usage.md`
- `../scripts/hyu-docker`
- `../README.md`

## 실행한 검증과 결과

### 0. conda `eufs` 환경 확인

명령:

```bash
conda env list
conda run -n eufs python -V
conda run -n eufs bash -lc 'source /opt/ros/galactic/setup.bash && python -c "import rclpy; print(rclpy.__file__)"'
conda run -n eufs bash -lc 'source /opt/ros/galactic/setup.bash && python -c "import sensor_msgs_py; print(sensor_msgs_py.__file__)"'
```

결과:

```text
eufs                   /home/dohyun/anaconda3/envs/eufs
Python 3.8.20
/opt/ros/galactic/lib/python3.8/site-packages/rclpy/__init__.py
ModuleNotFoundError: No module named 'sensor_msgs_py'
```

판정: `eufs` conda 환경은 ROS Galactic Python 3.8과 맞아 `rclpy`는 import된다. 하지만 `sensor_msgs_py`는 이 환경에서도 없다.

### 1. 패키지 선택 빌드

명령:

```bash
conda run -n eufs bash -lc \
  'source /opt/ros/galactic/setup.bash &&
   colcon build --symlink-install --packages-select hyu_perception'
```

결과:

```text
Finished <<< hyu_perception [0.71s]
Summary: 1 package finished [0.81s]
```

판정: 통과. `setuptools`의 `tests_require` warning과 `Could not connect: Operation not permitted` stderr가 있었지만 빌드는 exit code 0으로 완료됐다.

### 2. ROS package 등록 확인

명령:

```bash
conda run -n eufs bash -lc \
  'source /opt/ros/galactic/setup.bash &&
   source install/setup.bash &&
   ros2 pkg prefix hyu_perception &&
   ros2 pkg executables hyu_perception'
```

결과:

```text
/home/dohyun/FS/HYU-Formula-Student/install/hyu_perception
hyu_perception perception_baseline_node
```

판정: 통과.

### 3. launch argument 파싱

기본 ROS log 경로(`/home/dohyun/.ros/log`)는 현재 파일시스템에서 쓸 수 없어 `ROS_LOG_DIR`를 workspace 안으로 지정했다.

명령:

```bash
conda run -n eufs bash -lc \
  'source /opt/ros/galactic/setup.bash &&
   source install/setup.bash &&
   export ROS_LOG_DIR=/home/dohyun/FS/HYU-Formula-Student/log/ros &&
   ros2 launch hyu_perception perception_baseline.launch.py --show-args'
```

결과:

```text
Arguments (pass arguments as '<name>:=<value>'):
    'image_topic': default: '/zed/left/image_rect_color'
    'pointcloud_topic': default: '/velodyne_points'
    'bbox_topic': default: '/noisy_bounding_boxes'
    'camera_info_topic': default: '/custom_camera_info'
    'camera_frame': default: 'zed_right_camera_optical_frame'
    'projection_model': default: 'eufs_bbox'
    'output_cones_topic': default: '/cones'
    'output_frame': default: 'base_footprint'
    'oracle_cones_topic': default: ''
    'publish_empty_on_sync': default: 'false'
    'fusion_enabled': default: 'true'
```

판정: launch 파일 파싱은 통과.

### 4. 실제 노드 실행 smoke test

명령:

```bash
conda run -n eufs bash -lc \
  'source /opt/ros/galactic/setup.bash &&
   source install/setup.bash &&
   export ROS_LOG_DIR=/home/dohyun/FS/HYU-Formula-Student/log/ros &&
   timeout 5 ros2 launch hyu_perception perception_baseline.launch.py \
   fusion_enabled:=false oracle_cones_topic:=/camera_0/cones'
```

결과:

```text
[INFO] [perception_baseline_node-1]: process started
Traceback (most recent call last):
  File ".../perception_baseline_node.py", line 18, in <module>
    from sensor_msgs_py import point_cloud2
ModuleNotFoundError: No module named 'sensor_msgs_py'
[ERROR] [perception_baseline_node-1]: process has died
```

판정: 실패. `eufs` conda 환경에서도 Oracle adapter 모드가 import 단계에서 죽기 때문에 fusion 여부와 무관하게 노드가 기동하지 않는다.

### 5. 직접 executable 실행

명령:

```bash
conda run -n eufs bash -lc \
  'source /opt/ros/galactic/setup.bash &&
   source install/setup.bash &&
   export ROS_LOG_DIR=/home/dohyun/FS/HYU-Formula-Student/log/ros &&
   timeout 5 ros2 run hyu_perception perception_baseline_node \
   --ros-args -p fusion_enabled:=false -p oracle_cones_topic:=/camera_0/cones'
```

결과:

```text
ModuleNotFoundError: No module named 'sensor_msgs_py'
```

판정: 실패. launch 문제가 아니라 노드 import/runtime 문제이며, `eufs` conda 환경에서도 재현된다.

### 6. colcon test

명령:

```bash
conda run -n eufs bash -lc \
  'source /opt/ros/galactic/setup.bash &&
   colcon test --packages-select hyu_perception --event-handlers console_direct+ &&
   colcon test-result --test-result-base build/hyu_perception --verbose'
```

결과:

```text
Ran 0 tests in 0.000s
OK
Summary: 1 package finished [0.32s]
Summary: 0 tests, 0 errors, 0 failures, 0 skipped
```

판정: 실패는 없지만 테스트가 0개라 파이프라인 동작을 증명하지 못한다.

### 7. Docker 기반 end-to-end 검증

명령:

```bash
./scripts/hyu-docker build-ws
./scripts/hyu-docker status
```

결과:

```text
./scripts/hyu-docker: line 59: docker: command not found
./scripts/hyu-docker: line 36: docker: command not found
```

판정: 미검증. 현재 호스트에 Docker CLI가 없어 README의 권장 Humble 컨테이너 경로를 실행할 수 없다.

## 문제 지점

### 1. 노드 런타임 실패: `sensor_msgs_py` 없음

위치:

- `hyu_perception/perception_baseline_node.py:18`
- `hyu_perception/perception_baseline_node.py:408`

현상:

```python
from sensor_msgs_py import point_cloud2
```

`conda run -n eufs`와 `/opt/ros/galactic/setup.bash`를 함께 사용해도 `sensor_msgs_py`는 import되지 않는다. `ros2 pkg list`에는 `sensor_msgs`만 있고 `sensor_msgs_py`는 없다. 따라서 노드는 `/cones`를 publish하기 전에 import 단계에서 종료된다.

영향:

- Fusion mode 실행 불가.
- Oracle adapter mode 실행 불가.
- `/cones` publisher 생성 전 실패.
- graph SLAM과의 `/cones` 연결도 현재 환경에서는 확인 불가.

비고:

루트 README와 `scripts/hyu-docker`는 ROS 2 Humble을 기준으로 한다. Humble 컨테이너에 `sensor_msgs_py`가 있으면 이 문제는 호스트 Galactic + conda `eufs` 환경 한정일 수 있다. 하지만 현재는 Docker가 없어 의도된 Humble 환경에서 확인하지 못했다.

### 2. 의존성 선언에 `sensor_msgs_py`가 없음

위치:

- `package.xml:12-20`

현상:

`package.xml`에는 `sensor_msgs`는 있지만 `sensor_msgs_py`가 별도 exec dependency로 선언되어 있지 않다.

영향:

`sensor_msgs_py`를 제공하는 환경에서는 우연히 동작할 수 있지만, 현재처럼 해당 Python 패키지가 없는 환경에서는 rosdep/package metadata만으로 누락을 잡기 어렵다.

### 3. 현재 검증 호스트가 문서의 기준 환경과 다름

위치:

- `../README.md:31`
- `../scripts/hyu-docker:5`
- `../scripts/hyu-docker:41-43`

현상:

- 문서와 helper script는 ROS 2 Humble을 기준으로 한다.
- 현재 호스트에서 확인된 ROS setup은 `/opt/ros/galactic/setup.bash`이다.
- `docker` 명령이 없어 helper script 경로를 실행할 수 없다.

영향:

현재 호스트의 `conda eufs` + ROS Galactic에서 나온 `sensor_msgs_py` 실패가 최종 Humble Docker 환경에서도 재현되는지 확정할 수 없다. 다만 현재 사용 가능한 `eufs` 환경에서는 파이프라인이 동작하지 않는다.

### 4. 기본 ROS log 경로 쓰기 실패

현상:

`ROS_LOG_DIR`를 지정하지 않고 launch argument를 확인하면 다음 오류가 발생했다.

```text
OSError: [Errno 30] Read-only file system: '/home/dohyun/.ros/log/...'
```

영향:

이 환경에서는 `ROS_LOG_DIR`를 workspace 내부 쓰기 가능한 경로로 지정해야 launch 검증이 가능하다. 패키지 결함이라기보다 현재 sandbox/파일시스템 제약이다.

## 정적 리스크

실제 기동 실패와 별개로 코드/launch에서 확인된 리스크다.

### 1. bbox confidence가 association score에 사실상 반영되지 않음

위치:

- `hyu_perception/perception_baseline_node.py:557`

현재 코드:

```python
score = inside_count * max(detection.probability, 1.0)
```

일반적인 probability가 `0.0-1.0` 범위라면 `max(probability, 1.0)` 때문에 모든 detection이 최소 1.0 가중치를 받는다. 즉 confidence가 ranking에 거의 영향을 주지 않는다.

### 2. launch 파일이 YAML config 전체를 로드하지 않음

위치:

- `launch/perception_baseline.launch.py:22-32`
- `config/perception_baseline.yaml`

launch argument로 노출된 일부 parameter만 launch에서 전달된다. YAML에 있는 `clip_bboxes_to_image`, `clip_projected_points_to_image`, ROI, clustering, covariance 관련 tuning 값은 launch에서 config file로 직접 로드되지 않고 노드 코드의 declare default에 의존한다.

### 3. 알 수 없는 `projection_model` 값이 경고 없이 raw projection으로 처리됨

위치:

- `hyu_perception/perception_baseline_node.py:577`

`projection_model == "eufs_bbox"`일 때만 특수 변환을 하고, 그 외 값은 그대로 projection한다. 잘못된 parameter 값이 들어가도 경고 없이 부정확한 `/cones`가 나올 수 있다.

### 4. TF 누락 시 fusion이 publish 없이 skip됨

위치:

- `hyu_perception/perception_baseline_node.py:293`
- `hyu_perception/perception_baseline_node.py:325-326`
- `hyu_perception/perception_baseline_node.py:643-648`

LiDAR->base, LiDAR->camera TF가 없으면 fusion은 skip된다. 코드가 throttled warning은 남기지만, live topic 검증 없이 `/cones`가 비어 있거나 publish되지 않는 원인을 구분하기 어렵다.

## 최종 판정

현재 확인 가능한 환경 기준:

- Conda `eufs` Python/ROS compatibility: `rclpy` import 통과.
- Build under conda `eufs`: 통과.
- Package registration: 통과.
- Launch parse: 통과.
- Node startup under conda `eufs`: 실패 (`sensor_msgs_py` 없음).
- `/cones` publish: 미도달.
- Simulator + fusion + graph SLAM end-to-end: Docker 부재로 미검증.

따라서 현재 상태는 "파이프라인 정상 동작"이 아니라 "`eufs` conda 환경에서는 Python 3.8/rclpy 문제는 없지만, 노드가 `sensor_msgs_py` import에서 실패하여 파이프라인이 기동하지 않음"이다.

## 권장 다음 조치

1. 의도된 Humble Docker 환경에서 다시 검증한다.
   - `docker` 설치/접근 가능 상태에서 `./scripts/hyu-docker build-ws`, `sim-gui-bg`, `fusion-bg`, `slam-bg`, `status`를 실행한다.
2. `sensor_msgs_py` 의존성을 명시하거나, 현재 target ROS 배포판에서 사용 가능한 PointCloud2 reader API로 교체한다.
3. 최소 smoke/regression test를 추가한다.
   - import test
   - `_pointcloud_to_xyz()` 단위 테스트
   - oracle callback contract test
   - launch file load test
4. launch에서 YAML config를 직접 로드하거나, 문서에 "launch arg로 노출된 값만 override된다"는 점을 명확히 적는다.

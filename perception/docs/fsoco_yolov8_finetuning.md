# FSOCO YOLOv8 Fine-Tuning Handoff

This note captures the current FSOCO-to-YOLOv8 training state and the exact
commands to resume full fine-tuning after CUDA is available.

## Current State

- Source dataset: `/home/dohyun/FS/fsoco_bounding_boxes_train`
- Converted YOLO dataset: `/home/dohyun/FS/datasets/fsoco_yolov8`
- Dataset YAML: `/home/dohyun/FS/datasets/fsoco_yolov8/data.yaml`
- Training artifact root: `/home/dohyun/FS/artifacts/yolov8`
- Smoke checkpoint:
  `/home/dohyun/FS/artifacts/yolov8/fsoco_yolov8n_smoke_fraction02/weights/best.pt`

The conversion output has 11,572 images and 11,572 label files. The class
contract is:

```text
0 blue_cone
1 yellow_cone
2 orange_cone
3 large_orange_cone
4 unknown_cone
```

The ROS class map is:

```text
blue_cone:blue,yellow_cone:yellow,orange_cone:orange,large_orange_cone:big_orange,unknown_cone:unknown
```

As of 2026-06-04, full training is blocked on this host because `nvidia-smi`
cannot communicate with the NVIDIA driver and `torch.cuda.is_available()` is
false. The CPU smoke run completed, but it is not a usable Formula Student cone
detector and must not be treated as the final model.

## Full Training

Run this only after CUDA is visible from the `eufs` conda environment.

```bash
cd /home/dohyun/FS/HYU-Formula-Student
source /home/dohyun/anaconda3/etc/profile.d/conda.sh
conda activate eufs

python - <<'PY'
import torch
print("cuda_available", torch.cuda.is_available())
print("cuda_device_count", torch.cuda.device_count())
PY

MPLCONFIGDIR=/tmp/matplotlib \
yolo detect train \
  model=/home/dohyun/FS/HYU-Formula-Student/yolov8n.pt \
  data=/home/dohyun/FS/datasets/fsoco_yolov8/data.yaml \
  epochs=100 \
  imgsz=640 \
  batch=16 \
  device=0 \
  workers=4 \
  project=/home/dohyun/FS/artifacts/yolov8 \
  name=fsoco_yolov8n \
  exist_ok=False \
  plots=True \
  cache=False
```

If GPU memory is insufficient, reduce `batch` first. Keep `project` outside the
repo so generated weights do not pollute git.

## Post-Train Validation

Check that the trained checkpoint still has the exact five FSOCO class names and
order:

```bash
cd /home/dohyun/FS/HYU-Formula-Student
source /home/dohyun/anaconda3/etc/profile.d/conda.sh
conda activate eufs

python - <<'PY'
from ultralytics import YOLO

model = YOLO("/home/dohyun/FS/artifacts/yolov8/fsoco_yolov8n/weights/best.pt")
print(model.names)
assert model.names == {
    0: "blue_cone",
    1: "yellow_cone",
    2: "orange_cone",
    3: "large_orange_cone",
    4: "unknown_cone",
}
PY
```

Run validation on the held-out test split:

```bash
MPLCONFIGDIR=/tmp/matplotlib \
yolo detect val \
  model=/home/dohyun/FS/artifacts/yolov8/fsoco_yolov8n/weights/best.pt \
  data=/home/dohyun/FS/datasets/fsoco_yolov8/data.yaml \
  split=test \
  imgsz=640 \
  device=0 \
  project=/home/dohyun/FS/artifacts/yolov8 \
  name=fsoco_yolov8n_test
```

Save representative held-out predictions before using the model in ROS:

```bash
MPLCONFIGDIR=/tmp/matplotlib \
yolo detect predict \
  model=/home/dohyun/FS/artifacts/yolov8/fsoco_yolov8n/weights/best.pt \
  source=/home/dohyun/FS/datasets/fsoco_yolov8/images/test \
  imgsz=640 \
  conf=0.25 \
  device=0 \
  save=True \
  save_txt=True \
  save_conf=True \
  project=/home/dohyun/FS/artifacts/yolov8 \
  name=fsoco_yolov8n_test_predict
```

Do not proceed to fusion tuning unless held-out predictions show plausible cone
detections for all five classes.

## ROS Handoff

Start YOLO-backed perception with the trained checkpoint:

```bash
cd /home/dohyun/FS/HYU-Formula-Student
source /home/dohyun/anaconda3/etc/profile.d/conda.sh
conda activate eufs
source /opt/ros/galactic/setup.bash
source install/setup.bash

LD_PRELOAD=/lib/x86_64-linux-gnu/libffi.so.7 \
ros2 launch hyu_perception perception_baseline.launch.py \
  bbox_source:=yolov8 \
  yolo_model_path:=/home/dohyun/FS/artifacts/yolov8/fsoco_yolov8n/weights/best.pt \
  yolo_class_map:=blue_cone:blue,yellow_cone:yellow,orange_cone:orange,large_orange_cone:big_orange,unknown_cone:unknown \
  publish_yolo_debug_image:=true
```

Verify detector topics before inspecting fusion:

```bash
ros2 topic echo /yolo_bounding_boxes --once
ros2 topic hz /yolo_bounding_boxes
ros2 topic echo /yolo_bounding_boxes/debug_image --once
```

Only after `/yolo_bounding_boxes` and `/yolo_bounding_boxes/debug_image` prove
that detector output is present and correctly labeled should `/cones` or
`/cones/viz` be used to debug fusion.

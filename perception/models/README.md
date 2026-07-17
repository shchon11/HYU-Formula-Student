# Perception model checkpoints

Weights used by the perception nodes. Pointed at by
`config/perception_baseline.yaml -> yolov8_bbox_node.ros__parameters.model_path`,
which holds a **package-relative** path; the launch file resolves it against the
installed share directory.

## Active weight

    models/cone_pose_8kpt/weights/best.pt        # 6.2 MB, tracked in git

YOLO26n-pose cone detector. One forward pass yields the bounding box, the cone
class, **and** the cone silhouette keypoints, so it replaces both the paper's
YOLOv5 detector and its separate per-crop RektNet keypoint network.

- box mAP50 **0.9597**, pose mAP50-95 **0.8772**
- classes: `blue`, `yellow`, `orange`, `orange_big`, `undefined`
- keypoints are **class-dependent**: 6 on a standard cone (three left/right
  stripe pairs), 8 on a big orange cone (four pairs)

It is committed in-tree deliberately. The previous default was an absolute path
under a single developer's home directory, so on any other machine the detector
raised at startup, the fusion node waited forever for boxes, and `/perception/cones`
published nothing — a silent, total perception failure. Keeping the weight with
the package removes that class of bug.

## Legacy weight

    models/fsoco_yolov8n/weights/best.pt         # 6.0 MB, tracked in git

FSOCO fine-tuned YOLOv8n. Detection only, no keypoints, so it cannot drive the
Tier-3 stereo stage. Kept for comparison.

## Adding another weight

Large binaries are gitignored by default. If a checkpoint must ship with the
package, add an explicit allowlist entry to `models/.gitignore` and install it
via `data_files` in `setup.py`, as the two above do. Anything else — training
runs, experiments, ONNX/TensorRT exports — stays out of git.

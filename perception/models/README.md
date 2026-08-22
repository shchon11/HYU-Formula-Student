# Perception model checkpoints

Weights used by the perception nodes. Pointed at by
`config/perception_baseline.yaml -> yolov8_bbox_node.ros__parameters.model_path`,
which holds a **package-relative** path; the launch file resolves it against the
installed share directory.

## Active weight

    models/cone_detect_yolo26n_3cls/weights/best.pt   # 5.4 MB, tracked in git

YOLO26n cone **detector** (detect task: box + class, no keypoints) — the final
fine-tune, 2026-08-13 (`finetune_nano_3cls`, 100 epochs from the FSOCO teacher;
see its README.md). Classes `BLUE`, `YELLOW`, `ORANGE` — **no big-orange
class**, big orange cones are reported as `orange`. Val mAP50 **0.965**,
mAP50-95 **0.847**. A TensorRT FP16 `best.engine` next to it is built per
machine (`scripts/export_tensorrt_engine.py`, untracked) and preferred by the
node when present.

## Previous weights (installed, switch `model_path` to use)

    models/cone_detect_yolo26n/weights/best.pt   # 5-class detect, 2026-08-02
                                                 # (BLUE/ORANGE_BIG/ORANGE/UNDEFINED/YELLOW)
    models/cone_pose_8kpt/weights/best.pt        # 6.2 MB, pose variant, tracked in git

`cone_pose_8kpt` is the YOLO26n-pose cone detector: one forward pass yields the
bounding box, the cone class, **and** the cone silhouette keypoints, so it
replaces both the paper's YOLOv5 detector and its separate per-crop RektNet
keypoint network (box mAP50 0.9597, pose mAP50-95 0.8772; keypoints are
class-dependent: 6 on a standard cone, 8 on a big orange cone). Set
`publish_keypoints: true` with it to get the keypoint debug topic back.

Weights are committed in-tree deliberately. The original default was an absolute
path under a single developer's home directory, so on any other machine the
detector raised at startup, the fusion node waited forever for boxes, and
`/perception/cones` published nothing — a silent, total perception failure.
Keeping the weight with the package removes that class of bug.

## Legacy weight

    models/fsoco_yolov8n/weights/best.pt         # 6.0 MB, tracked in git

FSOCO fine-tuned YOLOv8n. Detection only, no keypoints, so it cannot drive the
Tier-3 stereo stage. Kept for comparison.

## Adding another weight

Large binaries are gitignored by default. If a checkpoint must ship with the
package, add an explicit allowlist entry to `models/.gitignore` and install it
via `data_files` in `setup.py`, as the two above do. Anything else — training
runs, experiments, ONNX/TensorRT exports — stays out of git.

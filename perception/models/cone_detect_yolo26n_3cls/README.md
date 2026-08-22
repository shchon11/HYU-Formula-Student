# cone_detect_yolo26n_3cls

Final cone detector fine-tune (the active `model_path` default).

| | |
|---|---|
| task | detect (box + class, no keypoints) |
| arch | YOLO26n (yolo26n.yaml), 2.50 M params, imgsz 640 |
| classes | 0 BLUE, 1 YELLOW, 2 ORANGE — **no big-orange class** (big orange cones are reported as orange) |
| trained | 2026-08-13, run `finetune_nano_3cls`, 100 epochs, batch 32, from the FSOCO teacher `teacher_fsoco_nano`, data `finetune_yolo/data.yaml` (ultralytics 8.4.93) |
| val metrics | mAP50 0.965, mAP50-95 0.847 (its own val split) |
| files | `weights/best.pt` (tracked); `weights/best.engine` = TensorRT FP16 plan exported ON THIS MACHINE (machine-specific, rebuild with `yolo export model=weights/best.pt format=engine quantize=16 imgsz=640 batch=1 device=0` after a JetPack/TensorRT change) |

The node prefers the sibling `.engine` when present (`prefer_engine: true`); it falls back to the `.pt` otherwise.
Previous weights: `../cone_detect_yolo26n` (5-class detect, 2026-08-02) and `../cone_pose_8kpt` (pose variant).

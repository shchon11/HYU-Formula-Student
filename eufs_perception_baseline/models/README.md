# Perception model checkpoints

YOLOv8 (and other) perception weights live here. They are intentionally kept
out of the sim launch's default absolute path assumptions and pointed at by
`config/perception_baseline.yaml -> yolov8_bbox_node.ros__parameters.model_path`.

Expected layout (matches the FSOCO fine-tuned YOLOv8n default):

    models/fsoco_yolov8n/weights/best.pt

Drop the checkpoint file at that path (or update `model_path` in
`config/perception_baseline.yaml` if you place it elsewhere). Large binary
weights should not be committed to git.

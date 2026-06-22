from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional

import numpy as np


COCO_PRETRAINED_YOLOV8_WEIGHTS = {
    "yolov8n.pt",
    "yolov8s.pt",
    "yolov8m.pt",
    "yolov8l.pt",
    "yolov8x.pt",
}


@dataclass(frozen=True)
class YoloDetection:
    color: str
    probability: float
    xmin: float
    ymin: float
    xmax: float
    ymax: float


def parse_class_map(value) -> Dict[str, str]:
    """Parse class-name aliases such as 'blue_cone:blue,yellow_cone:yellow'."""
    if value is None:
        return {}
    if isinstance(value, Mapping):
        items = value.items()
    else:
        text = str(value).strip()
        if not text:
            return {}
        pairs = []
        for entry in text.split(","):
            entry = entry.strip()
            if not entry:
                continue
            if ":" in entry:
                source, target = entry.split(":", 1)
            elif "=" in entry:
                source, target = entry.split("=", 1)
            else:
                continue
            pairs.append((source, target))
        items = pairs

    parsed = {}
    for source, target in items:
        key = _class_key(str(source))
        normalized = _canonical_color(str(target))
        if key and normalized:
            parsed[key] = normalized
    return parsed


def detections_from_ultralytics_results(
    results: Iterable,
    names=None,
    class_map: Optional[Mapping[str, str]] = None,
    confidence_threshold: float = 0.0,
    unknown_color_policy: str = "unknown",
) -> List[YoloDetection]:
    detections = []
    parsed_class_map = parse_class_map(class_map)
    policy = str(unknown_color_policy).strip().lower()

    for result in results or []:
        result_names = getattr(result, "names", None) or names
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            continue

        xyxy = _to_numpy(getattr(boxes, "xyxy", []), shape=(-1, 4))
        confidences = _to_numpy(getattr(boxes, "conf", []), shape=(-1,))
        classes = _to_numpy(getattr(boxes, "cls", []), shape=(-1,))

        count = min(len(xyxy), len(confidences), len(classes))
        for index in range(count):
            probability = float(confidences[index])
            if probability < confidence_threshold:
                continue

            class_id = int(classes[index])
            raw_name = _class_name(result_names, class_id)
            color = normalize_color(raw_name, parsed_class_map, policy)
            if color is None:
                continue

            xmin, ymin, xmax, ymax = sorted_box(xyxy[index])
            if xmax <= xmin or ymax <= ymin:
                continue

            detections.append(
                YoloDetection(
                    color=color,
                    probability=probability,
                    xmin=xmin,
                    ymin=ymin,
                    xmax=xmax,
                    ymax=ymax,
                )
            )
    return detections


def normalize_color(
    raw_name: str,
    class_map: Optional[Mapping[str, str]] = None,
    unknown_color_policy: str = "unknown",
) -> Optional[str]:
    key = _class_key(raw_name)
    if class_map and key in class_map:
        return class_map[key]

    inferred = _canonical_color(raw_name)
    if inferred:
        return inferred

    if str(unknown_color_policy).strip().lower() == "skip":
        return None
    return "unknown"


def sorted_box(values) -> tuple:
    xmin, ymin, xmax, ymax = [float(value) for value in values[:4]]
    xmin, xmax = sorted((xmin, xmax))
    ymin, ymax = sorted((ymin, ymax))
    return xmin, ymin, xmax, ymax


def looks_like_coco_pretrained_yolov8_weight(model_path: str) -> bool:
    """Return true for Ultralytics YOLOv8 stock COCO weight names."""
    return Path(str(model_path)).name.lower() in COCO_PRETRAINED_YOLOV8_WEIGHTS


def _class_name(names, class_id: int) -> str:
    if isinstance(names, Mapping):
        return str(names.get(class_id, names.get(str(class_id), class_id)))
    if isinstance(names, (list, tuple)) and 0 <= class_id < len(names):
        return str(names[class_id])
    return str(class_id)


def _to_numpy(value, shape):
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    array = np.asarray(value)
    return array.reshape(shape).astype(np.float64, copy=False)


def _class_key(value: str) -> str:
    return value.strip().lower().replace("-", "_").replace(" ", "_")


def _canonical_color(value: str) -> Optional[str]:
    key = _class_key(value)
    if "big" in key and "orange" in key:
        return "big_orange"
    if "blue" in key:
        return "blue"
    if "yellow" in key:
        return "yellow"
    if "orange" in key:
        return "orange"
    if key in ("unknown", "unknown_color"):
        return "unknown"
    return None

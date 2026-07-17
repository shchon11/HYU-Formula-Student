import math
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
    # Pose models attach the cone silhouette keypoints for this same detection.
    # (N, 2) pixel coordinates and (N,) confidences, N in {6, 8}; None for a
    # detect-only model.  Index i here matches bounding box i on the wire.
    keypoints: Optional[np.ndarray] = None
    keypoint_confidence: Optional[np.ndarray] = None


# The cone silhouette is annotated as left/right stripe pairs, top-down.  Small
# cones carry three rows; big orange cones carry a fourth.  Anything else means
# the weight does not match this pipeline's contract.
SMALL_CONE_KEYPOINTS = 6
BIG_CONE_KEYPOINTS = 8
VALID_KEYPOINT_COUNTS = (SMALL_CONE_KEYPOINTS, BIG_CONE_KEYPOINTS)


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

        # A pose model exposes result.keypoints parallel to result.boxes.  Read
        # it once per result; per-detection slices stay aligned by index.
        keypoints_xy, keypoints_conf = _result_keypoints(result, len(xyxy))

        count = min(len(xyxy), len(confidences), len(classes))
        for index in range(count):
            probability = float(confidences[index])
            class_value = float(classes[index])
            coordinates = np.asarray(xyxy[index], dtype=np.float64)
            if (
                not math.isfinite(probability)
                or not 0.0 <= probability <= 1.0
                or not math.isfinite(class_value)
                or class_value < 0.0
                or not class_value.is_integer()
                or coordinates.size != 4
                or not np.all(np.isfinite(coordinates))
            ):
                continue
            if probability < confidence_threshold:
                continue

            class_id = int(class_value)
            raw_name = _class_name(result_names, class_id)
            color = normalize_color(raw_name, parsed_class_map, policy)
            if color is None:
                continue

            xmin, ymin, xmax, ymax = sorted_box(coordinates)
            if xmax <= xmin or ymax <= ymin:
                continue

            cone_keypoints = None
            cone_keypoint_conf = None
            if keypoints_xy is not None and index < len(keypoints_xy):
                cone_keypoints, cone_keypoint_conf = _validated_keypoints(
                    keypoints_xy[index],
                    None if keypoints_conf is None else keypoints_conf[index],
                    color,
                )
                # A pose weight that returns unusable keypoints for this cone
                # must not silently degrade to a bbox-only detection: the
                # stereo tier would then have nothing to propagate.  Drop it.
                if cone_keypoints is None:
                    continue

            detections.append(
                YoloDetection(
                    color=color,
                    probability=probability,
                    xmin=xmin,
                    ymin=ymin,
                    xmax=xmax,
                    ymax=ymax,
                    keypoints=cone_keypoints,
                    keypoint_confidence=cone_keypoint_conf,
                )
            )
    return detections


def _result_keypoints(result, box_count: int):
    """Return (xy, conf) arrays from an Ultralytics pose result, or (None, None)."""
    keypoints = getattr(result, "keypoints", None)
    if keypoints is None:
        return None, None
    raw_xy = getattr(keypoints, "xy", None)
    if raw_xy is None:
        return None, None
    try:
        xy = _to_numpy(raw_xy, shape=(-1,))
    except (TypeError, ValueError):
        return None, None
    if xy.size == 0 or box_count <= 0 or xy.size % (box_count * 2) != 0:
        return None, None
    xy = xy.reshape(box_count, -1, 2)

    conf = None
    raw_conf = getattr(keypoints, "conf", None)
    if raw_conf is not None:
        try:
            flat = _to_numpy(raw_conf, shape=(-1,))
        except (TypeError, ValueError):
            flat = None
        if flat is not None and flat.size == box_count * xy.shape[1]:
            conf = flat.reshape(box_count, xy.shape[1])
    return xy, conf


def _validated_keypoints(points, confidence, color: str):
    """Trim a detection's keypoints to the count its cone class actually uses.

    The pose weight always emits a fixed-width tensor (8 slots), but only big
    orange cones define the fourth stripe row.  For a small cone the trailing
    slots are unlabeled padding, so they are dropped rather than fed to PnP as
    if they were real observations.
    """
    array = np.asarray(points, dtype=np.float64)
    if array.ndim != 2 or array.shape[1] != 2:
        return None, None

    expected = (
        BIG_CONE_KEYPOINTS
        if str(color).strip().lower() == "big_orange"
        else SMALL_CONE_KEYPOINTS
    )
    if array.shape[0] < expected:
        return None, None
    array = array[:expected]

    scores = None
    if confidence is not None:
        scores = np.asarray(confidence, dtype=np.float64).reshape(-1)
        if scores.size < expected:
            scores = None
        else:
            scores = scores[:expected]

    if not np.all(np.isfinite(array)):
        return None, None
    # Ultralytics writes (0, 0) for a slot it could not localize.  A cone whose
    # apex sits exactly on the origin is not physically reachable, so an exact
    # zero row is a dropped keypoint, not a measurement.
    if np.any(np.all(array == 0.0, axis=1)):
        return None, None
    if scores is not None and not np.all(np.isfinite(scores)):
        return None, None
    return array, scores


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
    # "large_orange_cone" (FSOCO) and "ORANGE_BIG" (this project's pose dataset)
    # are both the big cone; matching only "big" silently demoted the FSOCO name
    # to a small orange cone, which then took the wrong PnP template.
    if ("big" in key or "large" in key) and "orange" in key:
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

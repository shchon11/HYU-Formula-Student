"""Convert FSOCO Supervisely-style cone boxes into Ultralytics YOLO data."""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import shutil
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import yaml
from PIL import Image, ImageDraw


FSOCO_CLASSES = (
    "blue_cone",
    "yellow_cone",
    "orange_cone",
    "large_orange_cone",
    "unknown_cone",
)

SPLITS = ("train", "val", "test")
SPLIT_RATIOS = {"train": 0.8, "val": 0.1, "test": 0.1}


@dataclass(frozen=True)
class Label:
    class_id: int
    x_center: float
    y_center: float
    width: float
    height: float
    class_name: str

    def as_yolo_row(self) -> str:
        return (
            f"{self.class_id} "
            f"{self.x_center:.6f} "
            f"{self.y_center:.6f} "
            f"{self.width:.6f} "
            f"{self.height:.6f}"
        )


@dataclass
class ImageRecord:
    image_id: str
    group: str
    annotation_path: Path
    source_image_path: Path
    output_stem: str
    labels: List[Label]
    tag_counts: Counter
    clamped_boxes: int = 0
    discarded_boxes: int = 0
    split: str = ""

    @property
    def class_counts(self) -> Counter:
        return Counter(label.class_name for label in self.labels)


class ConversionError(RuntimeError):
    pass


def actual_image_size(image_path: Path) -> Tuple[int, int]:
    with Image.open(image_path) as image:
        return image.size


def yolo_label_from_rectangle(
    class_id: int,
    class_name: str,
    exterior: Sequence[Sequence[float]],
    image_width: int,
    image_height: int,
) -> Tuple[Optional[Label], bool]:
    """Return a normalized YOLO label and whether clamping was needed."""
    if len(exterior) != 2:
        raise ConversionError("rectangle exterior must contain exactly two points")

    try:
        x1, y1 = float(exterior[0][0]), float(exterior[0][1])
        x2, y2 = float(exterior[1][0]), float(exterior[1][1])
    except (IndexError, TypeError, ValueError) as exc:
        raise ConversionError("rectangle exterior points must be numeric xy pairs") from exc

    xmin, xmax = sorted((x1, x2))
    ymin, ymax = sorted((y1, y2))
    clamped_xmin = min(max(xmin, 0.0), float(image_width))
    clamped_xmax = min(max(xmax, 0.0), float(image_width))
    clamped_ymin = min(max(ymin, 0.0), float(image_height))
    clamped_ymax = min(max(ymax, 0.0), float(image_height))
    clamped = (
        clamped_xmin != xmin
        or clamped_xmax != xmax
        or clamped_ymin != ymin
        or clamped_ymax != ymax
    )

    box_width = clamped_xmax - clamped_xmin
    box_height = clamped_ymax - clamped_ymin
    if box_width <= 0.0 or box_height <= 0.0:
        return None, clamped

    label = Label(
        class_id=class_id,
        class_name=class_name,
        x_center=((clamped_xmin + clamped_xmax) / 2.0) / float(image_width),
        y_center=((clamped_ymin + clamped_ymax) / 2.0) / float(image_height),
        width=box_width / float(image_width),
        height=box_height / float(image_height),
    )

    values = (label.x_center, label.y_center, label.width, label.height)
    if not all(math.isfinite(value) and 0.0 <= value <= 1.0 for value in values):
        raise ConversionError("normalized YOLO label is nonfinite or out of range")

    return label, clamped


def paired_image_path(annotation_path: Path, group_root: Path) -> Path:
    image_name = annotation_path.name
    if image_name.endswith(".json"):
        image_name = image_name[: -len(".json")]
    return group_root / "img" / image_name


def collect_tag_counts(annotation: dict, obj: Optional[dict] = None) -> Counter:
    tags = Counter()
    if obj is None:
        for tag in annotation.get("tags", []) or []:
            name = tag.get("name") if isinstance(tag, dict) else None
            if name:
                tags[name] += 1
    if obj is not None:
        for tag in obj.get("tags", []) or []:
            name = tag.get("name") if isinstance(tag, dict) else None
            if name:
                tags[name] += 1
    return tags


def read_annotation_record(
    annotation_path: Path,
    dataset_root: Path,
    class_to_id: Dict[str, int],
) -> ImageRecord:
    group_root = annotation_path.parent.parent
    group = group_root.name
    image_path = paired_image_path(annotation_path, group_root)
    if not image_path.exists():
        raise ConversionError(f"missing paired image: {image_path}")

    try:
        annotation = json.loads(annotation_path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise ConversionError(f"unreadable annotation JSON: {annotation_path}") from exc

    size = annotation.get("size") or {}
    try:
        json_width = int(size["width"])
        json_height = int(size["height"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ConversionError(f"annotation size missing or invalid: {annotation_path}") from exc

    actual_width, actual_height = actual_image_size(image_path)
    if (actual_width, actual_height) != (json_width, json_height):
        raise ConversionError(
            "annotation size mismatch for "
            f"{annotation_path}: json=({json_width}, {json_height}) "
            f"actual=({actual_width}, {actual_height})"
        )

    labels: List[Label] = []
    tag_counts = collect_tag_counts(annotation)
    clamped_boxes = 0
    discarded_boxes = 0

    for obj in annotation.get("objects", []) or []:
        if obj.get("geometryType") != "rectangle":
            raise ConversionError(
                f"unsupported geometry {obj.get('geometryType')} in {annotation_path}"
            )

        class_name = obj.get("classTitle")
        if class_name not in class_to_id:
            raise ConversionError(f"unknown class {class_name!r} in {annotation_path}")

        tag_counts.update(collect_tag_counts(annotation, obj))
        points = obj.get("points") or {}
        label, clamped = yolo_label_from_rectangle(
            class_id=class_to_id[class_name],
            class_name=class_name,
            exterior=points.get("exterior") or [],
            image_width=json_width,
            image_height=json_height,
        )
        if clamped:
            clamped_boxes += 1
        if label is None:
            discarded_boxes += 1
            continue
        labels.append(label)

    relative_image = image_path.relative_to(dataset_root)
    output_stem = "__".join(relative_image.with_suffix("").parts)
    return ImageRecord(
        image_id=str(relative_image),
        group=group,
        annotation_path=annotation_path,
        source_image_path=image_path,
        output_stem=output_stem,
        labels=labels,
        tag_counts=tag_counts,
        clamped_boxes=clamped_boxes,
        discarded_boxes=discarded_boxes,
    )


def discover_annotation_paths(dataset_root: Path) -> List[Path]:
    paths: List[Path] = []
    for group_root in sorted(path for path in dataset_root.iterdir() if path.is_dir()):
        ann_dir = group_root / "ann"
        img_dir = group_root / "img"
        if not ann_dir.is_dir() or not img_dir.is_dir():
            continue
        paths.extend(sorted(ann_dir.glob("*.json")))
    return paths


def group_records(records: Iterable[ImageRecord]) -> Dict[str, List[ImageRecord]]:
    groups: Dict[str, List[ImageRecord]] = defaultdict(list)
    for record in records:
        groups[record.group].append(record)
    return groups


def records_class_counts(records: Iterable[ImageRecord]) -> Counter:
    counts = Counter()
    for record in records:
        counts.update(record.class_counts)
    return counts


def split_score(
    split_records: Dict[str, List[ImageRecord]],
    totals_by_class: Counter,
    total_images: int,
) -> float:
    score = 0.0
    for split in SPLITS:
        expected_images = total_images * SPLIT_RATIOS[split]
        actual_images = len(split_records[split])
        score += abs(actual_images - expected_images) / max(expected_images, 1.0)
        actual_classes = records_class_counts(split_records[split])
        for class_name in FSOCO_CLASSES:
            expected_class = totals_by_class[class_name] * SPLIT_RATIOS[split]
            score += abs(actual_classes[class_name] - expected_class) / max(
                expected_class, 1.0
            )
    return score


def assign_group_split(records: List[ImageRecord], seed: int = 42) -> str:
    grouped = group_records(records)
    split_records = {split: [] for split in SPLITS}
    totals_by_class = records_class_counts(records)
    total_images = len(records)
    groups = list(grouped.items())
    random.Random(seed).shuffle(groups)
    groups.sort(
        key=lambda item: (
            -sum(records_class_counts(item[1]).values()),
            -len(item[1]),
            item[0],
        )
    )

    group_assignment: Dict[str, str] = {}
    for group, group_items in groups:
        best_split = min(
            SPLITS,
            key=lambda split: split_score(
                {
                    name: (
                        split_records[name] + group_items
                        if name == split
                        else split_records[name]
                    )
                    for name in SPLITS
                },
                totals_by_class,
                total_images,
            ),
        )
        split_records[best_split].extend(group_items)
        group_assignment[group] = best_split

    for split in ("val", "test"):
        missing = [
            class_name
            for class_name in FSOCO_CLASSES
            if records_class_counts(split_records[split])[class_name] == 0
            and totals_by_class[class_name] > 0
        ]
        for class_name in missing:
            candidates = [
                (group, group_items)
                for group, group_items in grouped.items()
                if group_assignment[group] != split
                and records_class_counts(group_items)[class_name] > 0
            ]
            if not candidates:
                continue
            donor_group, donor_items = min(
                candidates,
                key=lambda item: (
                    len(item[1]),
                    -records_class_counts(item[1])[class_name],
                    item[0],
                ),
            )
            from_split = group_assignment[donor_group]
            split_records[from_split] = [
                record
                for record in split_records[from_split]
                if record.group != donor_group
            ]
            split_records[split].extend(donor_items)
            group_assignment[donor_group] = split

    for record in records:
        record.split = group_assignment[record.group]
    return "deterministic_greedy_group"


def output_paths(output_root: Path, record: ImageRecord) -> Tuple[Path, Path]:
    image_suffix = record.source_image_path.suffix.lower() or ".jpg"
    image_path = output_root / "images" / record.split / f"{record.output_stem}{image_suffix}"
    label_path = output_root / "labels" / record.split / f"{record.output_stem}.txt"
    return image_path, label_path


def symlink_or_replace(source: Path, target: Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists() or target.is_symlink():
        target.unlink()
    target.symlink_to(source.resolve())


def write_yolo_outputs(records: Sequence[ImageRecord], output_root: Path) -> None:
    for split in SPLITS:
        (output_root / "images" / split).mkdir(parents=True, exist_ok=True)
        (output_root / "labels" / split).mkdir(parents=True, exist_ok=True)

    for record in records:
        image_path, label_path = output_paths(output_root, record)
        symlink_or_replace(record.source_image_path, image_path)
        label_path.parent.mkdir(parents=True, exist_ok=True)
        rows = [label.as_yolo_row() for label in record.labels]
        label_path.write_text(("\n".join(rows) + "\n") if rows else "", encoding="utf-8")


def write_data_yaml(output_root: Path) -> Path:
    data = {
        "path": str(output_root.resolve()),
        "train": "images/train",
        "val": "images/val",
        "test": "images/test",
        "names": {index: name for index, name in enumerate(FSOCO_CLASSES)},
    }
    path = output_root / "data.yaml"
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return path


def write_split_manifest(records: Sequence[ImageRecord], output_root: Path) -> Path:
    path = output_root / "split_manifest.csv"
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "image_id",
                "group",
                "split",
                "annotation_path",
                "source_image_path",
                "output_image_path",
                *FSOCO_CLASSES,
            ],
        )
        writer.writeheader()
        for record in records:
            image_path, _ = output_paths(output_root, record)
            row = {
                "image_id": record.image_id,
                "group": record.group,
                "split": record.split,
                "annotation_path": str(record.annotation_path),
                "source_image_path": str(record.source_image_path),
                "output_image_path": str(image_path),
            }
            row.update({name: record.class_counts[name] for name in FSOCO_CLASSES})
            writer.writerow(row)
    return path


def validate_outputs(records: Sequence[ImageRecord], output_root: Path) -> Dict[str, object]:
    split_counts = Counter(record.split for record in records)
    split_class_counts = {
        split: dict(records_class_counts(record for record in records if record.split == split))
        for split in SPLITS
    }
    missing_by_split = {
        split: [
            class_name
            for class_name in FSOCO_CLASSES
            if split_class_counts[split].get(class_name, 0) == 0
        ]
        for split in ("val", "test")
    }

    unresolved_symlinks = []
    invalid_labels = []
    for record in records:
        image_path, label_path = output_paths(output_root, record)
        if not image_path.exists():
            unresolved_symlinks.append(str(image_path))
        if not label_path.exists():
            invalid_labels.append(f"missing label {label_path}")
            continue
        for line_no, line in enumerate(label_path.read_text(encoding="utf-8").splitlines(), 1):
            fields = line.split()
            if len(fields) != 5:
                invalid_labels.append(f"{label_path}:{line_no}: expected 5 fields")
                continue
            try:
                class_id = int(fields[0])
                coords = [float(value) for value in fields[1:]]
            except ValueError:
                invalid_labels.append(f"{label_path}:{line_no}: nonnumeric label")
                continue
            if class_id < 0 or class_id >= len(FSOCO_CLASSES):
                invalid_labels.append(f"{label_path}:{line_no}: class id out of range")
            if not all(math.isfinite(value) and 0.0 <= value <= 1.0 for value in coords):
                invalid_labels.append(f"{label_path}:{line_no}: coord out of range")

    return {
        "split_counts": dict(split_counts),
        "split_class_counts": split_class_counts,
        "missing_by_split": missing_by_split,
        "unresolved_symlinks": unresolved_symlinks,
        "invalid_labels": invalid_labels,
    }


def render_sample_overlays(records: Sequence[ImageRecord], output_root: Path) -> List[str]:
    overlay_dir = output_root / "reports" / "overlays"
    overlay_dir.mkdir(parents=True, exist_ok=True)
    selected = {}
    for split in SPLITS:
        for class_name in FSOCO_CLASSES:
            selected[(split, class_name)] = next(
                (
                    record
                    for record in records
                    if record.split == split and record.class_counts[class_name] > 0
                ),
                None,
            )

    written = []
    colors = {
        "blue_cone": "blue",
        "yellow_cone": "yellow",
        "orange_cone": "orange",
        "large_orange_cone": "red",
        "unknown_cone": "white",
    }
    for (split, class_name), record in selected.items():
        if record is None:
            continue
        with Image.open(record.source_image_path) as image:
            draw = ImageDraw.Draw(image)
            width, height = image.size
            for label in record.labels:
                x_center = label.x_center * width
                y_center = label.y_center * height
                box_width = label.width * width
                box_height = label.height * height
                xmin = x_center - box_width / 2.0
                ymin = y_center - box_height / 2.0
                xmax = x_center + box_width / 2.0
                ymax = y_center + box_height / 2.0
                color = colors.get(label.class_name, "white")
                draw.rectangle((xmin, ymin, xmax, ymax), outline=color, width=3)
                draw.text((xmin, max(0, ymin - 12)), label.class_name, fill=color)
            out_path = overlay_dir / f"{split}_{class_name}_{record.output_stem}.jpg"
            image.save(out_path, quality=90)
            written.append(str(out_path))
    return written


def write_report(
    records: Sequence[ImageRecord],
    output_root: Path,
    split_method: str,
    validation: Dict[str, object],
    overlays: Sequence[str],
) -> Path:
    tag_counts = Counter()
    for record in records:
        tag_counts.update(record.tag_counts)
    report = {
        "source_images": len(records),
        "source_annotations": len(records),
        "classes": list(FSOCO_CLASSES),
        "class_counts": dict(records_class_counts(records)),
        "split_method": split_method,
        "split_counts": validation["split_counts"],
        "split_class_counts": validation["split_class_counts"],
        "missing_by_split": validation["missing_by_split"],
        "tag_counts": dict(tag_counts),
        "clamped_boxes": sum(record.clamped_boxes for record in records),
        "discarded_boxes": sum(record.discarded_boxes for record in records),
        "unresolved_symlinks": validation["unresolved_symlinks"],
        "invalid_labels": validation["invalid_labels"],
        "sample_overlays": list(overlays),
    }
    reports_dir = output_root / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)
    path = reports_dir / "conversion_report.json"
    path.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    return path


def convert_dataset(
    dataset_root: Path,
    output_root: Path,
    overwrite: bool = False,
    require_split_classes: bool = True,
) -> Dict[str, object]:
    dataset_root = dataset_root.resolve()
    output_root = output_root.resolve()
    if not dataset_root.is_dir():
        raise ConversionError(f"dataset root does not exist: {dataset_root}")
    if output_root.exists():
        if not overwrite:
            raise ConversionError(f"output root already exists: {output_root}")
        shutil.rmtree(output_root)
    output_root.mkdir(parents=True)

    class_to_id = {class_name: index for index, class_name in enumerate(FSOCO_CLASSES)}
    annotation_paths = discover_annotation_paths(dataset_root)
    records = [
        read_annotation_record(path, dataset_root=dataset_root, class_to_id=class_to_id)
        for path in annotation_paths
    ]
    split_method = assign_group_split(records)
    write_yolo_outputs(records, output_root)
    data_yaml = write_data_yaml(output_root)
    split_manifest = write_split_manifest(records, output_root)
    validation = validate_outputs(records, output_root)
    overlays = render_sample_overlays(records, output_root)
    report = write_report(records, output_root, split_method, validation, overlays)

    if validation["unresolved_symlinks"] or validation["invalid_labels"]:
        raise ConversionError(
            "output validation failed; inspect "
            f"{report} for symlink and label errors"
        )
    missing = validation["missing_by_split"]
    if require_split_classes and any(missing[split] for split in missing):
        raise ConversionError(
            "validation/test split is missing classes; inspect "
            f"{report}"
        )

    return {
        "records": len(records),
        "data_yaml": str(data_yaml),
        "split_manifest": str(split_manifest),
        "report": str(report),
        "overlays": overlays,
        "validation": validation,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert FSOCO Supervisely-style bbox data to YOLOv8 format."
    )
    parser.add_argument(
        "--dataset-root",
        default="/home/dohyun/FS/fsoco_bounding_boxes_train",
        help="FSOCO bounding-box dataset root.",
    )
    parser.add_argument(
        "--output-root",
        default="/home/dohyun/FS/datasets/fsoco_yolov8",
        help="Output Ultralytics dataset directory.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Delete and recreate output-root if it already exists.",
    )
    parser.add_argument(
        "--allow-missing-split-classes",
        action="store_true",
        help=(
            "Do not fail if validation/test splits miss a class. "
            "Intended only for tiny fixtures and diagnostics."
        ),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = convert_dataset(
        dataset_root=Path(args.dataset_root),
        output_root=Path(args.output_root),
        overwrite=args.overwrite,
        require_split_classes=not args.allow_missing_split_classes,
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()

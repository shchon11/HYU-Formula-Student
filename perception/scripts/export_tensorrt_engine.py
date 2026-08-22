#!/usr/bin/env python3
"""Export the perception YOLO .pt to a TensorRT .engine on THIS machine.

The .engine is GPU- and TensorRT-version-specific, so it is untracked
(.gitignore) and must be (re)built wherever perception runs the accelerated
path -- new machine, new GPU, or a TensorRT upgrade. yolov8_bbox_node
auto-prefers a .engine sitting next to the configured .pt (parameter
``prefer_engine`` defaults true), so once this finishes the stack uses it with
no config change; delete the .engine (or set prefer_engine:=false) to fall back
to the portable .pt.

Prereq: a matching TensorRT python binding, e.g.
    pip install --user "tensorrt-cu12==10.13.3.9"   # cu12 runtime, works on CUDA 12/13 drivers
plus onnx + onnxslim for the ONNX stage:
    pip install --user "onnx>=1.12.0,<2.0.0" "onnxslim>=0.1.71"

Usage:
    python3 scripts/export_tensorrt_engine.py                 # default cone model, FP16, imgsz 640
    python3 scripts/export_tensorrt_engine.py --model <PATH> --imgsz 640 --fp32
"""
import argparse
import sys
from pathlib import Path

DEFAULT_MODEL = "models/cone_detect_yolo26n_3cls/weights/best.pt"  # the perception.yaml model


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default=DEFAULT_MODEL, help="path to the .pt weights")
    parser.add_argument("--imgsz", type=int, default=640, help="network input size")
    parser.add_argument("--fp32", action="store_true", help="FP32 engine (default: FP16/half)")
    parser.add_argument("--device", default="0", help="CUDA device index")
    args = parser.parse_args()

    model = Path(args.model)
    if not model.is_file():
        # tolerate being run from the package root or the scripts dir
        alt = Path(__file__).resolve().parent.parent / args.model
        model = alt if alt.is_file() else model
    if not model.is_file():
        print(f"model not found: {args.model}", file=sys.stderr)
        return 1

    try:
        import tensorrt  # noqa: F401
    except ImportError:
        print(
            "tensorrt not importable -- install the python binding first, e.g.\n"
            "  pip install --user 'tensorrt-cu12==10.13.3.9'",
            file=sys.stderr,
        )
        return 2

    from ultralytics import YOLO

    yolo = YOLO(str(model))
    out = yolo.export(
        format="engine",
        imgsz=args.imgsz,
        half=not args.fp32,
        device=args.device,
        verbose=True,
    )
    print(f"engine written: {out}  (node uses it automatically via prefer_engine)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

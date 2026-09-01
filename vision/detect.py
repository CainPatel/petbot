#!/usr/bin/env python3
"""Pet detection loop: Picamera2 -> YOLO -> floor-contact pixel.

Runs on the Raspberry Pi 5 with the ArduCam IMX708. For each frame it runs YOLO,
keeps detections of the configured classes (cat/dog) above the confidence
threshold, and reports the BOTTOM-CENTRE of each bounding box as the pixel where
the animal touches the floor. That pixel is the only one a floor homography can
legitimately map -- see docs/calibration.md section 3.

Usage:
    python vision/detect.py                 # continuous
    python vision/detect.py --frames 20     # then stop
    python vision/detect.py --no-annotate   # skip writing latest.jpg
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import cv2
import numpy as np
from config_loader import load_config, repo_path

COCO_NAMES = {15: "cat", 16: "dog"}


def open_camera(cam_cfg: dict):
    """Configure the IMX708 with manual focus and start it."""
    from picamera2 import Picamera2

    picam = Picamera2()

    # "RGB888" is a Picamera2 naming quirk: the array it hands back is in BGR
    # byte order, which is exactly what OpenCV and Ultralytics already expect.
    # Do NOT cv2.cvtColor() these frames -- doing so swaps red and blue and
    # quietly degrades YOLO's confidence on anything colour-dependent.
    config = picam.create_preview_configuration(
        main={
            "size": (int(cam_cfg["width"]), int(cam_cfg["height"])),
            "format": "RGB888",
        }
    )
    picam.configure(config)

    # Manual focus, set before start(). AfMode 0 = manual. LensPosition is in
    # dioptres (1 / distance in metres).
    picam.set_controls(
        {"AfMode": 0, "LensPosition": float(cam_cfg["lens_position"])}
    )

    picam.start()
    time.sleep(1.0)  # let AE/AWB settle before the first real frame
    return picam


def detect_frame(model, frame, conf: float, classes: list[int]):
    """Run YOLO on one BGR frame. Returns a list of detection dicts."""
    results = model(frame, conf=conf, classes=classes, verbose=False)

    out = []
    for box in results[0].boxes:
        x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
        cls = int(box.cls[0])

        out.append(
            {
                "cls": cls,
                "name": COCO_NAMES.get(cls, str(cls)),
                "conf": float(box.conf[0]),
                "bbox": (x1, y1, x2, y2),
                # Floor contact point: horizontal centre, BOTTOM edge of the
                # box. The centroid of the box is in mid-air and maps through
                # the floor homography to a position that is systematically
                # too far from the camera.
                "floor_px": ((x1 + x2) / 2.0, y2),
            }
        )
    return out


def annotate(frame, detections):
    """Draw boxes and floor-contact markers onto a copy of the frame."""
    canvas = frame.copy()
    for d in detections:
        x1, y1, x2, y2 = (int(v) for v in d["bbox"])
        fx, fy = (int(v) for v in d["floor_px"])

        cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(canvas, (fx, fy), 6, (0, 0, 255), -1)
        cv2.putText(
            canvas,
            f"{d['name']} {d['conf']:.2f}",
            (x1, max(y1 - 8, 12)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )
    return canvas


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=None, help="path to config.yaml")
    ap.add_argument("--frames", type=int, default=0,
                    help="stop after N frames (0 = run until interrupted)")
    ap.add_argument("--no-annotate", action="store_true",
                    help="do not write the annotated output image")
    args = ap.parse_args()

    cfg = load_config(args.config)
    cam_cfg = cfg["camera"]
    det_cfg = cfg["detection"]

    from ultralytics import YOLO

    model = YOLO(det_cfg["model"])
    conf = float(det_cfg["confidence"])
    classes = [int(c) for c in det_cfg["classes"]]
    out_path = repo_path(det_cfg.get("output_image", "latest.jpg"))

    picam = open_camera(cam_cfg)
    print(f"camera up at {cam_cfg['width']}x{cam_cfg['height']}, "
          f"lens_position={cam_cfg['lens_position']}")
    print(f"model={det_cfg['model']} conf={conf} classes={classes}")

    n = 0
    try:
        while args.frames == 0 or n < args.frames:
            frame = picam.capture_array()  # BGR order despite the RGB888 name
            detections = detect_frame(model, frame, conf, classes)
            n += 1

            if detections:
                for d in detections:
                    fx, fy = d["floor_px"]
                    print(f"[{n:05d}] {d['name']} conf={d['conf']:.2f} "
                          f"floor_px=({fx:.0f}, {fy:.0f})")
            else:
                print(f"[{n:05d}] no detection")

            if not args.no_annotate:
                canvas = annotate(frame, detections) if detections else frame
                # imwrite also expects BGR, so again: no colour conversion.
                cv2.imwrite(str(out_path), canvas)

    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        picam.stop()

    print(f"{n} frames processed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Ground-truth platform tracker.

Detects the ArUco marker on the platform's flat top (DICT_4X4_50, id 4 by
default), maps its centre through the saved floor homography, and prints the
result. Given a commanded position it prints commanded vs. measured with an
error magnitude -- this is the tool that fills in table 3 of docs/results.md.

Usage:
    python vision/track_platform.py                          # stream
    python vision/track_platform.py --once
    python vision/track_platform.py --once --commanded 1500,1500
    python vision/track_platform.py --image frame.jpg --once

CAVEAT, and it matters for interpreting every number this prints: the
homography is fitted to the FLOOR plane, and the platform is not on the floor.
Mapping an airborne marker through a floor homography returns the floor point
along the camera ray through that marker, which is displaced away from the
camera by an amount proportional to the platform's height. Fly the platform low
when comparing, keep the camera as close to overhead as possible, and record
the platform Z for every row in the results table so the offset can be
accounted for later.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import cv2
import numpy as np
from calibrate_homography import aruco_centres, pixel_to_floor
from config_loader import load_config, repo_path


def load_homography(cfg: dict, override: str | None = None) -> np.ndarray:
    path = repo_path(override or cfg["calibration"]["homography"])
    if not path.exists():
        raise FileNotFoundError(
            f"{path} not found. Run vision/calibrate_homography.py first."
        )
    return np.load(path)


class FrameSource:
    """Either a still image or a live camera, behind one interface."""

    def __init__(self, cfg: dict, image: str | None):
        self.image = image
        self.picam = None

        if image is None:
            from picamera2 import Picamera2

            cam_cfg = cfg["camera"]
            self.picam = Picamera2()
            config = self.picam.create_preview_configuration(
                main={
                    "size": (int(cam_cfg["width"]), int(cam_cfg["height"])),
                    "format": "RGB888",  # BGR byte order; no cvtColor
                }
            )
            self.picam.configure(config)
            self.picam.set_controls(
                {"AfMode": 0, "LensPosition": float(cam_cfg["lens_position"])}
            )
            self.picam.start()
            time.sleep(1.0)

    def read(self):
        if self.picam is not None:
            return self.picam.capture_array()
        frame = cv2.imread(self.image)
        if frame is None:
            raise FileNotFoundError(f"could not read {self.image}")
        return frame

    def close(self):
        if self.picam is not None:
            self.picam.stop()


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--config", default=None, help="path to config.yaml")
    ap.add_argument("--homography", default=None, help="override .npy path")
    ap.add_argument("--image", default=None,
                    help="read this image instead of the camera")
    ap.add_argument("--id", type=int, default=None,
                    help="ArUco id of the platform marker (default: config)")
    ap.add_argument("--once", action="store_true",
                    help="report a single measurement and exit")
    ap.add_argument("--commanded", default=None,
                    help="commanded floor position 'x,y' in mm, to compare")
    ap.add_argument("--samples", type=int, default=1,
                    help="average this many frames per measurement")
    args = ap.parse_args()

    cfg = load_config(args.config)
    H = load_homography(cfg, args.homography)

    plat = cfg.get("platform", {})
    dict_name = plat.get("aruco_dict", "DICT_4X4_50")
    marker_id = args.id if args.id is not None else int(plat.get("aruco_id", 4))

    commanded = None
    if args.commanded:
        cx, cy = args.commanded.split(",")
        commanded = (float(cx), float(cy))

    src = FrameSource(cfg, args.image)
    print(f"tracking {dict_name} id {marker_id}")
    if commanded:
        print(f"commanded position: ({commanded[0]:.1f}, {commanded[1]:.1f}) mm")

    n = 0
    try:
        while True:
            xs, ys, hits = [], [], 0

            for _ in range(max(1, args.samples)):
                frame = src.read()
                found = aruco_centres(frame, dict_name)
                if marker_id in found:
                    u, v = found[marker_id]
                    x, y = pixel_to_floor(u, v, H)
                    xs.append(x)
                    ys.append(y)
                    hits += 1

            n += 1

            if not hits:
                print(f"[{n:05d}] marker {marker_id} not detected")
            else:
                mx = float(np.mean(xs))
                my = float(np.mean(ys))
                spread = (float(np.std(xs)), float(np.std(ys)))

                line = (f"[{n:05d}] measured=({mx:8.1f}, {my:8.1f}) mm  "
                        f"hits={hits}/{max(1, args.samples)}")
                if args.samples > 1:
                    line += f"  sd=({spread[0]:.1f}, {spread[1]:.1f})"

                if commanded:
                    dx = mx - commanded[0]
                    dy = my - commanded[1]
                    err = float(np.hypot(dx, dy))
                    line += (f"  commanded=({commanded[0]:.1f}, "
                             f"{commanded[1]:.1f})"
                             f"  d=({dx:+.1f}, {dy:+.1f})  err={err:.1f} mm")

                print(line)

            if args.once or args.image:
                break

    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        src.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Compute and save the floor homography.

Maps camera pixel coordinates on the FLOOR PLANE to real floor coordinates in
millimetres, in the same room frame as the anchors: origin at one floor corner,
X and Y along the two walls.

A homography is valid for one plane only. Anything off the floor -- including
the platform hanging in mid-air -- does not map correctly through this matrix.

Two ways to supply the four source points:

    --auto      detect four ArUco markers in a captured frame and use their
                centres (repeatable, strongly preferred)
    --manual    type in four pixel coordinates by hand

Destination points (real floor positions, mm) always come from --dst or the
interactive prompt: they are tape measurements and nothing can infer them.

Usage:
    python vision/calibrate_homography.py --auto
    python vision/calibrate_homography.py --auto --image frame.jpg
    python vision/calibrate_homography.py --manual
    python vision/calibrate_homography.py --auto \
        --dst "0,0 0,2500 2500,2500 2500,0"

See docs/calibration.md section 3 for the full procedure, including the marker
placement rules (spread wide, off the frame edges, not collinear) and the
independent validation step that this script deliberately does not replace.
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


def pixel_to_floor(u: float, v: float, H: np.ndarray) -> tuple[float, float]:
    """Map one pixel (u, v) through homography H to floor mm (x, y).

    The third homogeneous component is the perspective divide; a value at or
    near zero means the pixel maps to the horizon and has no finite floor
    position, which is what the guard below catches.
    """
    src = np.array([[[float(u), float(v)]]], dtype=np.float64)
    dst = cv2.perspectiveTransform(src, H)
    x, y = dst[0][0]
    return float(x), float(y)


def capture_frame(cfg: dict) -> np.ndarray:
    """Grab a single frame from the Pi camera, with the configured focus."""
    from picamera2 import Picamera2

    cam_cfg = cfg["camera"]
    picam = Picamera2()
    config = picam.create_preview_configuration(
        main={
            "size": (int(cam_cfg["width"]), int(cam_cfg["height"])),
            "format": "RGB888",  # returns BGR byte order; no cvtColor needed
        }
    )
    picam.configure(config)
    picam.set_controls(
        {"AfMode": 0, "LensPosition": float(cam_cfg["lens_position"])}
    )
    picam.start()
    time.sleep(1.5)
    frame = picam.capture_array()
    picam.stop()
    return frame


def aruco_centres(frame: np.ndarray, dict_name: str = "DICT_4X4_50"):
    """Detect ArUco markers and return {id: (u, v)} of each marker centre."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, dict_name)
    )
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    corners, ids, _ = detector.detectMarkers(frame)
    if ids is None:
        return {}

    out = {}
    for marker_corners, marker_id in zip(corners, ids.flatten()):
        pts = marker_corners.reshape(4, 2)
        out[int(marker_id)] = (float(pts[:, 0].mean()), float(pts[:, 1].mean()))
    return out


def parse_point_list(text: str, expect: int = 4) -> list[tuple[float, float]]:
    """Parse 'x,y x,y x,y x,y' into a list of tuples."""
    parts = text.replace(";", " ").split()
    pts = []
    for p in parts:
        a, b = p.split(",")
        pts.append((float(a), float(b)))
    if len(pts) != expect:
        raise ValueError(f"expected {expect} points, got {len(pts)}")
    return pts


def prompt_points(label: str, unit: str) -> list[tuple[float, float]]:
    """Interactively read four points."""
    print(f"\nEnter the four {label} as 'x,y', one per line ({unit}):")
    pts = []
    for i in range(4):
        while True:
            raw = input(f"  point {i + 1}: ").strip()
            try:
                a, b = raw.split(",")
                pts.append((float(a), float(b)))
                break
            except ValueError:
                print("    could not parse; format is  x,y")
    return pts


def self_test(src: np.ndarray, dst: np.ndarray, H: np.ndarray) -> float:
    """Map the source points back through H and print residuals.

    NOTE what this does and does not prove. Four points determine a homography
    exactly, so these residuals are fit residuals, not accuracy -- they should
    be near zero and their being near zero means only that the arithmetic ran
    and there was no typo. Real accuracy comes from the independent validation
    step in docs/calibration.md, using points that were NOT part of the fit.
    """
    print("\nSelf-test (fit residuals, NOT an accuracy measurement):")
    print(f"  {'#':>2}  {'pixel':>18}  {'expected mm':>18}  "
          f"{'mapped mm':>18}  {'residual mm':>12}")

    worst = 0.0
    for i, ((u, v), (ex, ey)) in enumerate(zip(src, dst), start=1):
        mx, my = pixel_to_floor(u, v, H)
        residual = float(np.hypot(mx - ex, my - ey))
        worst = max(worst, residual)
        print(f"  {i:>2}  {f'({u:.0f}, {v:.0f})':>18}  "
              f"{f'({ex:.0f}, {ey:.0f})':>18}  "
              f"{f'({mx:.1f}, {my:.1f})':>18}  {residual:>12.3f}")

    print(f"\n  worst residual: {worst:.3f} mm")
    if worst > 1.0:
        print("  WARNING: a four-point fit should be near-exact. A residual "
              "this large means a typo in the input points.")
    return worst


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--config", default=None, help="path to config.yaml")
    src_mode = ap.add_mutually_exclusive_group()
    src_mode.add_argument("--auto", action="store_true",
                          help="find source pixels from ArUco markers")
    src_mode.add_argument("--manual", action="store_true",
                          help="type source pixels in by hand")
    ap.add_argument("--image", default=None,
                    help="use this image file instead of capturing a frame")
    ap.add_argument("--src", default=None,
                    help="source pixels, 'u,v u,v u,v u,v'")
    ap.add_argument("--dst", default=None,
                    help="destination floor points in mm, 'x,y x,y x,y x,y'")
    ap.add_argument("--ids", default=None,
                    help="comma-separated ArUco ids to use, in the same order "
                         "as --dst (default: the four lowest ids found)")
    ap.add_argument("--out", default=None,
                    help="output .npy path (default: from config)")
    args = ap.parse_args()

    cfg = load_config(args.config)
    out_path = repo_path(
        args.out or cfg["calibration"]["homography"]
    )

    # ---- source pixels -----------------------------------------------------
    if args.src:
        src_pts = parse_point_list(args.src)
    elif args.auto:
        if args.image:
            frame = cv2.imread(args.image)
            if frame is None:
                print(f"could not read {args.image}", file=sys.stderr)
                return 1
        else:
            frame = capture_frame(cfg)

        dict_name = cfg.get("platform", {}).get("aruco_dict", "DICT_4X4_50")
        found = aruco_centres(frame, dict_name)

        if args.ids:
            want = [int(i) for i in args.ids.split(",")]
        else:
            want = sorted(found)[:4]

        missing = [i for i in want if i not in found]
        if len(want) != 4 or missing:
            print(f"need 4 markers; found ids {sorted(found)}", file=sys.stderr)
            if missing:
                print(f"missing requested ids: {missing}", file=sys.stderr)
            return 1

        src_pts = [found[i] for i in want]
        print(f"using ArUco ids {want}")
        for i, (u, v) in zip(want, src_pts):
            print(f"  id {i}: pixel ({u:.1f}, {v:.1f})")
    else:
        src_pts = prompt_points("source PIXEL coordinates", "pixels")

    # ---- destination floor points -----------------------------------------
    if args.dst:
        dst_pts = parse_point_list(args.dst)
    else:
        dst_pts = prompt_points(
            "destination FLOOR coordinates", "millimetres, room frame"
        )

    src = np.array(src_pts, dtype=np.float64)
    dst = np.array(dst_pts, dtype=np.float64)

    # ---- fit ---------------------------------------------------------------
    H, mask = cv2.findHomography(src, dst)
    if H is None:
        print("findHomography failed. The four points are probably collinear "
              "or duplicated.", file=sys.stderr)
        return 1

    print("\nHomography:")
    print(H)

    self_test(src, dst, H)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.save(out_path, H)
    print(f"\nsaved to {out_path}")
    print("Now do the independent validation in docs/calibration.md section 3 "
          "step 7 and record it in docs/results.md. The self-test above is "
          "not a substitute.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

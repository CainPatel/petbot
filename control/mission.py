#!/usr/bin/env python3
"""Mission loop: see a pet, fly the platform to it, drop a treat.

Ties the pieces together:

    1. read config.yaml
    2. capture a frame and run YOLO (vision/detect.py)
    3. map the detection's floor-contact pixel through the homography
       (vision/calibrate_homography.pixel_to_floor)
    4. clamp the target into the safe workspace and send a move
       (control/serial_link.py)
    5. optionally hit the ESP32's /release endpoint to drop the treat

Usage:
    python -m control.mission --dry-run          # print, do not send
    python -m control.mission --once
    python -m control.mission --no-dispense
    python -m control.mission --home 1000,1000,1200

--dry-run touches no hardware except the camera: it prints the exact serial
commands and HTTP requests it would have issued. Use it first, every time.

The firmware only WARNS about targets outside its safe box and then moves
anyway, so the clamp in this script is the real guard: a pet standing in a
corner produces the nearest reachable point, not a slack cable.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "vision"))

import numpy as np
from config_loader import load_config, repo_path

from control import serial_link


def load_homography(cfg: dict) -> np.ndarray:
    path = repo_path(cfg["calibration"]["homography"])
    if not path.exists():
        raise FileNotFoundError(
            f"{path} not found. Run vision/calibrate_homography.py first."
        )
    return np.load(path)


def workspace_box(cfg: dict):
    """Return the safe box the firmware also checks, as {axis: (lo, hi)}.

    Mirrors inWorkspace() in firmware/uno_winches.cpp. The firmware only
    WARNS on a target outside this box and then moves anyway, so clamping
    here is what actually keeps the cables taut. Values come from the
    `workspace:` block of config.yaml; keep them identical to the firmware.
    """
    w = cfg["workspace"]
    return {
        "x": (float(w["x_min"]), float(w["x_max"])),
        "y": (float(w["y_min"]), float(w["y_max"])),
        "z": (float(w["z_min"]), float(w["z_max"])),
    }


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def dispense(cfg: dict, dry_run: bool) -> None:
    base = cfg["esp32"]["base_url"].rstrip("/")
    # With the claw platform "dispense" means open the jaws and drop whatever
    # they are holding: GET /release on esp32/esp32_claw.cpp.
    url = f"{base}{cfg['esp32'].get('dispense_endpoint', '/release')}"
    timeout = float(cfg["esp32"].get("request_timeout_s", 5.0))

    if dry_run:
        print(f"  [dry-run] GET {url}")
        return

    import requests

    try:
        r = requests.get(url, timeout=timeout)
        print(f"  dispense -> {r.status_code} {r.text.strip()}")
    except Exception as exc:  # noqa: BLE001 - network errors are all equivalent here
        # A failed treat drop should not abort the mission; the platform is
        # already where it needs to be and the operator can retry.
        print(f"  dispense FAILED: {exc}")


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--config", default=None, help="path to config.yaml")
    ap.add_argument("--dry-run", action="store_true",
                    help="print commands instead of sending them")
    ap.add_argument("--once", action="store_true",
                    help="handle one detection and exit")
    ap.add_argument("--no-dispense", action="store_true",
                    help="move to the pet but do not drop a treat")
    ap.add_argument("--home", default=None,
                    help="declare the platform's current position 'x,y,z' "
                         "in mm before doing anything else")
    ap.add_argument("--interval", type=float, default=5.0,
                    help="seconds between detection attempts")
    args = ap.parse_args()

    cfg = load_config(args.config)
    H = load_homography(cfg)
    box = workspace_box(cfg)
    cruise_z = clamp(float(cfg.get("platform", {}).get("cruise_z_mm", 1800)),
                     *box["z"])

    print(f"workspace x{box['x']} y{box['y']} cruise_z={cruise_z}")
    if args.dry_run:
        print("DRY RUN - no serial, no HTTP")

    # ---- camera + model ----------------------------------------------------
    import detect as detect_mod
    from calibrate_homography import pixel_to_floor
    from ultralytics import YOLO

    det_cfg = cfg["detection"]
    model = YOLO(det_cfg["model"])
    conf = float(det_cfg["confidence"])
    classes = [int(c) for c in det_cfg["classes"]]

    picam = detect_mod.open_camera(cfg["camera"])

    # ---- arduino -----------------------------------------------------------
    link = None
    if not args.dry_run:
        link = serial_link.from_config(cfg)
        print(f"arduino ready on {cfg['serial']['port']}")

    try:
        if args.home:
            hx, hy, hz = (float(v) for v in args.home.split(","))
            if args.dry_run:
                print(f"  [dry-run] H {hx:.2f} {hy:.2f} {hz:.2f}")
            else:
                print("  home:", link.set_home(hx, hy, hz))

        while True:
            frame = picam.capture_array()
            detections = detect_mod.detect_frame(frame=frame, model=model,
                                                 conf=conf, classes=classes)

            if not detections:
                print("no pet detected")
            else:
                # Most confident detection wins.
                best = max(detections, key=lambda d: d["conf"])
                u, v = best["floor_px"]
                fx, fy = pixel_to_floor(u, v, H)

                tx = clamp(fx, *box["x"])
                ty = clamp(fy, *box["y"])
                clamped = (tx != fx) or (ty != fy)

                print(f"{best['name']} conf={best['conf']:.2f} "
                      f"pixel=({u:.0f}, {v:.0f}) floor=({fx:.0f}, {fy:.0f})")
                if clamped:
                    print(f"  target clamped into workspace -> "
                          f"({tx:.0f}, {ty:.0f})")

                if args.dry_run:
                    print(f"  [dry-run] M {tx:.2f} {ty:.2f} {cruise_z:.2f}")
                else:
                    try:
                        print("  moved:", link.move(tx, ty, cruise_z))
                        if link.last_warning:
                            print("  " + link.last_warning)
                    except serial_link.SerialLinkError as exc:
                        print(f"  move failed: {exc}")
                        if args.once:
                            return 1
                        time.sleep(args.interval)
                        continue

                if not args.no_dispense:
                    dispense(cfg, args.dry_run)

            if args.once:
                break

            time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        picam.stop()
        if link is not None:
            # Leave the motors energised: releasing them drops the platform.
            # The operator disables deliberately with `D`.
            link.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

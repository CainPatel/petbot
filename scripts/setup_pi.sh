#!/usr/bin/env bash
#
# Provision a Raspberry Pi 5 for the cdpr-petbot vision stack.
#
# Idempotent: safe to run repeatedly. Everything it does is either a no-op when
# already done, or a re-check.
#
# Assumes RASPBERRY PI OS (64-bit). It will not work on Ubuntu -- picamera2 and
# the rpicam stack are packaged for Raspberry Pi OS, which is why the Ubuntu
# attempt in docs/build-log.md was abandoned.
#
# Usage:
#     bash scripts/setup_pi.sh
#     VENV=~/othervenv bash scripts/setup_pi.sh

set -euo pipefail

VENV="${VENV:-$HOME/cv}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REQUIREMENTS="$REPO_ROOT/vision/requirements.txt"

say() { printf '\n\033[1m== %s\033[0m\n' "$*"; }
warn() { printf '\033[33mwarning: %s\033[0m\n' "$*" >&2; }

# ---------------------------------------------------------------------------

say "Checking platform"

if [ ! -f /etc/rpi-issue ] && ! grep -qi raspbian /etc/os-release 2>/dev/null; then
  warn "this does not look like Raspberry Pi OS."
  warn "picamera2 and rpicam are packaged for Raspberry Pi OS; other distros"
  warn "will need the camera stack sorted out by hand. Continuing anyway."
fi

uname -a

# ---------------------------------------------------------------------------

say "Updating apt"

sudo apt-get update
sudo apt-get -y upgrade

say "Installing system packages"

# python3-picamera2 and python3-opencv come from apt, NOT pip. The pip builds
# either fail to compile on the Pi or install without camera-stack access.
sudo apt-get install -y \
  python3-picamera2 \
  python3-opencv \
  python3-venv \
  python3-pip \
  git

# ---------------------------------------------------------------------------

say "Creating virtualenv at $VENV"

# --system-site-packages is REQUIRED. Without it the venv cannot see the two
# apt-installed packages above and `import cv2` / `import picamera2` both fail.
if [ -d "$VENV" ]; then
  echo "$VENV already exists, leaving it alone"
  if [ ! -f "$VENV/pyvenv.cfg" ] || \
     ! grep -q 'include-system-site-packages = true' "$VENV/pyvenv.cfg"; then
    warn "$VENV was NOT created with --system-site-packages."
    warn "cv2 and picamera2 will not import inside it."
    warn "Fix with:  rm -rf $VENV && bash scripts/setup_pi.sh"
  fi
else
  python3 -m venv --system-site-packages "$VENV"
fi

# shellcheck disable=SC1091
source "$VENV/bin/activate"

python -m pip install --upgrade pip

# ---------------------------------------------------------------------------

say "Installing Python requirements"

if [ ! -f "$REQUIREMENTS" ]; then
  echo "missing $REQUIREMENTS" >&2
  exit 1
fi

pip install -r "$REQUIREMENTS"

# ---------------------------------------------------------------------------

say "Pre-fetching YOLO weights"

# Ultralytics downloads yolov8n.pt on first use. Doing it here means the first
# real run does not stall on a download, and it fails loudly now if there is no
# network rather than mid-session.
python - <<'PY'
from ultralytics import YOLO

model = YOLO("yolov8n.pt")
print("yolov8n.pt ready")
PY

# ---------------------------------------------------------------------------

say "Verifying imports"

python - <<'PY'
import sys

failed = []
for name in ("cv2", "numpy", "yaml", "serial", "requests",
             "picamera2", "ultralytics"):
    try:
        mod = __import__(name)
        version = getattr(mod, "__version__", "?")
        print(f"  ok    {name:<12} {version}")
    except Exception as exc:
        print(f"  FAIL  {name:<12} {exc}")
        failed.append(name)

if failed:
    print()
    print(f"failed imports: {', '.join(failed)}")
    if "cv2" in failed or "picamera2" in failed:
        print("Both of those come from apt and need the venv to have been")
        print("created with --system-site-packages.")
    sys.exit(1)

print("\nall imports ok")
PY

# ---------------------------------------------------------------------------

say "Cameras detected"

# If this list is empty: check the ribbon. The Pi 5 needs a 22-pin to 15-pin
# cable, and at the Pi 5 end the gold contacts face TOWARD the USB ports.
# See docs/build-log.md, 2026-08-17.
if command -v rpicam-hello >/dev/null 2>&1; then
  rpicam-hello --list-cameras || warn "no cameras listed -- check the ribbon"
else
  python - <<'PY'
from picamera2 import Picamera2

cams = Picamera2.global_camera_info()
if not cams:
    print("no cameras found -- check the ribbon cable and its orientation")
else:
    for c in cams:
        print(f"  {c}")
PY
fi

# ---------------------------------------------------------------------------

say "Config"

if [ -f "$REPO_ROOT/vision/config.yaml" ]; then
  echo "vision/config.yaml exists"
else
  cp "$REPO_ROOT/vision/config.example.yaml" "$REPO_ROOT/vision/config.yaml"
  echo "created vision/config.yaml from the example -- EDIT IT"
fi

# ---------------------------------------------------------------------------

cat <<EOF

Done.

Next:
    source $VENV/bin/activate
    \$EDITOR $REPO_ROOT/vision/config.yaml
    python $REPO_ROOT/vision/detect.py --frames 5

Still to fill in: anchor coordinates, steps_per_mm, serial port, ESP32 URL,
and the homography. See docs/calibration.md.
EOF

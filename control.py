#!/usr/bin/env python3
"""petbot control UI: live YOLO-annotated feed plus move, claw and delivery.

    python control.py                 # then open http://<pi-ip>:5000
    python control.py --dry-run       # camera runs; serial and ESP32 are faked
    python control.py --no-camera     # UI + hardware only (e.g. on a laptop)

This is the only place decisions are made. The Arduino receives `M x y z`
over USB serial (control/serial_link.py); the ESP32 on the platform receives
HTTP requests (/open /close /grab /release /face). Neither knows the other
exists.

Everything is read from vision/config.yaml: camera, model, serial port,
ESP32 address, the safe workspace box and the named waypoints.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(REPO_ROOT / "vision"))

from flask import Flask, Response, jsonify, request  # noqa: E402

from config_loader import load_config  # noqa: E402
from control import serial_link  # noqa: E402

app = Flask(__name__)


# ---------------------------------------------------------------------------
# Camera thread: capture -> YOLO -> annotated JPEG, continuously.
# ---------------------------------------------------------------------------
class Camera(threading.Thread):
    BOUNDARY = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"

    def __init__(self, cfg: dict):
        super().__init__(daemon=True, name="camera")
        self.cfg = cfg
        self.lock = threading.Lock()
        self.jpeg: bytes | None = None
        self.status = "starting camera"
        self.fps = 0.0
        self.detections: list[dict] = []

    def run(self) -> None:
        import cv2
        import detect as detect_mod
        from ultralytics import YOLO

        det = self.cfg["detection"]
        model = YOLO(det["model"])
        conf = float(det["confidence"])
        classes = [int(c) for c in det["classes"]]
        picam = detect_mod.open_camera(self.cfg["camera"])

        last = time.time()
        while True:
            frame = picam.capture_array()          # BGR despite "RGB888"
            dets = detect_mod.detect_frame(model, frame, conf, classes)
            canvas = detect_mod.annotate(frame, dets) if dets else frame
            ok, buf = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 80])

            now = time.time()
            fps = 1.0 / max(now - last, 1e-6)
            last = now

            if dets:
                best = max(dets, key=lambda d: d["conf"])
                status = f"{best['name']} {best['conf']:.2f}"
            else:
                status = "nothing detected"

            with self.lock:
                if ok:
                    self.jpeg = buf.tobytes()
                self.status = status
                self.fps = 0.8 * self.fps + 0.2 * fps if self.fps else fps
                self.detections = dets

    def mjpeg(self):
        """Generator for the multipart stream."""
        while True:
            with self.lock:
                jpeg = self.jpeg
            if jpeg is None:
                time.sleep(0.1)
                continue
            yield self.BOUNDARY + jpeg + b"\r\n"
            time.sleep(0.03)


# ---------------------------------------------------------------------------
# Robot: serial to the Arduino, HTTP to the ESP32, and the delivery sequence.
# One hardware command at a time -- moves block for up to a minute.
# ---------------------------------------------------------------------------
class Robot:
    def __init__(self, cfg: dict, dry_run: bool):
        self.cfg = cfg
        self.dry_run = dry_run
        self.lock = threading.Lock()
        self.busy = False
        self.log: list[str] = []
        self.position: dict | None = None
        self.link: serial_link.SerialLink | None = None

        w = cfg["workspace"]
        self.box = {
            "x": (float(w["x_min"]), float(w["x_max"])),
            "y": (float(w["y_min"]), float(w["y_max"])),
            "z": (float(w["z_min"]), float(w["z_max"])),
        }
        self.waypoints = {
            name: tuple(float(v) for v in xyz)
            for name, xyz in cfg["waypoints"].items()
        }

        if not dry_run:
            self.link = serial_link.from_config(cfg)
            self.position = self.link.report_parsed()
            self.say(f"arduino ready on {cfg['serial']['port']}, at {self.position}")
        else:
            self.say("DRY RUN: serial and ESP32 calls are printed, not sent")

    # -- logging ------------------------------------------------------------

    def say(self, msg: str) -> None:
        line = f"{time.strftime('%H:%M:%S')}  {msg}"
        print(line, flush=True)
        self.log.append(line)
        del self.log[:-40]

    # -- primitives ---------------------------------------------------------

    def clamp(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        c = lambda v, lo, hi: max(lo, min(hi, v))  # noqa: E731
        return (c(x, *self.box["x"]), c(y, *self.box["y"]), c(z, *self.box["z"]))

    def move(self, x: float, y: float, z: float) -> None:
        tx, ty, tz = self.clamp(x, y, z)
        if (tx, ty, tz) != (x, y, z):
            self.say(f"target ({x:.0f}, {y:.0f}, {z:.0f}) clamped to "
                     f"({tx:.0f}, {ty:.0f}, {tz:.0f})")
        self.say(f"M {tx:.0f} {ty:.0f} {tz:.0f}")
        if self.dry_run:
            time.sleep(0.5)
            self.position = {"x": tx, "y": ty, "z": tz}
            return
        x2, y2, z2 = self.link.move(tx, ty, tz)
        if self.link.last_warning:
            self.say(self.link.last_warning)
        self.position = {"x": x2, "y": y2, "z": z2}

    def home(self, x: float, y: float, z: float) -> None:
        self.say(f"H {x:.0f} {y:.0f} {z:.0f}")
        if self.dry_run:
            self.position = {"x": x, "y": y, "z": z}
            return
        x2, y2, z2 = self.link.set_home(x, y, z)
        self.position = {"x": x2, "y": y2, "z": z2}

    def motors(self, enable: bool) -> None:
        self.say("E" if enable else "D")
        if not self.dry_run:
            self.link.enable() if enable else self.link.disable()

    def esp32(self, path: str) -> str:
        e = self.cfg["esp32"]
        url = e["base_url"].rstrip("/") + path
        self.say(f"GET {url}")
        if self.dry_run:
            return "dry-run"
        import requests
        r = requests.get(url, timeout=float(e.get("request_timeout_s", 5.0)))
        r.raise_for_status()
        return r.text.strip()

    def claw(self, cmd: str) -> str:
        if cmd not in ("open", "close", "grab", "release"):
            raise ValueError(f"unknown claw command {cmd!r}")
        return self.esp32(f"/{cmd}")

    def face(self, mood: str) -> None:
        try:
            self.esp32(f"/face?m={mood}")
        except Exception as exc:  # noqa: BLE001 - the face is cosmetic
            self.say(f"face failed: {exc}")

    # -- the demo sequence --------------------------------------------------

    def deliver(self) -> None:
        """Park -> bowl -> grab -> lift -> crate -> release -> park."""
        wp = self.waypoints
        settle = float(self.cfg.get("platform", {}).get("settle_s", 1.5))

        self.face("busy")
        self.move(*wp["park"])
        self.claw("open")
        self.move(*wp["bowl"])
        time.sleep(settle)                     # let the platform stop swinging
        self.claw("grab")
        self.face("grab")
        time.sleep(settle)
        self.move(*wp["park"])                 # lift straight up first
        self.move(*wp["crate"])
        time.sleep(settle)
        self.claw("release")
        self.face("happy")
        self.move(*wp["park"])
        self.say("delivery complete")

    # -- background execution ----------------------------------------------

    def start(self, label: str, fn, *args) -> bool:
        """Run one hardware action on a worker thread. False if already busy."""
        if not self.lock.acquire(blocking=False):
            return False
        self.busy = True

        def worker():
            try:
                fn(*args)
            except Exception as exc:  # noqa: BLE001
                self.say(f"{label} FAILED: {exc}")
            finally:
                self.busy = False
                self.lock.release()

        threading.Thread(target=worker, daemon=True, name=label).start()
        return True


# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------
camera: Camera | None = None
robot: Robot | None = None

PAGE = """<!doctype html>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>petbot vision</title>
<style>
  body { margin: 0; padding: 20px 24px; background: #111; color: #ddd;
         font: 15px/1.4 system-ui, -apple-system, sans-serif; }
  h1 { font-size: 24px; margin: 0 0 14px; color: #fff; }
  img { display: block; width: min(100%, 1000px); aspect-ratio: 16/9;
        background: #000; border-radius: 4px; }
  #status { margin: 10px 0 18px; color: #4ade80; }
  #status.busy { color: #fbbf24; }
  .row { display: flex; flex-wrap: wrap; gap: 8px; align-items: center;
         margin: 0 0 10px; }
  .row span.label { width: 64px; color: #888; }
  button { background: #222; color: #eee; border: 1px solid #444;
           border-radius: 4px; padding: 6px 12px; cursor: pointer; }
  button:hover { background: #333; }
  button.go { background: #14532d; border-color: #166534; }
  button.warn { background: #7f1d1d; border-color: #991b1b; }
  button:disabled { opacity: .4; cursor: default; }
  input { width: 70px; background: #000; color: #eee; border: 1px solid #444;
          border-radius: 4px; padding: 5px 6px; }
  pre { background: #000; color: #9ca3af; padding: 10px; border-radius: 4px;
        max-width: 1000px; min-height: 6em; white-space: pre-wrap; margin: 0; }
  small { color: #666; }
</style>

<h1>petbot vision</h1>
<img id="feed" src="/stream" alt="camera feed">
<div id="status">connecting</div>

<div class="row"><span class="label">go to</span>
  __WAYPOINT_BUTTONS__
</div>
<div class="row"><span class="label">move</span>
  x <input id="x" type="number" step="10"> y <input id="y" type="number" step="10">
  z <input id="z" type="number" step="10">
  <button onclick="move()">M x y z</button>
  <button onclick="home()">H (set home here)</button>
</div>
<div class="row"><span class="label">claw</span>
  <button onclick="claw('open')">open</button>
  <button onclick="claw('close')">close</button>
  <button onclick="claw('grab')">grab</button>
  <button onclick="claw('release')">release</button>
</div>
<div class="row"><span class="label">mission</span>
  <button class="go" onclick="post('/api/deliver')">deliver treat</button>
  <button onclick="post('/api/motors/enable')">enable motors</button>
  <button class="warn" onclick="if(confirm('Disabling drops the platform. Sure?'))post('/api/motors/disable')">disable motors</button>
</div>
<pre id="log"></pre>
<p><small>Position is the firmware's belief, not a measurement. Re-home after any skipped steps.</small></p>

<script>
async function post(url, body) {
  const r = await fetch(url, {method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(body || {})});
  if (!r.ok) alert(await r.text());
}
function xyz() { return {x: +x.value, y: +y.value, z: +z.value}; }
function move()  { post('/api/move', xyz()); }
function home()  { post('/api/home', xyz()); }
function goto(n) { post('/api/goto/' + n); }
function claw(c) { post('/api/claw/' + c); }
async function poll() {
  try {
    const s = await (await fetch('/api/status')).json();
    const el = document.getElementById('status');
    let t = s.detection;
    if (s.fps) t += `  ·  ${s.fps.toFixed(1)} fps`;
    if (s.position) t += `  ·  at ${s.position.x.toFixed(0)}, ${s.position.y.toFixed(0)}, ${s.position.z.toFixed(0)}`;
    if (s.busy) t += '  ·  moving';
    if (s.dry_run) t += '  ·  DRY RUN';
    el.textContent = t; el.className = s.busy ? 'busy' : '';
    document.querySelectorAll('button').forEach(b => b.disabled = s.busy);
    document.getElementById('log').textContent = s.log.join('\\n');
    if (s.position && document.activeElement.tagName !== 'INPUT' && !x.value) {
      x.value = s.position.x; y.value = s.position.y; z.value = s.position.z;
    }
  } catch (e) { document.getElementById('status').textContent = 'no connection'; }
  setTimeout(poll, 500);
}
poll();
</script>
"""


@app.route("/")
def index():
    buttons = "".join(
        f'<button onclick="goto(\'{n}\')">{n} ({x:.0f}, {y:.0f}, {z:.0f})</button>'
        for n, (x, y, z) in robot.waypoints.items()
    )
    return PAGE.replace("__WAYPOINT_BUTTONS__", buttons)


@app.route("/stream")
def stream():
    if camera is None:
        return Response("no camera", status=503)
    return Response(camera.mjpeg(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/frame.jpg")
def frame():
    if camera is None or camera.jpeg is None:
        return Response("no frame yet", status=503)
    return Response(camera.jpeg, mimetype="image/jpeg")


@app.route("/api/status")
def status():
    return jsonify(
        detection=camera.status if camera else "no camera",
        fps=camera.fps if camera else 0.0,
        detections=camera.detections if camera else [],
        position=robot.position,
        busy=robot.busy,
        dry_run=robot.dry_run,
        log=robot.log,
    )


def _xyz_from_request() -> tuple[float, float, float]:
    b = request.get_json(force=True, silent=True) or {}
    try:
        return float(b["x"]), float(b["y"]), float(b["z"])
    except (KeyError, TypeError, ValueError):
        raise ValueError("body must be JSON with numeric x, y, z")


def _start(label: str, fn, *args):
    if not robot.start(label, fn, *args):
        return Response("busy: a move or sequence is already running", status=409)
    return jsonify(ok=True, started=label)


@app.route("/api/move", methods=["POST"])
def api_move():
    try:
        x, y, z = _xyz_from_request()
    except ValueError as exc:
        return Response(str(exc), status=400)
    return _start("move", robot.move, x, y, z)


@app.route("/api/home", methods=["POST"])
def api_home():
    try:
        x, y, z = _xyz_from_request()
    except ValueError as exc:
        return Response(str(exc), status=400)
    return _start("home", robot.home, x, y, z)


@app.route("/api/goto/<name>", methods=["POST"])
def api_goto(name: str):
    if name not in robot.waypoints:
        return Response(f"unknown waypoint {name!r}", status=404)
    return _start(f"goto {name}", robot.move, *robot.waypoints[name])


@app.route("/api/claw/<cmd>", methods=["POST"])
def api_claw(cmd: str):
    if cmd not in ("open", "close", "grab", "release"):
        return Response(f"unknown claw command {cmd!r}", status=404)
    return _start(f"claw {cmd}", robot.claw, cmd)


@app.route("/api/deliver", methods=["POST"])
def api_deliver():
    return _start("deliver", robot.deliver)


@app.route("/api/motors/<state>", methods=["POST"])
def api_motors(state: str):
    if state not in ("enable", "disable"):
        return Response("use enable or disable", status=404)
    return _start(f"motors {state}", robot.motors, state == "enable")


# ---------------------------------------------------------------------------

def main() -> int:
    global camera, robot

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None, help="path to config.yaml")
    ap.add_argument("--dry-run", action="store_true",
                    help="print serial and HTTP commands instead of sending")
    ap.add_argument("--no-camera", action="store_true",
                    help="skip the camera and YOLO (UI and hardware only)")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5000)
    args = ap.parse_args()

    cfg = load_config(args.config)
    robot = Robot(cfg, dry_run=args.dry_run)

    if not args.no_camera:
        camera = Camera(cfg)
        camera.start()

    print(f"petbot control UI on http://{args.host}:{args.port}")
    # threaded=True so the MJPEG stream does not starve the API routes.
    app.run(host=args.host, port=args.port, threaded=True, debug=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

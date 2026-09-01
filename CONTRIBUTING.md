# Contributing

Small project, short rules.

## Setup

```bash
git clone <repo> && cd cdpr-petbot
bash scripts/setup_pi.sh          # on the Pi
source ~/cv/bin/activate
cp vision/config.example.yaml vision/config.yaml
```

On a laptop, without camera hardware, `pip install -r vision/requirements.txt`
into any venv is enough for everything except `detect.py` and
`track_platform.py`.

Never commit `vision/config.yaml` — it is gitignored, and it holds your serial
port and network details. Edit `config.example.yaml` when you add a *key*; edit
`config.yaml` when you set a *value*.

## Flashing firmware

**Arduino Uno.** Install the **AccelStepper** library through the Arduino IDE
Library Manager (it ships `MultiStepper`). Open
`firmware/cdpr_controller/cdpr_controller.ino`, select Arduino Uno and the
right port, upload.

Before uploading a change that touches motion, re-read the two hard rules in
`docs/hardware.md` and confirm the pin map still matches the wiring.

**ESP32.** Install the ESP32 board support package and the **ESP32Servo**
library. Put your WiFi credentials in `esp32/platform_node/platform_node.ino`
and **do not commit them** — either leave that edit unstaged, or move the two
`#define`s into a `secrets.h`, which is already gitignored.

## Code style

Python: **black**, default settings.

```bash
pip install black
black vision/ control/
```

Arduino/C++: follow what is already in the sketches — two-space indent, braces
on the same line, `const` on anything that is not meant to change.

## Comments

Comment the *why*, not the *what*. The comments that earn their place in this
repo are the ones recording something that cost time to learn: that
`AccelStepper`'s DRIVER constructor takes STEP before DIR, that Picamera2's
`"RGB888"` hands back BGR, that `MultiStepper` ignores acceleration. Keep
writing those.

## Hardware changes

**Log them in `docs/build-log.md`, with a date.** A dated entry saying what
changed and why is worth more later than a tidy diff.

That includes failures. The most useful entries in that file are the ones about
things that broke.

## Measurements

`docs/results.md` holds measured numbers only.

- Do not fill in a row you have not measured.
- Record the raw samples, not just the mean.
- Record the conditions: drum fill level, whether it was re-homed, model and
  confidence threshold, resolution.
- If a result is bad, write it down anyway. A limitation that is documented is
  a known limitation; one that is not is a bug waiting for someone else to
  find.

The README's **Known limitations** list is meant to stay honest and current.
When you fix something on it, remove it. When you find something new, add it.

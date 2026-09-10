# Build log

Newest entry first. One entry per thing that was learned or broken; add a dated
entry whenever the hardware changes.

> **Seeded entries.** The entries below were reconstructed when this repo was
> scaffolded on 2026-09-01. The events are real; the exact dates are
> approximate. Correct any that are wrong — from this point on, log as you go.

---

## 2026-09-09 — Demo recorded

Full delivery sequence on video: park → treat bowl → grab → lift → crate →
release → park, driven from the `control.py` page while the YOLO feed ran.
`docs/media/demo.mp4`. Platform powered from a USB bank, not the LiPo (see
2026-09-07).

## 2026-09-07 — v2 drum in service; D8 dead; claw power (dates approximate)

Several things landed in the final week and were not logged at the time.
Reconstructed from the code and the README on 2026-09-10.

- **v2 40 mm drum printed and fitted.** One calibration: 16000 steps →
  1524 mm, `steps_per_mm = 10.5`. No repeat samples and no fill-level
  series, so its spread is not known. `docs/results.md` stays empty for v2.
- **Uno D8 stopped driving.** Winch 4 DIR moved to D11. Winch 4's coil
  pairs are also swapped relative to the other three motors — wire colours
  did not identify pairs, a meter did (1–5 Ω across a coil, open across two)
  — so its direction is set in hardware, not with an invert flag.
- **Breadboard power distribution failed under four motors.** Three would
  run and a fourth would drop out, a different one each time. Breadboard
  contacts are rated 1–2 A and get worse with every reseat. Moved VM/GND
  distribution off the breadboard.
- **End effector is a servo claw**, not the gravity gate: SG90 with rubber
  bands on the jaws for compliance, driven by `esp32/esp32_claw.cpp`. One
  servo was destroyed earlier by commanding it past its mechanical limit —
  it stalled at full current until it cooked. The reachable range is now
  found with the horn detached, and the code detaches after every move.
- **SSD1306 face and ArduinoOTA** added to the ESP32 so it can be reflashed
  on the platform without unhooking it.
- **LiPo + TP4056 cannot power the claw.** The servo's surge collapses the
  3.7 V rail and resets the ESP32. Tried 100, 470 and 1470 µF of bulk
  capacitance and ramping the servo in 2° steps; the servo always finishes
  its move, the board always resets. Root cause is the unregulated TP4056
  output path. Demo ran from a 5 V USB power bank; the proper fix is a boost
  converter. `setup()` deliberately does not command the servo, so a
  brownout reset cannot retrigger it and loop.
- **One pulley detached** from its VHB mount during testing. The safety
  tether caught it. Re-stuck; mechanical anchors are on the future-work list.
- **Workspace edge measured.** Pushing along Y, the winch 2 cable went slack
  at y ≈ 3494 mm of 4343 (about 80%). The treat bowl had been against a wall;
  it moved.

## 2026-08-24 — Spool radius variation found; v2 drum designed

Calibrating `steps_per_mm` gave results that would not settle. Eleven samples on
the 15.6 mm core drum ranged from 52.39 to 60.33 mm per revolution, mean 56.72,
sample SD 2.57 mm/rev — 4.5% of the mean. Raw data in `docs/calibration.md`.

The cause is not measurement noise. The line wraps in layers, so the effective
radius the cable actually leaves at depends on how full the drum is, and with a
15.6 mm core each new layer of 1.5 mm line is a large fractional change in
radius. On a nearly empty drum the payout per rev is small; on a full one it is
noticeably larger.

At 2 m of paid-out cable a 4.5% error is roughly ±90 mm of platform position
uncertainty, which is larger than every other error source on the machine
combined. This is currently *the* limiting factor.

Actions:

- v1 spool marked superseded in `cad/README.md`.
- v2 drum designed: **40 mm core**, 30 mm wide, 60 mm flanges, 5 mm D-bore. A
  bigger core makes each layer a much smaller fraction of the radius.
- Adding a **lead-in guide** so the line lays in a single even layer instead of
  piling up. Even wrapping matters as much as core diameter.
- `steps_per_mm` in the firmware left non-`const`, with a note that it may need
  to become a function of paid-out length rather than a scalar.

Not yet resolved. v2 is in design, not printed, and has no dataset of its own.

## 2026-08-17 — Pi 5 camera ribbon orientation

The camera stayed undetected through several reseats. Two separate problems:

1. The Pi 5 uses the **narrow 22-pin** camera connector. The 15-pin ribbon in
   the ArduCam box does not fit it. A 22-pin → 15-pin adapter cable is
   required, and is not included with either the Pi or the camera.
2. Orientation: on the **Pi 5 end, the gold contacts face *toward* the USB
   ports.** Backwards gives no error, no warning, and no camera — just an empty
   list from `rpicam-hello --list-cameras`.

Recorded here because it is invisible from the software side and cost most of
an evening.

## 2026-08-10 — Wrong OS image flashed

Flashed Ubuntu onto the Pi 5 out of habit. `picamera2` and the `rpicam` stack
are packaged for Raspberry Pi OS; getting the camera working under Ubuntu was
not worth the fight.

Reflashed **Raspberry Pi OS (64-bit)** and the camera stack worked from apt with
no manual intervention. `scripts/setup_pi.sh` assumes Raspberry Pi OS for this
reason and installs `python3-picamera2` and `python3-opencv` from apt rather
than pip — which is also why the venv must be created with
`--system-site-packages`.

## 2026-07-27 — TMC2209 destroyed by reversed VM/GND

Wired a driver's motor supply backwards. The driver died instantly and the
supply wire got hot enough to soften its insulation before power was cut. No
fire, but only because someone was standing there.

Two changes, both permanent:

- **Inline 5 A blade fuse** on the 12 V supply. Should have been there from the
  start.
- Polarity now gets checked with a meter before power is applied to anything,
  every time.

Also confirmed the other two rules the hard way while rebuilding: one 100 µF
electrolytic per driver at its own VM/GND pins, and every ground on a single
common net. Both are written up in `docs/hardware.md`.

Cost: one driver. Buy a spare.

## 2026-07-13 — Printer swapped: Ender 3 → Bambu A1 mini

The Ender 3 was consuming more time in tuning than the project was getting back
in parts, and the pulley housings need dimensional repeatability — the 608
bearing pocket and the M8 axle bore have to come out right first time or the
sheave binds.

Switched to a **Bambu A1 mini**. Print settings now standardised at 5 walls,
35–40% infill, PETG preferred, with load running *along* layer lines rather than
across them. See `cad/README.md`.

Constraint worth remembering: the A1 mini's build volume is 180 mm cubed, which
is what sets the 60 mm flange diameter on the v2 spool and means larger parts
have to be split.

## 2026-06-29 — Pivot: laundry picker → pet monitoring

The project started as a cable robot that would pick laundry off a bedroom
floor. Grasping arbitrary crumpled fabric turned out to be a hard manipulation
problem in its own right, entirely separate from the cable-robot problem that
was actually interesting.

Pivoted to **pet room monitoring**: the platform carries a camera view and a
gravity treat gate instead of a gripper. The motion problem is unchanged — four
cables, four winches, inverse kinematics — but the end effector becomes a servo
flap that drops a treat, which is tractable.

Everything upstream of the end effector carried over unchanged.

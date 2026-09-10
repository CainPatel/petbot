# How it works

This is the maths and the software, explained for someone who built the
hardware and wants to understand the rest. It starts from the physical
picture and only introduces an equation when the picture needs one.

Contents:

1. [The one idea](#1-the-one-idea)
2. [Coordinates and anchors](#2-coordinates-and-anchors)
3. [Inverse kinematics: position to cable lengths](#3-inverse-kinematics-position-to-cable-lengths)
4. [Cable lengths to motor steps](#4-cable-lengths-to-motor-steps)
5. [Homing: how the firmware knows where it is](#5-homing-how-the-firmware-knows-where-it-is)
6. [Straight lines: coordinating four motors](#6-straight-lines-coordinating-four-motors)
7. [Why forward kinematics is not solved](#7-why-forward-kinematics-is-not-solved)
8. [Tension and the reachable workspace](#8-tension-and-the-reachable-workspace)
9. [Where the error comes from](#9-where-the-error-comes-from)
10. [The three computers](#10-the-three-computers)
11. [The Arduino firmware](#11-the-arduino-firmware)
12. [The serial link](#12-the-serial-link)
13. [Vision: camera, YOLO, homography](#13-vision-camera-yolo-homography)
14. [control.py: the page you drove the demo from](#14-controlpy-the-page-you-drove-the-demo-from)
15. [The ESP32 claw node](#15-the-esp32-claw-node)
16. [mission.py: the autonomous loop](#16-missionpy-the-autonomous-loop)
17. [Glossary](#17-glossary)

---

## 1. The one idea

Everything in the maths comes from a single physical fact: **a taut cable is
a straight line, and its length is the distance between its two ends.**

One end of each cable is fixed at a pulley in a ceiling corner. The other end
is on the platform. So if you know where the platform is, you know how long
each cable must be. That is the whole inverse kinematics, and it is why the
"robot" needs no more computation than a distance calculation.

The rest of this document is that fact, applied four times, plus the
practical business of turning a length in millimetres into a number of motor
steps, and of making four motors move together.

## 2. Coordinates and anchors

To talk about "where the platform is" you need a coordinate frame. The one in
this project:

- **Origin** is the floor corner under winch 3, the corner below the camera.
- **X** runs along one wall, **Y** along the other, **Z** is straight up.
- Everything is in **millimetres**.

The four **anchors** are the points where each line leaves its pulley sheave.
Not the pulley axle, not the winch: the point the line actually pivots about
as the platform moves. Their coordinates, as measured with a tape:

| Anchor | X | Y | Z |
|---|---|---|---|
| A1, winch 1 (by the door) | 3632 | 4343 | 2794 |
| A2, winch 2 | 3632 | 0 | 2794 |
| A3, winch 3 (origin corner) | 0 | 0 | 2794 |
| A4, winch 4 | 0 | 4343 | 2794 |

These four numbers per anchor are the only description of the room the
firmware has. It lives in `A[4][3]` in `firmware/uno_winches.cpp`, and a
copy lives in `vision/config.yaml` for the Pi. If the two copies disagree the
robot and the camera will confidently disagree about where the platform is.

The platform is treated as a single **point**. Its four cable ears are close
enough together that, at room scale, the difference does not matter. That is
an approximation and it is one of the error sources in section 9.

## 3. Inverse kinematics: position to cable lengths

"Inverse kinematics" sounds grand. Here is all it means.

**Kinematics** is the relationship between what the motors do and where the
platform ends up. **Forward** kinematics goes from motor positions to
platform position. **Inverse** kinematics goes the other way: given where you
want the platform, what must each motor do?

For this machine the inverse direction is the easy one. Take a target point
**P** = (x, y, z) and an anchor **A**ᵢ = (xᵢ, yᵢ, zᵢ). The cable from that
anchor to the platform is a straight line, and its length is the
three-dimensional distance between the two points. That is Pythagoras,
applied twice:

```
Lᵢ = √( (x − xᵢ)² + (y − yᵢ)² + (z − zᵢ)² )
```

Read it as: how far apart are they along X, along Y and along Z; square each,
add them, take the square root. Do that once per anchor and you have the four
cable lengths. No iteration, no matrices, no trigonometry. Four square roots.

### A worked example with the real numbers

Send the platform to the **park** waypoint, P = (1500, 1750, 2400).

Cable 1 to anchor A1 = (3632, 4343, 2794):

```
dx = 1500 − 3632 = −2132
dy = 1750 − 4343 = −2593
dz = 2400 − 2794 =  −394

L1 = √(2132² + 2593² + 394²)
   = √(4 545 424 + 6 723 649 + 155 236)
   = √11 424 309
   ≈ 3380 mm
```

The same arithmetic for the other three:

| Cable | Anchor | Length at park (mm) |
|---|---|---|
| 1 | (3632, 4343, 2794) | 3380 |
| 2 | (3632, 0, 2794) | 2786 |
| 3 | (0, 0, 2794) | 2338 |
| 4 | (0, 4343, 2794) | 3021 |

Now drop to the **bowl**, P = (1500, 1750, 1800). Only z changed, by 600 mm:

| Cable | Length at bowl (mm) | Change (mm) |
|---|---|---|
| 1 | 3501 | +121 |
| 2 | 2932 | +146 |
| 3 | 2510 | +172 |
| 4 | 3156 | +135 |

Two things worth noticing. Descending 600 mm only pays out 121 to 172 mm of
cable, because the cables run diagonally and most of their length is
horizontal. And every cable changes by a *different* amount, which is the
problem section 6 solves.

The code that does this is `computeLengths()` in
`firmware/uno_winches.cpp`. It is nine lines.

## 4. Cable lengths to motor steps

The motor does not know about millimetres. It knows about **steps**: a
stepper turns a fixed small angle per pulse. So the last thing the firmware
does is convert each length into a step count:

```
stepsᵢ = Lᵢ × steps_per_mm
```

Where does `steps_per_mm` come from? Two facts multiplied together.

**Steps per revolution.** The NEMA 17 has 200 full steps per turn (1.8° each).
The TMC2209 with MS1 and MS2 unconnected runs 1/8 microstepping, so the
driver expects 8 pulses per full step. 200 × 8 = **1600 steps per
revolution**.

**Millimetres per revolution.** One turn of the drum pays out one
circumference of line. That is π × (effective diameter). For the 40 mm core
drum you measured it directly: 16000 steps, which is 10 revolutions, paid out
1524 mm, so **152.4 mm per revolution**.

Divide: 1600 / 152.4 = **10.5 steps per mm**. That is the number in the
firmware.

A side note that falls out of the measurement: 152.4 mm per revolution means
an effective diameter of 152.4 / π = 48.5 mm. The core is 40 mm. So during
that calibration there were about three layers of 1.5 mm line on the drum.
That is exactly the problem in section 9: the *effective* diameter depends on
how much line is wound on, so `steps_per_mm` is only right at the fill level
you measured it at.

The `(long)` cast in the code just rounds to a whole number of steps. You
cannot command a fraction of a microstep.

## 5. Homing: how the firmware knows where it is

Steppers have no idea where they are. They count pulses from wherever they
were when the power came on. So the firmware needs to be *told* the starting
position once, and from then on it tracks position by counting.

That is what `H x y z` does. `setHome()` computes the four cable lengths for
the position you typed, converts them to steps, and writes each motor's step
counter to that value with `setCurrentPosition()`. Nothing moves. The
firmware now believes the platform is at (x, y, z), and every later `M`
command is computed relative to that belief.

On boot the firmware calls `setHome(2232, 1829, 1600)` automatically, which
is the room centre at about 1.6 m. That is only correct if the platform was
actually parked there when the Uno powered up. If it was not, send a real `H`
before the first move.

### Why the winch-to-pulley run does not matter

Each line actually runs from the winch on the floor, up the corner, over the
pulley, then out to the platform. The firmware only models the last part,
pulley to platform. Is that a mistake?

No, and the reason is homing. The floor-to-pulley run is a **constant**
length: the winch and the pulley are both fixed to the building. A constant
added to every cable length is indistinguishable from a constant added to
the motor's zero point, and homing sets that zero point. So the constant
simply never appears. This is also why the anchor is defined as the pulley
exit point rather than the winch.

## 6. Straight lines: coordinating four motors

From section 3, moving park to bowl needs cable 3 to lengthen by 172 mm and
cable 1 by only 121 mm. Suppose you drove all four motors at the same speed.
Cable 1 would finish first, then cable 2, then 4, then 3. During the move
the platform would be pulled off course by whichever cables were still
running, and it would trace a curve, not a line.

The fix is to make every motor **finish at the same instant**. If cable 3 has
the most steps to go, it runs at full speed, and every other motor runs at
full speed × (its steps / cable 3's steps). Cable 1 would run at about 70%
speed. Now all four lengths change *in proportion* throughout the move, and
a platform whose four cable lengths change in proportion moves in a straight
line.

This is exactly what the AccelStepper library's `MultiStepper` class does.
`winches.moveTo(targets)` takes the four step targets and works out the four
speeds; `runSpeedToPosition()` then pulses all four until they arrive.

Two consequences you will have felt on the hardware:

- **`MultiStepper` ignores acceleration.** It runs each motor at a constant
  speed with an instant start and stop. That is why `setMaxSpeed(600)` is
  modest: a stepper asked to start from rest at too high a speed does not
  start, it skips, and a skipped step corrupts the position count silently.
- **`runSpeedToPosition()` blocks.** The Uno does nothing else until the move
  ends, including reading serial. A move cannot be aborted from the host.
  `D` disables the drivers, but that drops the platform. It is not a brake.

The straight line is a statement about the *commanded* path. Cable stretch,
spool variation and the pulley exit point moving all bend the real one.

## 7. Why forward kinematics is not solved

Going the other way, from four cable lengths back to one position, is
harder than it looks. Three cable lengths already pin down a point (three
spheres intersect at two points, and you know which one is below the
ceiling). A fourth length is extra information. On a perfect machine it
would agree; on a real one it never quite does, so there is no single point
that satisfies all four, and you would have to pick a best fit.

There is also nothing to feed it. The only cable lengths the Uno knows are
the ones it commanded, so "solving" forward kinematics would return the
commanded position and tell you nothing new. Real ground truth has to come
from outside the motors, which is what the camera and the ArUco marker in
`vision/track_platform.py` are for.

## 8. Tension and the reachable workspace

A cable can pull but never push. So for the platform to sit still at a
point, gravity plus the four cable pulls must add to zero **using only
positive tensions**. Write the direction of cable *i* from platform to anchor
as the unit vector **û**ᵢ. Then:

```
T₁û₁ + T₂û₂ + T₃û₃ + T₄û₄ + m·g = 0      with every Tᵢ > 0
```

Three equations (X, Y, Z) and four unknowns (the tensions), so there is a
family of solutions rather than one. That extra freedom is what makes it
possible to find a set where every tension is positive. But only inside part
of the room.

The intuition: near the middle of the room, each cable points up and
outward toward its own corner, so together they can pull the platform in
any horizontal direction and up. Push the platform toward one wall and the
two cables on that side steepen until they lie almost in the wall's plane.
They can now only pull along the wall and up. Nothing pulls *toward* the
wall, while the two far cables pull *away* from it. The far cables win, the
near ones go slack, and the platform swings back. That is what you saw at
y ≈ 3494 mm: the winch 2 cable went slack at about 80% of the way to the
wall.

The firmware does not solve the tension equations. `inWorkspace()` checks a
plain box (x 400 to 3200, y 500 to 3800, z 300 to 2300) and prints a warning
if the target is outside it, then moves anyway. The clamp in `control.py`
and `control/mission.py` is the real guard.

## 9. Where the error comes from

If you command (1500, 1750, 2400) the platform will not be at exactly
(1500, 1750, 2400). The sources, roughly in order of size:

**Spool radius.** The drum's effective diameter grows as line piles up in
layers. On the 15.6 mm core the payout per revolution varied by ±3.5% within
a fill level and 44% across the drum. At 2 m of paid-out cable, 3.5% is
±70 mm. The 40 mm core shrinks the per-layer change from 19% to 7.5% but has
only been calibrated once, so its actual spread is unknown.

**Skipped steps.** Open-loop steppers do not report a skip. The count keeps
going, the drum does not. The error is permanent until you re-home, and its
signature is drift that grows over a session.

**Anchor measurement.** The four anchor coordinates came from a tape measure.
An error there is systematic: it shifts every commanded position in the same
region the same way and does not average out.

**Sheave exit point.** The line leaves the pulley at a slightly different
point depending on which direction it heads off in. About ±15 mm per cable.
Not modelled.

**Point-mass platform.** The four ears are not one point, and the platform
can tilt. Small at this scale.

`docs/results.md` is the place these get measured. It is deliberately empty
until they are.

## 10. The three computers

There are three processors and each has exactly one job.

| Board | Job | Talks to |
|---|---|---|
| Raspberry Pi 5 | Sees the room, runs the web page, decides everything | Arduino over USB serial, ESP32 over WiFi |
| Arduino Uno | Turns "go to x y z" into pulses on four stepper drivers | Nothing but the Pi |
| ESP32 | Opens and closes the claw, draws the face | Nothing but the Pi |

The Arduino and the ESP32 never communicate. This was the best decision in
the build. When the claw's power supply was resetting the ESP32 every move,
the winches kept working and you could keep testing. When the D8 pin died on
the Uno, the claw did not care.

It also keeps each program small enough to read in one sitting: the Uno
sketch is about 120 lines, the ESP32 sketch about 150.

## 11. The Arduino firmware

`firmware/uno_winches.cpp`, top to bottom:

1. **Four `AccelStepper` objects**, one per winch, each told its STEP and DIR
   pins. Note the constructor order is STEP then DIR; winch 4 uses D11 for
   DIR because D8 died.
2. **The anchor table** `A[4][3]` and the **safe box** `inWorkspace()`.
3. **`steps_per_mm = 10.5`**, deliberately not `const`, because the long-term
   fix for spool variation is to make it a function of paid-out length.
4. **`computeLengths()`**: the four square roots from section 3, times
   `steps_per_mm`.
5. **`moveToXYZ()`**: warn if outside the box, compute the four targets, hand
   them to `MultiStepper`, block until done, remember the new position.
6. **`setHome()`**: section 5.
7. **`setup()`**: open serial at 115200, enable the drivers (EN is active
   low, so `LOW` means on), set max speed, register the four motors with
   `MultiStepper`, assume the platform is at room centre, print `ready`.
8. **`loop()`**: read one character. `M` reads three floats and moves. `R`
   reports. `H` homes. `D` and `E` toggle the driver enable pin.

`Serial.parseFloat()` skips anything that is not part of a number, which is
why `M 1500 1750 2400` works with spaces between the values and a newline at
the end.

## 12. The serial link

`control/serial_link.py` is a small Python wrapper so the rest of the code
never touches raw serial.

The one non-obvious thing it handles: **opening the port resets the Uno.**
The USB-serial chip toggles the DTR line when a program opens the port, and
the Uno treats that as a reset. So the wrapper opens the port, then waits up
to ten seconds for the firmware to print `ready` before it sends anything.
Without that, the first command lands during the bootloader and is lost.

Each method sends one command and waits for the reply line that command
produces (`moved:`, `pos:`, `home:`, `enabled`, `disabled`). Because a move
blocks on the Uno, `move()` waits up to 60 seconds; everything else 2. If the
firmware printed its `WARN: outside safe workspace` line first, the wrapper
keeps it in `last_warning` so the caller can show it.

## 13. Vision: camera, YOLO, homography

Three pieces, in the order the data flows.

**Camera.** `vision/detect.py` opens the IMX708 through Picamera2 at
1280×720 with **manual focus**. The room does not move, and autofocus
hunting would make every frame slightly different, which matters for the
homography below. `capture_array()` returns the frame as a NumPy array,
which is just a 720 × 1280 × 3 grid of numbers, one triple of colour values
per pixel.

The gotcha you hit: Picamera2's format called `RGB888` hands back the bytes
in **BGR** order. OpenCV and Ultralytics both *expect* BGR, so the right
amount of colour conversion is none. Adding one gives blue dogs.

**YOLO.** YOLOv8n is a neural network trained on the COCO dataset, which has
80 object classes including cat (15) and dog (16). You give it a frame, it
returns a list of **detections**: a bounding box, a class and a confidence
between 0 and 1. The code keeps only cats and dogs above 0.4 confidence.
"n" is the smallest variant, which is why it runs on the Pi's CPU at all,
at about 3 frames per second.

For each detection the code takes the **bottom centre** of the box, not the
middle. The middle of a dog is in mid-air. The bottom centre is where it
touches the floor, and the floor is the only surface the next step can map.

**Homography.** The camera sees the floor as a distorted quadrilateral. A
homography is a 3×3 matrix that converts a pixel on that quadrilateral into
a floor coordinate in millimetres, and it is fully determined by four known
points: put four markers on the floor, measure where they really are with a
tape, note which pixel each appears at, and `cv2.findHomography` solves for
the matrix. `vision/calibrate_homography.py` does that and saves it;
`pixel_to_floor()` applies it. It is only valid for points *on the floor*,
which is why the bottom-centre pixel matters.

The output of the whole chain is: "there is a dog, at floor position
(x, y) mm, with confidence 0.87." That is what `mission.py` flies to.

## 14. control.py: the page you drove the demo from

`control.py` is a Flask web server. Flask is a small Python library that
lets a function answer a web address: `@app.route("/api/status")` means
"when a browser asks for `/api/status`, run this function and send back
what it returns."

Three things run at once, on separate threads:

**The camera thread** loops forever: capture a frame, run YOLO, draw the
boxes, compress to JPEG, store the bytes and a status string like
`nothing detected` or `dog 0.87`. It never talks to the hardware.

**The Flask thread** answers browser requests. `/stream` sends the stored
JPEGs one after another as a **multipart MJPEG stream**, which is a format
browsers know how to show in a plain `<img>` tag, so the live feed needs no
JavaScript at all. `/api/status` returns the status string, the frame rate,
the firmware's believed position and the log as JSON, and the page polls it
twice a second to update the green line and the buttons.

**A worker thread**, started on demand, runs any hardware action: a move, a
claw command, the whole delivery sequence. It exists because a move blocks
for up to a minute and the page must stay responsive while it does. A lock
ensures only one worker runs at a time; a second request while one is
running gets HTTP 409 and the page shows its buttons disabled.

Every move goes through `clamp()` first, which pushes the target inside the
box from `config.yaml`. The delivery sequence is a plain list of steps:

```
face busy → park → claw open → bowl → wait → grab → face grab → wait
→ park (lift) → crate → wait → release → face happy → park
```

The `wait`s are `settle_s` from the config, 1.5 seconds, to let the platform
stop swinging before the claw acts. The lift back to park before the
traverse is what keeps the treat from being dragged across the bowl.

`--dry-run` replaces the serial and HTTP calls with printed lines, so you can
see the exact command sequence without any cables moving. `--no-camera`
skips YOLO for testing on a laptop.

## 15. The ESP32 claw node

`esp32/esp32_claw.cpp` connects to WiFi with a fixed IP and runs a tiny HTTP
server. Each address does one thing: `/open`, `/close`, `/grab`,
`/release`, `/face?m=happy`, `/status`. The Pi drives it with ordinary HTTP
GET requests, the same thing a browser does when you type a URL.

The servo handling reflects two hardware lessons:

- **`attach()`, `write()`, wait, `detach()`.** A detached servo receives no
  signal and applies no torque, so it cannot stall against a stop and cook
  itself, and it draws no current between moves. `/close` is the exception:
  it stays attached so the jaws hold against the rubber bands.
- **`setup()` never moves the servo.** If it did, a brownout reset would
  retrigger the move, brown out again, and loop forever. That was the
  failure mode on the LiPo.

Everything else is quality of life. `ArduinoOTA` lets you reflash the board
over WiFi while it hangs from the ceiling. The U8g2 library draws the three
faces on the SSD1306. `/status` returns uptime, WiFi signal and the last
commanded angle.

## 16. mission.py: the autonomous loop

`control/mission.py` is the "see a pet, go to it, drop a treat" loop that
strings sections 13, 12 and 15 together without a human:

1. Load `config.yaml` and the saved homography.
2. Capture a frame, run YOLO, take the most confident cat or dog.
3. Map its floor-contact pixel through the homography to (x, y) mm.
4. Clamp that into the safe box and send `M x y cruise_z`.
5. Call the ESP32's `/release` to drop the treat.
6. Wait `--interval` seconds and repeat, or stop after one with `--once`.

`--dry-run` prints every command instead of sending it. Run that first,
every time.

## 17. Glossary

- **Anchor**: the point where a line leaves its pulley. The fixed end of a
  cable, as far as the maths is concerned.
- **CDPR**: cable-driven parallel robot. "Parallel" because all four cables
  act on the platform at once, as opposed to a serial arm where joints are
  chained one after another.
- **Homography**: a 3×3 matrix that maps points on one flat surface (the
  camera's image of the floor) to points on another (the real floor).
- **Inverse kinematics (IK)**: computing what the motors must do to put the
  platform at a chosen point. Here, four distances.
- **Microstepping**: driving a stepper in fractions of a full step. 1/8 here,
  giving 1600 steps per turn instead of 200.
- **MJPEG**: a stream of JPEG images sent one after another over one HTTP
  connection. Browsers display it as live video.
- **Open loop**: no sensor confirms that a commanded motion happened. The
  firmware trusts its own count.
- **Positive tension**: a cable pulling. Zero tension is slack; negative
  would be pushing, which a cable cannot do.
- **steps_per_mm**: how many step pulses pay out one millimetre of line.
  Depends on microstepping and the drum's *effective* diameter.
- **YOLO**: "You Only Look Once", a family of object-detection networks.
  YOLOv8n is the smallest of the v8 generation.

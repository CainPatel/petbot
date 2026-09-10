# Build guide

How to build your own petbot from the parts list in this repo. Written for
someone comfortable with a 3D printer, a soldering iron and a multimeter,
who has not built a cable robot before.

Budget about **$350** in parts and **three weeks** of evenings. The order below
is the order that would have saved the most time on the original build:
measure and test the scary things first, and keep every subsystem working on
its own before joining them.

Contents:

1. [Read this first: safety](#1-read-this-first-safety)
2. [Tools and parts](#2-tools-and-parts)
3. [Print the parts](#3-print-the-parts)
4. [Build one winch and test it alone](#4-build-one-winch-and-test-it-alone)
5. [Build the other three and the power distribution](#5-build-the-other-three-and-the-power-distribution)
6. [Assemble and mount the corner pulleys](#6-assemble-and-mount-the-corner-pulleys)
7. [Measure the room](#7-measure-the-room)
8. [Flash the Arduino](#8-flash-the-arduino)
9. [Build the platform and claw](#9-build-the-platform-and-claw)
10. [Flash the ESP32](#10-flash-the-esp32)
11. [String the lines and home](#11-string-the-lines-and-home)
12. [Calibrate steps per millimetre](#12-calibrate-steps-per-millimetre)
13. [Set up the Raspberry Pi](#13-set-up-the-raspberry-pi)
14. [First flight](#14-first-flight)
15. [Run the delivery sequence](#15-run-the-delivery-sequence)
16. [Troubleshooting](#16-troubleshooting)

---

## 1. Read this first: safety

This is a 350 g object hanging over a room from four fishing lines glued to
the ceiling. Treat it that way.

- **Safety tether, always.** A slack line from the platform to something
  solid, independent of the four cables and the pulleys. One pulley came off
  the ceiling during the original build and the tether is the reason there
  was nothing to clean up.
- **Never leave it powered and unattended.** There is no closed-loop
  control and no limit switch. A skipped step is silent.
- **Check polarity with a meter before every first power-up.** Reversing VM
  and GND on a TMC2209 kills it instantly and heats the supply wire enough to
  melt insulation. The inline fuse is not optional.
- **Disabling the motors drops the platform.** `D` is not a brake. Park the
  platform somewhere soft, or hold it, before you send `D`.
- **Keep pets out of the room until the workspace is proven.** The whole
  point is a pet room, but not while you are still finding the edges.

## 2. Tools and parts

Parts and prices are in the [README bill of materials](../README.md#bill-of-materials)
with the reasoning for each in [hardware.md](hardware.md). Beyond those:

| Tool | Used for |
|---|---|
| 3D printer, ~180 mm build volume | spools, pulley housings, brackets, platform, claw |
| Multimeter | VREF, coil pairs, polarity, continuity |
| Soldering iron | driver headers, capacitor leads, screw terminal pigtails |
| Tape measure, 5 m | anchors, calibration, everything |
| Plumb line (a nut on a string) | dropping anchor points to the floor |
| Small screwdrivers, hex keys | pulley axles, brackets, servo horn |
| Isopropyl alcohol | surface prep for the VHB tape |
| Laptop with PlatformIO installed | flashing both boards |

Buy a **spare TMC2209 and a spare SG90**. The original build destroyed one of
each, and both were the cheapest parts in the box.

## 3. Print the parts

The printable files are hosted on [cainpatel.com](https://cainpatel.com/projects/petbot/),
not in this repo. [cad/README.md](../cad/README.md) is the index of what
each part is, what it was dimensioned around, and whether it worked.

| Part | Qty | Notes |
|---|---|---|
| Spool, v2 (40 mm core) | 4 | do not print v1; its 15.6 mm core is why section 12 exists |
| Corner pulley housing, 45° | 4 | takes a 608 U-groove bearing on an M8 axle |
| NEMA 17 bracket | 4 | standard 31 mm bolt pattern |
| Platform box | 1 | four symmetric cable ears, cutouts for the OLED and servo stalk |
| Claw | 1 | two jaws, servo horn mount; design inspired by [this SG90 gripper on GrabCAD](https://grabcad.com/library/gripper-servo-sg90-1) |

Settings that matter: **PETG**, 5 walls, 35 to 40% infill. PLA creeps under
constant load and the pulley housings are loaded every second the machine is
on.

Orientation matters more than the settings. Every one of these parts carries
cable tension, and FDM parts fail between layers. Print so the pull runs
*along* a layer, not across the layer boundary. The [orientation rule](../cad/README.md#orientation-rule)
in the CAD notes goes part by part.

## 4. Build one winch and test it alone

Do one completely before you build four. Everything you learn on the first
one is free on the other three.

1. **Bolt the motor to its bracket.** The 17HS19-2004S has a 5 mm D shaft;
   the v2 spool's D-bore slides straight on. A drop of thread locker on the
   grub screw if the spool has one.
2. **Identify the coil pairs.** Do not trust wire colours. With the motor
   unplugged, measure resistance between wires: **1 to 5 Ω is one coil,
   open circuit means two different coils.** Label them A+ A- B+ B-. One
   motor in the original batch was wired differently from its three
   siblings.
3. **Wire one driver.** BTT TMC2209 v1.2 in standalone mode: no UART, MS1
   and MS2 left unconnected for 1/8 microstepping. Motor coils to the four
   motor pins. STEP and DIR to the Uno (winch 1 is STEP D3, DIR D2). EN to
   D10. Logic VIO from the Uno's 5 V. **Motor VM and GND to the 12 V
   supply through the fuse**, and solder a **100 µF electrolytic across VM
   and GND right at the driver pins**. Polarity of the capacitor matters;
   the stripe is negative.
4. **Check polarity with the meter** before you plug in the supply. Then
   plug it in.
5. **Set VREF.** With 12 V on and the motor connected but idle, measure
   from the trimpot wiper to GND and adjust to **1.10 V**, which is about
   0.78 A RMS on this board's 0.11 Ω sense resistors. Never unplug a motor
   while the driver is powered.
6. **Spin it.** Flash the Uno (section 8) or a one-motor test sketch, and
   drive it back and forth. Wind a metre of line on the spool and watch it
   lay. If the motor hums and holds but does not turn, STEP and DIR are
   swapped: `AccelStepper(DRIVER, step, dir)` takes STEP first.

## 5. Build the other three and the power distribution

Repeat section 4 for winches 2, 3 and 4 using the pin map in
[hardware.md](hardware.md#arduino-uno-pin-map). Note winch 4's DIR is on
**D11**; on the original Uno D8 stopped driving.

Then take the power off the breadboard. Four motors together will not run
from breadboard rails: contacts are rated 1 to 2 A, are springy, and get
worse every time a wire is reseated. The symptom is three motors running and
a fourth dropping out, with a different victim each time. Use **screw
terminals** or a soldered bus for the 12 V and ground distribution.

Two rules with no exceptions:

- One 100 µF capacitor **per driver, at that driver's pins**.
- **Every ground on one net**: PSU negative, Arduino GND, all four driver
  grounds.

Run all four at once for a few minutes before you move on. If one drops out
now, fix it now.

## 6. Assemble and mount the corner pulleys

1. Press the 608 U-groove bearing into the housing. It must be a **U** groove;
   a V groove pinches braided line and wears it through.
2. Fit the M8 × 45 axle with narrow washers each side and a nyloc nut. The
   bearing should spin freely with no side play.
3. Choose the four ceiling corners. They do not have to form a perfect
   rectangle; the maths does not assume one. They do have to be solid.
4. Clean each mounting surface with isopropyl alcohol and let it dry. Apply
   the 3M VHB 5952, press the housing on hard for a full minute, and give it
   a day before loading it if you can.
5. The housing sits at 45° so the line comes straight up the corner from the
   floor and leaves toward the room centre.

VHB is a permanent installation and it is also the weakest link. Mechanical
anchors are on the future-work list for a reason. This is what the tether is
for.

## 7. Measure the room

The firmware's only knowledge of the room is four anchor coordinates. Get
them right and everything else is calibration; get them wrong and nothing
else can fix it.

Follow [calibration.md section 2](calibration.md#2-anchor-coordinates)
exactly. The short version:

1. Pick the origin: the floor corner under winch 3, which is also the corner
   the camera sits above. X along one wall, Y along the other, Z up.
2. For each pulley, hang a plumb line from the **point the line leaves the
   sheave** to the floor, mark it, and measure that mark along the two walls.
   Then measure straight up from the floor to the same point for Z.
3. Sanity-check the two room diagonals with the tape against what your
   coordinates predict. More than 10 mm off means a mistake; find it now.
4. Write the four coordinates into **both** `A[4][3]` in
   `firmware/uno_winches.cpp` and `anchors:` in `vision/config.yaml`.

The original room measured 3632 × 4343 mm with anchors at 2794 mm.

## 8. Flash the Arduino

```bash
pio run -e uno -t upload
pio device monitor -e uno
```

You should see `ready`. Type `R` and Enter: the firmware reports the position
it assumed at boot, the room centre. Type `E` to make sure the drivers are
enabled. Do not send an `M` yet; there are no cables.

## 9. Build the platform and claw

1. Mount the ESP32, the SSD1306 (SDA GPIO 21, SCL GPIO 22) and the servo
   (signal GPIO 13) in the platform box. Route the servo lead down the stalk
   to the claw.
2. **Find the servo's real range before you attach the horn to anything.**
   A servo driven into a mechanical stop stalls at full current and cooks
   itself in under a minute. With the horn loose, command angles from a
   serial sketch and note where the jaws would fully open and fully close.
   Put those numbers in `OPEN_ANGLE` and `CLOSED_ANGLE` in
   `esp32/esp32_claw.cpp` (the original used 100 and 175).
3. Fit rubber bands across the jaws. They are the compliance: the claw has
   no force feedback, and the bands let it close on a treat without the
   servo having to be exactly right.
4. **Power the ESP32 from a USB power bank**, not a LiPo through a TP4056.
   The servo's current surge collapses the unregulated cell voltage and
   resets the board mid-move. Capacitors up to 1470 µF and ramped servo
   moves were both tried and neither fixed it. A 5 V boost converter would;
   see future work.
5. Strap everything down. Nothing on the platform may dangle or swing.
6. Tie the tether on.

## 10. Flash the ESP32

```bash
cp esp32/secrets.example.h esp32/secrets.h              # SSID, WiFi password, OTA password
cp platformio.local.example.ini platformio.local.ini    # the same OTA password, for uploads
pio run -e esp32_usb -t upload                          # first time, over USB
pio device monitor -e esp32                    # watch it join WiFi, note the IP
```

The sketch requests a static IP (192.168.1.119 in the source). Change it to
suit your network, and set the same address in `platformio.ini` under
`[env:esp32]` and in `vision/config.yaml` under `esp32: base_url`. From then
on you can reflash over the air while it hangs from the ceiling:

```bash
pio run -e esp32 -t upload
```

Test it from any machine on the network:

```bash
curl http://192.168.1.119/status
curl http://192.168.1.119/open
curl http://192.168.1.119/grab
curl "http://192.168.1.119/face?m=happy"
```

If the board reboots when the servo moves, the power supply is the problem,
not the code.

Neither the ESP32's HTTP server nor the control page on the Pi has any
authentication. Anyone on your WiFi can move the robot. Keep it on a network
you trust, and never port-forward either of them to the internet.

## 11. String the lines and home

1. Put each winch on the floor directly below its pulley and fix it so it
   cannot tip or slide. It sees the full cable tension.
2. Tie the line to the spool, wind on enough to reach the far side of the
   room plus a margin, run it straight up the corner, over the sheave, and
   out to the platform ear for that corner. Use the same knot on all four.
3. With the drivers **disabled** (`D`), pull the platform by hand to a point
   you can measure: room centre at a convenient height is easiest. Take up
   slack on all four spools by hand until every line is taut.
4. Measure the platform's position from the origin with the tape and tell
   the firmware: `E`, then `H x y z`. The firmware now believes the platform
   is there.
5. Send a small move, 100 mm in one axis: `M x+100 y z`. Watch all four
   motors turn and the platform track. Send it back.

If the platform lurches or a line goes slack on a small move, one motor is
turning the wrong way. That winch's coil pair is reversed; swap one coil's
two wires.

## 12. Calibrate steps per millimetre

The number in the firmware, 10.5, is for the original spool with about three
layers of line on it. Yours will differ. Follow
[calibration.md section 1](calibration.md#1-steps_per_mm): mark the line
where it leaves the drum, command a known number of steps under load,
measure what paid out, repeat at least five times, and repeat again at empty,
half and full drum.

`steps_per_mm = 1600 / (mm per revolution)`. Put the result in
`firmware/uno_winches.cpp` and reflash. Record every raw sample in
[results.md](results.md); the spread is the interesting part.

## 13. Set up the Raspberry Pi

Use **Raspberry Pi OS 64-bit**, not Ubuntu; the camera stack is packaged for
it. Connect the IMX708 with the **22-pin to 15-pin** ribbon, gold contacts
toward the USB ports at the Pi end. Backwards gives no error and no camera.

```bash
git clone https://github.com/CainPatel/petbot.git && cd petbot
bash scripts/setup_pi.sh
source ~/cv/bin/activate
cp vision/config.example.yaml vision/config.yaml
```

Edit `config.yaml`: anchors from section 7, your serial port, the ESP32
address, and `lens_position` as 1 / (camera-to-floor distance in metres).
Then check the camera and the model:

```bash
python vision/detect.py --frames 5
```

For the autonomous loop you also need a homography, which maps floor pixels
to floor millimetres. Put four markers on the floor, measure them, and follow
[calibration.md section 3](calibration.md#3-homography). The manual page in
`control.py` does not need it.

## 14. First flight

```bash
python control.py --dry-run
```

Open `http://<pi-ip>:5000`. You should see the live feed and the status line.
Every button prints the command it *would* send to the log on the page.
Click through a delivery and read the sequence.

Then for real:

```bash
python control.py
```

Move in small steps. Find your workspace edge the way the original did:
walk the platform toward each wall in 100 mm steps until a line goes slack,
note the coordinate, and step back. Set `workspace:` in `config.yaml` inside
those limits. Expect the usable volume to be roughly the inner 70 to 80% of
the anchor footprint, and expect the middle of each wall to be unreachable.

Pick your three waypoints (park, bowl, crate) inside that volume and put them
in `waypoints:`. Check each one is taut on all four lines before you trust
it.

## 15. Run the delivery sequence

Put a treat in the bowl, the bowl under the bowl waypoint, and click
**deliver treat**. The sequence is park, open, bowl, settle, grab, lift,
crate, settle, release, park.

Expect to tune the grip. On the original build every move landed but the
jaws did not close far enough to pick the treat up, and the crate leg flew
empty. The three things to adjust, in order: the bowl waypoint's z (the jaws
hang 140 mm below the cable plane on the original platform), `CLOSED_ANGLE`
in the ESP32 sketch, and the rubber-band tension on the jaws.

Then write down what happened in `docs/build-log.md`, including what did not
work. That is the part of this repo that was most useful to write.

## 16. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Motor hums and holds but never turns | STEP and DIR swapped | constructor takes STEP first |
| One of four motors drops out, different one each time | breadboard power rails | screw terminals for VM and GND |
| Driver dead on first power-up, hot wire | VM/GND reversed | new driver; check with a meter next time; fuse |
| Platform lurches on a small move | one winch turning backwards | swap that motor's coil wires |
| Position drifts over a session | skipped steps | lower `setMaxSpeed`, re-home |
| Position wrong by a consistent amount in one region | anchor coordinate error | remeasure that anchor |
| Payout per revolution will not settle | spool layering | bigger core, even winding, calibrate per fill level |
| ESP32 reboots when the servo moves | unregulated LiPo rail | USB power bank or 5 V boost converter |
| Servo hot and buzzing | driven past its stop | find range with horn detached; `detach()` after moves |
| `rpicam-hello --list-cameras` empty | ribbon backwards or wrong cable | contacts toward USB ports; 22-to-15 pin cable |
| YOLO sees blue dogs | colour conversion added | remove it; RGB888 is already BGR |
| `import picamera2` fails in the venv | venv without system packages | recreate with `--system-site-packages` |
| No `ready` on serial | wrong port, wrong baud, or not flashed | `ls /dev/serial/by-id/`, 115200 |

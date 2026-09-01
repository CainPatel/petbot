# Hardware

## Bill of materials

Costs are approximate, in GBP, for what was actually paid or a realistic
current price. Quantities are for one complete machine.

| Item | Qty | Notes | Approx cost |
|---|---|---|---|
| NEMA 17 stepper (42×42, ~1.5 A/phase, 5 mm shaft) | 4 | One per winch. 5 mm D-shaft matches the v2 spool bore. | £48 (4× £12) |
| TMC2209 stepper driver, BTT V1.1 | 4 | **Standalone mode** — no UART. 0.11 Ω sense resistors, which sets the VREF formula below. One spare is worth buying; see build log. | £24 (4× £6) |
| Arduino Uno | 1 | Runs the IK and MultiStepper group. D2–D10 used. | £20 |
| 12 V PSU, 6 A or better | 1 | Four steppers at ~0.78 A RMS plus driver overhead. Do not undersize. | £15 |
| Inline blade fuse holder + 5 A fuse | 1 | Added after a driver was destroyed by a reversed VM/GND. Non-optional. | £4 |
| 100 µF electrolytic capacitor, 25 V+ | 4 | **One per driver, at its own VM/GND pins.** Not one shared cap on the rail. | £3 |
| U-groove bearing, 608 size (30 mm OD, 8 mm bore, 10 mm wide) | 4 | The corner pulley sheave. U-groove, not V-groove — the line must sit in the channel without pinching. | £10 |
| M8×45 bolt + nyloc nut + narrow washers | 4 sets | Pulley axle. Narrow washers so they clear the printed cheeks. | £6 |
| Dyneema / braided fishing line, ~50 kg rating | 1 spool | Low stretch is the whole point. Nylon monofilament creeps and is unusable here. | £12 |
| 3M VHB 5952 tape | 1 roll | Pulley housing to ceiling corner. See the warning in the README. | £12 |
| Isopropyl alcohol | 1 bottle | Surface prep for the VHB. Adhesion is poor without it. | £4 |
| Raspberry Pi 5, 8 GB | 1 | Runs YOLO and the ArUco tracker. | £75 |
| ArduCam IMX708 | 1 | Autofocus module; driven in **manual** focus mode for a fixed room. | £25 |
| Pi 5 camera ribbon, 22-pin → 15-pin | 1 | The Pi 5 connector is the narrow 22-pin type. The cable in the camera box does **not** fit. | £5 |
| ESP32 DevKit | 1 | On the moving platform. WiFi HTTP server. | £8 |
| SG90 servo (MG996R as the upgrade path) | 1 | Actuates the gravity treat gate. SG90 is adequate for a flap; MG996R if the gate binds. | £3 |
| TP4056 charging module + LiPo (~1000 mAh) | 1 | Platform power. Keep the pack strapped, not dangling. | £10 |
| Safety tether cordage | 1 | Slack line under the platform, anchored independently of the four cables. | £5 |
| Bambu A1 mini + PETG | — | Printer already in hand; filament ~£20/kg. | — |

Rough total, excluding printer and Pi accessories: **£290**.

---

## Wiring

### Arduino Uno pin map

`AccelStepper` in `DRIVER` mode takes **STEP first, then DIR** —
`AccelStepper(AccelStepper::DRIVER, stepPin, dirPin)`. Getting this backwards
produces motors that hum but do not turn, so the table below is written in that
same order.

| Winch | STEP | DIR |
|---|---|---|
| 1 | D3 | D2 |
| 2 | D5 | D4 |
| 3 | D7 | D6 |
| 4 | D9 | D8 |

Shared **EN on D10**, wired to all four drivers' `EN` pins in parallel. EN on
the TMC2209 is **active LOW**: drive it LOW to energise the motors, HIGH to
release them. The firmware drives it HIGH at boot so nothing moves before the
anchor table and home position have been set.

So, in one line each:

```
D2  -> W1 DIR      D3  -> W1 STEP
D4  -> W2 DIR      D5  -> W2 STEP
D6  -> W3 DIR      D7  -> W3 STEP
D8  -> W4 DIR      D9  -> W4 STEP
D10 -> EN (all four drivers, active LOW)
```

### Power

```
12 V PSU (+) ──[ 5 A blade fuse ]──┬── driver 1 VM ──┐
                                   ├── driver 2 VM   │  100 µF electrolytic
                                   ├── driver 3 VM   │  at EACH driver's
                                   └── driver 4 VM ──┘  own VM/GND pins

12 V PSU (−) ──┬── driver 1..4 GND
               ├── Arduino GND
               └── (single common ground net — see rule 2 below)
```

The Arduino is powered over USB from the Pi. The 12 V rail powers only the
motor side of the drivers. The drivers' logic side takes its 5 V/3.3 V
reference from the Arduino header.

---

## Driver setup

**Mode.** BTT TMC2209 V1.1 boards in **standalone** mode — no UART, no jumper
on the UART pads, configuration is entirely by pin state.

**Microstepping.** With MS1 and MS2 both left unconnected, the TMC2209 runs
**1/8 microstepping**. With a 1.8°/step motor:

```
200 full steps/rev × 8 = 1600 steps/rev
```

That 1600 is the number the firmware's `steps_per_mm` is derived from — see
`docs/calibration.md`.

**Current (VREF).** The BTT V1.1 board uses 0.11 Ω sense resistors, which gives

```
I_RMS ≈ VREF × 0.71
```

Target **VREF ≈ 1.10 V**, i.e. **≈ 0.78 A RMS** per motor. Set it with the
board's trimpot, measured between the pot wiper and GND, with 12 V applied and
the motor connected but idle. Do not unplug a motor while the driver is
powered.

### Two hard rules

1. **One 100 µF electrolytic capacitor per driver, physically at that driver's
   own VM and GND pins.** Not one big cap at the PSU, not one cap for the
   board. Stepper drivers dump large current transients into the rail every
   commutation; without local bulk capacitance the resulting voltage spikes
   kill the driver. This is the single most common way TMC2209s die.

2. **All grounds tied to one net.** PSU negative, Arduino GND, and every
   driver's GND must be electrically common. STEP and DIR are voltage levels
   referenced to ground; if the Arduino's ground and the driver's ground float
   relative to each other, the step pulses are meaningless and the behaviour is
   erratic rather than obviously broken.

And one earned warning, from `docs/build-log.md`: **check VM/GND polarity
before applying power.** Reversing them destroys the driver instantly and heats
the supply wire enough to melt insulation. The inline 5 A fuse exists because
of that failure.

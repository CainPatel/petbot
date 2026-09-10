# Calibration

Three procedures. Each is written so that someone who has never touched this
machine can repeat it and get a comparable number.

Do all three in this order, the homography depends on nothing, but the anchor
coordinates and `steps_per_mm` both feed the firmware, and validating motion
before those are right will only waste line.

---

## 1. `steps_per_mm`

### What is being measured

How far the cable pays out per motor revolution. With 1/8 microstepping and a
1.8° motor there are **1600 steps/rev** (see `docs/hardware.md`), so

```
steps_per_mm = 1600 / (mm paid out per revolution)
```

This is *not* a fixed property of the spool. The line wraps in layers, so the
effective radius grows as the drum fills and shrinks as it empties. The number
you get depends on how much line was already on the drum when you measured.

### Procedure

1. **Load the drum realistically.** Wind on the amount of line the machine will
   actually carry. Measuring a bare drum gives a number that is wrong
   everywhere except at one end of travel.
2. **Apply load.** Hang the platform, or an equivalent mass, from the line.
   Dyneema under tension seats differently in the wrap than slack line does,
   and an unloaded measurement reads consistently high.
3. **Mark the reference.** Put a fine marker line across both the cable and a
   fixed point on the bracket, at the point where the line leaves the drum.
4. **Command a known step count.** One full revolution, 1600 steps, is the
   convenient unit. Use several revolutions if the resolution of your tape is
   the limiting factor, and divide.
5. **Measure the paid-out length** from the mark to its new position, in mm,
   with the line still under load.
6. **Repeat at least 5 times.** More is better; the dataset below is 11.
7. **Repeat the whole set at three drum fill levels**, near empty, half, full
  , and record all three separately in `docs/results.md`. A single mean hides
   exactly the error that dominates this machine.
8. Compute mean and spread for each fill level.

### Recorded dataset: 15.6 mm core drum (v1)

11 samples, mm of line paid out per revolution:

```
53.98  60.33  58.74  58.74  58.74  57.15  55.56  53.98  52.39  58.74  55.56
```

| Statistic | Value |
|---|---|
| n | 11 |
| Mean | **56.72 mm/rev** |
| Min | 52.39 mm/rev (−7.6% of mean) |
| Max | 60.33 mm/rev (+6.4% of mean) |
| Sample standard deviation | 2.57 mm/rev (4.5% of mean) |
| Implied `steps_per_mm` at the mean | **28.21** |

> The headline figure quoted in the README is "±3.5%". That is a working
> shorthand, and it is optimistic against this dataset: the true 1σ spread is
> 4.5% and the full range spans −7.6%/+6.4%. Treat 3.5% as a floor, not a
> bound, until the v2 drum has its own dataset.

At 2 m of paid-out cable, a 4.5% length error is roughly **±90 mm** of cable
length error, which is the single largest contributor to platform position
error on this machine. That is the entire reason for the v2 drum redesign
(40 mm core, lead-in guide to force even layering) in `cad/README.md`.

### Recording the result

Put the per-fill-level numbers in the first table of `docs/results.md`, then
copy the working mean into `steps_per_mm` in
`firmware/uno_winches.cpp`. It is deliberately **not**
`const` there, the long-term fix is to make it a function of paid-out length.

---

## 2. Anchor coordinates

### What is being measured

The position of each of the four anchors, in the room frame, in mm.

**The anchor is the point where the line leaves the pulley sheave, not the
pulley axle centre, and not the winch.** The IK measures distance from the
platform to the last point the cable touches before its free span. Using the
axle centre puts every anchor off by roughly the sheave radius (~15 mm here) in
a direction that changes with platform position. Using the winch is off by the
entire vertical run.

This error does not average out. A systematic 20 mm error in one anchor puts a
systematic error into the computed length for that cable at *every* point in
the workspace.

### Procedure

1. **Define the origin.** Pick the floor corner below winch 3, the corner
   the camera sits above. Mark it. X runs
   along one wall, Y along the other, Z is up. Write down which wall is which
   and stick to it, swapping X and Y halfway through is the classic way to
   lose an afternoon.
2. **Measure X and Y for each anchor** by dropping a plumb line (a weight on a
   string) from the pulley's line-exit point to the floor, marking the floor,
   and measuring that mark from the origin along the two walls. Do not measure
   diagonally and trust trigonometry; measure along each wall.
3. **Measure Z** from the floor to the line-exit point, vertically. Measure
   each anchor separately even if the ceiling looks level.
4. **Sanity-check the diagonals.** Measure A1→A3 and A2→A4 directly with the
   tape and compare against the distances implied by your coordinates. If they
   disagree by more than ~10 mm, something is wrong; find it before moving on.
5. **Record to the nearest 5 mm** and be honest about it. Do not write down
   1247 mm when the tape was read to the nearest centimetre.

### Recording the result

Fill in the anchor table in `docs/kinematics.md`, then copy the same values
into **both**:

- `A[4][3]` in `firmware/uno_winches.cpp`
- `anchors:` in `vision/config.yaml`

They must match exactly. A mismatch produces a machine that moves confidently
to the wrong place and a camera that confidently reports it is elsewhere.

---

## 3. Homography

### What is being measured

The 3×3 matrix `H` mapping camera pixel coordinates on the floor plane to real
floor coordinates in mm. This is what turns "the dog is at pixel (812, 640)"
into "the dog is at (1830 mm, 1140 mm)".

A homography is only valid for **one plane**. This one is the floor. Anything
off the floor, including the platform itself, hanging in mid-air, maps
incorrectly through it, which is why `vision/track_platform.py` needs the
platform's marker to be interpreted with that caveat in mind.

### Procedure

1. **Fix the camera and the focus first.** Mount the camera where it will
   permanently live, set manual focus (`AfMode: 0`, `LensPosition` in dioptres
  , see `vision/config.example.yaml`), and do not touch either again. Any
   change to camera pose or lens invalidates `H` completely.
2. **Place four floor markers.** Requirements, all of which matter:
   - **Spread wide.** They should cover as much of the usable floor area as
     possible. Four points bunched in the middle give a matrix that is accurate
     nowhere else.
   - **Not at the frame edges.** Lens distortion is worst there, and a marker
     partly out of frame gives a corner detection that is quietly wrong.
   - **Not collinear.** Any three of the four on a line makes the system
     degenerate.
   - A printed ArUco marker at each point makes step 4 automatic and much more
     repeatable than clicking pixels by eye.
3. **Measure their real positions in mm** from the same origin used for the
   anchors, along the same X and Y walls. Same frame, same origin, mixing
   frames here is a silent, systematic error.
4. **Read their pixel coordinates.** Either enter them manually, or let
   `vision/calibrate_homography.py` find the ArUco corners automatically. Use
   the marker centre consistently for both the pixel and the real measurement.
5. **Compute and save:**

   ```bash
   python vision/calibrate_homography.py --auto      # ArUco corners
   python vision/calibrate_homography.py --manual    # type in pixels
   ```

   This calls `cv2.findHomography` and writes
   `data/calibration/homography.npy`.
6. **Read the self-test.** The script maps the four source points back through
   `H` and prints the residual against the measured destination for each. These
   are fit residuals, not accuracy, four points fit a homography exactly, so
   residuals near zero prove only that the arithmetic ran. They are still worth
   reading: a large residual means a typo.
7. **Validate independently.** This is the step that actually tells you
   anything. Place an object at 6–8 *new* positions that were not used in the
   fit, spread across the workspace. For each: measure the true position with
   the tape, read the mapped position from `pixel_to_floor(u, v, H)`, and
   tabulate commanded vs. measured with an error magnitude.

### Recording the result

Validation points go in `docs/results.md`. Expect the error to grow toward the
edges of frame; note where it becomes unacceptable, because that boundary is
the real limit of the vision system regardless of what the mechanics can reach.

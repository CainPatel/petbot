# Results

**Every table on this page is empty on purpose.** Nothing here is filled in
until it has actually been measured on the machine. If a row has no number, the
measurement has not been done, not "roughly this" and not "should be about".

Procedures for producing each table are in `docs/calibration.md`.

---

## 1. `steps_per_mm` vs. drum fill level

Method: `docs/calibration.md` §1. Record every raw sample, not just the mean.

### v1 drum: 15.6 mm core (superseded)

| Fill level | n | Mean (mm/rev) | Min | Max | SD | SD as % of mean | Implied steps/mm |
|---|---|---|---|---|---|---|---|
| Empty | | | | | | | |
| Half | | | | | | | |
| Full | | | | | | | |
| **Pooled** | 11 | 56.72 | 52.39 | 60.33 | 2.57 | 4.5% | 28.21 |

The pooled row is the dataset recorded in `docs/calibration.md`; it was taken
without separating fill levels, which is precisely the gap the three rows above
it are meant to close.

### v2 drum: 40 mm core

| Fill level | n | Mean (mm/rev) | Min | Max | SD | SD as % of mean | Implied steps/mm |
|---|---|---|---|---|---|---|---|
| Empty | | | | | | | |
| Half | | | | | | | |
| Full | | | | | | | |
| **Pooled** | | | | | | | |

Only one number exists for the v2 drum so far: **16000 steps paid out
1524 mm**, i.e. 152.4 mm/rev, `steps_per_mm = 10.5`. That is a single
sample at one unrecorded fill level, so the table stays empty, it says
nothing about spread.

Target: SD under 1% of mean. If the v2 drum does not beat the v1 spread
substantially, the redesign did not work and that should be written down here
plainly.

---

## 2. Commanded position vs. tape-measured actual

Method: home the platform, command each target, let it settle, measure the
actual platform position with a tape from the room origin. At least 8 points,
spread across the workspace including near the edges of the usable volume , 
not 8 points clustered in the easy middle.

Error magnitude is `√(Δx² + Δy² + Δz²)`.

| # | Cmd X | Cmd Y | Cmd Z | Meas X | Meas Y | Meas Z | ΔX | ΔY | ΔZ | Error (mm) |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | | | | | | | | | | |
| 2 | | | | | | | | | | |
| 3 | | | | | | | | | | |
| 4 | | | | | | | | | | |
| 5 | | | | | | | | | | |
| 6 | | | | | | | | | | |
| 7 | | | | | | | | | | |
| 8 | | | | | | | | | | |

| Summary | Value |
|---|---|
| Mean error (mm) | |
| Max error (mm) | |
| Drum fill level during test | |
| Re-homed before run? | |

---

## 3. ArUco-measured platform position vs. commanded

Method: `vision/track_platform.py`, same commanded points as table 2 so the two
are directly comparable. This isolates how much of the table 2 error the camera
can actually see.

Note the plane caveat from `docs/calibration.md` §3: the homography is fitted to
the floor, and the platform is not on the floor. Record the platform Z for every
row so this can be accounted for later.

| # | Cmd X | Cmd Y | Cmd Z | ArUco X | ArUco Y | ΔX | ΔY | Error (mm) | Marker detected? |
|---|---|---|---|---|---|---|---|---|---|
| 1 | | | | | | | | | |
| 2 | | | | | | | | | |
| 3 | | | | | | | | | |
| 4 | | | | | | | | | |
| 5 | | | | | | | | | |
| 6 | | | | | | | | | |
| 7 | | | | | | | | | |
| 8 | | | | | | | | | |

| Summary | Value |
|---|---|
| Mean error (mm) | |
| Max error (mm) | |
| Detection rate across all attempts | |

---

## 4. YOLO detection rate by scenario

Method: run `vision/detect.py` against each scenario for a fixed number of
frames. Count a frame as a detection if the correct class (15 cat / 16 dog) is
returned above the configured confidence threshold. Record false positives
separately, a system that detects a cushion as a cat is not working, however
good the recall column looks.

| Scenario | Frames | Detections | Rate | Confidence range | False positives |
|---|---|---|---|---|---|
| Standing | | | | | |
| Sitting | | | | | |
| Curled / sleeping | | | | | |
| Partially occluded | | | | | |
| Low light | | | | | |

| Condition | Value |
|---|---|
| Model | |
| Confidence threshold | |
| Resolution | |
| Lens position (dioptres) | |

---

## Interpretation: which error source dominates

*Written once tables 1–3 exist. Do not write it before.*

The question this section has to answer is: of the error measured in table 2,
how much comes from each source?

The candidates, and how to separate them:

- **Spool radius variation.** Table 1 gives its magnitude directly. Multiply the
  SD as a fraction by the typical paid-out length to get expected cable length
  error, and compare against table 2's mean error. If it accounts for most of
  the error, the v2 drum is the priority and nothing else matters much yet.
- **Anchor coordinate error.** Systematic and position-dependent. Its signature
  is error that varies smoothly across the workspace rather than randomly, if
  the table 2 errors all point roughly the same direction in one region and the
  opposite direction in another, suspect the anchors.
- **Sheave exit-point migration.** Also position-dependent, bounded at roughly
  ±15 mm per cable. If total error is well above that, this is not the main
  problem.
- **Skipped steps.** Its signature is error that *grows over the session* and
  is fixed by re-homing. Test for it by repeating point 1 of table 2 at the end
  of the run without re-homing and comparing to the first measurement. If those
  two differ meaningfully, the position estimate is drifting and every other
  number in the table is contaminated.
- **Camera / homography error.** The gap between table 2 and table 3 bounds
  this: table 2 is mechanical truth, table 3 is what the camera thinks, and the
  difference is the vision system's own error.

State the conclusion in one sentence, with the number that supports it.

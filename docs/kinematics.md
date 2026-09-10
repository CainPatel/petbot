# Kinematics

## Coordinate frame

Origin is one floor corner of the room. **X** and **Y** run along the two walls
meeting at that corner. **Z** is up. All lengths in millimetres.

The platform is treated as a **point mass**. Its four cable attachment ears are
close enough together, relative to room scale, that platform orientation is not
modelled. That approximation is one of the error sources listed in the README.

## Inverse kinematics

For platform position **P** = (x, y, z) and anchor *i* at
**A**ᵢ = (xᵢ, yᵢ, zᵢ), the required cable length is just the Euclidean
distance:

```
Lᵢ = ‖P − Aᵢ‖ = √( (x − xᵢ)² + (y − yᵢ)² + (z − zᵢ)² )

steps_i = Lᵢ × steps_per_mm
```

Four independent square roots, no iteration, no matrix. This is why an Arduino
Uno is sufficient: a full four-axis IK solve is four `sqrt()` calls.

`computeLengths(x, y, z, out[4])` in `firmware/uno_winches.cpp`
is exactly this expression.

## What is deliberately not solved

**Forward kinematics.** Going from four measured lengths back to one position
is over-constrained: three lengths already determine a point (up to reflection),
so four measurements of an imperfect system will not intersect anywhere. Solving
it properly means a least-squares fit over the four constraints, and with
open-loop steppers there is nothing to feed it that is more trustworthy than the
commanded position anyway. Ground truth comes from the camera instead, see
`vision/track_platform.py`.

## The fixed winch-to-pulley run

Each cable actually runs winch → corner pulley → platform. The firmware models
only the **pulley → platform** span.

That is legitimate because the winch → pulley run is a **constant length**: the
winch and its pulley are both bolted to the building and neither moves. A
constant offset in Lᵢ is indistinguishable from a constant offset in the
motor's zero position, so it is absorbed entirely by homing. When
`setHome(x, y, z)` runs, it writes each motor's step counter to
`computeLengths(...) × steps_per_mm`, the fixed run simply never appears in
the arithmetic.

The anchor **A**ᵢ is therefore the point where the line *leaves the pulley
sheave*, not the pulley axle centre and not the winch. See
`docs/calibration.md`.

## Positive tension bounds the workspace

Cables pull; they cannot push. For the platform to be in static equilibrium at
**P**, there must exist a set of tensions T₁..T₄ with **every Tᵢ > 0** such
that

```
Σ Tᵢ · ûᵢ + m·g = 0        where ûᵢ = (Aᵢ − P) / ‖Aᵢ − P‖
```

With four cables and three translational degrees of freedom the system is
redundant by one, which is what makes non-negative tension solutions possible at
all, but only inside part of the volume. Near an anchor, or near the floor
under the anchor footprint, or outside the footprint entirely, at least one
cable would have to push, and the platform simply falls or goes slack instead.

Practical consequence: **the usable volume is roughly the inner 70–80% of
the anchor footprint**, and it shrinks as the platform gets close to the
ceiling plane where the anchors are. Measured on this room: pushing along Y,
the winch 2 cable went slack at y ≈ 3494 mm of 4343. The firmware does not
solve the tension problem; `inWorkspace()` checks a conservative axis-aligned
box and **warns** if a target is outside it, then moves anyway. The host
(`control.py`, `control/mission.py`) clamps to the same box, and that clamp is
the real guard.

## Coordinated motion with MultiStepper

`MultiStepper::moveTo()` takes an absolute target for every motor, then scales
each motor's constant speed so that **all four arrive at the same instant**.
Since each cable length changes monotonically and proportionally along the way,
the platform traces an approximately straight line between the two points
rather than an arbitrary curve.

Two properties worth stating plainly, because both surprise people:

- **`MultiStepper` ignores acceleration.** `setAcceleration()` on the member
  `AccelStepper` objects has no effect during a `MultiStepper` move. Motion is
  constant-speed, with an instantaneous start and stop. Keep `setMaxSpeed()`
  low enough that the motors can start at that speed without stalling , 
  a stall here is a silent skipped step, which corrupts position permanently.
- **`runSpeedToPosition()` blocks** until every motor arrives. The firmware
  cannot service serial during a move. That is accepted for now; an E-stop
  would need a hardware interrupt or the non-blocking `run()` loop instead.

The straight-line claim is about the *commanded* path. Real cable stretch,
spool radius variation, and sheave exit-point migration all bend it. Quantifying
that is what `docs/results.md` is for.

## Anchor coordinate table

These are the values in `A[4][3]` in `firmware/uno_winches.cpp` and in
`anchors:` in `vision/config.yaml`. They must agree; a mismatch between
firmware and host is a very confusing bug.

Origin = floor corner below winch 3 (under the ArduCam). X along one wall,
Y along the other, Z up. Winches sit on the floor directly below their
pulleys; the anchor is the pulley exit point, not the winch.

| Anchor | Corner | X (mm) | Y (mm) | Z (mm) |
|---|---|---|---|---|
| A1 | winch 1, by the door | 3632 | 4343 | 2794 |
| A2 | winch 2 | 3632 | 0 | 2794 |
| A3 | winch 3, origin | 0 | 0 | 2794 |
| A4 | winch 4 | 0 | 4343 | 2794 |

| Quantity | Value (mm) |
|---|---|
| Room X extent | 3632 |
| Room Y extent | 4343 |
| Anchor height (Z) | 2794 |
| Winch height (floor level, Z) | 0 |

Notes:

- All four anchor Z values were taken as equal. They are usually *nearly*
  equal but rarely exactly so; remeasure each one if position error looks
  systematic near one corner.
- The IK never assumes a rectangle. Only the safe box in `inWorkspace()`
  does, which is why it is conservative: x 400–3200, y 500–3800,
  z 300–2300. The host's box in `vision/config.yaml` allows z up to 2400
  because the park waypoint at that height was verified taut; the firmware
  prints its WARN line for that move and proceeds. The measured tension
  limit on the Y axis was y ≈ 3494 mm, where the winch 2 cable went slack.

# CAD

Printed parts. Source files live in the subdirectories; this page is the index
of what exists, what it is dimensioned for, and whether it works.

## Parts

| Part | Key dimensions | Status |
|---|---|---|
| Spool v1 | 15.6 mm core | **superseded**, radius variation too high |
| Spool v2 | 40 mm core, 30 mm wide, 60 mm flanges, 5 mm D-bore | printed, in use |
| Corner pulley housing | 45° wedge, U-groove 608 bearing (30 mm OD / 8 mm bore / 10 mm wide), M8 axle, 12 mm cheek gap, 5 mm cheeks | printed |
| NEMA 17 bracket | 42.3 mm motor face, 31 mm hole pattern, M3 | printed |
| Platform | 4 symmetric cable ears, ESP32 + SG90 claw + OLED mounts, flat top for the ArUco marker | printed, flew the demo |

## Notes per part

### Spool v1: superseded

15.6 mm core. Measured payout varied from 52.39 to 60.33 mm/rev across 11
samples (mean 56.72, sample SD 4.5%). On a core that small, each new layer of
1.5 mm line is a large fractional change in effective radius, so the payout
per revolution depends on how full the drum is. That variation dominates the
whole machine's position error. Full dataset in `docs/calibration.md`.

Keep the file for reference; do not print more.

### Spool v2: printed, in use

40 mm core, 30 mm wide, 60 mm flanges, 5 mm D-bore to match the NEMA 17 shaft.

Two changes, and both matter:

- **Bigger core.** At 40 mm, one layer of line is a much smaller fraction of
  the radius, so payout per revolution changes far less across the drum's
  range.
- **Lead-in guide.** A bigger core alone does not help if the line piles up
  unevenly. The guide forces a single flat layer. Even wrapping is at least as
  important as core diameter.

60 mm flanges are set by the Bambu A1 mini's 180 mm build volume with room to
spare for a raft-free flat print; going larger means splitting the part.

Calibrated once: 16000 steps paid out 1524 mm, so `steps_per_mm = 10.5`
(152.4 mm/rev). That is the only measurement taken on this drum, no repeat
samples, no fill-level series, so its *spread* is unknown. The per-layer
diameter change drops from 19% to 7.5% by geometry, but that has not been
confirmed by measurement. Target when it is: SD under 1% of mean, three fill
levels, same procedure as v1.

### Corner pulley housing: printed

45° wedge so it sits flat in the ceiling corner. Holds a U-groove 608 bearing
as the sheave: 30 mm OD, 8 mm bore, 10 mm wide. The groove must be a **U**, not
a V, a V-groove pinches braided line and accelerates wear.

- **M8×45 axle** through the 8 mm bore, nyloc nut, narrow washers so they clear
  the cheeks.
- **12 mm cheek gap** for the 10 mm bearing: 1 mm clearance each side. Enough
  that the bearing spins free, tight enough that the line cannot walk out
  between bearing and cheek.
- **5 mm cheeks**, because this part carries the full cable tension in bending.

Mounted with 3M VHB 5952 onto an isopropyl-cleaned surface. Adhesive, not
mechanical, hence the mandatory safety tether.

Known limitation, listed in the README: the line's exit point migrates around
the sheave by roughly ±15 mm as the platform moves. The firmware treats the
anchor as a fixed point and does not model this.

### NEMA 17 bracket: printed

Standard NEMA 17 interface: 42.3 mm face, 31 mm bolt circle, M3 fasteners.
Nothing unusual. Print orientation matters more than geometry here, see below.

### Platform: printed

Carries the ESP32, the SG90 claw servo, the SSD1306 face and its power, and
presents a flat top face for the ArUco marker.

Requirements:

- **Four symmetric cable ears.** Symmetry keeps the four attachment points
  close to a single effective point, which is what makes the point-mass
  approximation in `docs/kinematics.md` defensible.
- **Flat, matte top face** sized for the marker. Gloss causes specular
  highlights that break ArUco corner detection under room lighting.
- **Battery strapped, not dangling.** A swinging mass changes the dynamics and
  can foul a cable.
- Keep total mass down. Every gram increases cable tension and, with it, cable
  stretch.

## Print settings

Applies to all parts unless a part says otherwise:

| Setting | Value |
|---|---|
| Walls | 5 |
| Infill | 35–40% |
| Material | PETG preferred |
| Printer | Bambu A1 mini (180 mm build volume) |

**PETG over PLA** because the parts are under sustained tension. PLA creeps
under constant load and the pulley housings are loaded every second the machine
is powered.

### Orientation rule

**Load must run along layer lines, not across them.** FDM parts are strong in
the plane of a layer and weak between layers, an interlayer bond is the
failure mode, every time.

In practice:

- Pulley housing: print so cable tension pulls *within* layers, not so that it
  tries to peel one layer off the next.
- Spool flanges: the flange-to-core junction is the stress riser. Orient so
  that junction is not a single interlayer interface.
- Platform ears: the pull direction should lie in the layer plane.

If a part must be oriented badly for printability, add material rather than
accepting the weak axis.

/*
 * cdpr_controller.ino
 *
 * Four-axis cable-driven parallel robot controller for an Arduino Uno.
 *
 * Takes Cartesian move commands over USB serial, solves the inverse kinematics
 * (four Euclidean distances), and drives four winches as a coordinated group so
 * that all of them arrive simultaneously and the platform travels in an
 * approximately straight line.
 *
 * Requires the AccelStepper library (Mike McCauley), which ships MultiStepper.
 *
 * Serial protocol, 115200 baud, newline-terminated:
 *
 *   M x y z    move to (x, y, z) in mm            -> "ok move ..."  / "err ..."
 *   R          report position and cable lengths  -> "ok pos ..."
 *   H x y z    declare the platform is at (x,y,z) -> "ok home ..."
 *   D          disable drivers (motors go limp)   -> "ok disabled"
 *   E          enable drivers                     -> "ok enabled"
 *
 * Every command replies with exactly one line beginning "ok" or "err".
 * On boot the sketch prints "ready".
 *
 * See docs/kinematics.md for the derivation and docs/hardware.md for wiring.
 */

#include <AccelStepper.h>
#include <MultiStepper.h>

// ---------------------------------------------------------------------------
// Pin map, Arduino Uno
//
// AccelStepper's DRIVER constructor takes STEP FIRST, then DIR:
//     AccelStepper(AccelStepper::DRIVER, stepPin, dirPin)
// Swapping them gives motors that buzz and hold but never turn.
//
//     Winch | STEP | DIR
//       1   |  D3  |  D2
//       2   |  D5  |  D4
//       3   |  D7  |  D6
//       4   |  D9  |  D8
//
// Shared enable on D10, wired to all four drivers. TMC2209 EN is ACTIVE LOW:
// LOW energises the motors, HIGH releases them.
// ---------------------------------------------------------------------------
const uint8_t EN_PIN = 10;

AccelStepper winch1(AccelStepper::DRIVER, 3, 2);
AccelStepper winch2(AccelStepper::DRIVER, 5, 4);
AccelStepper winch3(AccelStepper::DRIVER, 7, 6);
AccelStepper winch4(AccelStepper::DRIVER, 9, 8);

AccelStepper *winch[4] = { &winch1, &winch2, &winch3, &winch4 };

MultiStepper group;

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

/*
 * MEASURE THIS.
 *
 * Anchor coordinates in mm, in the room frame: origin at one floor corner,
 * X and Y along the two walls, Z up. Rows are A1..A4, columns are {x, y, z}.
 *
 * The anchor is the point where the line LEAVES THE PULLEY SHEAVE, not the
 * pulley axle centre, and not the winch. See docs/calibration.md section 2.
 *
 * These placeholders describe a 3000 x 3000 x 2400 mm room and are wrong for
 * your room. Anchor error propagates systematically into position error
 * everywhere in the workspace; it does not average out. Measure, then copy the
 * same numbers into vision/config.yaml so firmware and host agree.
 */
const float A[4][3] = {
  {    0.0f,    0.0f, 2400.0f },   // A1  origin corner
  {    0.0f, 3000.0f, 2400.0f },   // A2  +Y from origin
  { 3000.0f, 3000.0f, 2400.0f },   // A3  far corner
  { 3000.0f,    0.0f, 2400.0f }    // A4  +X from origin
};

/*
 * MEASURE THIS TOO.
 *
 * Motor steps per mm of cable paid out.
 *
 *   1/8 microstepping x 200 full steps = 1600 steps/rev
 *   steps_per_mm = 1600 / (mm paid out per rev)
 *
 * Deliberately NOT const. The effective drum radius is not a constant: line
 * wraps in layers, so payout per revolution grows as the drum fills. The v1
 * 15.6 mm drum measured 56.72 mm/rev mean with a 4.5% sample SD, giving the
 * 28.21 below at the mean and roughly +/-90 mm of position error at 2 m of
 * paid-out cable.
 *
 * The v2 40 mm drum should shrink that spread. If it does not shrink it
 * enough, the correct fix is to replace this scalar with a function of
 * paid-out length, e.g. stepsPerMm(L), which is why this is a variable.
 */
float steps_per_mm = 28.21f;

// ---------------------------------------------------------------------------
// Motion limits
// ---------------------------------------------------------------------------

/*
 * MultiStepper runs CONSTANT SPEED and ignores setAcceleration() entirely.
 * Motors must therefore be able to start from rest at MAX_SPEED without
 * stalling. A stall is a silently skipped step, and a skipped step corrupts the
 * position estimate permanently until the platform is re-homed by hand.
 * Increase this cautiously.
 */
const float MAX_SPEED = 800.0f;   // steps/s per motor

/*
 * Conservative axis-aligned safe volume, as an inset from the anchor footprint.
 *
 * The real constraint is that all four cable tensions must stay positive
 * (docs/kinematics.md), whose boundary is a curved surface this firmware does
 * not compute. WORKSPACE_MARGIN is a blunt, deliberately over-restrictive
 * stand-in for it. Roughly the inner 70% of the footprint is actually usable.
 */
const float WORKSPACE_MARGIN = 400.0f;   // mm inset from the anchor extremes
const float Z_MIN            = 150.0f;   // mm, keep the platform off the floor
const float Z_CEILING_GAP    = 300.0f;   // mm below the lowest anchor

// Computed from A[][] in setup().
float lim_x_min, lim_x_max;
float lim_y_min, lim_y_max;
float lim_z_min, lim_z_max;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

// Last known platform position, mm. Only meaningful after H has been issued.
float posX = 0.0f, posY = 0.0f, posZ = 0.0f;

bool homed   = false;
bool enabled = false;

char  cmdBuf[64];
uint8_t cmdLen = 0;

// ---------------------------------------------------------------------------
// Kinematics
// ---------------------------------------------------------------------------

/*
 * Inverse kinematics: cable length from the platform to each anchor.
 *
 *   Li = || P - Ai ||
 *
 * The fixed winch-to-pulley run is a constant and is absorbed by homing, so it
 * does not appear here. See docs/kinematics.md.
 */
void computeLengths(float x, float y, float z, float out[4]) {
  for (uint8_t i = 0; i < 4; i++) {
    float dx = x - A[i][0];
    float dy = y - A[i][1];
    float dz = z - A[i][2];
    out[i] = sqrt(dx * dx + dy * dy + dz * dz);
  }
}

void computeLimits() {
  float xmin = A[0][0], xmax = A[0][0];
  float ymin = A[0][1], ymax = A[0][1];
  float zmin = A[0][2];

  for (uint8_t i = 1; i < 4; i++) {
    if (A[i][0] < xmin) xmin = A[i][0];
    if (A[i][0] > xmax) xmax = A[i][0];
    if (A[i][1] < ymin) ymin = A[i][1];
    if (A[i][1] > ymax) ymax = A[i][1];
    if (A[i][2] < zmin) zmin = A[i][2];
  }

  lim_x_min = xmin + WORKSPACE_MARGIN;
  lim_x_max = xmax - WORKSPACE_MARGIN;
  lim_y_min = ymin + WORKSPACE_MARGIN;
  lim_y_max = ymax - WORKSPACE_MARGIN;
  lim_z_min = Z_MIN;
  lim_z_max = zmin - Z_CEILING_GAP;
}

bool inWorkspace(float x, float y, float z) {
  return (x >= lim_x_min && x <= lim_x_max &&
          y >= lim_y_min && y <= lim_y_max &&
          z >= lim_z_min && z <= lim_z_max);
}

// ---------------------------------------------------------------------------
// Reporting
//
// C++ cannot concatenate a float onto a string literal with '+'. There is no
// operator+ for (const char*, float), and the usual String tricks either fail
// to compile or silently produce garbage. Use successive Serial.print() calls.
// ---------------------------------------------------------------------------
void reportPos() {
  float L[4];
  computeLengths(posX, posY, posZ, L);

  Serial.print(F("ok pos x="));
  Serial.print(posX, 1);
  Serial.print(F(" y="));
  Serial.print(posY, 1);
  Serial.print(F(" z="));
  Serial.print(posZ, 1);

  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(F(" L"));
    Serial.print(i + 1);
    Serial.print('=');
    Serial.print(L[i], 1);
  }

  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(F(" S"));
    Serial.print(i + 1);
    Serial.print('=');
    Serial.print(winch[i]->currentPosition());
  }

  Serial.print(F(" homed="));
  Serial.print(homed ? 1 : 0);
  Serial.print(F(" enabled="));
  Serial.println(enabled ? 1 : 0);
}

// ---------------------------------------------------------------------------
// Actions
// ---------------------------------------------------------------------------

void setEnabled(bool on) {
  // TMC2209 EN is active LOW.
  digitalWrite(EN_PIN, on ? LOW : HIGH);
  enabled = on;
}

/*
 * Declare where the platform currently is. There are no limit switches and no
 * absolute reference: the operator physically places the platform at a known
 * point and tells the firmware about it.
 */
void setHome(float x, float y, float z) {
  float L[4];
  computeLengths(x, y, z, L);

  for (uint8_t i = 0; i < 4; i++) {
    winch[i]->setCurrentPosition((long)(L[i] * steps_per_mm));
  }

  posX = x;
  posY = y;
  posZ = z;
  homed = true;

  Serial.print(F("ok home x="));
  Serial.print(posX, 1);
  Serial.print(F(" y="));
  Serial.print(posY, 1);
  Serial.print(F(" z="));
  Serial.println(posZ, 1);
}

/*
 * Move the platform. Blocks until every motor arrives, MultiStepper's
 * runSpeedToPosition() does not return early, so serial is not serviced during
 * a move and there is no software E-stop. Cut power or pull the safety tether.
 */
void moveToXYZ(float x, float y, float z) {
  if (!homed) {
    Serial.println(F("err not homed - issue H x y z first"));
    return;
  }

  if (!inWorkspace(x, y, z)) {
    Serial.print(F("err target outside workspace: "));
    Serial.print(x, 1); Serial.print(' ');
    Serial.print(y, 1); Serial.print(' ');
    Serial.print(z, 1);
    Serial.print(F(" | allowed x["));
    Serial.print(lim_x_min, 0); Serial.print(':'); Serial.print(lim_x_max, 0);
    Serial.print(F("] y["));
    Serial.print(lim_y_min, 0); Serial.print(':'); Serial.print(lim_y_max, 0);
    Serial.print(F("] z["));
    Serial.print(lim_z_min, 0); Serial.print(':'); Serial.print(lim_z_max, 0);
    Serial.println(']');
    return;   // reject; do not move
  }

  if (!enabled) setEnabled(true);

  float L[4];
  long  target[4];
  computeLengths(x, y, z, L);
  for (uint8_t i = 0; i < 4; i++) {
    target[i] = (long)(L[i] * steps_per_mm);
  }

  group.moveTo(target);
  group.runSpeedToPosition();   // blocking

  posX = x;
  posY = y;
  posZ = z;

  Serial.print(F("ok move x="));
  Serial.print(posX, 1);
  Serial.print(F(" y="));
  Serial.print(posY, 1);
  Serial.print(F(" z="));
  Serial.println(posZ, 1);
}

// ---------------------------------------------------------------------------
// Serial command parser
// ---------------------------------------------------------------------------

// Pulls the next whitespace-delimited float out of the buffer.
// Returns false if there is no next token.
bool nextFloat(char **cursor, float *out) {
  char *tok = strtok(*cursor, " \t\r\n");
  *cursor = NULL;                  // subsequent strtok calls continue the scan
  if (tok == NULL) return false;
  *out = atof(tok);
  return true;
}

void handleCommand(char *line) {
  // Skip leading whitespace.
  while (*line == ' ' || *line == '\t') line++;
  if (*line == '\0') return;

  char cmd = *line;
  char *rest = line + 1;
  char *cursor = rest;

  // Single-quoted char literals, 'M' is a char, "M" is a string and will not
  // compare correctly here.
  switch (cmd) {

    case 'M':
    case 'm': {
      float x, y, z;
      if (!nextFloat(&cursor, &x) ||
          !nextFloat(&cursor, &y) ||
          !nextFloat(&cursor, &z)) {
        Serial.println(F("err usage: M x y z"));
        return;
      }
      moveToXYZ(x, y, z);
      break;
    }

    case 'H':
    case 'h': {
      float x, y, z;
      if (!nextFloat(&cursor, &x) ||
          !nextFloat(&cursor, &y) ||
          !nextFloat(&cursor, &z)) {
        Serial.println(F("err usage: H x y z"));
        return;
      }
      setHome(x, y, z);
      break;
    }

    case 'R':
    case 'r':
      reportPos();
      break;

    case 'D':
    case 'd':
      setEnabled(false);
      Serial.println(F("ok disabled"));
      break;

    case 'E':
    case 'e':
      setEnabled(true);
      Serial.println(F("ok enabled"));
      break;

    default:
      Serial.print(F("err unknown command '"));
      Serial.print(cmd);
      Serial.println(F("' - expected M R H D E"));
      break;
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        handleCommand(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    } else {
      // Overlong line: discard it rather than truncating into a bad command.
      cmdLen = 0;
      Serial.println(F("err command too long"));
    }
  }
}

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN, OUTPUT);
  setEnabled(false);   // stay released until told otherwise

  for (uint8_t i = 0; i < 4; i++) {
    winch[i]->setMaxSpeed(MAX_SPEED);
    // setAcceleration() is intentionally omitted: MultiStepper ignores it.
    group.addStepper(*winch[i]);
  }

  computeLimits();

  Serial.println(F("ready"));
}

void loop() {
  pollSerial();
}

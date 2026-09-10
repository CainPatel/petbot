#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

// constructor(interface, STEP, DIR) — all four wired DIR-first, so swapped here
AccelStepper w[4] = {
  AccelStepper(AccelStepper::DRIVER, 3, 2),   // winch 1
  AccelStepper(AccelStepper::DRIVER, 5, 4),   // winch 2
  AccelStepper(AccelStepper::DRIVER, 7, 6),   // winch 3 (origin)
  AccelStepper(AccelStepper::DRIVER, 9, 11)    // winch 4
};

const int EN_PIN = 10;
MultiStepper winches;

// Pulley exit points, mm. Origin = winch 3 corner (under the ArduCam).
const float A[4][3] = {
  {3632, 4343, 2794},   // winch 1 — by the door
  {3632,    0, 2794},   // winch 2
  {   0,    0, 2794},   // winch 3 — origin
  {   0, 4343, 2794}    // winch 4
};

bool inWorkspace(float x, float y, float z) {
  return x > 400 && x < 3200
      && y > 500 && y < 3800
      && z > 300 && z < 2300;
}

float steps_per_mm = 10.5;    // measured on 40mm drum: 16000 steps = 1524mm

float px, py, pz;

void computeLengths(float x, float y, float z, long out[4]);
void moveToXYZ(float x, float y, float z);
void setHome(float x, float y, float z);
void reportPos(const char* label);


void computeLengths(float x, float y, float z, long out[4]) {
  for (int i = 0; i < 4; i++) {
    float dx = x - A[i][0];
    float dy = y - A[i][1];
    float dz = z - A[i][2];
    out[i] = (long)(sqrt(dx*dx + dy*dy + dz*dz) * steps_per_mm);
  }
}

void moveToXYZ(float x, float y, float z) {
  if (!inWorkspace(x, y, z)) {
    Serial.println("WARN: outside safe workspace, cables may go slack");
  }
  long targets[4];
  computeLengths(x, y, z, targets);
  winches.moveTo(targets);
  winches.runSpeedToPosition();
  px = x; py = y; pz = z;
}

void setHome(float x, float y, float z) {
  long lengths[4];
  computeLengths(x, y, z, lengths);
  for (int i = 0; i < 4; i++) w[i].setCurrentPosition(lengths[i]);
  px = x; py = y; pz = z;
}

void reportPos(const char* label) {
  Serial.print(label);
  Serial.print(px); Serial.print(" ");
  Serial.print(py); Serial.print(" ");
  Serial.println(pz);
}

void setup() {
  Serial.begin(115200);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  for (int i = 0; i < 4; i++) {
    w[i].setMaxSpeed(600);
    winches.addStepper(w[i]);
  }

  setHome(2232, 1829, 1600);    // room centre, platform ~1.2m up, Home: M 2232 1829 1600
  Serial.println("ready");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();

  if (c == 'M') {
    float x = Serial.parseFloat();
    float y = Serial.parseFloat();
    float z = Serial.parseFloat();
    moveToXYZ(x, y, z);
    reportPos("moved: ");
  }
  else if (c == 'R') reportPos("pos: ");
  else if (c == 'H') {
    float x = Serial.parseFloat();
    float y = Serial.parseFloat();
    float z = Serial.parseFloat();
    setHome(x, y, z);
    reportPos("home: ");
  }
  else if (c == 'D') { digitalWrite(EN_PIN, HIGH); Serial.println("disabled"); }
  else if (c == 'E') { digitalWrite(EN_PIN, LOW);  Serial.println("enabled"); }
}
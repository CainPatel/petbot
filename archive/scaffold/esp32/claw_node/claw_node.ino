/*
 * claw_node.ino
 *
 * ESP32 servo claw for the cdpr-petbot moving platform.
 *
 * Joins WiFi and serves an HTTP API for a two-servo gripper: a claw (open /
 * close / grip to an angle) and an optional wrist (tilt). Replaces the gravity
 * treat gate in esp32/platform_node/ when the platform is carrying a gripper
 * instead of a treat hopper. Both sketches speak the same style of JSON API so
 * the host side barely changes.
 *
 *   GET /                    plain-text help
 *   GET /open                open the claw
 *   GET /close               close the claw onto an object
 *   GET /grip?deg=&speed=    claw to an explicit angle
 *   GET /wrist?deg=&speed=   wrist to an explicit angle
 *   GET /preset?name=        stow | pickup | carry | release
 *   GET /stop                freeze both axes where they are, now
 *   GET /relax               detach both servos (NO holding torque)
 *   GET /status              JSON: angles, motion state, battery, uptime
 *
 * Motion is NON-BLOCKING. A command returns immediately with "moving":true and
 * the servos sweep toward their targets from loop(). Poll /status and wait for
 * "moving":false. This matters for two reasons: the web server stays responsive
 * (so /stop actually works mid-sweep), and the sweep speed is controllable --
 * a servo slammed to its target imparts a reaction torque that swings the whole
 * platform on its cables, which then takes several seconds to damp out.
 *
 * Board: ESP32 DevKit (Arduino core for ESP32).
 * Libraries: ESP32Servo (Library Manager). WiFi, WebServer, ESPmDNS ship with
 * the core.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ESP32Servo.h>

// ---------------------------------------------------------------------------
// Credentials
//
// DO NOT COMMIT REAL CREDENTIALS. Replace these locally and keep the
// replacement out of git -- either leave the edit unstaged, or move the two
// defines into a secrets.h (already listed in .gitignore) and #include it.
// ---------------------------------------------------------------------------
#define WIFI_SSID     "YOUR_SSID_HERE"
#define WIFI_PASSWORD "YOUR_PASSWORD_HERE"

// Reachable as http://clawbot.local/ so the host does not need a fixed IP.
// mDNS is flaky on some networks; /status still reports the raw IP as a
// fallback and that is what goes in vision/config.yaml if .local fails.
#define MDNS_NAME "clawbot"

// Set to 0 for a claw-only build with no wrist servo.
#define USE_WRIST 1

// ---------------------------------------------------------------------------
// Pins
//
// Pin choice on the ESP32 is not free. Avoid:
//   GPIO 6-11   wired to the flash chip
//   GPIO 12     strapping pin (MTDI). Held HIGH at boot it selects 1.8 V flash
//               and the board may not boot at all. A servo signal line with a
//               pull-up here is a genuinely hard fault to diagnose.
//   GPIO 34-39  input only, cannot drive a servo
//   ADC2 pins   (0,2,4,12-15,25-27) analogRead() returns garbage while WiFi is
//               active. The battery sense MUST be on ADC1 (32-39).
// ---------------------------------------------------------------------------
const int CLAW_PIN    = 13;   // safe output
const int WRIST_PIN   = 14;   // safe output
const int BATTERY_PIN = 34;   // ADC1, input-only, fed by a divider off the LiPo

// ---------------------------------------------------------------------------
// Claw geometry -- MEASURE THESE on the printed claw.
//
// The numbers below are placeholders. Find yours by relaxing the servo, moving
// the claw by hand to each position, then bisecting with /grip?deg=N until the
// jaws land where you want. Set MIN/MAX first: a servo driven into a mechanical
// hard stop stalls, draws its full stall current, and cooks itself.
// ---------------------------------------------------------------------------
const float CLAW_OPEN_DEG   = 100.0f;   // jaws wide
const float CLAW_GRIP_DEG   =  35.0f;   // closed onto a typical object
const float CLAW_MIN_DEG    =  10.0f;   // hard limit, never commanded past
const float CLAW_MAX_DEG    = 110.0f;   // hard limit

const float WRIST_UP_DEG    = 120.0f;
const float WRIST_LEVEL_DEG =  75.0f;
const float WRIST_DOWN_DEG  =  30.0f;
const float WRIST_MIN_DEG   =  20.0f;
const float WRIST_MAX_DEG   = 140.0f;

// Sweep rates, degrees per second. Deliberately slow -- see the note about
// reaction torque at the top of this file. Raise them only after watching what
// the platform does.
const float CLAW_SPEED_DEG_S  = 90.0f;
const float WRIST_SPEED_DEG_S = 60.0f;

// Servo pulse width range, microseconds. MG996R is 500-2500; SG90 is 500-2400.
// Using a range wider than the servo supports drives it into its internal stop.
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

// ---------------------------------------------------------------------------
// Grip behaviour
//
// There is NO force feedback here. The servo is commanded to an angle and has
// no idea whether it closed on an object, on air, or on your finger. Two
// partial mitigations:
//
// GRIP_BACKOFF_DEG: after a close completes, ease back by this much. When the
//   claw has stalled against an object, that stall is continuous full current
//   and continuous heat; backing off a degree or two relieves the peak while
//   keeping contact. It does nothing useful when the claw closed on air.
//
// GRIP_MAX_HOLD_MS: relax the claw after holding this long. Protects the servo
//   and the LiPo, but DROPS WHATEVER IS BEING HELD when it fires. Default 0
//   (disabled) because a gripper that silently opens is a worse surprise than a
//   warm servo. Set it to e.g. 120000 for unattended runs.
// ---------------------------------------------------------------------------
const float    GRIP_BACKOFF_DEG  = 2.0f;
const uint32_t GRIP_MAX_HOLD_MS  = 0;

// ---------------------------------------------------------------------------
// Battery divider. 12-bit ADC over a nominal 0-3.3 V; a 2:1 divider keeps a 1S
// LiPo (3.0-4.2 V) in range. The ESP32 ADC is not accurate enough for this to
// be better than indicative, which is why /status returns the raw count too.
// ---------------------------------------------------------------------------
const float ADC_MAX       = 4095.0f;
const float ADC_REF_V     = 3.3f;
const float DIVIDER_RATIO = 2.0f;

// ---------------------------------------------------------------------------
// A single servo axis with a rate-limited, non-blocking sweep.
// ---------------------------------------------------------------------------
class Axis {
public:
  Axis(const char *name, int pin, float minDeg, float maxDeg, float speedDegPerS)
    : _name(name), _pin(pin), _min(minDeg), _max(maxDeg), _speed(speedDegPerS),
      _current(0.0f), _target(0.0f), _attached(false), _lastMs(0) {}

  void begin(float startDeg) {
    _current = clampDeg(startDeg);
    _target  = _current;
    attach();
    _servo.write((int)lroundf(_current));
  }

  void attach() {
    if (_attached) return;
    _servo.setPeriodHertz(50);                          // hobby servo frame rate
    _servo.attach(_pin, SERVO_MIN_US, SERVO_MAX_US);
    _attached = true;
    _lastMs = millis();                                 // no dt jump on resume
    _servo.write((int)lroundf(_current));
  }

  // Cut drive entirely. The axis goes limp and holds nothing.
  void relax() {
    if (!_attached) return;
    _servo.detach();
    _attached = false;
    _target = _current;
  }

  void moveTo(float deg) {
    attach();
    _target = clampDeg(deg);
  }

  // Freeze here. Does not relax -- the axis keeps holding position.
  void stop() { _target = _current; }

  void update() {
    uint32_t now = millis();
    float dt = (now - _lastMs) * 0.001f;
    _lastMs = now;

    if (!_attached || _current == _target) return;

    float step  = _speed * dt;
    float delta = _target - _current;

    if (fabsf(delta) <= step) _current = _target;
    else                      _current += (delta > 0.0f) ? step : -step;

    _servo.write((int)lroundf(_current));
  }

  void  setSpeed(float s)  { if (s > 0.0f) _speed = s; }
  bool  moving()     const { return _attached && _current != _target; }
  float current()    const { return _current; }
  float target()     const { return _target; }
  float speed()      const { return _speed; }
  bool  attached()   const { return _attached; }
  const char *name() const { return _name; }

private:
  float clampDeg(float d) const {
    if (d < _min) return _min;
    if (d > _max) return _max;
    return d;
  }

  Servo       _servo;
  const char *_name;
  int         _pin;
  float       _min, _max, _speed;
  float       _current, _target;
  bool        _attached;
  uint32_t    _lastMs;
};

Axis claw("claw", CLAW_PIN, CLAW_MIN_DEG, CLAW_MAX_DEG, CLAW_SPEED_DEG_S);
#if USE_WRIST
Axis wrist("wrist", WRIST_PIN, WRIST_MIN_DEG, WRIST_MAX_DEG, WRIST_SPEED_DEG_S);
#endif

WebServer server(80);

// Grip state
bool     gripping        = false;   // last claw command was a close, not an open
bool     backoffApplied  = false;
uint32_t gripSettledMs   = 0;       // when the closing sweep finished
uint32_t gripCount       = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

float argFloat(const char *name, float fallback) {
  if (!server.hasArg(name)) return fallback;
  return server.arg(name).toFloat();
}

void applySpeedArg(Axis &axis) {
  if (server.hasArg("speed")) axis.setSpeed(server.arg("speed").toFloat());
}

bool anyMoving() {
#if USE_WRIST
  return claw.moving() || wrist.moving();
#else
  return claw.moving();
#endif
}

// Appends  "key":value  with no leading comma. Written as successive appends
// rather than  body += "key" + String(v)  because that form leans on an
// implicit const char* -> StringSumHelper conversion; this one cannot surprise
// anyone.
void appendKV(String &out, const char *key, const String &value) {
  out += "\"";
  out += key;
  out += "\":";
  out += value;
}

void appendAxis(String &out, const Axis &axis) {
  out += "\"";
  out += axis.name();
  out += "\":{";
  appendKV(out, "deg",    String(axis.current(), 1));
  out += ",";
  appendKV(out, "target", String(axis.target(), 1));
  out += ",";
  appendKV(out, "speed",  String(axis.speed(), 1));
  out += ",";
  appendKV(out, "moving",  axis.moving()   ? "true" : "false");
  out += ",";
  appendKV(out, "holding", axis.attached() ? "true" : "false");
  out += "}";
}

String statusJson(bool ok = true, const char *action = nullptr) {
  int   raw   = analogRead(BATTERY_PIN);
  float volts = (raw / ADC_MAX) * ADC_REF_V * DIVIDER_RATIO;

  String body = "{";
  appendKV(body, "ok", ok ? "true" : "false");

  if (action) {
    body += ",";
    appendKV(body, "action", String("\"") + action + "\"");
  }

  body += ",";
  appendAxis(body, claw);
#if USE_WRIST
  body += ",";
  appendAxis(body, wrist);
#endif

  body += ",";
  appendKV(body, "moving",      anyMoving() ? "true" : "false");
  body += ",";
  appendKV(body, "gripping",    gripping ? "true" : "false");
  body += ",";
  appendKV(body, "grip_count",  String(gripCount));
  body += ",";
  appendKV(body, "battery_raw", String(raw));
  body += ",";
  appendKV(body, "battery_v",   String(volts, 2));
  body += ",";
  appendKV(body, "uptime_s",    String(millis() / 1000UL));
  body += ",";
  appendKV(body, "rssi",        String(WiFi.RSSI()));
  body += ",";
  appendKV(body, "ip",          String("\"") + WiFi.localIP().toString() + "\"");
  body += "}";
  return body;
}

void reply(const char *action) {
  server.send(200, "application/json", statusJson(true, action));
}

void replyError(const char *message) {
  String body = "{\"ok\":false,\"error\":\"";
  body += message;
  body += "\"}";
  server.send(400, "application/json", body);
}

// ---------------------------------------------------------------------------
// Claw actions
// ---------------------------------------------------------------------------

void openClaw(float deg) {
  claw.moveTo(deg);
  gripping       = false;
  backoffApplied = false;
  gripSettledMs  = 0;
}

void closeClaw(float deg) {
  claw.moveTo(deg);
  gripping       = true;
  backoffApplied = false;
  gripSettledMs  = 0;
  gripCount++;
}

// ---------------------------------------------------------------------------
// Handlers. None of these block: they set a target and return.
// ---------------------------------------------------------------------------

void handleRoot() {
  String help =
    "cdpr-petbot claw node\n\n"
    "GET /open                 open the claw\n"
    "GET /close                close onto an object\n"
    "GET /grip?deg=&speed=     claw to an explicit angle\n"
#if USE_WRIST
    "GET /wrist?deg=&speed=    wrist to an explicit angle\n"
#endif
    "GET /preset?name=         stow | pickup | carry | release\n"
    "GET /stop                 freeze both axes\n"
    "GET /relax                detach servos (no holding torque)\n"
    "GET /status               JSON state\n\n"
    "Motion is non-blocking. Poll /status until \"moving\":false.\n";
  server.send(200, "text/plain", help);
}

void handleOpen() {
  openClaw(argFloat("deg", CLAW_OPEN_DEG));
  applySpeedArg(claw);
  reply("open");
}

void handleClose() {
  closeClaw(argFloat("deg", CLAW_GRIP_DEG));
  applySpeedArg(claw);
  reply("close");
}

void handleGrip() {
  if (!server.hasArg("deg")) { replyError("missing deg"); return; }

  float deg = server.arg("deg").toFloat();
  applySpeedArg(claw);

  // Treat "more closed than current" as a grip so the backoff and hold logic
  // applies to a manual angle command too.
  if (deg < claw.current()) closeClaw(deg);
  else                      openClaw(deg);

  reply("grip");
}

#if USE_WRIST
void handleWrist() {
  if (!server.hasArg("deg")) { replyError("missing deg"); return; }
  applySpeedArg(wrist);
  wrist.moveTo(server.arg("deg").toFloat());
  reply("wrist");
}
#endif

void handlePreset() {
  if (!server.hasArg("name")) { replyError("missing name"); return; }
  String name = server.arg("name");

  if (name == "stow") {
    // Parked for travel: claw closed on nothing, wrist tucked up. Smallest
    // swinging profile while the platform is moving.
    closeClaw(CLAW_GRIP_DEG);
#if USE_WRIST
    wrist.moveTo(WRIST_UP_DEG);
#endif
  } else if (name == "pickup") {
    openClaw(CLAW_OPEN_DEG);
#if USE_WRIST
    wrist.moveTo(WRIST_DOWN_DEG);
#endif
  } else if (name == "carry") {
    // Keeps whatever the claw is holding; only the wrist levels off.
#if USE_WRIST
    wrist.moveTo(WRIST_LEVEL_DEG);
#endif
  } else if (name == "release") {
    openClaw(CLAW_OPEN_DEG);
#if USE_WRIST
    wrist.moveTo(WRIST_DOWN_DEG);
#endif
  } else {
    replyError("unknown preset; expected stow|pickup|carry|release");
    return;
  }

  reply(name.c_str());
}

void handleStop() {
  claw.stop();
#if USE_WRIST
  wrist.stop();
#endif
  reply("stop");
}

void handleRelax() {
  claw.relax();
#if USE_WRIST
  wrist.relax();
#endif
  gripping       = false;
  backoffApplied = false;
  gripSettledMs  = 0;
  reply("relax");
}

void handleStatus() {
  server.send(200, "application/json", statusJson());
}

void handleNotFound() {
  server.send(404, "application/json",
              "{\"ok\":false,\"error\":\"not found\"}");
}

// ---------------------------------------------------------------------------
// Grip maintenance, called every loop.
// ---------------------------------------------------------------------------
void serviceGrip() {
  if (!gripping || claw.moving()) return;

  if (gripSettledMs == 0) gripSettledMs = millis();

  // Ease off the stall once, just after the closing sweep finishes.
  if (!backoffApplied && GRIP_BACKOFF_DEG > 0.0f) {
    backoffApplied = true;
    claw.moveTo(claw.current() + GRIP_BACKOFF_DEG);
    return;
  }

  // Optional hold timeout. Disabled by default; see the note above -- this
  // drops whatever is being held.
  if (GRIP_MAX_HOLD_MS > 0 && (millis() - gripSettledMs) > GRIP_MAX_HOLD_MS) {
    Serial.println("grip hold timeout, relaxing claw");
    claw.relax();
    gripping = false;
  }
}

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------
void connectWifi(uint32_t timeoutMs) {
  Serial.print("connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - started) < timeoutMs) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
}

void pollWifi() {
  // Non-blocking reconnect. Never spin here: the claw may be holding something
  // and loop() has to keep running.
  static uint32_t lastCheck = 0;
  if (millis() - lastCheck < 5000) return;
  lastCheck = millis();

  if (WiFi.status() == WL_CONNECTED) return;

  Serial.println("wifi down, retrying");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(200);

  // ESP32Servo needs its LEDC timers reserved before any attach(). Without
  // this, a second servo can silently steal the first one's timer and both
  // start jittering.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  analogReadResolution(12);

  // Start closed-but-empty and wrist up: the compact travel pose. Coming up in
  // a wide-open pose risks the jaws fouling a cable on power-on.
  claw.begin(CLAW_GRIP_DEG);
#if USE_WRIST
  wrist.begin(WRIST_UP_DEG);
#endif

  connectWifi(20000);

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("connected, ip ");
    Serial.println(WiFi.localIP());
    Serial.println("put this address in vision/config.yaml as esp32.base_url");

    if (MDNS.begin(MDNS_NAME)) {
      MDNS.addService("http", "tcp", 80);
      Serial.printf("also reachable at http://%s.local/\n", MDNS_NAME);
    }
  } else {
    Serial.println("no wifi yet; server starting anyway, will keep retrying");
  }

  server.on("/",        HTTP_GET, handleRoot);
  server.on("/open",    HTTP_GET, handleOpen);
  server.on("/close",   HTTP_GET, handleClose);
  server.on("/grip",    HTTP_GET, handleGrip);
#if USE_WRIST
  server.on("/wrist",   HTTP_GET, handleWrist);
#endif
  server.on("/preset",  HTTP_GET, handlePreset);
  server.on("/stop",    HTTP_GET, handleStop);
  server.on("/relax",   HTTP_GET, handleRelax);
  server.on("/status",  HTTP_GET, handleStatus);
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println("http server up on port 80");
}

void loop() {
  server.handleClient();
  pollWifi();

  claw.update();
#if USE_WRIST
  wrist.update();
#endif

  serviceGrip();
}

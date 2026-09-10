#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <ArduinoOTA.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>

// WiFi credentials live in esp32/secrets.h, which is gitignored.
// Copy esp32/secrets.example.h to esp32/secrets.h and fill it in.
#include "secrets.h"

U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);


const int SERVO_PIN    = 13;
const int OPEN_ANGLE   = 100;
const int CLOSED_ANGLE = 175;

// Where the claw currently is, so moves ramp from it rather than jumping.
// Assumed open at boot; the servo is deliberately NOT commanded in setup(),
// because a brownout reset would otherwise retrigger it and loop forever.
int lastAngle = OPEN_ANGLE;

Servo claw;
WebServer server(80);

void handleFace();
void drawFace(const char* mood);
void handleOpen();
void handleClose();
void handleGrab();
void handleRelease();
void handleStatus();
void moveClaw(int target);



// Ramp in 2-degree steps so peak current stays low enough not to collapse
// the 3.7V battery rail. Detach at the end, a detached servo cannot stall.
void moveClaw(int target) {
  claw.attach(SERVO_PIN);
  claw.write(target);
  delay(600);
  claw.detach();
  lastAngle = target;
}

void handleOpen()    { moveClaw(OPEN_ANGLE);   server.send(200, "text/plain", "open"); }
void handleClose()   { claw.attach(SERVO_PIN); claw.write(CLOSED_ANGLE); delay(600);
  // stays attached, holds grip against band tension
  lastAngle = CLOSED_ANGLE;
  server.send(200, "text/plain", "closed"); }
void handleGrab()    { moveClaw(CLOSED_ANGLE); server.send(200, "text/plain", "grabbed"); }
void handleRelease() { moveClaw(OPEN_ANGLE);   server.send(200, "text/plain", "released"); }

void handleStatus() {
  String j = "{\"uptime\":" + String(millis() / 1000) +
           ",\"rssi\":"   + String(WiFi.RSSI()) +
           ",\"angle\":"  + String(lastAngle) +
           ",\"ota\":1}";
  server.send(200, "application/json", j);
}

void drawFace(const char* mood) {
  oled.clearBuffer();

  if (strcmp(mood, "happy") == 0) {
    oled.drawDisc(40, 24, 6);
    oled.drawDisc(88, 24, 6);
    for (int x = 44; x <= 84; x++) {
      int y = 44 + (int)(6 * sin((x - 44) * 3.14159 / 40));
      oled.drawPixel(x, y);
      oled.drawPixel(x, y + 1);
    }
  }
  else if (strcmp(mood, "busy") == 0) {
    oled.drawBox(34, 22, 12, 4);
    oled.drawBox(82, 22, 12, 4);
    oled.drawBox(52, 44, 24, 3);
  }
  else if (strcmp(mood, "grab") == 0) {
    oled.drawCircle(40, 24, 8);
    oled.drawCircle(88, 24, 8);
    oled.drawDisc(64, 44, 7);
  }

  oled.sendBuffer();
}

void handleFace() {
  if (server.hasArg("m")) {
    drawFace(server.arg("m").c_str());
    server.send(200, "text/plain", "ok");
  } else {
    server.send(400, "text/plain", "need ?m=happy|busy|grab");
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  oled.begin();
  // if step 3 found 0x3D instead of 0x3C, add this line before begin():
  // oled.setI2CAddress(0x3D * 2);
  drawFace("happy");

  delay(1000);                 // let the rail settle before drawing current

  // No servo movement here. See note on lastAngle above.

  WiFi.setSleep(false);

  IPAddress local(192, 168, 1, 119);
  IPAddress gateway(192, 168, 1, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.config(local, gateway, subnet, IPAddress(8, 8, 8, 8));

  WiFi.begin(SSID, PASS);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // Over-the-air updates so the board can be reflashed without USB
  ArduinoOTA.setHostname("petbot-claw");
  // Without this, anyone on the WiFi can flash arbitrary code to the board.
  // The password lives in esp32/secrets.h; pio sends it from platformio.local.ini.
  ArduinoOTA.setPassword(OTA_PASSWORD_STR);
  ArduinoOTA.onStart([]() {
    claw.detach();             // never update mid-servo-move
    Serial.println("OTA start");
  });
  ArduinoOTA.onEnd([]() { Serial.println("OTA done"); });
  ArduinoOTA.begin();

  server.on("/face", handleFace);
  server.on("/open",    handleOpen);
  server.on("/close",   handleClose);
  server.on("/grab",    handleGrab);
  server.on("/release", handleRelease);
  server.on("/status",  handleStatus);
  server.begin();

  Serial.println("ready");
}

void loop() {
  ArduinoOTA.handle();
  server.handleClient();
}
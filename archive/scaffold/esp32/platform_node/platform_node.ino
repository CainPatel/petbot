/*
 * platform_node.ino
 *
 * ESP32 node that rides on the moving platform. Joins WiFi and serves two
 * endpoints:
 *
 *   GET /dispense   sweep the servo open, hold, close. Returns JSON.
 *   GET /status     battery ADC reading and uptime, as JSON.
 *
 * The servo actuates a gravity treat gate: opening the flap lets one treat
 * fall. There is no feedback that a treat actually left, so /dispense reports
 * only that the motion ran.
 *
 * Board: ESP32 DevKit (Arduino core for ESP32).
 * Libraries: ESP32Servo (Library Manager). WiFi and WebServer ship with the core.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// ---------------------------------------------------------------------------
// Credentials
//
// DO NOT COMMIT REAL CREDENTIALS. Replace these placeholders locally and keep
// the replacement out of git — either leave the edit unstaged, or move the two
// defines into a secrets.h (already listed in .gitignore) and #include it.
// ---------------------------------------------------------------------------
#define WIFI_SSID     "YOUR_SSID_HERE"
#define WIFI_PASSWORD "YOUR_PASSWORD_HERE"

// ---------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------
const int SERVO_PIN   = 13;   // PWM-capable pin driving the treat gate servo
const int BATTERY_PIN = 34;   // ADC1 input-only pin, fed by a divider off the LiPo

// ---------------------------------------------------------------------------
// Gate geometry — measure these on the printed flap, they are not universal
// ---------------------------------------------------------------------------
const int GATE_CLOSED_DEG = 10;
const int GATE_OPEN_DEG   = 90;
const int GATE_OPEN_MS    = 400;   // how long the flap stays open per dispense

// ---------------------------------------------------------------------------
// Battery divider
//
// The ESP32 ADC is 12-bit over a nominal 0-3.3 V. With a 2:1 divider from a
// 1S LiPo (3.0-4.2 V) the reading stays in range. These constants convert the
// raw count to an approximate pack voltage; the ESP32 ADC is not accurate
// enough for this to be better than indicative, which is why /status returns
// the raw count as well.
// ---------------------------------------------------------------------------
const float ADC_MAX        = 4095.0f;
const float ADC_REF_V      = 3.3f;
const float DIVIDER_RATIO  = 2.0f;

WebServer server(80);
Servo gate;

unsigned long dispenseCount = 0;

// ---------------------------------------------------------------------------

void handleDispense() {
  gate.write(GATE_OPEN_DEG);
  delay(GATE_OPEN_MS);
  gate.write(GATE_CLOSED_DEG);
  delay(200);                 // let the flap settle before reporting done

  dispenseCount++;

  String body = "{";
  body += "\"ok\":true,";
  body += "\"action\":\"dispense\",";
  body += "\"open_ms\":" + String(GATE_OPEN_MS) + ",";
  body += "\"count\":" + String(dispenseCount);
  body += "}";

  server.send(200, "application/json", body);
}

void handleStatus() {
  int   raw     = analogRead(BATTERY_PIN);
  float volts   = (raw / ADC_MAX) * ADC_REF_V * DIVIDER_RATIO;
  unsigned long uptimeS = millis() / 1000UL;

  String body = "{";
  body += "\"ok\":true,";
  body += "\"battery_raw\":" + String(raw) + ",";
  body += "\"battery_v\":" + String(volts, 2) + ",";
  body += "\"uptime_s\":" + String(uptimeS) + ",";
  body += "\"dispense_count\":" + String(dispenseCount) + ",";
  body += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  body += "\"ip\":\"" + WiFi.localIP().toString() + "\"";
  body += "}";

  server.send(200, "application/json", body);
}

void handleRoot() {
  server.send(200, "text/plain",
              "cdpr-petbot platform node\n"
              "GET /dispense\n"
              "GET /status\n");
}

void handleNotFound() {
  server.send(404, "application/json", "{\"ok\":false,\"error\":\"not found\"}");
}

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(200);

  gate.setPeriodHertz(50);              // standard hobby servo frame rate
  gate.attach(SERVO_PIN, 500, 2400);    // SG90-compatible pulse range, in us
  gate.write(GATE_CLOSED_DEG);

  analogReadResolution(12);

  Serial.print("connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Retry rather than block forever: the platform may power up before the
  // access point is reachable.
  unsigned long started = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
    if (millis() - started > 20000UL) {
      Serial.println();
      Serial.println("wifi timeout, retrying");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      started = millis();
    }
  }

  Serial.println();
  Serial.print("connected, ip ");
  Serial.println(WiFi.localIP());
  Serial.println("put this address in vision/config.yaml as esp32.base_url");

  server.on("/",         HTTP_GET, handleRoot);
  server.on("/dispense", HTTP_GET, handleDispense);
  server.on("/status",   HTTP_GET, handleStatus);
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println("http server up on port 80");
}

void loop() {
  server.handleClient();
}

// Copy this file to esp32/secrets.h (gitignored) and fill in your network.
#pragma once

#define WIFI_SSID_STR "your-ssid"
#define WIFI_PASS_STR "your-password"

// Password ArduinoOTA demands before accepting a firmware upload. Put the
// same string in platformio.local.ini (see platformio.local.example.ini).
#define OTA_PASSWORD_STR "choose-a-long-random-string"

static const char* SSID = WIFI_SSID_STR;
static const char* PASS = WIFI_PASS_STR;

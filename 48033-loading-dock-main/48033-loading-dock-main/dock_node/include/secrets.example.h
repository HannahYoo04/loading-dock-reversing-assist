// Copy this file to secrets.h (same folder) and fill in your WiFi.
// secrets.h is in .gitignore, so your password never goes into the repo.
// The web page client (phone/laptop) must be on the same network as the ESP32.

#pragma once

#define WIFI_SSID     "your-network-name"
#define WIFI_PASSWORD "your-password"

// MQTT commands on .../cmd must carry this as "key". Ask Jon for the real one.
#define COMMAND_KEY   "ask-jon-for-the-key"

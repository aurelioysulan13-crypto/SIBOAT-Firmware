#pragma once
// Compile-safe example only. Replace these values in a PRIVATE repository
// using the GitHub Actions secret SIBOAT_SECRETS_H.
// Never commit real Wi-Fi passwords or OTA passwords.

#define SIBOAT_SSID1 "YOUR_WIFI_NAME_1"
#define SIBOAT_PASS1 "YOUR_WIFI_PASSWORD_1"

#define SIBOAT_SSID2 ""
#define SIBOAT_PASS2 ""

#define SIBOAT_SSID3 ""
#define SIBOAT_PASS3 ""

// Main ESP32 BLE PIN: 4-8 digits. Replace before enabling BLE control.
#define SIBOAT_BLE_PIN "123456"

// Optional OTA password. Uncomment and replace with a private password.
// #define SIBOAT_OTA_PASSWORD "REPLACE_WITH_A_LONG_RANDOM_PASSWORD"

#pragma once

// Copy to include/config.local.h; the local copy is ignored by git.
#define WIFI_SSID "GANTI-DENGAN-SSID"
#define WIFI_PASSWORD "GANTI-DENGAN-PASSWORD"

// Eduroam uses WPA2-Enterprise (PEAP/MSCHAPv2), not a personal Wi-Fi PSK.
// When WIFI_SSID is exactly "eduroam", set these to your institution credentials.
// Configure identity as required by your institution; often same as username.
#define EDUROAM_USERNAME "nama.pengguna@institusi.ac.id"
#define EDUROAM_IDENTITY EDUROAM_USERNAME
#define EDUROAM_PASSWORD "GANTI-DENGAN-PASSWORD"
// If your institution requires server certificate validation, insert its CA PEM here.
// Obtain the correct CA from campus IT; do not disable certificate validation as a shortcut.
#define EDUROAM_CA_CERT ""
#define MQTT_HOST "192.168.1.10"
#define MQTT_PORT 1883
#define MQTT_USER ""
#define MQTT_PASSWORD ""

// Must match the 2.4 GHz Wi-Fi AP channel used by the gateway.
#define ESPNOW_CHANNEL 6

// BH1750 I2C address is usually 0x23 (ADDR low) or 0x5C (ADDR high).
#define BH1750_I2C_ADDRESS 0x23

// Verify each ratio with the exact resistors used.
#define WIND_DIVIDER_RATIO 2.0f
#define BATTERY_DIVIDER_RATIO 2.0f

// Verify against the thermistor datasheet.
#define NTC_BETA 3950.0f

// Keep awake while testing. Deep sleep resets ESP-NOW every sample interval.
#define NODE_DEEP_SLEEP 0

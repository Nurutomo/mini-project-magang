#include <Arduino.h>
#include <esp_mac.h>
#include <string.h>
#include "config.h"
#include "telemetry.h"
#include "telemetry-module.h"
#include "voltage-read.h"

namespace {
RTC_DATA_ATTR uint32_t sequenceNumber = 0;
}

String deviceId() {
  uint8_t mac[6] = {};
  // Reads the eFuse MAC; unlike WiFi.macAddress() this works before WiFi.mode().
  if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK) return String("node-unknown");
  // Use all 48 eFuse MAC bits; the previous 24-bit suffix could collide.
  char id[20];
  snprintf(id, sizeof(id), "node-%02X%02X%02X%02X%02X%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(id);
}

TelemetryPacket collectTelemetry() {
  TelemetryPacket packet = makeEmptyTelemetry();
  const String id = deviceId();
  strlcpy(packet.nodeId, id.c_str(), sizeof(packet.nodeId));
  strlcpy(packet.from, id.c_str(), sizeof(packet.from));
  packet.sequence = ++sequenceNumber;
  packet.uptimeMs = millis();
  packet.hop = 0;
  packet.ttl = ESPNOW_TTL;

  float lux = NAN;
  if (readBh1750(lux)) {
    packet.flags |= FLAG_LUX_VALID;
    packet.luxCenti = static_cast<uint32_t>(constrain(lux * 100.0f, 0.0f, 42949672.0f));
  }

  bool ntcValid = false;
  const float tempC = readNtcCelsius(ntcValid);
  if (ntcValid) {
    packet.flags |= FLAG_NTC_VALID;
    packet.batteryTempCentiC = static_cast<int16_t>(constrain(tempC * 100.0f, -32768.0f, 32767.0f));
  }

#if FEATURE_BATTERY_SENSE
  const float batteryV = readAdcMillivolts(BATTERY_ADC_PIN) * BATTERY_DIVIDER_RATIO / 1000.0f;
  if (isfinite(batteryV) && batteryV > 0.1f && batteryV < 6.0f) {
    packet.flags |= FLAG_BATTERY_VALID;
    packet.batteryMilliVolts = static_cast<uint16_t>(constrain(batteryV * 1000.0f, 0.0f, 65535.0f));
  }
#endif

#if FEATURE_WIND_ADC
  const float windV = readAdcMillivolts(WIND_ADC_PIN) * WIND_DIVIDER_RATIO / 1000.0f;
  if (isfinite(windV) && windV >= 0.0f && windV <= 6.0f) {
    packet.flags |= FLAG_WIND_VALID;
    packet.windMilliVolts = static_cast<uint16_t>(constrain(windV * 1000.0f, 0.0f, 65535.0f));
  }
#endif
  return packet;
}

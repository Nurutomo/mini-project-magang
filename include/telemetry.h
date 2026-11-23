#pragma once

#include <Arduino.h>
#include <stdint.h>

// Versioned application packet shared by sensor/relay nodes and gateway.
static constexpr uint8_t TELEMETRY_MAGIC = 0xA7;
static constexpr uint8_t TELEMETRY_VERSION = 2;
static constexpr size_t NODE_ID_MAX_LEN = 20;
static constexpr size_t TELEMETRY_WIRE_SIZE = 46;

struct __attribute__((packed)) TelemetryPacket {
  uint8_t magic;
  uint8_t version;
  char nodeId[NODE_ID_MAX_LEN];
  uint32_t sequence;
  uint32_t uptimeMs;
  uint8_t hop;
  uint8_t ttl;
  uint8_t flags;
  uint32_t luxCenti;
  int16_t batteryTempCentiC;
  uint16_t batteryMilliVolts;
  uint16_t windMilliVolts;
  uint8_t reserved[3];
};

static_assert(sizeof(TelemetryPacket) == TELEMETRY_WIRE_SIZE,
              "Unexpected telemetry packet layout");

static constexpr uint8_t FLAG_LUX_VALID = 1 << 0;
static constexpr uint8_t FLAG_NTC_VALID = 1 << 1;
static constexpr uint8_t FLAG_BATTERY_VALID = 1 << 2;
static constexpr uint8_t FLAG_WIND_VALID = 1 << 3;
static constexpr uint8_t FLAG_RELAYED = 1 << 4;

inline bool isValidTelemetryPacket(const uint8_t *data, size_t len) {
  if (data == nullptr || len != sizeof(TelemetryPacket)) return false;
  const auto *packet = reinterpret_cast<const TelemetryPacket *>(data);
  if (packet->magic != TELEMETRY_MAGIC || packet->version != TELEMETRY_VERSION) return false;
  if (packet->nodeId[NODE_ID_MAX_LEN - 1] != '\0' || packet->nodeId[0] == '\0') return false;
  return packet->ttl <= 8 && packet->hop <= 8;
}

inline TelemetryPacket makeEmptyTelemetry() {
  TelemetryPacket packet{};
  packet.magic = TELEMETRY_MAGIC;
  packet.version = TELEMETRY_VERSION;
  return packet;
}

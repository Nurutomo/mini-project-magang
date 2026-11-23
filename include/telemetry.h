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

// Repeated gateway broadcast. remaining_sleep_ms is the time to the next
// synchronized sensor window, measured when the beacon is assembled.
static constexpr uint8_t SYNC_BEACON_MAGIC = 0xB8;
static constexpr uint8_t SYNC_BEACON_VERSION = 1;
struct __attribute__((packed)) SyncBeacon {
  uint8_t magic;
  uint8_t version;
  uint8_t channel;
  uint8_t window_open;
  uint32_t sequence;
  uint32_t remaining_sleep_ms;
  uint32_t sensor_interval_ms;
  uint32_t beacon_interval_ms;
  uint32_t active_window_ms;
  uint32_t window_remaining_ms;
};

inline bool isValidSyncBeacon(const uint8_t *data, size_t len) {
  if (!data || len != sizeof(SyncBeacon)) return false;
  const auto *beacon = reinterpret_cast<const SyncBeacon *>(data);
  return beacon->magic == SYNC_BEACON_MAGIC &&
         beacon->version == SYNC_BEACON_VERSION &&
         beacon->channel >= 1 && beacon->channel <= 13 &&
         beacon->sensor_interval_ms >= 10000 && beacon->sensor_interval_ms <= 86400000UL &&
         beacon->beacon_interval_ms >= 100 && beacon->beacon_interval_ms <= 60000 &&
         beacon->active_window_ms >= 500 && beacon->active_window_ms <= 10000 &&
         beacon->sensor_interval_ms > beacon->active_window_ms &&
         beacon->beacon_interval_ms <= beacon->active_window_ms / 2 &&
         beacon->remaining_sleep_ms <= beacon->sensor_interval_ms &&
         beacon->window_remaining_ms <= beacon->active_window_ms &&
         beacon->window_open <= 1 &&
         (!beacon->window_open || beacon->window_remaining_ms > 0);
}

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

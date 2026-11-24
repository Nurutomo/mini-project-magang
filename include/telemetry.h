#pragma once

#include <Arduino.h>
#include <stdint.h>

// Versioned application packet shared by sensor/relay nodes and gateway.
static constexpr uint8_t TELEMETRY_MAGIC = 0xA7;
static constexpr uint8_t TELEMETRY_VERSION = 3;
static constexpr size_t NODE_ID_MAX_LEN = 20;
static constexpr size_t TELEMETRY_WIRE_SIZE = 63;

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
  char from[NODE_ID_MAX_LEN];
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
// fromId is the originating gateway; relays preserve it, so a node that hears
// only a relayed beacon still knows the schedule came from the gateway.
static constexpr uint8_t SYNC_BEACON_MAGIC = 0xB8;
static constexpr uint8_t SYNC_BEACON_VERSION = 3;
static constexpr size_t SYNC_BEACON_WIRE_SIZE = 55;
static constexpr uint8_t FLAG_BEACON_RELAYED = 1 << 0;
struct __attribute__((packed)) SyncBeacon {
  uint8_t magic;
  uint8_t version;
  uint8_t channel;
  uint8_t window_open;
  uint8_t next_channel;
  uint32_t channel_switch_delay_ms;
  uint32_t sequence;
  uint32_t remaining_sleep_ms;
  uint32_t sensor_interval_ms;
  uint32_t beacon_interval_ms;
  uint32_t active_window_ms;
  uint32_t window_remaining_ms;
  char fromId[NODE_ID_MAX_LEN];
  uint8_t ttl;
  uint8_t flags;
};

static_assert(sizeof(SyncBeacon) == SYNC_BEACON_WIRE_SIZE,
              "Unexpected sync beacon layout");

inline bool isValidSyncBeacon(const uint8_t *data, size_t len) {
  if (!data || len != SYNC_BEACON_WIRE_SIZE) return false;
  const auto *beacon = reinterpret_cast<const SyncBeacon *>(data);
  return beacon->magic == SYNC_BEACON_MAGIC &&
         beacon->version == SYNC_BEACON_VERSION &&
         beacon->fromId[NODE_ID_MAX_LEN - 1] == '\0' && beacon->fromId[0] != '\0' &&
         beacon->ttl <= 8 &&
         beacon->channel >= 1 && beacon->channel <= 13 &&
         beacon->next_channel >= 1 && beacon->next_channel <= 13 &&
         beacon->channel_switch_delay_ms <= 60000 &&
         beacon->sensor_interval_ms >= 500 && beacon->sensor_interval_ms <= 86400000UL &&
         beacon->beacon_interval_ms >= 100 && beacon->beacon_interval_ms <= 30000 &&
         beacon->active_window_ms >= 200 && beacon->active_window_ms <= 86400000UL &&
         beacon->sensor_interval_ms > beacon->active_window_ms &&
         beacon->beacon_interval_ms <= beacon->active_window_ms / 2 &&
         beacon->remaining_sleep_ms <= beacon->sensor_interval_ms &&
         beacon->window_remaining_ms <= beacon->active_window_ms &&
         beacon->window_open <= 1 &&
         (beacon->next_channel == beacon->channel ||
          beacon->channel_switch_delay_ms >= 1000) &&
         (!beacon->window_open || beacon->window_remaining_ms > 0);
}

inline bool isValidTelemetryPacket(const uint8_t *data, size_t len) {
  if (data == nullptr || len != sizeof(TelemetryPacket)) return false;
  const auto *packet = reinterpret_cast<const TelemetryPacket *>(data);
  if (packet->magic != TELEMETRY_MAGIC || packet->version != TELEMETRY_VERSION) return false;
  if (packet->nodeId[NODE_ID_MAX_LEN - 1] != '\0' || packet->nodeId[0] == '\0' ||
      packet->from[NODE_ID_MAX_LEN - 1] != '\0' || packet->from[0] == '\0') return false;
  return packet->ttl <= 8 && packet->hop <= 8;
}

inline TelemetryPacket makeEmptyTelemetry() {
  TelemetryPacket packet{};
  packet.magic = TELEMETRY_MAGIC;
  packet.version = TELEMETRY_VERSION;
  return packet;
}

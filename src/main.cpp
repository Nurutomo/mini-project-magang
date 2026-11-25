#include <Arduino.h>
#include "config.h"
#include "telemetry.h"
#include "runtime.h"
#include "voltage-read.h"
#include "telemetry-module.h"
#include "display.h"
#include "mqtt.h"
#include "wifi-manager.h"
#include "serial-commands.h"
#include "data-transceive.h"

#include <WiFi.h>
#include <Preferences.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include <esp_mac.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
#include <ArduinoJson.h>
#if CONFIG_IDF_TARGET_ESP32C3
#include <driver/usb_serial_jtag.h>
#endif
#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
#include <time.h>
#include <sys/time.h>
#include <esp_rtc_time.h>
#include <esp_sntp.h>
#endif

namespace {
constexpr uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
constexpr uint32_t SENSOR_SETTLE_MS = 4;

uint8_t runtimeMeshChannel = ESPNOW_CHANNEL;
uint32_t runtimeSensorIntervalMs = SAMPLE_INTERVAL_S * 1000UL;
uint32_t runtimeBeaconIntervalMs = SYNC_BEACON_INTERVAL_MS;
uint32_t runtimeActiveWindowMs = NODE_ACTIVE_WINDOW_MS;

bool serialConsoleConnected() {
#if defined(ARDUINO_USB_MODE) && ARDUINO_USB_MODE && CONFIG_IDF_TARGET_ESP32C3
  return usb_serial_jtag_is_connected();
#else
  return true; // UART-connected boards have no reliable host-presence signal.
#endif
}

#if defined(DEVICE_ROLE_SENSOR) && DEVICE_ROLE_SENSOR
void pollNodeSerial();
void applyPendingSyncBeacon();
void onNodeReceive(const esp_now_recv_info_t *, const uint8_t *, int);
void serviceNodeMesh();
void onSampleSend(const esp_now_send_info_t *, esp_now_send_status_t);
bool publishSample();
void queueUnsentSample(const TelemetryPacket &packet);
#endif
#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
bool sendSyncBeacon(bool windowOpen);
void onGatewayReceive(const esp_now_recv_info_t *, const uint8_t *, int);
extern uint8_t gatewayRadioChannel;
#endif

RTC_DATA_ATTR uint8_t rtcMeshChannel = ESPNOW_CHANNEL;
RTC_DATA_ATTR uint32_t rtcSensorIntervalMs = SAMPLE_INTERVAL_S * 1000UL;
RTC_DATA_ATTR uint32_t rtcBeaconIntervalMs = SYNC_BEACON_INTERVAL_MS;
RTC_DATA_ATTR uint32_t rtcActiveWindowMs = NODE_ACTIVE_WINDOW_MS;
RTC_DATA_ATTR uint32_t lastSyncSleepMs = 0;
RTC_DATA_ATTR uint16_t missedSyncCount = 0;
RTC_DATA_ATTR bool rtcNodeSynced = false;

struct __attribute__((packed)) MeshConfigRecord {
  uint8_t version;
  uint8_t channel;
  uint32_t sensorIntervalMs;
  uint32_t beaconIntervalMs;
  uint32_t activeWindowMs;
  uint32_t checksum;
};
constexpr uint8_t MESH_CONFIG_RECORD_VERSION = 1;

uint32_t meshConfigChecksum(const MeshConfigRecord &record) {
  const auto *bytes = reinterpret_cast<const uint8_t *>(&record);
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < offsetof(MeshConfigRecord, checksum); ++i) {
    hash = (hash ^ bytes[i]) * 16777619UL;
  }
  return hash;
}

void loadMeshConfig() {
  Preferences meshPreferences;
  if (meshPreferences.begin("mesh", true)) {
    MeshConfigRecord record{};
    const bool hasAtomicRecord = meshPreferences.getBytesLength("config_v1") == sizeof(record) &&
        meshPreferences.getBytes("config_v1", &record, sizeof(record)) == sizeof(record) &&
        record.version == MESH_CONFIG_RECORD_VERSION &&
        record.checksum == meshConfigChecksum(record);
    if (hasAtomicRecord) {
      runtimeMeshChannel = record.channel;
      runtimeSensorIntervalMs = record.sensorIntervalMs;
      runtimeBeaconIntervalMs = record.beaconIntervalMs;
      runtimeActiveWindowMs = record.activeWindowMs;
    } else if (!meshPreferences.isKey("config_v1") && meshPreferences.isKey("channel")) {
      runtimeMeshChannel = meshPreferences.getUChar("channel", runtimeMeshChannel);
      runtimeSensorIntervalMs = meshPreferences.getUInt("sensor_ms", runtimeSensorIntervalMs);
      runtimeBeaconIntervalMs = meshPreferences.getUInt("beacon_ms", runtimeBeaconIntervalMs);
      runtimeActiveWindowMs = meshPreferences.getUInt("active_ms", runtimeActiveWindowMs);
    }
    meshPreferences.end();
  }
  if (runtimeMeshChannel < 1 || runtimeMeshChannel > 13) runtimeMeshChannel = ESPNOW_CHANNEL;
  if (runtimeSensorIntervalMs < 500 || runtimeSensorIntervalMs > 86400000UL)
    runtimeSensorIntervalMs = SAMPLE_INTERVAL_S * 1000UL;
  if (runtimeActiveWindowMs < 100 || runtimeActiveWindowMs > 86400000UL)
    runtimeActiveWindowMs = NODE_ACTIVE_WINDOW_MS;
  if (runtimeSensorIntervalMs <= runtimeActiveWindowMs)
    runtimeActiveWindowMs = runtimeSensorIntervalMs > 200
        ? runtimeSensorIntervalMs - 1 : 200;
  if (runtimeBeaconIntervalMs < 100 || runtimeBeaconIntervalMs > 30000 ||
      runtimeBeaconIntervalMs > runtimeActiveWindowMs / 2) {
    runtimeBeaconIntervalMs = SYNC_BEACON_INTERVAL_MS;
    if (runtimeBeaconIntervalMs > runtimeActiveWindowMs / 2)
      runtimeBeaconIntervalMs = runtimeActiveWindowMs / 2;
    if (runtimeBeaconIntervalMs < 100) runtimeBeaconIntervalMs = 100;
  }
  rtcMeshChannel = runtimeMeshChannel;
  rtcSensorIntervalMs = runtimeSensorIntervalMs;
  rtcBeaconIntervalMs = runtimeBeaconIntervalMs;
  rtcActiveWindowMs = runtimeActiveWindowMs;
}

bool saveMeshConfig() {
  Preferences meshPreferences;
  if (!meshPreferences.begin("mesh", false)) return false;
  MeshConfigRecord record{};
  record.version = MESH_CONFIG_RECORD_VERSION;
  record.channel = runtimeMeshChannel;
  record.sensorIntervalMs = runtimeSensorIntervalMs;
  record.beaconIntervalMs = runtimeBeaconIntervalMs;
  record.activeWindowMs = runtimeActiveWindowMs;
  record.checksum = meshConfigChecksum(record);
  const size_t bytesWritten = meshPreferences.putBytes("config_v1", &record, sizeof(record));
  const bool saved = bytesWritten == sizeof(record);
  meshPreferences.end();
  if (!saved) return false;
  rtcMeshChannel = runtimeMeshChannel;
  rtcSensorIntervalMs = runtimeSensorIntervalMs;
  rtcBeaconIntervalMs = runtimeBeaconIntervalMs;
  rtcActiveWindowMs = runtimeActiveWindowMs;
  return true;
}

#if defined(DEVICE_ROLE_SENSOR) && DEVICE_ROLE_SENSOR
portMUX_TYPE syncBeaconMux = portMUX_INITIALIZER_UNLOCKED;
SyncBeacon pendingSyncBeacon{};
volatile bool hasPendingSyncBeacon = false;
bool nodeSynced = false;
bool nodeWindowOpen = false;
bool nodeSentThisWindow = false;
bool nodeConsoleEnabled = false;
bool nodeRadioReady = false;
uint8_t nodeRadioChannel = 0;
RTC_DATA_ATTR uint8_t channelScanCandidate = 1;
uint32_t lastChannelScanMs = 0;
uint32_t nodeNextSlotAt = 0;
uint32_t nodeTransmitAt = 0;
uint32_t nodeWakeStartedMs = 0;
uint32_t nodeFallbackTransmitAt = 0;
uint32_t nodeWindowStartedAt = 0;
uint32_t nodeWindowEndsAt = 0;
uint32_t nodeInitialJitterMs = 0;
RTC_DATA_ATTR TelemetryPacket unsentSamples[8]{};
RTC_DATA_ATTR uint8_t unsentSampleCount = 0;
RTC_DATA_ATTR bool channelSwitchPending = false;
RTC_DATA_ATTR uint8_t channelSwitchTarget = 0;
uint32_t channelSwitchAt = 0;
RTC_DATA_ATTR uint32_t channelSwitchLastBeaconSequence = 0;
RTC_DATA_ATTR bool channelSwitchHasBeaconSequence = false;
String nodeSerialCommand;
#endif

#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
uint32_t gatewayNextBeaconAt = 0;
uint32_t gatewayNextSensorSlotAt = 0;
uint32_t gatewayNextWindowAt = 0;
uint32_t gatewayWindowOpenedAt = 0;
uint32_t gatewayBeaconSequence = 0;
bool gatewayWindowActive = false;
bool gatewayChannelChangePending = false;
uint8_t gatewayPendingChannel = ESPNOW_CHANNEL;
uint32_t gatewayChannelSwitchAt = 0;
constexpr uint32_t GATEWAY_CHANNEL_ANNOUNCE_MS = 15000;
#endif

#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
String serialCommand;
bool wifiApplyRequested = false;
bool wifiScanRequested = false;
uint32_t wifiTimeSyncStartedMs = 0;
constexpr int64_t TIME_VALID_EPOCH = RTC_TIME_VALID;
constexpr unsigned long RTC_TIME_SAVE_INTERVAL_MS = 3600000UL;
RTC_DATA_ATTR int64_t rtcBackupUnixTime = 0;
RTC_DATA_ATTR uint64_t rtcBackupTimerUs = 0;
RTC_DATA_ATTR bool rtcBackupValid = false;
bool saveMeshConfig();

void handleMeshCommand(String command) {
  command.trim();
  const uint8_t previousChannel = runtimeMeshChannel;
  const uint32_t previousSensorIntervalMs = runtimeSensorIntervalMs;
  const uint32_t previousBeaconIntervalMs = runtimeBeaconIntervalMs;
  const uint32_t previousActiveWindowMs = runtimeActiveWindowMs;
  if (command == "mesh show") {
    const uint8_t displayedChannel = WiFi.status() == WL_CONNECTED
        ? static_cast<uint8_t>(WiFi.channel()) : runtimeMeshChannel;
    Serial.printf("[mesh] channel=%u sensor_interval_ms=%lu beacon_interval_ms=%lu active_window_ms=%lu\n",
                  displayedChannel, static_cast<unsigned long>(runtimeSensorIntervalMs),
                  static_cast<unsigned long>(runtimeBeaconIntervalMs),
                  static_cast<unsigned long>(runtimeActiveWindowMs));
    return;
  }
  if (command.startsWith("mesh window ")) {
    String value = command.substring(12); value.trim();
    char *end = nullptr;
    const unsigned long windowMs = strtoul(value.c_str(), &end, 10);
    if (value.isEmpty() || !end || *end || windowMs < 200 || windowMs > 86400000UL) {
      Serial.println("[mesh] usage: mesh window <active_window_ms 200..86400000>");
      return;
    }
    if (windowMs >= runtimeSensorIntervalMs) {
      Serial.printf("[mesh] active window must be shorter than the sensor interval (%lu ms)\n",
                    static_cast<unsigned long>(runtimeSensorIntervalMs));
      return;
    }
    const uint32_t maxBeaconMs = static_cast<uint32_t>(windowMs / 2);
    const uint32_t newBeaconMs = runtimeBeaconIntervalMs > maxBeaconMs
        ? (maxBeaconMs < 100 ? 100 : maxBeaconMs) : runtimeBeaconIntervalMs;
    runtimeActiveWindowMs = static_cast<uint32_t>(windowMs);
    runtimeBeaconIntervalMs = newBeaconMs;
    if (!saveMeshConfig()) {
      runtimeActiveWindowMs = previousActiveWindowMs;
      runtimeBeaconIntervalMs = previousBeaconIntervalMs;
      rtcActiveWindowMs = previousActiveWindowMs;
      rtcBeaconIntervalMs = previousBeaconIntervalMs;
      Serial.println("[mesh] NVS save failed; previous configuration retained");
    } else {
      const uint32_t now = millis();
      gatewayWindowActive = false;
      gatewayNextWindowAt = now + runtimeSensorIntervalMs;
      gatewayNextSensorSlotAt = gatewayNextWindowAt;
      gatewayNextBeaconAt = now;
      Serial.printf("[mesh] active window=%lu ms beacon=%lu ms saved in NVS\n",
                    static_cast<unsigned long>(runtimeActiveWindowMs),
                    static_cast<unsigned long>(runtimeBeaconIntervalMs));
    }
    return;
  }
  if (!command.startsWith("mesh set ")) return;
  String fields = command.substring(9); fields.trim();
  String values[3]; uint8_t count = 0;
  while (fields.length() && count < 3) {
    const int split = fields.indexOf(' ');
    values[count++] = split < 0 ? fields : fields.substring(0, split);
    if (split < 0) fields = ""; else { fields = fields.substring(split + 1); fields.trim(); }
  }
  char *end = nullptr;
  const long channel = count > 0 ? strtol(values[0].c_str(), &end, 10) : 0;
  bool valid = count == 3 && fields.isEmpty() && end && !*end && channel >= 1 && channel <= 13;
  const long sensorMs = count > 1 ? strtol(values[1].c_str(), &end, 10) : 0;
  valid = valid && end && !*end && sensorMs >= 500 && sensorMs <= 86400000L &&
          sensorMs > runtimeActiveWindowMs;
  const long beaconMs = count > 2 ? strtol(values[2].c_str(), &end, 10) : 0;
  valid = valid && end && !*end && beaconMs >= 100 && beaconMs <= 30000 &&
          beaconMs <= runtimeActiveWindowMs / 2;
  if (!valid) {
    Serial.println("[mesh] usage: mesh set <channel 1..13> <sensor_interval_ms 500..86400000> <beacon_interval_ms 100..30000>; sensor interval must exceed active window and beacon <= half window");
    return;
  }
  if (WiFi.status() == WL_CONNECTED && channel != WiFi.channel()) {
    Serial.printf("[mesh] channel %ld conflicts with Wi-Fi AP channel %ld; AP channel is authoritative\n",
                  channel, static_cast<long>(WiFi.channel()));
    return;
  }
  if (channel != previousChannel) {
    gatewayChannelChangePending = true;
    gatewayPendingChannel = static_cast<uint8_t>(channel);
    gatewayChannelSwitchAt = millis() + GATEWAY_CHANNEL_ANNOUNCE_MS;
  } else if (channel == previousChannel) {
    gatewayChannelChangePending = false;
    gatewayPendingChannel = previousChannel;
    gatewayChannelSwitchAt = 0;
  }
  runtimeMeshChannel = static_cast<uint8_t>(channel);
  runtimeSensorIntervalMs = static_cast<uint32_t>(sensorMs);
  runtimeBeaconIntervalMs = static_cast<uint32_t>(beaconMs);
  if (!saveMeshConfig()) {
    gatewayChannelChangePending = false;
    gatewayPendingChannel = previousChannel;
    gatewayChannelSwitchAt = 0;
    runtimeMeshChannel = previousChannel;
    runtimeSensorIntervalMs = previousSensorIntervalMs;
    runtimeBeaconIntervalMs = previousBeaconIntervalMs;
    runtimeActiveWindowMs = previousActiveWindowMs;
    rtcMeshChannel = previousChannel;
    rtcSensorIntervalMs = previousSensorIntervalMs;
    rtcBeaconIntervalMs = previousBeaconIntervalMs;
    rtcActiveWindowMs = previousActiveWindowMs;
    Serial.println("[mesh] NVS save failed; previous configuration retained");
  } else {
    const uint32_t now = millis();
    if (gatewayWindowActive) {
      gatewayNextWindowAt = gatewayWindowOpenedAt + runtimeSensorIntervalMs;
      if (static_cast<int32_t>(now - gatewayNextWindowAt) >= 0)
        gatewayNextWindowAt = now + runtimeSensorIntervalMs;
    } else gatewayNextWindowAt = now + runtimeSensorIntervalMs;
    gatewayNextSensorSlotAt = gatewayWindowActive ? gatewayWindowOpenedAt : gatewayNextWindowAt;
    gatewayNextBeaconAt = now;
    if (gatewayChannelChangePending)
      Serial.printf("[mesh] saved; announcing channel %u on old channel for %lu ms before switch\n",
                    gatewayPendingChannel,
                    static_cast<unsigned long>(GATEWAY_CHANNEL_ANNOUNCE_MS));
    else
      Serial.println("[mesh] saved in NVS; new intervals take effect after current window");
  }
}

bool currentTimeValid() { return time(nullptr) >= TIME_VALID_EPOCH; }

void restoreRtcTime() {
  if (!rtcBackupValid || rtcBackupUnixTime < TIME_VALID_EPOCH) return;
  const uint64_t nowUs = esp_rtc_get_time_us();
  if (nowUs <= rtcBackupTimerUs) return;
  const uint64_t elapsedMs = (nowUs - rtcBackupTimerUs) / 1000ULL;
  if (elapsedMs > 30ULL * 24ULL * 60ULL * 60ULL * 1000ULL) return;
  timeval tv{};
  tv.tv_sec = static_cast<time_t>(rtcBackupUnixTime + static_cast<int64_t>(elapsedMs / 1000ULL));
  tv.tv_usec = static_cast<suseconds_t>((elapsedMs % 1000ULL) * 1000ULL);
  if (settimeofday(&tv, nullptr) != 0) {
    Serial.println("[time] failed to restore RTC estimate"); return;
  }
  Serial.println("[time] restored approximate RTC time; NTP will correct it");
}

void markNtpSyncStarted() {
  wifiTimeSyncStartedMs = millis();
}

void saveRtcTime() {
  const time_t now = time(nullptr);
  if (now < TIME_VALID_EPOCH) return;
  rtcBackupUnixTime = now;
  rtcBackupTimerUs = esp_rtc_get_time_us();
  rtcBackupValid = true;
}

void pollTimeSync() {
  static bool ntpConfirmed = false;
  static bool timeoutReported = false;
  static uint32_t lastRtcSaveMs = 0;
  if (WiFi.status() != WL_CONNECTED) { timeoutReported = false; return; }

  if (sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED) {
    if (!ntpConfirmed) {
      ntpConfirmed = true;
      saveRtcTime();
      lastRtcSaveMs = millis();
      struct tm now{};
      if (getLocalTime(&now, 0)) {
        char formatted[24]; strftime(formatted, sizeof(formatted), "%Y-%m-%d %H:%M:%S UTC", &now);
        Serial.printf("[time] NTP synchronized; system time=%s\n", formatted);
      }
    } else if (millis() - lastRtcSaveMs >= RTC_TIME_SAVE_INTERVAL_MS) {
      saveRtcTime();
      lastRtcSaveMs = millis();
    }
  } else if (millis() - wifiTimeSyncStartedMs >= TIME_SYNC_TIMEOUT_MS && !timeoutReported) {
    timeoutReported = true;
    Serial.printf("[time] NTP not confirmed; %s\n",
                  currentTimeValid() ? "using approximate RTC time" : "timestamps unavailable");
  }
}
#endif

#if defined(DEVICE_ROLE_SENSOR) && DEVICE_ROLE_SENSOR
#if defined(DEVICE_ROLE_RELAY) && DEVICE_ROLE_RELAY && ESPNOW_ENABLE_RELAY
void onRelayReceive(const esp_now_recv_info_t *, const uint8_t *data, int len);
void onRelayBeacon(const uint8_t *data);
#endif

void onNodeReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (isValidSyncBeacon(data, static_cast<size_t>(len))) {
    const auto *beacon = reinterpret_cast<const SyncBeacon *>(data);
    portENTER_CRITICAL(&syncBeaconMux);
    pendingSyncBeacon = *beacon;
    hasPendingSyncBeacon = true;
    portEXIT_CRITICAL(&syncBeaconMux);
    blinkStatusLed(1);
#if defined(DEVICE_ROLE_RELAY) && DEVICE_ROLE_RELAY && ESPNOW_ENABLE_RELAY
    onRelayBeacon(data);
#endif
    return;
  }
#if defined(DEVICE_ROLE_RELAY) && DEVICE_ROLE_RELAY && ESPNOW_ENABLE_RELAY
  if (isValidTelemetryPacket(data, static_cast<size_t>(len))) onRelayReceive(info, data, len);
#endif
}

void applyPendingSyncBeacon() {
  SyncBeacon beacon{};
  bool pending = false;
  portENTER_CRITICAL(&syncBeaconMux);
  if (hasPendingSyncBeacon) { beacon = pendingSyncBeacon; hasPendingSyncBeacon = false; pending = true; }
  portEXIT_CRITICAL(&syncBeaconMux);
  if (!pending) return;
  if (beacon.channel != nodeRadioChannel) {
    if (esp_wifi_set_channel(beacon.channel, WIFI_SECOND_CHAN_NONE) != ESP_OK) return;
    if (esp_now_is_peer_exist(BROADCAST_MAC)) esp_now_del_peer(BROADCAST_MAC);
    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, BROADCAST_MAC, sizeof(BROADCAST_MAC));
    peer.channel = beacon.channel;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) return;
    nodeRadioChannel = beacon.channel;
  }
  if (beacon.next_channel != beacon.channel && beacon.channel_switch_delay_ms > 0) {
    const uint32_t switchAt = millis() + beacon.channel_switch_delay_ms;
    const uint32_t remainingToWindow = beacon.window_open ? beacon.window_remaining_ms
        : beacon.remaining_sleep_ms;
    const bool sequenceChanged = !channelSwitchHasBeaconSequence ||
        static_cast<int32_t>(beacon.sequence - channelSwitchLastBeaconSequence) > 0;
    if (sequenceChanged && !beacon.window_open && remainingToWindow >
        beacon.channel_switch_delay_ms + beacon.active_window_ms + 500UL) {
      channelSwitchLastBeaconSequence = beacon.sequence;
      channelSwitchHasBeaconSequence = true;
      channelSwitchTarget = beacon.next_channel;
      channelSwitchPending = true;
      channelSwitchAt = switchAt;
    }
  } else if (beacon.next_channel == beacon.channel) {
    channelSwitchPending = false;
    channelSwitchHasBeaconSequence = false;
  }
  const bool newSensorSchedule = !nodeSynced ||
      beacon.sensor_interval_ms != runtimeSensorIntervalMs ||
      beacon.active_window_ms != runtimeActiveWindowMs;
  runtimeMeshChannel = beacon.channel;
  runtimeSensorIntervalMs = beacon.sensor_interval_ms;
  runtimeBeaconIntervalMs = beacon.beacon_interval_ms;
  runtimeActiveWindowMs = beacon.active_window_ms;
  const bool wasWindowOpen = nodeWindowOpen;
  nodeWindowOpen = beacon.window_open != 0;
  const uint32_t now = millis();
  if (nodeWindowOpen && !wasWindowOpen) {
    nodeSentThisWindow = false;
    nodeWindowStartedAt = now;
    nodeWindowEndsAt = now + beacon.window_remaining_ms;
    nodeTransmitAt = now + nodeInitialJitterMs + static_cast<uint32_t>(random(5, 25));
  } else if (nodeWindowOpen) {
    const uint32_t advertisedEnd = now + beacon.window_remaining_ms;
    if (static_cast<int32_t>(advertisedEnd - nodeWindowEndsAt) < 0)
      nodeWindowEndsAt = advertisedEnd;
  }
  nodeNextSlotAt = nodeWindowOpen ? nodeWindowEndsAt : now + beacon.remaining_sleep_ms;
  if (!nodeWindowOpen) {
    const uint32_t jitter = newSensorSchedule
        ? nodeInitialJitterMs + static_cast<uint32_t>(random(5, 25)) : 0;
    nodeTransmitAt = nodeNextSlotAt + jitter;
  }
  nodeSynced = true;
  rtcNodeSynced = true;
  nodeWakeStartedMs = now;
  channelScanCandidate = beacon.channel;
  rtcMeshChannel = beacon.channel;
  rtcSensorIntervalMs = beacon.sensor_interval_ms;
  rtcBeaconIntervalMs = beacon.beacon_interval_ms;
  rtcActiveWindowMs = beacon.active_window_ms;
  if (missedSyncCount > 0) --missedSyncCount;
  lastSyncSleepMs = beacon.remaining_sleep_ms;
  nodeWindowOpen = beacon.window_open != 0;
  if (!nodeWindowOpen) nodeSentThisWindow = false;
  else if (beacon.window_remaining_ms == beacon.active_window_ms) nodeSentThisWindow = false;
  if (nodeConsoleEnabled) Serial.printf("[mesh] sync seq=%lu channel=%u window=%s next=%lums\n",
      static_cast<unsigned long>(beacon.sequence), beacon.channel,
      nodeWindowOpen ? "open" : "closed", static_cast<unsigned long>(beacon.remaining_sleep_ms));
}

void serviceNodeMesh() {
  if (nodeConsoleEnabled && !serialConsoleConnected()) nodeConsoleEnabled = false;
  else if (!nodeConsoleEnabled && serialConsoleConnected()) {
    nodeConsoleEnabled = true;
    Serial.begin(115200);
    Serial.printf("[node] console active; channel=%u interval=%lus\n",
        runtimeMeshChannel, static_cast<unsigned long>(runtimeSensorIntervalMs / 1000UL));
  }
  if (nodeConsoleEnabled) pollNodeSerial();
  applyPendingSyncBeacon();

  const uint32_t now = millis();
  if (channelSwitchPending && nodeWindowOpen) {
    const uint32_t remainingToSwitch = channelSwitchAt - now;
    const uint32_t remainingToWindow = nodeWindowEndsAt - now;
    if (static_cast<int32_t>(remainingToSwitch) <= 0 ||
        remainingToSwitch <= remainingToWindow + 500UL) {
      channelSwitchPending = false;
      channelSwitchHasBeaconSequence = false;
    }
  }
  if (channelSwitchPending && static_cast<int32_t>(now - channelSwitchAt) >= 0) {
    if (nodeRadioReady) {
      esp_now_deinit();
      nodeRadioReady = false;
    }
    nodeRadioChannel = channelSwitchTarget;
    channelScanCandidate = nodeRadioChannel;
    nodeSynced = false;
    nodeWindowOpen = false;
    nodeSentThisWindow = false;
    nodeWakeStartedMs = now;
    channelSwitchPending = false;
    if (initMeshRadio(nodeRadioChannel)) {
      nodeRadioReady = true;
      esp_now_register_send_cb(onSampleSend);
      esp_now_register_recv_cb(onNodeReceive);
    }
    if (nodeConsoleEnabled)
      Serial.printf("[mesh] switched to announced channel %u\n", nodeRadioChannel);
  }
  if (!nodeSynced && nodeRadioReady && static_cast<uint32_t>(now - nodeWakeStartedMs) >= runtimeActiveWindowMs) {
    if (nodeConsoleEnabled) {
      nodeWakeStartedMs = now;
      nodeRadioChannel = static_cast<uint8_t>((nodeRadioChannel % 13) + 1);
      channelScanCandidate = nodeRadioChannel;
      if (nodeRadioReady) esp_now_deinit();
      nodeRadioReady = false;
      if (initMeshRadio(nodeRadioChannel)) {
        nodeRadioReady = true;
        esp_now_register_send_cb(onSampleSend);
        esp_now_register_recv_cb(onNodeReceive);
      }
      return;
    }
    if (nodeRadioReady) {
      esp_now_deinit();
      nodeRadioReady = false;
    }
    nodeRadioChannel = static_cast<uint8_t>((nodeRadioChannel % 13) + 1);
    channelScanCandidate = nodeRadioChannel;
    if (initMeshRadio(nodeRadioChannel)) {
      nodeRadioReady = true;
      esp_now_register_send_cb(onSampleSend);
      esp_now_register_recv_cb(onNodeReceive);
    }
    nodeWakeStartedMs = now;
    nodeSynced = false;
    nodeWindowOpen = false;
    nodeSentThisWindow = false;
    if (missedSyncCount < UINT16_MAX) ++missedSyncCount;
  }

  if (nodeSynced && nodeWindowOpen && !nodeSentThisWindow &&
      static_cast<int32_t>(now - nodeTransmitAt) >= 0 &&
      static_cast<int32_t>(now - nodeWindowEndsAt) < 0) {
    if (publishSample()) nodeSentThisWindow = true;
    else nodeTransmitAt = millis() + 250UL;
  }
  if (nodeSynced && nodeWindowOpen && !nodeSentThisWindow &&
      static_cast<int32_t>(now - nodeWindowEndsAt) >= 0) {
    queueUnsentSample(collectTelemetry());
    nodeSentThisWindow = true;
  }
  // No beacon yet: still take a reading instead of staying silent.
  if (!nodeSynced && nodeRadioReady && static_cast<int32_t>(now - nodeFallbackTransmitAt) >= 0) {
    publishSample();
    nodeFallbackTransmitAt = millis() + runtimeSensorIntervalMs;
  }
  if (nodeSynced && nodeWindowOpen && static_cast<int32_t>(now - nodeWindowEndsAt) >= 0) {
    nodeWindowOpen = false;
  }

#if NODE_DEEP_SLEEP
  const uint32_t awakeLimit = runtimeActiveWindowMs + 1500UL;
  if (nodeConsoleEnabled) return;
  if (nodeSynced && !nodeWindowOpen && !channelSwitchPending &&
      static_cast<int32_t>(nodeNextSlotAt - now) > 0) {
    const uint32_t sleepMs = nodeNextSlotAt - now;
    lastSyncSleepMs = sleepMs;
    if (nodeRadioReady) { esp_now_deinit(); nodeRadioReady = false; }
    WiFi.mode(WIFI_OFF);
    if (nodeConsoleEnabled) Serial.flush();
    esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(sleepMs) * 1000ULL);
    esp_deep_sleep_start();
  }
  if (!nodeSynced && now - nodeWakeStartedMs >= awakeLimit &&
      (unsentSampleCount == 0 || static_cast<int32_t>(now - nodeFallbackTransmitAt) < 0)) {
    if (nodeConsoleEnabled) return;
    if (nodeRadioReady) { esp_now_deinit(); nodeRadioReady = false; }
    WiFi.mode(WIFI_OFF);
    channelScanCandidate = static_cast<uint8_t>((nodeRadioChannel % 13) + 1);
    esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(runtimeBeaconIntervalMs) * 1000ULL);
    esp_deep_sleep_start();
  }
#endif
}

void pollNodeSerial() {
  if (!serialConsoleConnected()) { nodeConsoleEnabled = false; return; }
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      nodeSerialCommand.trim();
      if (nodeSerialCommand.startsWith("mesh set ")) {
        String args = nodeSerialCommand.substring(9);
        const int first = args.indexOf(' '), second = args.indexOf(' ', first + 1);
        if (first > 0 && second > first) {
          const String channelText = args.substring(0, first);
          const String intervalText = args.substring(first + 1, second);
          const String beaconText = args.substring(second + 1);
          char *end = nullptr;
          const long channel = strtol(channelText.c_str(), &end, 10);
          const bool channelValid = end && !*end;
          const long interval = strtol(intervalText.c_str(), &end, 10);
          const bool intervalValid = end && !*end;
          const long beacon = strtol(beaconText.c_str(), &end, 10);
          const bool beaconValid = end && !*end;
          if (channelValid && intervalValid && beaconValid && channel >= 1 && channel <= 13 &&
              interval >= 500 && interval <= 86400000L && interval > runtimeActiveWindowMs &&
              beacon >= 100 && beacon <= 30000 && beacon <= runtimeActiveWindowMs / 2) {
            const uint8_t previousNodeChannel = runtimeMeshChannel;
            if (channel != previousNodeChannel) Serial.println("[mesh-info] configured channel; gateway announcement takes precedence when received");
            runtimeMeshChannel = static_cast<uint8_t>(channel);
            runtimeSensorIntervalMs = static_cast<uint32_t>(interval);
            runtimeBeaconIntervalMs = static_cast<uint32_t>(beacon);
            if (saveMeshConfig()) Serial.println("[mesh] saved; reboot to apply local configuration");
            else {
              runtimeMeshChannel = previousNodeChannel;
              rtcMeshChannel = previousNodeChannel;
              Serial.println("[mesh] failed to save configuration");
            }
          } else Serial.println("[mesh] values out of range; sensor interval (ms) must exceed active window and beacon must be <= half window");
        } else Serial.println("[mesh] usage: mesh set <channel> <sensor_interval_ms 500..86400000> <beacon_interval_ms 100..30000>");
      } else if (nodeSerialCommand.startsWith("mesh window ")) {
        String value = nodeSerialCommand.substring(12); value.trim();
        char *end = nullptr;
        const unsigned long windowMs = strtoul(value.c_str(), &end, 10);
        if (value.isEmpty() || !end || *end || windowMs < 200 || windowMs > 86400000UL) {
          Serial.println("[mesh] usage: mesh window <active_window_ms 200..86400000>");
        } else if (windowMs >= runtimeSensorIntervalMs) {
          Serial.printf("[mesh] active window must be shorter than the sensor interval (%lu ms)\n",
                        static_cast<unsigned long>(runtimeSensorIntervalMs));
        } else {
          const uint32_t previousActiveWindowMs = runtimeActiveWindowMs;
          const uint32_t previousBeaconIntervalMs = runtimeBeaconIntervalMs;
          const uint32_t maxBeaconMs = static_cast<uint32_t>(windowMs / 2);
          runtimeActiveWindowMs = static_cast<uint32_t>(windowMs);
          runtimeBeaconIntervalMs = runtimeBeaconIntervalMs > maxBeaconMs
              ? (maxBeaconMs < 100 ? 100 : maxBeaconMs) : runtimeBeaconIntervalMs;
          if (saveMeshConfig()) {
            Serial.printf("[mesh] active window=%lu ms beacon=%lu ms saved; reboot to apply local configuration\n",
                          static_cast<unsigned long>(runtimeActiveWindowMs),
                          static_cast<unsigned long>(runtimeBeaconIntervalMs));
          } else {
            runtimeActiveWindowMs = previousActiveWindowMs;
            runtimeBeaconIntervalMs = previousBeaconIntervalMs;
            rtcActiveWindowMs = previousActiveWindowMs;
            rtcBeaconIntervalMs = previousBeaconIntervalMs;
            Serial.println("[mesh] failed to save configuration");
          }
        }
      } else if (nodeSerialCommand == "mesh show") {
        Serial.printf("[mesh] channel=%u sensor_interval_ms=%lu beacon_interval_ms=%lu active_window_ms=%lu missed_sync=%u\n",
                      runtimeMeshChannel, static_cast<unsigned long>(runtimeSensorIntervalMs),
                      static_cast<unsigned long>(runtimeBeaconIntervalMs),
                      static_cast<unsigned long>(runtimeActiveWindowMs), missedSyncCount);
      }
      nodeSerialCommand = "";
    } else if (nodeSerialCommand.length() < 96) nodeSerialCommand += c;
    else nodeSerialCommand = "";
  }
}

volatile bool sampleSendDone = false;
volatile esp_now_send_status_t sampleSendStatus = ESP_NOW_SEND_FAIL;

void onSampleSend(const esp_now_send_info_t *, esp_now_send_status_t status) {
  sampleSendStatus = status;
  sampleSendDone = true;
  if (status == ESP_NOW_SEND_SUCCESS) blinkStatusLed(1);
}

void queueUnsentSample(const TelemetryPacket &packet);

bool transmitPacket(const TelemetryPacket &packet) {
  if (nodeConsoleEnabled) {
    Serial.printf("[sensor] id=%s seq=%lu lux=", packet.nodeId,
                static_cast<unsigned long>(packet.sequence));
  if (packet.flags & FLAG_LUX_VALID) Serial.printf("%.2f lx", packet.luxCenti / 100.0f);
    else Serial.print("invalid");
    Serial.print(" batteryTemp=");
    if (packet.flags & FLAG_NTC_VALID) Serial.printf("%.2f C", packet.batteryTempCentiC / 100.0f);
    else Serial.print("invalid");
    Serial.print(" windInput=");
    if (packet.flags & FLAG_WIND_VALID) Serial.printf("%.3f V", packet.windMilliVolts / 1000.0f);
    else Serial.print("invalid");
    Serial.println();
  }

  blinkStatusLed(2);
  sampleSendDone = false;
  const esp_err_t result = esp_now_send(BROADCAST_MAC,
                                        reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
  if (result != ESP_OK) {
    if (nodeConsoleEnabled) Serial.printf("[sensor] ESP-NOW enqueue error=%d\n", result);
    return false;
  }
  const uint32_t started = millis();
  while (!sampleSendDone && millis() - started < 1200) delay(5);
  const bool sent = sampleSendDone && sampleSendStatus == ESP_NOW_SEND_SUCCESS;
  if (nodeConsoleEnabled) Serial.printf("[sensor] ESP-NOW callback=%s\n", sent ? "success" : "failure/timeout");
  return sent;
}

void queueUnsentSample(const TelemetryPacket &packet) {
  if (unsentSampleCount == sizeof(unsentSamples) / sizeof(unsentSamples[0])) {
    memmove(unsentSamples, unsentSamples + 1, sizeof(unsentSamples[0]) * (unsentSampleCount - 1));
    --unsentSampleCount;
  }
  unsentSamples[unsentSampleCount++] = packet;
}

bool publishSample() {
  bool sent = true;
  for (uint8_t i = 0; i < unsentSampleCount; ++i) {
    if (!transmitPacket(unsentSamples[i])) { sent = false; break; }
    memmove(unsentSamples + i, unsentSamples + i + 1, sizeof(unsentSamples[0]) * (unsentSampleCount - i - 1));
    --unsentSampleCount;
    --i;
  }
  const TelemetryPacket packet = collectTelemetry();
  if (!transmitPacket(packet)) {
    queueUnsentSample(packet);
    return false;
  }
  if (!sent) return false;
  return true;
}
#endif

#if defined(DEVICE_ROLE_RELAY) && DEVICE_ROLE_RELAY
struct RelayItem { TelemetryPacket packet; };
RelayItem relayQueue[GATEWAY_QUEUE_SIZE];
volatile uint8_t relayHead = 0, relayTail = 0;
portMUX_TYPE relayMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t relayQueueDropped = 0;
struct SeenPacket { char nodeId[NODE_ID_MAX_LEN]; uint32_t sequence; };
SeenPacket relaySeen[RELAY_DEDUP_SIZE]{};
uint16_t relaySeenCursor = 0;

struct RelayBeaconItem { SyncBeacon beacon; };
RelayBeaconItem relayBeaconQueue[RELAY_DEDUP_SIZE];
volatile uint8_t relayBeaconHead = 0, relayBeaconTail = 0;
portMUX_TYPE relayBeaconMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t relayBeaconDropped = 0;

bool wasBeaconRelayed(const SyncBeacon &beacon) {
  static SeenPacket beaconSeen[RELAY_DEDUP_SIZE]{};
  static uint16_t beaconSeenCursor = 0;
  for (const auto &seen : beaconSeen) {
    if (seen.nodeId[0] && strncmp(seen.nodeId, beacon.fromId, NODE_ID_MAX_LEN) == 0 &&
        seen.sequence == beacon.sequence) return true;
  }
  auto &slot = beaconSeen[beaconSeenCursor++ % RELAY_DEDUP_SIZE];
  strlcpy(slot.nodeId, beacon.fromId, sizeof(slot.nodeId));
  slot.sequence = beacon.sequence;
  return false;
}

void onRelayBeacon(const uint8_t *data) {
  const auto *beacon = reinterpret_cast<const SyncBeacon *>(data);
  if (beacon->ttl == 0 || (beacon->flags & FLAG_BEACON_RELAYED) != 0) return;
  RelayBeaconItem item{};
  item.beacon = *beacon;
  portENTER_CRITICAL(&relayBeaconMux);
  const uint8_t next = static_cast<uint8_t>((relayBeaconHead + 1) % RELAY_DEDUP_SIZE);
  if (next == relayBeaconTail) {
    ++relayBeaconDropped;
    portEXIT_CRITICAL(&relayBeaconMux);
    return;
  }
  relayBeaconQueue[relayBeaconHead] = item;
  relayBeaconHead = next;
  portEXIT_CRITICAL(&relayBeaconMux);
}

void relayQueuedBeacons() {
  while (true) {
    RelayBeaconItem item{};
    bool found = false;
    portENTER_CRITICAL(&relayBeaconMux);
    if (relayBeaconTail != relayBeaconHead) {
      item = relayBeaconQueue[relayBeaconTail];
      relayBeaconTail = static_cast<uint8_t>((relayBeaconTail + 1) % RELAY_DEDUP_SIZE);
      found = true;
    }
    portEXIT_CRITICAL(&relayBeaconMux);
    if (!found) break;
    auto &beacon = item.beacon;
    if (strncmp(beacon.fromId, deviceId().c_str(), NODE_ID_MAX_LEN) == 0 ||
        wasBeaconRelayed(beacon)) continue;
    delay(random(0, RELAY_JITTER_MS));
    blinkStatusLed(1);
    --beacon.ttl;
    beacon.flags |= FLAG_BEACON_RELAYED;
    const esp_err_t result = esp_now_send(BROADCAST_MAC,
                                           reinterpret_cast<const uint8_t *>(&beacon), sizeof(beacon));
    Serial.printf("[relay] %s beacon from=%s seq=%lu ttl=%u\n",
                  result == ESP_OK ? "forwarded" : "send error", beacon.fromId,
                  static_cast<unsigned long>(beacon.sequence), beacon.ttl);
  }
}

bool wasRelayed(const TelemetryPacket &packet) {
  for (const auto &seen : relaySeen) {
    if (seen.nodeId[0] && strncmp(seen.nodeId, packet.nodeId, NODE_ID_MAX_LEN) == 0 &&
        seen.sequence == packet.sequence) return true;
  }
  auto &slot = relaySeen[relaySeenCursor++ % RELAY_DEDUP_SIZE];
  strlcpy(slot.nodeId, packet.nodeId, sizeof(slot.nodeId));
  slot.sequence = packet.sequence;
  return false;
}

void onRelayReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (isValidSyncBeacon(data, static_cast<size_t>(len))) {
    onRelayBeacon(data);
    return;
  }
  if (!isValidTelemetryPacket(data, static_cast<size_t>(len))) return;
  blinkStatusLed(1);
  RelayItem item{};
  memcpy(&item.packet, data, sizeof(TelemetryPacket));
  if (info && info->src_addr) {
    const String relayId = deviceId();
    strlcpy(item.packet.from, relayId.c_str(), sizeof(item.packet.from));
  }
  portENTER_CRITICAL(&relayMux);
  const uint8_t next = static_cast<uint8_t>((relayHead + 1) % GATEWAY_QUEUE_SIZE);
  if (next == relayTail) {
    ++relayQueueDropped;
    portEXIT_CRITICAL(&relayMux);
    return;
  }
  relayQueue[relayHead] = item;
  relayHead = next;
  portEXIT_CRITICAL(&relayMux);
}

void relayQueuedPackets() {
  while (true) {
    RelayItem item{};
    bool found = false;
    portENTER_CRITICAL(&relayMux);
    if (relayTail != relayHead) {
      item = relayQueue[relayTail];
      relayTail = static_cast<uint8_t>((relayTail + 1) % GATEWAY_QUEUE_SIZE);
      found = true;
    }
    portEXIT_CRITICAL(&relayMux);
    if (!found) break;
    auto &packet = item.packet;
    if (strncmp(packet.nodeId, deviceId().c_str(), NODE_ID_MAX_LEN) == 0 ||
        packet.ttl == 0 || packet.hop >= ESPNOW_TTL || wasRelayed(packet)) continue;
    delay(random(0, RELAY_JITTER_MS));
    blinkStatusLed(1);
    --packet.ttl;
    ++packet.hop;
    packet.flags |= FLAG_RELAYED;
    const String relayId = deviceId();
    strlcpy(packet.from, relayId.c_str(), sizeof(packet.from));
    const esp_err_t result = esp_now_send(BROADCAST_MAC,
                                           reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
    Serial.printf("[relay] %s node=%s seq=%lu hop=%u ttl=%u\n",
                  result == ESP_OK ? "forwarded" : "send error", packet.nodeId,
                  static_cast<unsigned long>(packet.sequence), packet.hop, packet.ttl);
  }
}
#endif

#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
bool gatewayWifiWasConnected = false;
uint8_t gatewayRadioChannel = ESPNOW_CHANNEL;
uint32_t lastWifiReconnectAttempt = 0;
struct QueueItem { TelemetryPacket packet; int8_t rssi; };
QueueItem queueItems[GATEWAY_QUEUE_SIZE];
using NodeDisplayInfo = GatewayDisplayNode;
NodeDisplayInfo gatewayNodes[GATEWAY_DEDUP_SIZE]{};
uint8_t gatewayNodeCursor = 0;
volatile uint8_t queueHead = 0, queueTail = 0;
portMUX_TYPE queueMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t queueDropped = 0, invalidPackets = 0, gatewayRxCount = 0;
char gatewayLastNode[NODE_ID_MAX_LEN] = "-";
float gatewayLastLux = NAN;
bool gatewayLastLuxValid = false;
struct SeenPacket { char nodeId[NODE_ID_MAX_LEN]; uint32_t sequence; };
SeenPacket seenPackets[GATEWAY_DEDUP_SIZE]{};
uint16_t seenCursor = 0;

GatewayDisplaySnapshot gatewayDisplaySnapshot() {
  return {gatewayNodes, GATEWAY_DEDUP_SIZE, runtimeSensorIntervalMs, gatewayRadioChannel,
          gatewayRxCount, queueDropped, WiFi.status() == WL_CONNECTED, mqttConnected()};
}

void onGatewayReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!info || !info->src_addr || !isValidTelemetryPacket(data, static_cast<size_t>(len))) {
    ++invalidPackets;
    return;
  }
  blinkStatusLed(1);
  ++gatewayRxCount;
  QueueItem item{};
  memcpy(&item.packet, data, sizeof(TelemetryPacket));
  item.rssi = info->rx_ctrl ? info->rx_ctrl->rssi : -127;
  portENTER_CRITICAL(&queueMux);
  const uint8_t next = static_cast<uint8_t>((queueHead + 1) % GATEWAY_QUEUE_SIZE);
  if (next == queueTail) {
    ++queueDropped;
    portEXIT_CRITICAL(&queueMux);
    return;
  }
  queueItems[queueHead] = item;
  queueHead = next;
  portEXIT_CRITICAL(&queueMux);
}

bool duplicatePacket(const TelemetryPacket &packet) {
  for (const auto &seen : seenPackets) {
    if (seen.nodeId[0] && strncmp(seen.nodeId, packet.nodeId, NODE_ID_MAX_LEN) == 0 &&
        seen.sequence == packet.sequence) return true;
  }
  auto &slot = seenPackets[seenCursor++ % GATEWAY_DEDUP_SIZE];
  strlcpy(slot.nodeId, packet.nodeId, sizeof(slot.nodeId));
  slot.sequence = packet.sequence;
  return false;
}

bool sendSyncBeacon(bool windowOpen) {
  SyncBeacon beacon{};
  beacon.magic = SYNC_BEACON_MAGIC;
  beacon.version = SYNC_BEACON_VERSION;
  beacon.channel = gatewayRadioChannel;
  beacon.window_open = windowOpen ? 1 : 0;
  beacon.next_channel = gatewayChannelChangePending ? gatewayPendingChannel : gatewayRadioChannel;
  const int32_t switchDelay = gatewayChannelChangePending
      ? static_cast<int32_t>(gatewayChannelSwitchAt - millis()) : 0;
  beacon.channel_switch_delay_ms = switchDelay > 0 ? static_cast<uint32_t>(switchDelay) : 0;
  beacon.sequence = ++gatewayBeaconSequence;
  const uint32_t now = millis();
  const int32_t untilWindow = static_cast<int32_t>(gatewayNextWindowAt - now);
  beacon.remaining_sleep_ms = untilWindow > 0 ? static_cast<uint32_t>(untilWindow) : 0;
  beacon.sensor_interval_ms = runtimeSensorIntervalMs;
  beacon.beacon_interval_ms = runtimeBeaconIntervalMs;
  beacon.active_window_ms = runtimeActiveWindowMs;
  const int32_t windowRemaining = static_cast<int32_t>(gatewayWindowOpenedAt + runtimeActiveWindowMs - now);
  beacon.window_remaining_ms = windowOpen && windowRemaining > 0
      ? static_cast<uint32_t>(windowRemaining) : 0;
  const String gatewayId = deviceId();
  strlcpy(beacon.fromId, gatewayId.c_str(), sizeof(beacon.fromId));
  beacon.ttl = ESPNOW_TTL;
  const esp_err_t result = esp_now_send(BROADCAST_MAC,
      reinterpret_cast<const uint8_t *>(&beacon), sizeof(beacon));
  if (result == ESP_OK) blinkStatusLed(1);
  return result == ESP_OK;
}

void processGatewayPacket(const TelemetryPacket &packet, int8_t rssi) {
  if (duplicatePacket(packet)) {
    Serial.printf("[gateway] duplicate node=%s seq=%lu ignored\n",
                  packet.nodeId, static_cast<unsigned long>(packet.sequence));
    return;
  }
  Serial.printf("[gateway] received node=%s seq=%lu from=%s rssi=%d dBm hop=%u ttl=%u",
                packet.nodeId, static_cast<unsigned long>(packet.sequence), packet.from,
                static_cast<int>(rssi), packet.hop, packet.ttl);
  if (packet.flags & FLAG_LUX_VALID)
    Serial.printf(" lux=%.2f lx", packet.luxCenti / 100.0f);
  if (packet.flags & FLAG_NTC_VALID)
    Serial.printf(" temp=%.2f C", packet.batteryTempCentiC / 100.0f);
  if (packet.flags & FLAG_BATTERY_VALID)
    Serial.printf(" battery=%.3f V", packet.batteryMilliVolts / 1000.0f);
  if (packet.flags & FLAG_WIND_VALID)
    Serial.printf(" wind=%.3f V", packet.windMilliVolts / 1000.0f);
  Serial.println();
  NodeDisplayInfo *nodeInfo = nullptr;
  for (auto &node : gatewayNodes) {
    if (node.seen && strncmp(node.packet.nodeId, packet.nodeId, NODE_ID_MAX_LEN) == 0) { nodeInfo = &node; break; }
  }
  if (!nodeInfo) {
    nodeInfo = &gatewayNodes[gatewayNodeCursor++ % GATEWAY_DEDUP_SIZE];
    *nodeInfo = NodeDisplayInfo{};
  }
  nodeInfo->packet = packet;
  nodeInfo->rssi = rssi;
  nodeInfo->lastSeenMs = millis();
  nodeInfo->seen = true;
  snprintf(gatewayLastNode, sizeof(gatewayLastNode), "%s", packet.nodeId);
  gatewayLastLuxValid = (packet.flags & FLAG_LUX_VALID) != 0;
  gatewayLastLux = gatewayLastLuxValid ? packet.luxCenti / 100.0f : NAN;
  publishPacket(packet, rssi);
}

void drainGatewayQueue() {
  while (true) {
    QueueItem item{};
    bool found = false;
    portENTER_CRITICAL(&queueMux);
    if (queueTail != queueHead) {
      item = queueItems[queueTail];
      queueTail = static_cast<uint8_t>((queueTail + 1) % GATEWAY_QUEUE_SIZE);
      found = true;
    }
    portEXIT_CRITICAL(&queueMux);
    if (!found) break;
    processGatewayPacket(item.packet, item.rssi);
  }
}
#endif
} // namespace

void setup() {
  initStatusLed();
#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
  Serial.begin(115200);
#if defined(ARDUINO_USB_MODE) && ARDUINO_USB_MODE
  delay(30);
#endif
#endif
  randomSeed(esp_random());
  Serial.printf("\n[boot] Solar sensor mesh prototype v%s\n", FW_VERSION);
  Serial.printf("[boot] chip=%s rev=%u cpu=%luMHz flash=%luMB\n", ESP.getChipModel(),
                ESP.getChipRevision(), static_cast<unsigned long>(getCpuFrequencyMhz()),
                static_cast<unsigned long>(ESP.getFlashChipSize() / (1024UL * 1024UL)));
#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
  Serial.println("[boot] role=gateway");
  Serial.println("[serial] enter 'wifi help' to configure Wi-Fi");
#elif defined(DEVICE_ROLE_RELAY) && DEVICE_ROLE_RELAY
  Serial.println("[boot] role=sensor-relay");
#else
  Serial.println("[boot] role=sensor-node");
#endif

#if (defined(DEVICE_ROLE_SENSOR) && DEVICE_ROLE_SENSOR) || (defined(DEVICE_ROLE_RELAY) && DEVICE_ROLE_RELAY)
  analogReadResolution(12);
#endif
#if defined(DEVICE_ROLE_SENSOR) && DEVICE_ROLE_SENSOR
  loadMeshConfig();
  nodeConsoleEnabled = serialConsoleConnected();
  if (nodeConsoleEnabled) {
    Serial.begin(115200);
#if defined(ARDUINO_USB_MODE) && ARDUINO_USB_MODE
    delay(30);
#endif
  }
  nodeWakeStartedMs = millis();
  const esp_sleep_wakeup_cause_t wakeCause = esp_sleep_get_wakeup_cause();
  if (wakeCause != ESP_SLEEP_WAKEUP_TIMER) {
    nodeSynced = false;
    rtcNodeSynced = false;
    missedSyncCount = 0;
    channelScanCandidate = runtimeMeshChannel;
  } else {
    nodeSynced = rtcNodeSynced;
    runtimeMeshChannel = rtcMeshChannel;
    runtimeSensorIntervalMs = rtcSensorIntervalMs;
    runtimeBeaconIntervalMs = rtcBeaconIntervalMs;
    runtimeActiveWindowMs = rtcActiveWindowMs;
  }
  const String nodeIdForJitter = deviceId();
  nodeInitialJitterMs = nodeIdForJitter.length() ? static_cast<uint8_t>(nodeIdForJitter[nodeIdForJitter.length() - 1]) % 500U : 0;
  nodeRadioChannel = (wakeCause == ESP_SLEEP_WAKEUP_TIMER && missedSyncCount > 0 &&
                      channelScanCandidate >= 1 && channelScanCandidate <= 13)
      ? channelScanCandidate : runtimeMeshChannel;
  if (nodeConsoleEnabled) {
    Serial.printf("[sensor] id=%s starting channel=%u interval=%lus\n",
                  deviceId().c_str(), runtimeMeshChannel, static_cast<unsigned long>(runtimeSensorIntervalMs / 1000UL));
  }
  analogSetPinAttenuation(NTC_ADC_PIN, ADC_11db);
  analogSetPinAttenuation(WIND_ADC_PIN, ADC_11db);
#if FEATURE_BATTERY_SENSE
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
#endif
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (nodeConsoleEnabled) {
    Serial.printf("[sensor] id=%s BH1750=0x%02X channel=%u interval=%lus\n",
                  deviceId().c_str(), BH1750_I2C_ADDRESS, runtimeMeshChannel,
                  static_cast<unsigned long>(runtimeSensorIntervalMs / 1000UL));
    Serial.printf("[sensor] ADC ntc=GPIO%u", NTC_ADC_PIN);
#if FEATURE_WIND_ADC
    Serial.printf(" wind=GPIO%u", WIND_ADC_PIN);
#endif
#if FEATURE_BATTERY_SENSE
    Serial.printf(" battery=GPIO%u", BATTERY_ADC_PIN);
#endif
    Serial.printf(" samples=%u\n", ADC_SAMPLES);
  }
  nodeSynced = wakeCause == ESP_SLEEP_WAKEUP_TIMER && rtcNodeSynced;
  nodeWindowOpen = nodeSynced;
  nodeSentThisWindow = false;
  nodeWakeStartedMs = millis();
  if (nodeSynced) {
    nodeWindowStartedAt = nodeWakeStartedMs;
    nodeWindowEndsAt = nodeWakeStartedMs + runtimeActiveWindowMs;
    nodeNextSlotAt = nodeWindowEndsAt;
    nodeTransmitAt = nodeWakeStartedMs + nodeInitialJitterMs + static_cast<uint32_t>(random(5, 25));
  }
  if (initMeshRadio(nodeRadioChannel)) {
    nodeRadioReady = true;
    esp_now_register_send_cb(onSampleSend);
    esp_now_register_recv_cb(onNodeReceive);
    if (nodeConsoleEnabled) Serial.printf("[sensor] listening for SYNC_BEACON on channel=%u\n", runtimeMeshChannel);
  } else if (nodeConsoleEnabled) Serial.println("[sensor] ESP-NOW initialization failed");
#endif

#if defined(DEVICE_ROLE_RELAY) && DEVICE_ROLE_RELAY && !(defined(DEVICE_ROLE_SENSOR) && DEVICE_ROLE_SENSOR)
  randomSeed(esp_random());
  if (initMeshRadio(ESPNOW_CHANNEL)) {
    esp_now_register_recv_cb(onRelayReceive);
    Serial.printf("[relay] listening channel=%u max_hops=%u\n", ESPNOW_CHANNEL, ESPNOW_TTL);
  } else Serial.println("[relay] ESP-NOW initialization failed");
#endif

#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
  loadMeshConfig();
  setMeshCommandHandler(handleMeshCommand);
  loadWifiConfig();
  loadMqttConfig();
  initMqtt();
  restoreRtcTime();
#if FEATURE_OLED
  initGatewayDisplay();
#endif
  WiFi.mode(WIFI_STA);
  // Broadcast ESP-NOW beacons on the configured mesh channel before Wi-Fi
  // associates.  Nodes that are awake in this window will learn the timing
  // and channel from the gateway without having to wait for the full Wi-Fi
  // connect / NTP cycle (which can take up to 30 s + NTP sync).
  {
    const uint8_t preWifiChannel = runtimeMeshChannel;
    gatewayRadioChannel = preWifiChannel;
    if (initMeshRadio(preWifiChannel)) {
      esp_now_register_recv_cb(onGatewayReceive);
      Serial.printf("[gateway] pre-WiFi ESP-NOW beacons on channel=%u\n", preWifiChannel);
      const uint32_t preBeaconEnd = millis() + 2000UL;  // 2 s window
      gatewayNextWindowAt     = millis() + runtimeSensorIntervalMs;
      gatewayNextSensorSlotAt = gatewayNextWindowAt;
      gatewayWindowOpenedAt   = 0;
      gatewayWindowActive     = false;
      gatewayNextBeaconAt     = millis();
      while (static_cast<int32_t>(preBeaconEnd - millis()) > 0) {
        if (static_cast<int32_t>(millis() - gatewayNextBeaconAt) >= 0) {
          sendSyncBeacon(false);
          gatewayNextBeaconAt += runtimeBeaconIntervalMs;
          if (static_cast<int32_t>(millis() - gatewayNextBeaconAt) >= 0)
            gatewayNextBeaconAt = millis() + runtimeBeaconIntervalMs;
        }
        delay(5);
      }
      esp_now_deinit();
      gatewayRadioChannel = 0;  // Will be set properly after Wi-Fi/channel resolution below.
      Serial.println("[gateway] pre-WiFi beacon window done");
    }
  }
  if (wifiProfileIsEmpty()) {
    Serial.println("[wifi] no profile configured; use 'wifi set ...' then 'wifi save' and 'wifi apply'");
  } else if (beginWifi()) {
    const uint32_t started = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - started < 30000) {
      delay(250);
#if FEATURE_OLED
      updateGatewayDisplay(gatewayDisplaySnapshot());
#endif
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    gatewayWifiWasConnected = true;
    gatewayRadioChannel = static_cast<uint8_t>(WiFi.channel());
    Serial.printf("[gateway] Wi-Fi IP=%s channel=%u (configure all nodes to match)\n",
                  WiFi.localIP().toString().c_str(), gatewayRadioChannel);
    configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2);
    markNtpSyncStarted();
    Serial.printf("[time] NTP sync started; valid=%s\n", currentTimeValid() ? "yes" : "no");
  } else {
    lastWifiReconnectAttempt = millis();
    gatewayRadioChannel = ESPNOW_CHANNEL;
    Serial.printf("[gateway] Wi-Fi unavailable; ESP-NOW channel=%u; MQTT disabled until Wi-Fi joins\n", gatewayRadioChannel);
  }
  gatewayRadioChannel = WiFi.status() == WL_CONNECTED
      ? static_cast<uint8_t>(WiFi.channel()) : runtimeMeshChannel;
  if (initMeshRadio(gatewayRadioChannel)) {
    esp_now_register_recv_cb(onGatewayReceive);
    gatewayNextWindowAt = millis() + runtimeSensorIntervalMs;
    gatewayNextSensorSlotAt = gatewayNextWindowAt;
    gatewayWindowOpenedAt = 0;
    gatewayWindowActive = false;
    gatewayNextBeaconAt = millis();
    Serial.printf("[gateway] ESP-NOW listening channel=%u sensor interval=%lus beacon=%lums MQTT=%s:%u\n",
                  gatewayRadioChannel, static_cast<unsigned long>(runtimeSensorIntervalMs / 1000UL),
                  static_cast<unsigned long>(runtimeBeaconIntervalMs), mqttHostName().c_str(), mqttBrokerPort());
  } else Serial.println("[gateway] ESP-NOW initialization failed");
  updateGatewayDisplay(gatewayDisplaySnapshot(), true);
#endif
}

void loop() {
  serviceStatusLed();
#if defined(DEVICE_ROLE_SENSOR) && DEVICE_ROLE_SENSOR
  serviceNodeMesh();
#endif
#if defined(DEVICE_ROLE_RELAY) && DEVICE_ROLE_RELAY && ESPNOW_ENABLE_RELAY
  // Relaying must not depend on a gateway sync beacon: an unsynchronized relay
  // still forwards queued telemetry/beacons while its radio is awake.
  relayQueuedPackets();
  relayQueuedBeacons();
#endif
#if (defined(DEVICE_ROLE_SENSOR) && DEVICE_ROLE_SENSOR) || (defined(DEVICE_ROLE_RELAY) && DEVICE_ROLE_RELAY)
  delay(5);
#endif
#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
  pollSerialCommands();
  const uint32_t scanCheckNow = millis();
  if (gatewayWindowActive &&
      static_cast<int32_t>(scanCheckNow - (gatewayWindowOpenedAt + runtimeActiveWindowMs)) >= 0) {
    gatewayWindowActive = false;
    if (static_cast<int32_t>(scanCheckNow - gatewayNextWindowAt) >= 0)
      gatewayNextWindowAt = scanCheckNow + runtimeSensorIntervalMs;
  }
  const int32_t scanWindowDelay = static_cast<int32_t>(gatewayNextWindowAt - millis());
  if (wifiScanRequestPending() && !gatewayWindowActive &&
      scanWindowDelay > static_cast<int32_t>(runtimeActiveWindowMs + 30000UL)) {
    clearWifiScanRequest();
    Serial.println("[wifi] scanning 2.4 GHz networks...");
    Serial.println("WIFI_SCAN_BEGIN");
    const int16_t count = WiFi.scanNetworks(false, true);
    if (count < 0) {
      Serial.printf("[wifi] scan failed (%d)\n", count);
      Serial.println("WIFI_SCAN_DONE");
    } else {
      Serial.printf("[wifi] scan found %d network(s)\n", count);
      for (int16_t i = 0; i < count; ++i) {
        JsonDocument scanResult;
        scanResult["ssid"] = WiFi.SSID(i);
        scanResult["rssi"] = WiFi.RSSI(i);
        scanResult["channel"] = WiFi.channel(i);
        scanResult["secure"] = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;
        Serial.print("WIFI_SCAN_JSON:");
        serializeJson(scanResult, Serial);
        Serial.println();
        Serial.printf("[wifi] network ssid=\"%s\" rssi=%ld dBm auth=%s channel=%ld\n",
                      WiFi.SSID(i).c_str(), static_cast<long>(WiFi.RSSI(i)),
                      WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "open" : "secured",
                      static_cast<long>(WiFi.channel(i)));
      }
      Serial.println("WIFI_SCAN_DONE");
    }
    WiFi.scanDelete();
  }
  const uint32_t meshNow = millis();
  if (wifiScanRequestPending()) {
    static uint32_t lastScanDeferralLog = 0;
    if (meshNow - lastScanDeferralLog > 2000) {
      Serial.println("[wifi] scan deferred until more than 30s outside the next mesh window");
      lastScanDeferralLog = meshNow;
    }
  }
  if (gatewayWindowActive && static_cast<int32_t>(meshNow - (gatewayWindowOpenedAt + runtimeActiveWindowMs)) >= 0) {
    gatewayWindowActive = false;
    if (static_cast<int32_t>(meshNow - gatewayNextWindowAt) >= 0)
      gatewayNextWindowAt = meshNow + runtimeSensorIntervalMs;
  }
  if (!gatewayWindowActive && static_cast<int32_t>(meshNow - gatewayNextWindowAt) >= 0) {
    gatewayWindowActive = true;
    gatewayWindowOpenedAt = meshNow;
    gatewayNextWindowAt += runtimeSensorIntervalMs;
    if (static_cast<int32_t>(meshNow - gatewayNextWindowAt) >= 0)
      gatewayNextWindowAt = meshNow + runtimeSensorIntervalMs;
    gatewayNextSensorSlotAt = gatewayWindowOpenedAt;
  }
  if (static_cast<int32_t>(meshNow - gatewayNextBeaconAt) >= 0) {
    const bool transitionBeacon = gatewayChannelChangePending;
    if (sendSyncBeacon(gatewayWindowActive) && transitionBeacon &&
        static_cast<int32_t>(millis() - gatewayChannelSwitchAt) >= 0) {
      delay(20);  // Let the final old-channel beacon leave the radio before deinit.
      if (WiFi.status() == WL_CONNECTED && WiFi.channel() != gatewayPendingChannel) {
        gatewayChannelChangePending = false;
        gatewayPendingChannel = gatewayRadioChannel;
        gatewayChannelSwitchAt = 0;
        Serial.printf("[gateway] AP is still on channel %u; ESP-NOW migration cancelled\n",
                      static_cast<unsigned>(WiFi.channel()));
      } else {
        esp_now_deinit();
        gatewayRadioChannel = gatewayPendingChannel;
        runtimeMeshChannel = gatewayPendingChannel;
        if (initMeshRadio(gatewayRadioChannel)) {
          esp_now_register_recv_cb(onGatewayReceive);
          gatewayNextWindowAt = millis() + runtimeSensorIntervalMs;
          gatewayNextSensorSlotAt = gatewayNextWindowAt;
          gatewayWindowOpenedAt = 0;
          gatewayWindowActive = false;
          gatewayChannelChangePending = false;
          gatewayNextBeaconAt = millis();
          Serial.printf("[gateway] switched ESP-NOW to channel %u\n", gatewayRadioChannel);
        } else {
          gatewayChannelChangePending = false;
          Serial.println("[gateway] channel switch failed; ESP-NOW is unavailable");
        }
      }
    }
    gatewayNextBeaconAt += runtimeBeaconIntervalMs;
    if (static_cast<int32_t>(meshNow - gatewayNextBeaconAt) >= 0)
      gatewayNextBeaconAt = meshNow + runtimeBeaconIntervalMs;
  }
  if (takeWifiApplyRequest()) {
    disconnectMqtt();
    if (WiFi.status() == WL_CONNECTED) { WiFi.disconnect(false, false); delay(100); }
    normalizeWifiProfile();
    if (beginWifi()) {
      Serial.printf("[wifi] connecting SSID=%s; waiting up to 30s\n", wifiSsid().c_str());
      uint32_t started = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - started < 30000) delay(250);
      if (WiFi.status() == WL_CONNECTED) {
        gatewayRadioChannel = static_cast<uint8_t>(WiFi.channel()); gatewayWifiWasConnected = true;
        esp_now_deinit();
        if (initMeshRadio(gatewayRadioChannel)) {
          esp_now_register_recv_cb(onGatewayReceive);
          gatewayNextWindowAt = millis() + runtimeSensorIntervalMs;
          gatewayWindowActive = false;
          gatewayWindowOpenedAt = 0;
          gatewayNextBeaconAt = millis();
        }
        configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2);
        markNtpSyncStarted();
        Serial.printf("[wifi] connected IP=%s channel=%u; NTP and ESP-NOW restarted\n", WiFi.localIP().toString().c_str(), gatewayRadioChannel);
      } else Serial.println("[wifi] failed; see the reason code above (SSID coverage vs. credentials)");
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    if (!gatewayWifiWasConnected) {
      gatewayWifiWasConnected = true;
      const uint8_t newChannel = static_cast<uint8_t>(WiFi.channel());
      if (newChannel != gatewayRadioChannel) {
        Serial.printf("[gateway] AP channel changed %u -> %u; restarting ESP-NOW on the new AP channel\n", gatewayRadioChannel, newChannel);
        esp_now_deinit();
        gatewayRadioChannel = newChannel;
        if (initMeshRadio(gatewayRadioChannel)) {
          esp_now_register_recv_cb(onGatewayReceive);
          gatewayNextWindowAt = millis() + runtimeSensorIntervalMs;
          gatewayWindowActive = false;
          gatewayWindowOpenedAt = 0;
          gatewayNextBeaconAt = millis();
        } else Serial.println("[gateway] ESP-NOW restart failed after AP channel change");
      }
      configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2);
      markNtpSyncStarted();
      Serial.println("[time] NTP sync restarted after Wi-Fi reconnect");
    }
    reconnectMqtt();
  } else {
    gatewayWifiWasConnected = false;
    // Only re-issue a connection once the driver reports a finished (failed)
    // attempt; disconnecting mid-attempt aborts otherwise healthy retries.
    const wl_status_t wifiStatus = WiFi.status();
    const bool attemptFinished = wifiStatus != WL_IDLE_STATUS &&
        wifiStatus != WL_SCAN_COMPLETED && wifiStatus != WL_CONNECTED;
    if (!wifiProfileIsEmpty() && attemptFinished &&
        millis() - lastWifiReconnectAttempt >= 15000) {
      lastWifiReconnectAttempt = millis();
      beginWifi();
    }
  }
  pollTimeSync();
  serviceMqtt();
  drainGatewayQueue();
#if FEATURE_OLED
  updateGatewayDisplay(gatewayDisplaySnapshot());
#endif
  static uint32_t lastStats = 0;
  if (millis() - lastStats >= 30000) {
    lastStats = millis();
    Serial.printf("[gateway] invalid=%lu queue_dropped=%lu mqtt=%s\n",
                  static_cast<unsigned long>(invalidPackets), static_cast<unsigned long>(queueDropped),
                  mqttConnected() ? "up" : "down");
  }
  delay(5);
#endif
}

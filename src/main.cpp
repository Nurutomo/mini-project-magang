#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include <esp_mac.h>
#if CONFIG_IDF_TARGET_ESP32C3
#include <driver/usb_serial_jtag.h>
#endif
#include <math.h>
#include <stddef.h>
#include <string.h>
#include <ArduinoJson.h>

#include "config.h"
#include "telemetry.h"

#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <time.h>
#include <sys/time.h>
#include <esp_rtc_time.h>
#include <esp_sntp.h>
#endif

namespace {
constexpr uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
constexpr uint32_t SENSOR_SETTLE_MS = 4;

#if STATUS_LED_PIN >= 0
volatile uint8_t statusLedPendingPulses = 0;
portMUX_TYPE statusLedMux = portMUX_INITIALIZER_UNLOCKED;
bool statusLedOn = false;
bool statusLedGap = false;
uint32_t statusLedNextTransitionMs = 0;
#endif

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

void initStatusLed() {
#if STATUS_LED_PIN >= 0
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, STATUS_LED_ACTIVE_HIGH ? LOW : HIGH);
#endif
}

void loadMeshConfig();
bool saveMeshConfig();
void configureMaxTxPower();

#if defined(DEVICE_ROLE_SENSOR) && DEVICE_ROLE_SENSOR
void pollNodeSerial();
void applyPendingSyncBeacon();
void onNodeReceive(const esp_now_recv_info_t *, const uint8_t *, int);
void serviceNodeMesh();
void onSampleSend(const esp_now_send_info_t *, esp_now_send_status_t);
bool publishSample();
#endif
#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
bool sendSyncBeacon(bool windowOpen);
void onGatewayReceive(const esp_now_recv_info_t *, const uint8_t *, int);
extern uint8_t gatewayRadioChannel;
#endif

void blinkStatusLed(uint8_t pulses = 1) {
#if STATUS_LED_PIN >= 0
  portENTER_CRITICAL(&statusLedMux);
  const uint16_t queued = static_cast<uint16_t>(statusLedPendingPulses) + pulses;
  statusLedPendingPulses = static_cast<uint8_t>(queued > 20 ? 20 : queued);
  portEXIT_CRITICAL(&statusLedMux);
#else
  (void)pulses;
#endif
}

void serviceStatusLed() {
#if STATUS_LED_PIN >= 0
  const uint32_t now = millis();
  if (statusLedOn && static_cast<int32_t>(now - statusLedNextTransitionMs) >= 0) {
    digitalWrite(STATUS_LED_PIN, STATUS_LED_ACTIVE_HIGH ? LOW : HIGH);
    statusLedOn = false;
    statusLedGap = true;
    statusLedNextTransitionMs = now + 45;
  }
  if (!statusLedOn && statusLedPendingPulses &&
      (!statusLedGap || static_cast<int32_t>(now - statusLedNextTransitionMs) >= 0)) {
    portENTER_CRITICAL(&statusLedMux);
    if (statusLedPendingPulses) --statusLedPendingPulses;
    portEXIT_CRITICAL(&statusLedMux);
    digitalWrite(STATUS_LED_PIN, STATUS_LED_ACTIVE_HIGH ? HIGH : LOW);
    statusLedOn = true;
    statusLedGap = false;
    statusLedNextTransitionMs = now + 70;
  }
#endif
}

RTC_DATA_ATTR uint8_t rtcMeshChannel = ESPNOW_CHANNEL;
RTC_DATA_ATTR uint32_t rtcSensorIntervalMs = SAMPLE_INTERVAL_S * 1000UL;
RTC_DATA_ATTR uint32_t rtcBeaconIntervalMs = SYNC_BEACON_INTERVAL_MS;
RTC_DATA_ATTR uint32_t rtcActiveWindowMs = NODE_ACTIVE_WINDOW_MS;
RTC_DATA_ATTR uint32_t lastSyncSleepMs = 0;
RTC_DATA_ATTR uint16_t missedSyncCount = 0;

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
  if (runtimeSensorIntervalMs < 10000 || runtimeSensorIntervalMs > 86400000UL)
    runtimeSensorIntervalMs = SAMPLE_INTERVAL_S * 1000UL;
  if (runtimeActiveWindowMs < 500 || runtimeActiveWindowMs > 10000)
    runtimeActiveWindowMs = NODE_ACTIVE_WINDOW_MS;
  if (runtimeSensorIntervalMs <= runtimeActiveWindowMs)
    runtimeSensorIntervalMs = SAMPLE_INTERVAL_S * 1000UL > runtimeActiveWindowMs
        ? SAMPLE_INTERVAL_S * 1000UL : runtimeActiveWindowMs + 10000UL;
  if (runtimeBeaconIntervalMs < 100 || runtimeBeaconIntervalMs > 60000 ||
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
uint32_t nodeWindowStartedAt = 0;
uint32_t nodeWindowEndsAt = 0;
uint32_t nodeInitialJitterMs = 0;
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

String deviceId() {
  if (strlen(NODE_ID) > 0) return String(NODE_ID);
  uint8_t mac[6] = {};
  // Reads the eFuse MAC; unlike WiFi.macAddress() this works before WiFi.mode().
  if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK) return String("node-unknown");
  char id[16];
  snprintf(id, sizeof(id), "node-%02X%02X%02X", mac[3], mac[4], mac[5]);
  return String(id);
}

#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
struct WifiConfig { String ssid, password, username, identity, caCert; bool enterprise = false; };
WifiConfig wifiConfig;
Preferences wifiPreferences;
String serialCommand;
bool wifiApplyRequested = false;
String mqttHost = MQTT_HOST;
uint16_t mqttPort = MQTT_PORT;
String mqttUser = MQTT_USER;
String mqttPassword = MQTT_PASSWORD;
Preferences mqttPreferences;
extern PubSubClient mqttClient;
bool wifiScanRequested = false;
uint32_t wifiTimeSyncStartedMs = 0;
constexpr int64_t TIME_VALID_EPOCH = RTC_TIME_VALID;
constexpr unsigned long RTC_TIME_SAVE_INTERVAL_MS = 3600000UL;
RTC_DATA_ATTR int64_t rtcBackupUnixTime = 0;
RTC_DATA_ATTR uint64_t rtcBackupTimerUs = 0;
RTC_DATA_ATTR bool rtcBackupValid = false;
bool beginWifi() {
  configureMaxTxPower();
#if defined(CONFIG_ESP_WIFI_ENTERPRISE_SUPPORT) && CONFIG_ESP_WIFI_ENTERPRISE_SUPPORT
  // Eduroam is WPA2-Enterprise (typically PEAP/MSCHAPv2), not WPA2-PSK.
  if (wifiConfig.enterprise) {
    if (wifiConfig.username.isEmpty() || wifiConfig.password.isEmpty()) {
      Serial.println("[wifi] eduroam username/password missing"); return false;
    }
    Serial.printf("[wifi] connecting WPA2-Enterprise SSID=%s identity=%s\n", wifiConfig.ssid.c_str(), wifiConfig.identity.c_str());
    if (wifiConfig.caCert.isEmpty()) {
      WiFi.begin(wifiConfig.ssid.c_str(), WPA2_AUTH_PEAP, wifiConfig.identity.c_str(),
                 wifiConfig.username.c_str(), wifiConfig.password.c_str());
    } else {
      WiFi.begin(wifiConfig.ssid.c_str(), WPA2_AUTH_PEAP, wifiConfig.identity.c_str(),
                 wifiConfig.username.c_str(), wifiConfig.password.c_str(), wifiConfig.caCert.c_str());
    }
    return true;
  }
#else
  if (wifiConfig.enterprise) { Serial.println("[wifi] enterprise support is unavailable in this build"); return false; }
#endif
  Serial.printf("[wifi] WPA2-PSK SSID=%s\n", wifiConfig.ssid.c_str());
  WiFi.begin(wifiConfig.ssid.c_str(), wifiConfig.password.c_str());
  return true;
}

void loadMqttConfig() {
  mqttPreferences.begin(MQTT_CONFIG_NAMESPACE, true);
  mqttHost = mqttPreferences.getString(MQTT_CONFIG_HOST_KEY, MQTT_HOST);
  mqttPort = mqttPreferences.getUShort(MQTT_CONFIG_PORT_KEY, MQTT_PORT);
  mqttUser = mqttPreferences.getString(MQTT_CONFIG_USER_KEY, MQTT_USER);
  mqttPassword = mqttPreferences.getString(MQTT_CONFIG_PASSWORD_KEY, MQTT_PASSWORD);
  mqttPreferences.end();
}

void loadWifiConfig() {
  wifiConfig.ssid = WIFI_SSID;
  wifiConfig.enterprise = wifiConfig.ssid.equalsIgnoreCase("eduroam");
  wifiConfig.password = wifiConfig.enterprise ? EDUROAM_PASSWORD : WIFI_PASSWORD;
  wifiConfig.username = EDUROAM_USERNAME; wifiConfig.identity = EDUROAM_IDENTITY;
  wifiConfig.caCert = EDUROAM_CA_CERT;
  wifiPreferences.begin(WIFI_CONFIG_NAMESPACE, true);
  if (wifiPreferences.isKey(WIFI_CONFIG_SSID_KEY)) {
    wifiConfig.ssid = wifiPreferences.getString(WIFI_CONFIG_SSID_KEY);
    wifiConfig.password = wifiPreferences.getString(WIFI_CONFIG_PASSWORD_KEY);
    wifiConfig.username = wifiPreferences.getString(WIFI_CONFIG_USERNAME_KEY);
    wifiConfig.identity = wifiPreferences.getString(WIFI_CONFIG_IDENTITY_KEY);
    wifiConfig.caCert = wifiPreferences.getString(WIFI_CONFIG_CA_CERT_KEY);
    wifiConfig.enterprise = wifiConfig.ssid.equalsIgnoreCase("eduroam");
  }
  wifiPreferences.end();
  if (wifiConfig.enterprise && wifiConfig.identity.isEmpty()) wifiConfig.identity = wifiConfig.username;
}

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
    Serial.printf("[mesh] channel=%u sensor_interval_s=%lu beacon_interval_ms=%lu active_window_ms=%lu\n",
                  displayedChannel, static_cast<unsigned long>(runtimeSensorIntervalMs / 1000UL),
                  static_cast<unsigned long>(runtimeBeaconIntervalMs),
                  static_cast<unsigned long>(runtimeActiveWindowMs));
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
  const long sensorSec = count > 1 ? strtol(values[1].c_str(), &end, 10) : 0;
  valid = valid && end && !*end && sensorSec >= 10 && sensorSec <= 86400 &&
          sensorSec * 1000L > runtimeActiveWindowMs;
  const long beaconMs = count > 2 ? strtol(values[2].c_str(), &end, 10) : 0;
  valid = valid && end && !*end && beaconMs >= 100 && beaconMs <= 60000;
  if (!valid) {
    Serial.println("[mesh] usage: mesh set <channel 1..13> <sensor_interval_seconds 10..86400> <beacon_interval_ms 100..1500>");
    return;
  }
  if (WiFi.status() == WL_CONNECTED && channel != WiFi.channel()) {
    Serial.printf("[mesh] channel %ld conflicts with Wi-Fi AP channel %ld; AP channel is authoritative\n",
                  channel, static_cast<long>(WiFi.channel()));
    return;
  }
  const uint32_t newActiveWindowMs = runtimeActiveWindowMs;
  if (beaconMs > static_cast<long>(newActiveWindowMs / 2)) {
    Serial.printf("[mesh] beacon interval must be <= half the active window (%lu ms)\n",
                  static_cast<unsigned long>(newActiveWindowMs / 2));
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
  runtimeSensorIntervalMs = static_cast<uint32_t>(sensorSec) * 1000UL;
  runtimeBeaconIntervalMs = static_cast<uint32_t>(beaconMs);
  runtimeActiveWindowMs = newActiveWindowMs;
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

void handleWifiCommand(String command) {
  command.trim();
  if (command.startsWith("mesh ")) { handleMeshCommand(command); return; }
  if (command == "mqtt show") {
    Serial.printf("[mqtt] host=%s port=%u username=%s password=[hidden]\n", mqttHost.c_str(), mqttPort, mqttUser.isEmpty() ? "(none)" : "(set)"); return;
  }
  if (command == "mqtt save") {
    mqttPreferences.begin(MQTT_CONFIG_NAMESPACE, false);
    mqttPreferences.putString(MQTT_CONFIG_HOST_KEY, mqttHost);
    mqttPreferences.putUShort(MQTT_CONFIG_PORT_KEY, mqttPort);
    mqttPreferences.putString(MQTT_CONFIG_USER_KEY, mqttUser);
    mqttPreferences.putString(MQTT_CONFIG_PASSWORD_KEY, mqttPassword);
    mqttPreferences.end();
    Serial.println("[mqtt] broker settings saved in NVS"); return;
  }
  if (command == "mqtt clear") {
    mqttPreferences.begin(MQTT_CONFIG_NAMESPACE, false); mqttPreferences.clear(); mqttPreferences.end();
    Serial.println("[mqtt] saved settings removed; reboot to use build defaults"); return;
  }
  if (command.startsWith("mqtt set ")) {
    String fields = command.substring(9); fields.trim();
    String values[4]; uint8_t count = 0;
    while (fields.length() && count < 4) {
      int split = fields.indexOf(' ');
      values[count++] = split < 0 ? fields : fields.substring(0, split);
      if (split < 0) fields = ""; else { fields = fields.substring(split + 1); fields.trim(); }
    }
    bool validHost = values[0].length() <= 253;
    for (size_t i = 0; i < values[0].length(); ++i) {
      const char c = values[0][i];
      if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
            (c >= '0' && c <= '9') || c == '.' || c == '-')) validHost = false;
    }
    if (count < 2 || count == 3 || !fields.isEmpty() || !validHost || values[0].isEmpty() ||
        values[0][0] == '.' || values[0][0] == '-' || values[0][values[0].length() - 1] == '.' ||
        values[0][values[0].length() - 1] == '-' || (count >= 3 && values[2].length() > 128) ||
        (count >= 4 && (values[2].isEmpty() || values[2].length() > 128 || values[3].length() > 128))) {
      Serial.println("[mqtt] usage: mqtt set <host> <port> [username password] (DNS/IPv4 host, tokens without spaces)"); return;
    }
    char *end = nullptr; const long port = strtol(values[1].c_str(), &end, 10);
    if (!end || *end || port < 1 || port > 65535) { Serial.println("[mqtt] port must be 1..65535"); return; }
    mqttHost = values[0]; mqttPort = static_cast<uint16_t>(port);
    mqttUser = count >= 3 ? values[2] : ""; mqttPassword = count >= 4 ? values[3] : "";
    mqttClient.disconnect(); mqttClient.setServer(mqttHost.c_str(), mqttPort);
    Serial.println("[mqtt] settings updated (password hidden); use 'mqtt save' to persist"); return;
  }
  if (command == "wifi" || command == "wifi help") {
    Serial.println("mesh set <channel> <sensor_interval_seconds> <beacon_interval_ms> | mesh show");
    Serial.println("wifi set <ssid> <password>");
    Serial.println("wifi set eduroam <username> <password> [identity]");
    Serial.println("wifi ca <single-line PEM; replace newlines with \\n> | wifi ca clear");
    Serial.println("wifi scan | wifi save | wifi apply | wifi show | wifi clear");
    Serial.println("mqtt set <host> <port> [username password]");
    Serial.println("mqtt show | mqtt save | mqtt clear");
    Serial.println("Fields are space-separated. Your serial terminal may echo typed credentials.");
    return;
  }
  if (command == "wifi show") {
    Serial.printf("[wifi] SSID=%s auth=%s password=[hidden]\n", wifiConfig.ssid.c_str(), wifiConfig.enterprise ? "PEAP" : "PSK"); return;
  }
  if (command == "wifi save") {
    if (wifiConfig.ssid.isEmpty()) { Serial.println("[wifi] set a profile first"); return; }
    if (wifiConfig.enterprise && wifiConfig.identity.isEmpty()) wifiConfig.identity = wifiConfig.username;
    wifiPreferences.begin(WIFI_CONFIG_NAMESPACE, false);
    wifiPreferences.putString(WIFI_CONFIG_SSID_KEY, wifiConfig.ssid);
    wifiPreferences.putString(WIFI_CONFIG_PASSWORD_KEY, wifiConfig.password);
    wifiPreferences.putString(WIFI_CONFIG_USERNAME_KEY, wifiConfig.username);
    wifiPreferences.putString(WIFI_CONFIG_IDENTITY_KEY, wifiConfig.identity);
    wifiPreferences.putString(WIFI_CONFIG_CA_CERT_KEY, wifiConfig.caCert);
    wifiPreferences.end();
    Serial.println("[wifi] profile saved in NVS"); return;
  }
  if (command == "wifi clear") {
    wifiPreferences.begin(WIFI_CONFIG_NAMESPACE, false); wifiPreferences.clear(); wifiPreferences.end();
    Serial.println("[wifi] saved profile removed; reboot to use build defaults"); return;
  }
  if (command == "wifi ca clear") {
    wifiConfig.caCert = "";
    Serial.println("[wifi] CA removed from active profile; use 'wifi save' to persist"); return;
  }
  if (command.startsWith("wifi ca ")) {
    String certificate = command.substring(8);
    certificate.replace("\\n", "\n");
    if (!certificate.startsWith("-----BEGIN CERTIFICATE-----") || !certificate.endsWith("-----END CERTIFICATE-----")) {
      Serial.println("[wifi] invalid PEM certificate"); return;
    }
    if (certificate.length() > 5000) { Serial.println("[wifi] CA certificate too long (max 5000 chars)"); return; }
    wifiConfig.caCert = certificate;
    Serial.println("[wifi] CA set; use 'wifi save' to persist"); return;
  }
  if (command.startsWith("wifi set eduroam ")) {
    String fields = command.substring(16); fields.trim();
    int p1 = fields.indexOf(' ');
    if (p1 <= 0) { Serial.println("[wifi] usage: wifi set eduroam <username> <password> [identity]"); return; }
    int p2 = fields.indexOf(' ', p1 + 1);
    String username = fields.substring(0, p1);
    String password = p2 < 0 ? fields.substring(p1 + 1) : fields.substring(p1 + 1, p2);
    String identity = p2 < 0 ? username : fields.substring(p2 + 1);
    if (username.isEmpty() || password.isEmpty() || password.length() > 63 || identity.isEmpty() || identity.indexOf(' ') >= 0) {
      Serial.println("[wifi] invalid fields; use single tokens without spaces"); return;
    }
    if (username.length() > 128 || identity.length() > 128) {
      Serial.println("[wifi] username/identity too long (max 128 chars)"); return;
    }
    wifiConfig.ssid = "eduroam"; wifiConfig.username = username;
    wifiConfig.password = password; wifiConfig.identity = identity;
    wifiConfig.enterprise = true;
    Serial.println("[wifi] eduroam profile set (password hidden); use 'wifi save' then 'wifi apply'"); return;
  }
  if (command.startsWith("wifi set ")) {
    String fields = command.substring(9); fields.trim();
    int p = fields.indexOf(' ');
    if (p <= 0 || p == static_cast<int>(fields.length()) - 1) { Serial.println("[wifi] usage: wifi set <ssid> <password>"); return; }
    String ssid = fields.substring(0, p);
    String password = fields.substring(p + 1);
    if (ssid.length() > 32 || password.length() > 63 || ssid.indexOf(' ') >= 0 || ssid.equalsIgnoreCase("eduroam")) {
      Serial.println("[wifi] invalid PSK profile; SSID must be space-free (max 32) and not eduroam; password max 63"); return;
    }
    wifiConfig.ssid = ssid; wifiConfig.password = password;
    wifiConfig.username = ""; wifiConfig.identity = ""; wifiConfig.caCert = "";
    wifiConfig.enterprise = wifiConfig.ssid.equalsIgnoreCase("eduroam");
    Serial.println("[wifi] PSK profile set (password hidden); use 'wifi save' then 'wifi apply'"); return;
  }
  if (command == "wifi scan") {
    wifiScanRequested = true;
    Serial.println("[wifi] scan queued"); return;
  }
  if (command == "wifi apply") {
    wifiApplyRequested = true;
    Serial.println("[wifi] will reconnect with the new profile"); return;
  }
  Serial.println("[serial] unknown command; use 'wifi help'");
}

void pollSerialCommands() {
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (!serialCommand.isEmpty()) { handleWifiCommand(serialCommand); serialCommand = ""; }
    } else if (serialCommand.length() < 6000) serialCommand += c;
    else { serialCommand = ""; Serial.println("[serial] command too long; discarded"); }
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

uint16_t readAdcMillivolts(uint8_t pin) {
  analogRead(pin); // discard conversion after channel switch
  delay(SENSOR_SETTLE_MS);
  uint32_t total = 0;
  uint8_t valid = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES; ++i) {
    const uint32_t millivolts = analogReadMilliVolts(pin);
    if (millivolts > 0) {
      total += millivolts;
      ++valid;
    }
    delay(2);
  }
  if (valid == 0) {
    static uint32_t lastWarnMs = 0;
    if (millis() - lastWarnMs >= 5000) {
      lastWarnMs = millis();
      Serial.printf("[sensor] GPIO%u ADC read failed; check wiring. On classic ESP32 use ADC1 "
                    "GPIO32-39 because ADC2 (GPIO0/2/4/12-15/25-27) does not work with Wi-Fi.\n",
                    pin);
    }
    return 0;
  }
  return static_cast<uint16_t>(total / valid);
}

bool readBh1750(float &lux) {
  Wire.beginTransmission(BH1750_I2C_ADDRESS);
  Wire.write(0x10); // continuous high-resolution mode, approx. 1 lx resolution
  if (Wire.endTransmission() != 0) return false;
  delay(180);
  if (Wire.requestFrom(static_cast<int>(BH1750_I2C_ADDRESS), 2) != 2) return false;
  const uint16_t raw = (static_cast<uint16_t>(Wire.read()) << 8) | Wire.read();
  lux = raw / 1.2f;
  return isfinite(lux) && lux >= 0.0f && lux <= 200000.0f;
}

float readNtcCelsius(bool &valid) {
  const float mv = readAdcMillivolts(NTC_ADC_PIN);
  const float vref = ADC_REFERENCE_MV;
  if (mv <= 1 || mv >= vref - 1 || NTC_BETA <= 0.0f) {
    valid = false;
    return NAN;
  }
  // 3V3 -> fixed resistor -> ADC -> NTC -> GND.
  const float resistance = NTC_SERIES_OHM * mv / (vref - mv);
  const float nominalK = NTC_NOMINAL_TEMP_C + 273.15f;
  const float invT = (1.0f / nominalK) + logf(resistance / NTC_NOMINAL_OHM) / NTC_BETA;
  if (!isfinite(invT) || invT <= 0.0f) {
    valid = false;
    return NAN;
  }
  const float celsius = (1.0f / invT) - 273.15f;
  valid = isfinite(celsius) && celsius > -40.0f && celsius < 150.0f;
  return valid ? celsius : NAN;
}

RTC_DATA_ATTR uint32_t sequenceNumber = 0;

TelemetryPacket collectTelemetry() {
  TelemetryPacket packet = makeEmptyTelemetry();
  const String id = deviceId();
  strlcpy(packet.nodeId, id.c_str(), sizeof(packet.nodeId));
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

void configureMaxTxPower() {
  // ESP-IDF expresses the limit in quarter-dBm units; 78 is 19.5 dBm,
  // the highest portable setting for the supported ESP32 targets.
  constexpr int8_t maxTxPowerQuarterDbm = 78;
  const esp_err_t result = esp_wifi_set_max_tx_power(maxTxPowerQuarterDbm);
  if (result != ESP_OK)
    Serial.printf("[wifi] max TX power setup failed (%d)\n", static_cast<int>(result));
}

bool initEspNow(uint8_t channel) {
  WiFi.mode(WIFI_STA);
  configureMaxTxPower();
  WiFi.setAutoReconnect(false);
  const bool stationConnected = WiFi.status() == WL_CONNECTED;
  if (stationConnected && channel != WiFi.channel()) return false;
  if (!stationConnected) {
    WiFi.disconnect(false, false);
    if (esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE) != ESP_OK) return false;
  }
  if (esp_now_init() != ESP_OK) return false;
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, BROADCAST_MAC, sizeof(BROADCAST_MAC));
  peer.channel = channel;
  peer.encrypt = false;
  if (!esp_now_is_peer_exist(BROADCAST_MAC) && esp_now_add_peer(&peer) != ESP_OK) return false;
  return true;
}

#if defined(DEVICE_ROLE_SENSOR) && DEVICE_ROLE_SENSOR
#if defined(DEVICE_ROLE_RELAY) && DEVICE_ROLE_RELAY && ESPNOW_ENABLE_RELAY
void onRelayReceive(const esp_now_recv_info_t *, const uint8_t *data, int len);
#endif

void onNodeReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (isValidSyncBeacon(data, static_cast<size_t>(len))) {
    const auto *beacon = reinterpret_cast<const SyncBeacon *>(data);
    portENTER_CRITICAL(&syncBeaconMux);
    pendingSyncBeacon = *beacon;
    hasPendingSyncBeacon = true;
    portEXIT_CRITICAL(&syncBeaconMux);
    blinkStatusLed(1);
    return;
  }
#if defined(DEVICE_ROLE_RELAY) && DEVICE_ROLE_RELAY && ESPNOW_ENABLE_RELAY
  if (isValidTelemetryPacket(data, static_cast<size_t>(len))) onRelayReceive(nullptr, data, len);
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
    if (initEspNow(nodeRadioChannel)) {
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
      if (initEspNow(nodeRadioChannel)) {
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
    if (initEspNow(nodeRadioChannel)) {
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
  if (!nodeSynced && now - nodeWakeStartedMs >= awakeLimit) {
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
              interval >= 10 && interval <= 86400 && interval * 1000L > runtimeActiveWindowMs &&
              beacon >= 100 && beacon <= 60000 && beacon <= runtimeActiveWindowMs / 2) {
            const uint8_t previousNodeChannel = runtimeMeshChannel;
            if (channel != previousNodeChannel) Serial.println("[mesh-info] configured channel; gateway announcement takes precedence when received");
            runtimeMeshChannel = static_cast<uint8_t>(channel);
            runtimeSensorIntervalMs = static_cast<uint32_t>(interval) * 1000UL;
            runtimeBeaconIntervalMs = static_cast<uint32_t>(beacon);
            runtimeActiveWindowMs = NODE_ACTIVE_WINDOW_MS;
            if (saveMeshConfig()) Serial.println("[mesh] saved; reboot to apply local configuration");
            else {
              runtimeMeshChannel = previousNodeChannel;
              rtcMeshChannel = previousNodeChannel;
              Serial.println("[mesh] failed to save configuration");
            }
          } else Serial.println("[mesh] values out of range; sensor interval must exceed active window and beacon must be 100..1500 ms");
        } else Serial.println("[mesh] usage: mesh set <channel> <sensor_interval_seconds> <beacon_interval_ms <= 1500>");
      } else if (nodeSerialCommand == "mesh show") {
        Serial.printf("[mesh] channel=%u sensor_interval_s=%lu beacon_interval_ms=%lu missed_sync=%u\n",
                      runtimeMeshChannel, static_cast<unsigned long>(runtimeSensorIntervalMs / 1000UL),
                      static_cast<unsigned long>(runtimeBeaconIntervalMs), missedSyncCount);
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

bool publishSample() {
  const TelemetryPacket packet = collectTelemetry();
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

void onRelayReceive(const esp_now_recv_info_t *, const uint8_t *data, int len) {
  if (!isValidTelemetryPacket(data, static_cast<size_t>(len))) return;
  blinkStatusLed(1);
  RelayItem item{};
  memcpy(&item.packet, data, sizeof(TelemetryPacket));
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
    const esp_err_t result = esp_now_send(BROADCAST_MAC,
                                           reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
    Serial.printf("[relay] %s node=%s seq=%lu hop=%u ttl=%u\n",
                  result == ESP_OK ? "forwarded" : "send error", packet.nodeId,
                  static_cast<unsigned long>(packet.sequence), packet.hop, packet.ttl);
  }
}
#endif

#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
Adafruit_SSD1306 gatewayDisplay(128, 64, &Wire, -1);
bool gatewayDisplayReady = false;
uint32_t displayLastRefresh = 0;
bool gatewayWifiWasConnected = false;
uint8_t gatewayRadioChannel = ESPNOW_CHANNEL;
uint32_t lastWifiReconnectAttempt = 0;
struct QueueItem { TelemetryPacket packet; uint8_t sourceMac[6]; int8_t rssi; };
QueueItem queueItems[GATEWAY_QUEUE_SIZE];
struct NodeDisplayInfo { TelemetryPacket packet{}; int8_t rssi = -127; uint32_t lastSeenMs = 0; bool seen = false; };
NodeDisplayInfo gatewayNodes[GATEWAY_DEDUP_SIZE]{};
uint8_t gatewayNodeCursor = 0;
uint8_t gatewayDisplayPage = 0;
uint32_t gatewayDisplayPageStarted = 0;
uint32_t nodeOnlineTimeoutMs() { return runtimeSensorIntervalMs * 3UL; }
volatile uint8_t queueHead = 0, queueTail = 0;
portMUX_TYPE queueMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t queueDropped = 0, invalidPackets = 0, gatewayRxCount = 0;
char gatewayLastNode[NODE_ID_MAX_LEN] = "-";
float gatewayLastLux = NAN;
bool gatewayLastLuxValid = false;
struct SeenPacket { char nodeId[NODE_ID_MAX_LEN]; uint32_t sequence; };
SeenPacket seenPackets[GATEWAY_DEDUP_SIZE]{};
uint16_t seenCursor = 0;

uint8_t countOnlineNodes(uint32_t now) {
  uint8_t count = 0;
  for (const auto &node : gatewayNodes) {
    if (node.seen && now - node.lastSeenMs <= nodeOnlineTimeoutMs()) ++count;
  }
  return count;
}

int findDisplayNode(uint8_t ordinal, uint32_t now) {
  for (uint8_t i = 0; i < GATEWAY_DEDUP_SIZE; ++i) {
    if (gatewayNodes[i].seen && now - gatewayNodes[i].lastSeenMs <= nodeOnlineTimeoutMs()) {
      if (ordinal-- == 0) return i;
    }
  }
  return -1;
}

void updateGatewayDisplay(bool force = false) {
  if (!gatewayDisplayReady) return;
  const uint32_t now = millis();
  if (!force && now - displayLastRefresh < 1000) return;
  displayLastRefresh = now;
  const uint8_t online = countOnlineNodes(now);
  if (now - gatewayDisplayPageStarted >= 5000) {
    gatewayDisplayPageStarted = now;
    gatewayDisplayPage = online > 1 ? (gatewayDisplayPage + 1) % (online + 1) : 0;
  }
  gatewayDisplay.clearDisplay();
  gatewayDisplay.setTextColor(SSD1306_WHITE);
  gatewayDisplay.setTextSize(1);
  gatewayDisplay.setCursor(0, 0);
  if (gatewayDisplayPage > online) gatewayDisplayPage = 0;
  if (online == 0 || online == 1 || gatewayDisplayPage == 0) {
    gatewayDisplay.println("GATEWAY STATUS");
    gatewayDisplay.printf("WiFi: %s\n", WiFi.status() == WL_CONNECTED ? "connected" : "disconnected");
    if (WiFi.status() == WL_CONNECTED) {
      String ssid = WiFi.SSID();
      if (ssid.length() > 20) ssid = ssid.substring(0, 19) + "~";
      gatewayDisplay.printf("SSID: %s\n", ssid.c_str());
    } else gatewayDisplay.println("SSID: --");
    gatewayDisplay.printf("Channel: %u  MQTT: %s\n", gatewayRadioChannel,
                          mqttClient.connected() ? "UP" : "DOWN");
    gatewayDisplay.printf("Nodes online: %u\n", online);
    gatewayDisplay.printf("RX:%lu Drop:%lu\n", static_cast<unsigned long>(gatewayRxCount),
                          static_cast<unsigned long>(queueDropped));
  } else {
    const int index = findDisplayNode(gatewayDisplayPage - 1, now);
    if (index >= 0) {
      const NodeDisplayInfo &node = gatewayNodes[index];
      const TelemetryPacket &packet = node.packet;
      gatewayDisplay.printf("NODE %u/%u %s\n", gatewayDisplayPage, online, packet.nodeId);
      gatewayDisplay.printf("RSSI: %d dBm  hop:%u\n", node.rssi, packet.hop);
      if (packet.flags & FLAG_LUX_VALID) gatewayDisplay.printf("Light: %.2f lx\n", packet.luxCenti / 100.0f);
      else gatewayDisplay.println("Light: --");
      if (packet.flags & FLAG_NTC_VALID) gatewayDisplay.printf("Battery: %.1f C\n", packet.batteryTempCentiC / 100.0f);
      else gatewayDisplay.println("Battery: --");
      if (packet.flags & FLAG_BATTERY_VALID) gatewayDisplay.printf("Voltage: %.2f V\n", packet.batteryMilliVolts / 1000.0f);
      else if (packet.flags & FLAG_WIND_VALID) gatewayDisplay.printf("Wind input: %.2f V\n", packet.windMilliVolts / 1000.0f);
      else gatewayDisplay.println("Voltage: --");
      gatewayDisplay.printf("Seq:%lu age:%lus\n", static_cast<unsigned long>(packet.sequence),
                            static_cast<unsigned long>((now - node.lastSeenMs) / 1000));
      gatewayDisplay.println("Dist: RSSI needs calibration");
    }
  }
  gatewayDisplay.display();
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
  memcpy(item.sourceMac, info->src_addr, 6);
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
  const esp_err_t result = esp_now_send(BROADCAST_MAC,
      reinterpret_cast<const uint8_t *>(&beacon), sizeof(beacon));
  if (result == ESP_OK) blinkStatusLed(1);
  return result == ESP_OK;
}

void reconnectMqtt() {
  if (mqttClient.connected() || mqttHost.isEmpty() || WiFi.status() != WL_CONNECTED) return;
  static uint32_t lastAttempt = 0;
  if (millis() - lastAttempt < 5000) return;
  lastAttempt = millis();
  String clientId = String(MQTT_CLIENT_ID);
  if (clientId.isEmpty()) clientId = String("ussc-gateway-") + WiFi.macAddress();
  clientId.replace(":", "");
  const String statusTopic = String(MQTT_TOPIC_PREFIX) + "/gateway/status";
  const bool ok = !mqttUser.isEmpty()
      ? mqttClient.connect(clientId.c_str(), mqttUser.c_str(), mqttPassword.c_str(), statusTopic.c_str(), 1, true, "offline")
      : mqttClient.connect(clientId.c_str(), statusTopic.c_str(), 1, true, "offline");
  if (ok) {
    mqttClient.publish(statusTopic.c_str(), "online", true);
    Serial.println("[gateway] MQTT connected");
  } else Serial.printf("[gateway] MQTT connect failed state=%d\n", mqttClient.state());
}

void publishPacket(const TelemetryPacket &packet, const uint8_t *sourceMac, int8_t rssi) {
  if (duplicatePacket(packet)) return;
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
  JsonDocument doc;
  doc["schema"] = 2;
  doc["node_id"] = packet.nodeId;
  doc["sequence"] = packet.sequence;
  doc["uptime_ms"] = packet.uptimeMs;
  doc["hop"] = packet.hop;
  doc["relayed"] = (packet.flags & FLAG_RELAYED) != 0;
  doc["rssi_dbm"] = rssi;
  doc["gateway_rx_epoch"] = static_cast<uint32_t>(time(nullptr));
  char mac[18];
  snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
           sourceMac[0], sourceMac[1], sourceMac[2], sourceMac[3], sourceMac[4], sourceMac[5]);
  doc["source_mac"] = mac;
  if (packet.flags & FLAG_LUX_VALID) doc["lux_lx"] = packet.luxCenti / 100.0f;
  if (packet.flags & FLAG_NTC_VALID) doc["battery_temp_c"] = packet.batteryTempCentiC / 100.0f;
  if (packet.flags & FLAG_BATTERY_VALID) doc["battery_v"] = packet.batteryMilliVolts / 1000.0f;
  if (packet.flags & FLAG_WIND_VALID) doc["wind_input_v"] = packet.windMilliVolts / 1000.0f;
  char payload[320];
  const size_t len = serializeJson(doc, payload, sizeof(payload));
  const String topic = String(MQTT_TOPIC_PREFIX) + "/" + packet.nodeId + "/state";
  if (mqttClient.connected()) {
    const bool ok = mqttClient.publish(topic.c_str(), reinterpret_cast<const uint8_t *>(payload), len, false);
    Serial.printf("[gateway] %s topic=%s payload=%s\n", ok ? "published" : "publish failed", topic.c_str(), payload);
  } else Serial.printf("[gateway] MQTT offline, dropped node=%s seq=%lu\n", packet.nodeId, static_cast<unsigned long>(packet.sequence));
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
    publishPacket(item.packet, item.sourceMac, item.rssi);
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
    missedSyncCount = 0;
    channelScanCandidate = runtimeMeshChannel;
  } else {
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
  nodeSynced = false;
  nodeWindowOpen = false;
  nodeSentThisWindow = false;
  nodeWakeStartedMs = millis();
  if (initEspNow(nodeRadioChannel)) {
    nodeRadioReady = true;
    esp_now_register_send_cb(onSampleSend);
    esp_now_register_recv_cb(onNodeReceive);
    if (nodeConsoleEnabled) Serial.printf("[sensor] listening for SYNC_BEACON on channel=%u\n", runtimeMeshChannel);
  } else if (nodeConsoleEnabled) Serial.println("[sensor] ESP-NOW initialization failed");
#endif

#if defined(DEVICE_ROLE_RELAY) && DEVICE_ROLE_RELAY && !(defined(DEVICE_ROLE_SENSOR) && DEVICE_ROLE_SENSOR)
  randomSeed(esp_random());
  if (initEspNow(ESPNOW_CHANNEL)) {
    esp_now_register_recv_cb(onRelayReceive);
    Serial.printf("[relay] listening channel=%u max_hops=%u\n", ESPNOW_CHANNEL, ESPNOW_TTL);
  } else Serial.println("[relay] ESP-NOW initialization failed");
#endif

#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
  loadMeshConfig();
  loadWifiConfig();
  loadMqttConfig();
  mqttClient.setServer(mqttHost.c_str(), mqttPort);
  restoreRtcTime();
#if FEATURE_OLED
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!gatewayDisplay.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
    Serial.println("[gateway] OLED not found; display disabled");
  } else {
    gatewayDisplayReady = true;
    Serial.printf("[gateway] OLED ready at 0x%02X\n", OLED_I2C_ADDRESS);
    updateGatewayDisplay(true);
  }
#endif
  WiFi.mode(WIFI_STA);
  if (wifiConfig.ssid.isEmpty()) {
    Serial.println("[wifi] no profile configured; use 'wifi set ...' then 'wifi save' and 'wifi apply'");
  } else if (beginWifi()) {
    const uint32_t started = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - started < 30000) {
      delay(250);
#if FEATURE_OLED
      updateGatewayDisplay();
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
    mqttClient.setBufferSize(384);
  } else {
    gatewayRadioChannel = ESPNOW_CHANNEL;
    Serial.printf("[gateway] Wi-Fi unavailable; ESP-NOW channel=%u; MQTT disabled until Wi-Fi joins\n", gatewayRadioChannel);
  }
  gatewayRadioChannel = WiFi.status() == WL_CONNECTED
      ? static_cast<uint8_t>(WiFi.channel()) : runtimeMeshChannel;
  if (initEspNow(gatewayRadioChannel)) {
    esp_now_register_recv_cb(onGatewayReceive);
    gatewayNextWindowAt = millis() + runtimeSensorIntervalMs;
    gatewayNextSensorSlotAt = gatewayNextWindowAt;
    gatewayWindowOpenedAt = 0;
    gatewayWindowActive = false;
    gatewayNextBeaconAt = millis();
    Serial.printf("[gateway] ESP-NOW listening channel=%u sensor interval=%lus beacon=%lums MQTT=%s:%u\n",
                  gatewayRadioChannel, static_cast<unsigned long>(runtimeSensorIntervalMs / 1000UL),
                  static_cast<unsigned long>(runtimeBeaconIntervalMs), mqttHost.c_str(), mqttPort);
  } else Serial.println("[gateway] ESP-NOW initialization failed");
  updateGatewayDisplay(true);
#endif
}

void loop() {
  serviceStatusLed();
#if defined(DEVICE_ROLE_SENSOR) && DEVICE_ROLE_SENSOR
  serviceNodeMesh();
#endif
#if defined(DEVICE_ROLE_RELAY) && DEVICE_ROLE_RELAY && ESPNOW_ENABLE_RELAY
#if defined(DEVICE_ROLE_SENSOR) && DEVICE_ROLE_SENSOR
  if (nodeWindowOpen) relayQueuedPackets();
#else
  relayQueuedPackets();
#endif
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
  if (wifiScanRequested && !gatewayWindowActive &&
      scanWindowDelay > static_cast<int32_t>(runtimeActiveWindowMs + 30000UL)) {
    wifiScanRequested = false;
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
  if (wifiScanRequested) {
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
        if (initEspNow(gatewayRadioChannel)) {
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
  if (wifiApplyRequested) {
    wifiApplyRequested = false;
    mqttClient.disconnect(); WiFi.disconnect(false, false); delay(100);
    wifiConfig.enterprise = wifiConfig.ssid.equalsIgnoreCase("eduroam");
    if (beginWifi()) {
      uint32_t started = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - started < 30000) delay(250);
      if (WiFi.status() == WL_CONNECTED) {
        gatewayRadioChannel = static_cast<uint8_t>(WiFi.channel()); gatewayWifiWasConnected = true;
        esp_now_deinit();
        if (initEspNow(gatewayRadioChannel)) {
          esp_now_register_recv_cb(onGatewayReceive);
          gatewayNextWindowAt = millis() + runtimeSensorIntervalMs;
          gatewayWindowActive = false;
          gatewayWindowOpenedAt = 0;
          gatewayNextBeaconAt = millis();
        }
        configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2);
        markNtpSyncStarted();
        Serial.printf("[wifi] connected IP=%s channel=%u; NTP and ESP-NOW restarted\n", WiFi.localIP().toString().c_str(), gatewayRadioChannel);
      } else Serial.println("[wifi] failed; check credentials and 2.4 GHz coverage");
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
        if (initEspNow(gatewayRadioChannel)) {
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
    if (!wifiConfig.ssid.isEmpty() && millis() - lastWifiReconnectAttempt >= 15000) {
      lastWifiReconnectAttempt = millis();
      WiFi.disconnect(false, false);
      beginWifi();
    }
  }
  pollTimeSync();
  mqttClient.loop();
  drainGatewayQueue();
#if FEATURE_OLED
  updateGatewayDisplay();
#endif
  static uint32_t lastStats = 0;
  if (millis() - lastStats >= 30000) {
    lastStats = millis();
    Serial.printf("[gateway] invalid=%lu queue_dropped=%lu mqtt=%s\n",
                  static_cast<unsigned long>(invalidPackets), static_cast<unsigned long>(queueDropped),
                  mqttClient.connected() ? "up" : "down");
  }
  delay(5);
#endif
}

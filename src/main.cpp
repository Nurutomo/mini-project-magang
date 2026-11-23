#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include <esp_mac.h>
#include <math.h>
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

void handleWifiCommand(String command) {
  command.trim();
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

bool initEspNow(uint8_t channel) {
  WiFi.mode(WIFI_STA);
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
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
volatile bool sampleSendDone = false;
volatile esp_now_send_status_t sampleSendStatus = ESP_NOW_SEND_FAIL;

void onSampleSend(const esp_now_send_info_t *, esp_now_send_status_t status) {
  sampleSendStatus = status;
  sampleSendDone = true;
}

void publishSample() {
  const TelemetryPacket packet = collectTelemetry();
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

  sampleSendDone = false;
  const esp_err_t result = esp_now_send(BROADCAST_MAC,
                                        reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
  if (result != ESP_OK) {
    Serial.printf("[sensor] ESP-NOW enqueue error=%d\n", result);
    return;
  }
  const uint32_t started = millis();
  while (!sampleSendDone && millis() - started < 1200) delay(5);
  Serial.printf("[sensor] ESP-NOW callback=%s\n",
                sampleSendDone && sampleSendStatus == ESP_NOW_SEND_SUCCESS ? "success" : "failure/timeout");
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
struct QueueItem { TelemetryPacket packet; uint8_t sourceMac[6]; };
QueueItem queueItems[GATEWAY_QUEUE_SIZE];
volatile uint8_t queueHead = 0, queueTail = 0;
portMUX_TYPE queueMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t queueDropped = 0, invalidPackets = 0, gatewayRxCount = 0;
char gatewayLastNode[NODE_ID_MAX_LEN] = "-";
float gatewayLastLux = NAN;
bool gatewayLastLuxValid = false;
struct SeenPacket { char nodeId[NODE_ID_MAX_LEN]; uint32_t sequence; };
SeenPacket seenPackets[GATEWAY_DEDUP_SIZE]{};
uint16_t seenCursor = 0;

void updateGatewayDisplay(bool force = false) {
  if (!gatewayDisplayReady) return;
  if (!force && millis() - displayLastRefresh < 1000) return;
  displayLastRefresh = millis();
  gatewayDisplay.clearDisplay();
  gatewayDisplay.setTextColor(SSD1306_WHITE);
  gatewayDisplay.setTextSize(1);
  gatewayDisplay.setCursor(0, 0);
  gatewayDisplay.println("GATEWAY / DEBUG");
  gatewayDisplay.printf("WiFi %s ch:%u\n", WiFi.status() == WL_CONNECTED ? "UP" : "DOWN",
                        gatewayRadioChannel);
  gatewayDisplay.printf("MQTT %s\n", mqttClient.connected() ? "UP" : "WAIT");
  gatewayDisplay.printf("RX %lu  QDROP %lu\n", static_cast<unsigned long>(gatewayRxCount),
                        static_cast<unsigned long>(queueDropped));
  gatewayDisplay.printf("INVALID %lu\n", static_cast<unsigned long>(invalidPackets));
  gatewayDisplay.print("Last ");
  gatewayDisplay.print(gatewayLastNode);
  if (gatewayLastLuxValid) {
    gatewayDisplay.print(' ');
    gatewayDisplay.print(gatewayLastLux, 0);
    gatewayDisplay.print("lx");
  }
  gatewayDisplay.display();
}

void onGatewayReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!info || !info->src_addr || !isValidTelemetryPacket(data, static_cast<size_t>(len))) {
    ++invalidPackets;
    return;
  }
  ++gatewayRxCount;
  QueueItem item{};
  memcpy(&item.packet, data, sizeof(TelemetryPacket));
  memcpy(item.sourceMac, info->src_addr, 6);
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

void publishPacket(const TelemetryPacket &packet, const uint8_t *sourceMac) {
  if (duplicatePacket(packet)) return;
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
  doc["rssi_dbm"] = WiFi.RSSI();
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
    publishPacket(item.packet, item.sourceMac);
  }
}
#endif
} // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
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
  analogSetPinAttenuation(NTC_ADC_PIN, ADC_11db);
  analogSetPinAttenuation(WIND_ADC_PIN, ADC_11db);
#if FEATURE_BATTERY_SENSE
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
#endif
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Serial.printf("[sensor] id=%s BH1750=0x%02X channel=%u interval=%us\n",
                deviceId().c_str(), BH1750_I2C_ADDRESS, ESPNOW_CHANNEL, SAMPLE_INTERVAL_S);
  Serial.printf("[sensor] ADC ntc=GPIO%u", NTC_ADC_PIN);
#if FEATURE_WIND_ADC
  Serial.printf(" wind=GPIO%u", WIND_ADC_PIN);
#endif
#if FEATURE_BATTERY_SENSE
  Serial.printf(" battery=GPIO%u", BATTERY_ADC_PIN);
#endif
  Serial.printf(" samples=%u\n", ADC_SAMPLES);
  if (!initEspNow(ESPNOW_CHANNEL)) Serial.println("[sensor] ESP-NOW initialization failed");
  else {
    esp_now_register_send_cb(onSampleSend);
#if defined(DEVICE_ROLE_RELAY) && DEVICE_ROLE_RELAY && ESPNOW_ENABLE_RELAY
    esp_now_register_recv_cb(onRelayReceive);
    Serial.println("[sensor] relay receive enabled");
#endif
    publishSample();
  }
#if NODE_DEEP_SLEEP
  esp_now_deinit();
  WiFi.mode(WIFI_OFF);
  esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(SAMPLE_INTERVAL_S) * 1000000ULL);
  esp_deep_sleep_start();
#endif
#endif

#if defined(DEVICE_ROLE_RELAY) && DEVICE_ROLE_RELAY && !(defined(DEVICE_ROLE_SENSOR) && DEVICE_ROLE_SENSOR)
  randomSeed(esp_random());
  if (initEspNow(ESPNOW_CHANNEL)) {
    esp_now_register_recv_cb(onRelayReceive);
    Serial.printf("[relay] listening channel=%u max_hops=%u\n", ESPNOW_CHANNEL, ESPNOW_TTL);
  } else Serial.println("[relay] ESP-NOW initialization failed");
#endif

#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
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
  if (initEspNow(gatewayRadioChannel)) {
    esp_now_register_recv_cb(onGatewayReceive);
    Serial.printf("[gateway] ESP-NOW listening channel=%u MQTT=%s:%u\n", gatewayRadioChannel, mqttHost.c_str(), mqttPort);
  } else Serial.println("[gateway] ESP-NOW initialization failed");
  updateGatewayDisplay(true);
#endif
}

void loop() {
#if defined(DEVICE_ROLE_SENSOR) && DEVICE_ROLE_SENSOR
#if !NODE_DEEP_SLEEP
  static uint32_t nextSampleAt = 0;
  if (static_cast<int32_t>(millis() - nextSampleAt) >= 0) {
    publishSample();
    nextSampleAt = millis() + static_cast<uint32_t>(SAMPLE_INTERVAL_S) * 1000UL;
  }
#endif
#endif
#if defined(DEVICE_ROLE_RELAY) && DEVICE_ROLE_RELAY && ESPNOW_ENABLE_RELAY
  relayQueuedPackets();
#endif
#if (defined(DEVICE_ROLE_SENSOR) && DEVICE_ROLE_SENSOR) || (defined(DEVICE_ROLE_RELAY) && DEVICE_ROLE_RELAY)
  delay(5);
#endif
#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
  pollSerialCommands();
  if (wifiScanRequested) {
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
        if (initEspNow(gatewayRadioChannel)) esp_now_register_recv_cb(onGatewayReceive);
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
        Serial.printf("[gateway] AP channel changed %u -> %u; restart required to rejoin ESP-NOW consistently\n", gatewayRadioChannel, newChannel);
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

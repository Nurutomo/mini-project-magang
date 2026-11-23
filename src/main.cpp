#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include <math.h>
#include <string.h>
#include <ArduinoJson.h>

#include "config.h"
#include "telemetry.h"

#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <time.h>
#endif

namespace {
constexpr uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
constexpr uint32_t SENSOR_SETTLE_MS = 4;

String deviceId() {
  if (strlen(NODE_ID) > 0) return String(NODE_ID);
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char id[16];
  snprintf(id, sizeof(id), "node-%02X%02X%02X", mac[3], mac[4], mac[5]);
  return String(id);
}

#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
bool beginWifi() {
#if defined(CONFIG_ESP_WIFI_ENTERPRISE_SUPPORT) && CONFIG_ESP_WIFI_ENTERPRISE_SUPPORT
  // Eduroam is WPA2-Enterprise (typically PEAP/MSCHAPv2), not WPA2-PSK.
  if (strcmp(WIFI_SSID, "eduroam") == 0) {
    if (strlen(EDUROAM_USERNAME) == 0 || strlen(EDUROAM_PASSWORD) == 0) {
      Serial.println("[wifi] eduroam selected but EDUROAM_USERNAME/EDUROAM_PASSWORD is empty");
      return false;
    }
    Serial.printf("[wifi] eduroam WPA2-Enterprise user=%s identity=%s\n", EDUROAM_USERNAME, EDUROAM_IDENTITY);
    if (strlen(EDUROAM_CA_CERT) > 0) {
      WiFi.begin(WIFI_SSID, WPA2_AUTH_PEAP, EDUROAM_IDENTITY, EDUROAM_USERNAME, EDUROAM_PASSWORD,
                 EDUROAM_CA_CERT);
    } else {
      WiFi.begin(WIFI_SSID, WPA2_AUTH_PEAP, EDUROAM_IDENTITY, EDUROAM_USERNAME, EDUROAM_PASSWORD);
    }
    return true;
  }
#endif
  Serial.printf("[wifi] WPA2-PSK SSID=%s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  return true;
}
#endif

uint16_t readAdcMillivolts(uint8_t pin) {
  analogRead(pin); // discard conversion after channel switch
  delay(SENSOR_SETTLE_MS);
  uint32_t total = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES; ++i) {
    total += analogReadMilliVolts(pin);
    delay(2);
  }
  return static_cast<uint16_t>(total / ADC_SAMPLES);
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
bool gatewayWifiWasConnected = false;
uint8_t gatewayRadioChannel = ESPNOW_CHANNEL;
uint32_t lastWifiReconnectAttempt = 0;
struct QueueItem { TelemetryPacket packet; uint8_t sourceMac[6]; };
QueueItem queueItems[GATEWAY_QUEUE_SIZE];
volatile uint8_t queueHead = 0, queueTail = 0;
portMUX_TYPE queueMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t queueDropped = 0, invalidPackets = 0;
struct SeenPacket { char nodeId[NODE_ID_MAX_LEN]; uint32_t sequence; };
SeenPacket seenPackets[GATEWAY_DEDUP_SIZE]{};
uint16_t seenCursor = 0;

void onGatewayReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!info || !info->src_addr || !isValidTelemetryPacket(data, static_cast<size_t>(len))) {
    ++invalidPackets;
    return;
  }
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
  if (mqttClient.connected() || strlen(MQTT_HOST) == 0 || WiFi.status() != WL_CONNECTED) return;
  static uint32_t lastAttempt = 0;
  if (millis() - lastAttempt < 5000) return;
  lastAttempt = millis();
  String clientId = String(MQTT_CLIENT_ID);
  if (clientId.isEmpty()) clientId = String("ussc-gateway-") + WiFi.macAddress();
  clientId.replace(":", "");
  const String statusTopic = String(MQTT_TOPIC_PREFIX) + "/gateway/status";
  const bool ok = strlen(MQTT_USER) > 0
      ? mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD, statusTopic.c_str(), 1, true, "offline")
      : mqttClient.connect(clientId.c_str(), statusTopic.c_str(), 1, true, "offline");
  if (ok) {
    mqttClient.publish(statusTopic.c_str(), "online", true);
    Serial.println("[gateway] MQTT connected");
  } else Serial.printf("[gateway] MQTT connect failed state=%d\n", mqttClient.state());
}

void publishPacket(const TelemetryPacket &packet, const uint8_t *sourceMac) {
  if (duplicatePacket(packet)) return;
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
#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
  Serial.println("[boot] role=gateway");
#elif defined(DEVICE_ROLE_RELAY) && DEVICE_ROLE_RELAY
  Serial.println("[boot] role=relay-node");
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
  WiFi.mode(WIFI_STA);
  if (strlen(WIFI_SSID) > 0) {
    if (beginWifi()) {
      const uint32_t started = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - started < 30000) delay(250);
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    gatewayWifiWasConnected = true;
    gatewayRadioChannel = static_cast<uint8_t>(WiFi.channel());
    Serial.printf("[gateway] Wi-Fi IP=%s channel=%u (configure all nodes to match)\n",
                  WiFi.localIP().toString().c_str(), gatewayRadioChannel);
    configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2);
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setBufferSize(384);
  } else {
    gatewayRadioChannel = ESPNOW_CHANNEL;
    Serial.printf("[gateway] Wi-Fi unavailable; ESP-NOW channel=%u; MQTT disabled until Wi-Fi joins\n", gatewayRadioChannel);
  }
  if (initEspNow(gatewayRadioChannel)) {
    esp_now_register_recv_cb(onGatewayReceive);
    Serial.printf("[gateway] ESP-NOW listening channel=%u MQTT=%s:%u\n", gatewayRadioChannel, MQTT_HOST, MQTT_PORT);
  } else Serial.println("[gateway] ESP-NOW initialization failed");
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
  if (WiFi.status() == WL_CONNECTED) {
    if (!gatewayWifiWasConnected) {
      gatewayWifiWasConnected = true;
      const uint8_t newChannel = static_cast<uint8_t>(WiFi.channel());
      if (newChannel != gatewayRadioChannel) {
        Serial.printf("[gateway] AP channel changed %u -> %u; restart required to rejoin ESP-NOW consistently\n", gatewayRadioChannel, newChannel);
      }
      configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2);
    }
    reconnectMqtt();
  } else {
    gatewayWifiWasConnected = false;
    if (strlen(WIFI_SSID) > 0 && millis() - lastWifiReconnectAttempt >= 15000) {
      lastWifiReconnectAttempt = millis();
      if (strcmp(WIFI_SSID, "eduroam") == 0) {
        WiFi.disconnect(false, false);
        beginWifi();
      } else {
        WiFi.reconnect();
      }
    }
  }
  mqttClient.loop();
  drainGatewayQueue();
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

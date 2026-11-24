#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <string.h>
#include "config.h"
#include "telemetry.h"
#include "mqtt.h"

#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
namespace {
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
Preferences mqttPreferences;
String mqttHost = MQTT_HOST;
uint16_t mqttPort = MQTT_PORT;
String mqttTopicPrefix = MQTT_TOPIC_PREFIX;
String mqttUser = MQTT_USER;
String mqttPassword = MQTT_PASSWORD;
}

void loadMqttConfig() {
  mqttPreferences.begin(MQTT_CONFIG_NAMESPACE, true);
  mqttHost = mqttPreferences.getString(MQTT_CONFIG_HOST_KEY, MQTT_HOST);
  mqttPort = mqttPreferences.getUShort(MQTT_CONFIG_PORT_KEY, MQTT_PORT);
  mqttTopicPrefix = mqttPreferences.getString(MQTT_CONFIG_TOPIC_KEY, MQTT_TOPIC_PREFIX);
  mqttUser = mqttPreferences.getString(MQTT_CONFIG_USER_KEY, MQTT_USER);
  mqttPassword = mqttPreferences.getString(MQTT_CONFIG_PASSWORD_KEY, MQTT_PASSWORD);
  mqttPreferences.end();
}

String mqttHostName() { return mqttHost; }
uint16_t mqttBrokerPort() { return mqttPort; }
bool mqttConnected() { return mqttClient.connected(); }
void initMqtt() {
  mqttClient.setServer(mqttHost.c_str(), mqttPort);
  mqttClient.setBufferSize(384);
}
void disconnectMqtt() { mqttClient.disconnect(); }
void serviceMqtt() { mqttClient.loop(); }

void reconnectMqtt() {
  if (mqttClient.connected() || mqttHost.isEmpty() || WiFi.status() != WL_CONNECTED) return;
  static uint32_t lastAttempt = 0;
  if (millis() - lastAttempt < 5000) return;
  lastAttempt = millis();
  String clientId = String(MQTT_CLIENT_ID);
  if (clientId.isEmpty()) clientId = String("ussc-gateway-") + WiFi.macAddress();
  clientId.replace(":", "");
  const String statusTopic = mqttTopicPrefix + "/gateway/status";
  const bool ok = !mqttUser.isEmpty()
      ? mqttClient.connect(clientId.c_str(), mqttUser.c_str(), mqttPassword.c_str(), statusTopic.c_str(), 1, true, "offline")
      : mqttClient.connect(clientId.c_str(), statusTopic.c_str(), 1, true, "offline");
  if (ok) {
    mqttClient.publish(statusTopic.c_str(), "online", true);
    Serial.println("[gateway] MQTT connected");
  } else Serial.printf("[gateway] MQTT connect failed state=%d\n", mqttClient.state());
}

void publishPacket(const TelemetryPacket &packet, int8_t rssi) {
  JsonDocument doc;
  if (packet.flags & FLAG_LUX_VALID) doc["lx"] = packet.luxCenti / 100.0f;
  if (packet.flags & FLAG_NTC_VALID) doc["tc"] = packet.batteryTempCentiC / 100.0f;
  if (packet.flags & FLAG_BATTERY_VALID) doc["bv"] = packet.batteryMilliVolts / 1000.0f;
  if (packet.flags & FLAG_WIND_VALID) doc["wv"] = packet.windMilliVolts / 1000.0f;
  doc["s"] = packet.sequence;
  doc["h"] = packet.hop;
  doc["r"] = rssi;
  char payload[192];
  const size_t len = serializeJson(doc, payload, sizeof(payload));
  const String topic = mqttTopicPrefix + "/" + String(packet.nodeId).substring(5) + "/state";
  if (mqttClient.connected()) {
    const bool ok = mqttClient.publish(topic.c_str(), reinterpret_cast<const uint8_t *>(payload), len, false);
    Serial.printf("[gateway] %s topic=%s payload=%s\n", ok ? "published" : "publish failed", topic.c_str(), payload);
  } else Serial.printf("[gateway] MQTT offline, dropped node=%s seq=%lu\n", packet.nodeId, static_cast<unsigned long>(packet.sequence));
}

void handleMqttCommand(const String &command) {
  if (command == "mqtt show") {
    Serial.printf("[mqtt] host=%s port=%u topic=%s username=%s password=[hidden]\n",
                  mqttHost.c_str(), mqttPort, mqttTopicPrefix.c_str(),
                  mqttUser.isEmpty() ? "(none)" : "(set)");
    return;
  }
  if (command == "mqtt save") {
    mqttPreferences.begin(MQTT_CONFIG_NAMESPACE, false);
    mqttPreferences.putString(MQTT_CONFIG_HOST_KEY, mqttHost);
    mqttPreferences.putUShort(MQTT_CONFIG_PORT_KEY, mqttPort);
    mqttPreferences.putString(MQTT_CONFIG_TOPIC_KEY, mqttTopicPrefix);
    mqttPreferences.putString(MQTT_CONFIG_USER_KEY, mqttUser);
    mqttPreferences.putString(MQTT_CONFIG_PASSWORD_KEY, mqttPassword);
    mqttPreferences.end();
    Serial.println("[mqtt] broker settings saved in NVS");
    return;
  }
  if (command == "mqtt clear") {
    mqttPreferences.begin(MQTT_CONFIG_NAMESPACE, false);
    mqttPreferences.clear();
    mqttPreferences.end();
    Serial.println("[mqtt] saved settings removed; reboot to use build defaults");
    return;
  }
  if (command.startsWith("mqtt topic ")) {
    String prefix = command.substring(11);
    prefix.trim();
    bool validPrefix = !prefix.isEmpty() && prefix.length() <= 200 &&
        prefix[0] != '/' && prefix[prefix.length() - 1] != '/';
    bool segmentHasCharacter = false;
    for (size_t i = 0; i < prefix.length(); ++i) {
      const char c = prefix[i];
      const bool allowed = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
          (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '/';
      if (!allowed) validPrefix = false;
      if (c == '/') {
        if (!segmentHasCharacter) validPrefix = false;
        segmentHasCharacter = false;
      } else segmentHasCharacter = true;
    }
    if (!validPrefix || !segmentHasCharacter) {
      Serial.println("[mqtt] invalid topic prefix; use 1..200 chars of letters, digits, '.', '_' or '-' separated by '/', with no empty segments");
      return;
    }
    mqttTopicPrefix = prefix;
    Serial.println("[mqtt] topic prefix updated; use 'mqtt save' to persist");
    return;
  }
  if (command.startsWith("mqtt set ")) {
    String fields = command.substring(9);
    fields.trim();
    String values[4];
    uint8_t count = 0;
    while (fields.length() && count < 4) {
      const int split = fields.indexOf(' ');
      values[count++] = split < 0 ? fields : fields.substring(0, split);
      if (split < 0) fields = "";
      else { fields = fields.substring(split + 1); fields.trim(); }
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
      Serial.println("[mqtt] usage: mqtt set <host> <port> [username password] (DNS/IPv4 host, tokens without spaces)");
      return;
    }
    char *end = nullptr;
    const long port = strtol(values[1].c_str(), &end, 10);
    if (!end || *end || port < 1 || port > 65535) {
      Serial.println("[mqtt] port must be 1..65535");
      return;
    }
    mqttHost = values[0];
    mqttPort = static_cast<uint16_t>(port);
    mqttUser = count >= 3 ? values[2] : "";
    mqttPassword = count >= 4 ? values[3] : "";
    mqttClient.disconnect();
    mqttClient.setServer(mqttHost.c_str(), mqttPort);
    Serial.println("[mqtt] settings updated (password hidden); use 'mqtt save' to persist");
  }
}
#endif

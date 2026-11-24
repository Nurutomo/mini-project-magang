#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "display.h"
#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY && FEATURE_OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

namespace {
Adafruit_SSD1306 gatewayDisplay(128, 64, &Wire, -1);
bool gatewayDisplayReady = false;
uint32_t displayLastRefresh = 0;
uint8_t gatewayDisplayPage = 0;
uint32_t gatewayDisplayPageStarted = 0;

uint8_t countOnlineNodes(const GatewayDisplaySnapshot &snapshot, uint32_t now) {
  uint8_t count = 0;
  for (size_t i = 0; i < snapshot.nodeCount; ++i) {
    const auto &node = snapshot.nodes[i];
    if (node.seen && now - node.lastSeenMs <= snapshot.sensorIntervalMs * 3UL) ++count;
  }
  return count;
}

int findDisplayNode(const GatewayDisplaySnapshot &snapshot, uint8_t ordinal, uint32_t now) {
  for (size_t i = 0; i < snapshot.nodeCount; ++i) {
    const auto &node = snapshot.nodes[i];
    if (node.seen && now - node.lastSeenMs <= snapshot.sensorIntervalMs * 3UL) {
      if (ordinal-- == 0) return static_cast<int>(i);
    }
  }
  return -1;
}
}

void initGatewayDisplay() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!gatewayDisplay.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
    Serial.println("[gateway] OLED not found; display disabled");
    return;
  }
  gatewayDisplayReady = true;
  Serial.printf("[gateway] OLED ready at 0x%02X\n", OLED_I2C_ADDRESS);
}

void updateGatewayDisplay(const GatewayDisplaySnapshot &snapshot, bool force) {
  if (!gatewayDisplayReady) return;
  const uint32_t now = millis();
  if (!force && now - displayLastRefresh < 1000) return;
  displayLastRefresh = now;
  const uint8_t online = countOnlineNodes(snapshot, now);
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
    gatewayDisplay.printf("WiFi: %s\n", snapshot.wifiConnected ? "connected" : "disconnected");
    if (snapshot.wifiConnected) {
      String ssid = WiFi.SSID();
      if (ssid.length() > 20) ssid = ssid.substring(0, 19) + "~";
      gatewayDisplay.printf("SSID: %s\n", ssid.c_str());
    } else gatewayDisplay.println("SSID: --");
    gatewayDisplay.printf("Channel: %u  MQTT: %s\n", snapshot.radioChannel,
                          snapshot.mqttConnected ? "UP" : "DOWN");
    gatewayDisplay.printf("Nodes online: %u\n", online);
    gatewayDisplay.printf("RX:%lu Drop:%lu\n", static_cast<unsigned long>(snapshot.receivedCount),
                          static_cast<unsigned long>(snapshot.droppedCount));
  } else {
    const int index = findDisplayNode(snapshot, gatewayDisplayPage - 1, now);
    if (index >= 0) {
      const GatewayDisplayNode &node = snapshot.nodes[index];
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
#else
void initGatewayDisplay() {}
void updateGatewayDisplay(const GatewayDisplaySnapshot &, bool) {}
#endif

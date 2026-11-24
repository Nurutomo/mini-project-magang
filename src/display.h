#pragma once
#include <Arduino.h>
#include "telemetry.h"

struct GatewayDisplayNode {
  TelemetryPacket packet{};
  int8_t rssi = -127;
  uint32_t lastSeenMs = 0;
  bool seen = false;
};

struct GatewayDisplaySnapshot {
  const GatewayDisplayNode *nodes;
  size_t nodeCount;
  uint32_t sensorIntervalMs;
  uint8_t radioChannel;
  uint32_t receivedCount;
  uint32_t droppedCount;
  bool wifiConnected;
  bool mqttConnected;
};

void initGatewayDisplay();
void updateGatewayDisplay(const GatewayDisplaySnapshot &snapshot, bool force = false);

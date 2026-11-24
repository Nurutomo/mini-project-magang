#pragma once
#include <Arduino.h>
#include "telemetry.h"

#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
void loadMqttConfig();
String mqttHostName();
uint16_t mqttBrokerPort();
void initMqtt();
void disconnectMqtt();
bool mqttConnected();
void reconnectMqtt();
void serviceMqtt();
void publishPacket(const TelemetryPacket &packet, int8_t rssi);
void handleMqttCommand(const String &command);
#endif

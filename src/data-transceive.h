#pragma once
#include <Arduino.h>
#include "telemetry.h"

bool initMeshRadio(uint8_t channel);
bool addMeshBroadcastPeer(uint8_t channel);

#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
struct SyncBeaconState {
  uint8_t channel;
  bool windowOpen;
  uint8_t nextChannel;
  uint32_t channelSwitchAt;
  uint32_t nextWindowAt;
  uint32_t sensorIntervalMs;
  uint32_t beaconIntervalMs;
  uint32_t activeWindowMs;
  uint32_t windowOpenedAt;
};
bool sendSyncBeacon(const SyncBeaconState &state);
#endif

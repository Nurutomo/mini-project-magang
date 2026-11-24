#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>
#include "config.h"
#include "data-transceive.h"

namespace {
constexpr uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void configureMaxTxPower() {
  constexpr int8_t maxTxPowerQuarterDbm = 78;
  const esp_err_t result = esp_wifi_set_max_tx_power(maxTxPowerQuarterDbm);
  if (result != ESP_OK)
    Serial.printf("[wifi] max TX power setup failed (%d)\n", static_cast<int>(result));
}
}

bool initMeshRadio(uint8_t channel) {
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
  return addMeshBroadcastPeer(channel);
}

bool addMeshBroadcastPeer(uint8_t channel) {
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, BROADCAST_MAC, sizeof(BROADCAST_MAC));
  peer.channel = channel;
  peer.encrypt = false;
  if (esp_now_is_peer_exist(BROADCAST_MAC)) return true;
  return esp_now_add_peer(&peer) == ESP_OK;
}

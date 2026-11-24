#pragma once
#include <Arduino.h>

#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
struct WifiProfile {
  String ssid;
  String password;
  String username;
  String identity;
  String caCert;
  bool enterprise = false;
};

void loadWifiConfig();
bool beginWifi();
String wifiSsid();
bool wifiIsEnterprise();
void normalizeWifiProfile();
void setWifiProfile(const WifiProfile &profile);
bool wifiProfileIsEmpty();
void saveWifiProfile();
void clearSavedWifiProfile();
void setWifiApplyRequested();
bool takeWifiApplyRequest();
void setWifiScanRequested();
bool wifiScanRequestPending();
void clearWifiScanRequest();
#endif

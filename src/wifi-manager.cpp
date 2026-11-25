#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <esp_wifi.h>
#include "config.h"
#include "wifi-manager.h"

#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
namespace {
WifiProfile profile;
Preferences preferences;
bool applyRequested = false;
bool scanRequested = false;

void configureMaxTxPower() {
  constexpr int8_t maxTxPowerQuarterDbm = 78;
  const esp_err_t result = esp_wifi_set_max_tx_power(maxTxPowerQuarterDbm);
  if (result != ESP_OK)
    Serial.printf("[wifi] max TX power setup failed (%d)\n", static_cast<int>(result));
}

const char *wifiReasonHint(wifi_err_reason_t reason) {
  switch (reason) {
    case WIFI_REASON_NO_AP_FOUND:
    case WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY:
    case WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD:
    case WIFI_REASON_NO_AP_FOUND_IN_RSSI_THRESHOLD:
      return "SSID not found; check the name and 2.4 GHz coverage";
    case WIFI_REASON_AUTH_FAIL:
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
      return "authentication failed; check password/identity";
    case WIFI_REASON_ASSOC_FAIL:
      return "AP rejected association; move closer or use another AP";
    case WIFI_REASON_BEACON_TIMEOUT:
      return "lost the AP beacon; weak signal or AP restart";
    case WIFI_REASON_ASSOC_LEAVE:
    case WIFI_REASON_AUTH_LEAVE:
      return "disconnected on purpose";
    default:
      return "connection attempt failed";
  }
}

void logWifiStatus() {
  static bool eventHandlerRegistered = false;
  if (eventHandlerRegistered) return;
  eventHandlerRegistered = true;
  WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t info) {
    const auto reason = static_cast<wifi_err_reason_t>(info.wifi_sta_disconnected.reason);
    Serial.printf("[wifi] disconnected reason=%u: %s\n",
                  static_cast<unsigned>(reason), wifiReasonHint(reason));
  }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
}
}

String wifiSsid() { return profile.ssid; }
bool wifiIsEnterprise() { return profile.enterprise; }
WifiProfile currentWifiProfile() { return profile; }
void normalizeWifiProfile() { profile.enterprise = profile.ssid.equalsIgnoreCase("eduroam"); }
void setWifiProfile(const WifiProfile &updatedProfile) { profile = updatedProfile; }
bool wifiProfileIsEmpty() { return profile.ssid.isEmpty(); }
void saveWifiProfile() {
  if (profile.enterprise && profile.identity.isEmpty()) profile.identity = profile.username;
  preferences.begin(WIFI_CONFIG_NAMESPACE, false);
  preferences.putString(WIFI_CONFIG_SSID_KEY, profile.ssid);
  preferences.putString(WIFI_CONFIG_PASSWORD_KEY, profile.password);
  preferences.putString(WIFI_CONFIG_USERNAME_KEY, profile.username);
  preferences.putString(WIFI_CONFIG_IDENTITY_KEY, profile.identity);
  preferences.putString(WIFI_CONFIG_CA_CERT_KEY, profile.caCert);
  preferences.end();
}
void clearSavedWifiProfile() {
  preferences.begin(WIFI_CONFIG_NAMESPACE, false);
  preferences.clear();
  preferences.end();
}
void setWifiApplyRequested() { applyRequested = true; }
bool takeWifiApplyRequest() {
  const bool requested = applyRequested;
  applyRequested = false;
  return requested;
}
void setWifiScanRequested() { scanRequested = true; }
bool wifiScanRequestPending() { return scanRequested; }
void clearWifiScanRequest() { scanRequested = false; }

void loadWifiConfig() {
  profile.ssid = WIFI_SSID;
  profile.enterprise = profile.ssid.equalsIgnoreCase("eduroam");
  profile.password = profile.enterprise ? EDUROAM_PASSWORD : WIFI_PASSWORD;
  profile.username = EDUROAM_USERNAME;
  profile.identity = EDUROAM_IDENTITY;
  profile.caCert = EDUROAM_CA_CERT;
  preferences.begin(WIFI_CONFIG_NAMESPACE, true);
  if (preferences.isKey(WIFI_CONFIG_SSID_KEY)) {
    profile.ssid = preferences.getString(WIFI_CONFIG_SSID_KEY);
    profile.password = preferences.getString(WIFI_CONFIG_PASSWORD_KEY);
    profile.username = preferences.getString(WIFI_CONFIG_USERNAME_KEY);
    profile.identity = preferences.getString(WIFI_CONFIG_IDENTITY_KEY);
    profile.caCert = preferences.getString(WIFI_CONFIG_CA_CERT_KEY);
    profile.enterprise = profile.ssid.equalsIgnoreCase("eduroam");
  }
  preferences.end();
  if (profile.enterprise && profile.identity.isEmpty()) profile.identity = profile.username;
  if (!profile.ssid.isEmpty() && profile.password.isEmpty() && !profile.enterprise)
    Serial.println("[wifi] warning: saved SSID has no password; re-enter it with 'wifi set <ssid> <password>' then 'wifi save'");
}

bool beginWifi() {
  configureMaxTxPower();
  logWifiStatus();
#if defined(CONFIG_ESP_WIFI_ENTERPRISE_SUPPORT) && CONFIG_ESP_WIFI_ENTERPRISE_SUPPORT
  // Eduroam is WPA2-Enterprise (typically PEAP/MSCHAPv2), not WPA2-PSK.
  if (profile.enterprise) {
    if (profile.username.isEmpty() || profile.password.isEmpty()) {
      Serial.println("[wifi] eduroam username/password missing");
      return false;
    }
    Serial.printf("[wifi] connecting WPA2-Enterprise SSID=%s identity=%s password=[set] ca=%s\n",
                  profile.ssid.c_str(), profile.identity.c_str(),
                  profile.caCert.isEmpty() ? "none" : "set");
    wl_status_t status;
    if (profile.caCert.isEmpty()) {
      status = WiFi.begin(profile.ssid.c_str(), WPA2_AUTH_PEAP, profile.identity.c_str(),
                          profile.username.c_str(), profile.password.c_str());
    } else {
      status = WiFi.begin(profile.ssid.c_str(), WPA2_AUTH_PEAP, profile.identity.c_str(),
                          profile.username.c_str(), profile.password.c_str(), profile.caCert.c_str());
    }
    if (status == WL_CONNECT_FAILED) {
      Serial.println("[wifi] begin failed; driver rejected the enterprise profile");
      return false;
    }
    return true;
  }
#else
  if (profile.enterprise) {
    Serial.println("[wifi] enterprise support is unavailable in this build");
    return false;
  }
#endif
  if (profile.ssid.isEmpty()) {
    Serial.println("[wifi] no SSID configured");
    return false;
  }
  Serial.printf("[wifi] WPA2-PSK SSID=%s password=%s\n", profile.ssid.c_str(),
                profile.password.isEmpty() ? "[missing; open network only]" : "[set]");
  if (WiFi.begin(profile.ssid.c_str(), profile.password.c_str()) == WL_CONNECT_FAILED) {
    Serial.println("[wifi] begin failed; driver rejected the PSK profile");
    return false;
  }
  return true;
}
#endif

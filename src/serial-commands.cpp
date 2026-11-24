#include <Arduino.h>
#include "config.h"
#include "wifi-manager.h"
#include "mqtt.h"
#include "serial-commands.h"

#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
namespace {
String serialCommand;
void (*meshCommandHandler)(String) = nullptr;
}

void setMeshCommandHandler(void (*handler)(String)) { meshCommandHandler = handler; }

void pollSerialCommands() {
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (serialCommand.isEmpty()) continue;
      serialCommand.trim();
      if (serialCommand.startsWith("mesh ")) {
        if (meshCommandHandler) meshCommandHandler(serialCommand);
      } else if (serialCommand.startsWith("mqtt ")) {
        handleMqttCommand(serialCommand);
      } else if (serialCommand == "wifi" || serialCommand == "wifi help") {
        Serial.println("mesh set <channel> <sensor_interval_ms 500..86400000> <beacon_interval_ms 100..30000> | mesh window <active_window_ms 200..86400000> | mesh show");
        Serial.println("wifi set <ssid> <password>");
        Serial.println("wifi set eduroam <username> <password> [identity]");
        Serial.println("wifi ca <single-line PEM; replace newlines with \\n> | wifi ca clear");
        Serial.println("wifi scan | wifi save | wifi apply | wifi show | wifi clear");
        Serial.println("mqtt set <host> <port> [username password]");
        Serial.println("mqtt topic <prefix> | mqtt show | mqtt save | mqtt clear");
        Serial.println("Fields are space-separated. Your serial terminal may echo typed credentials.");
      } else {
        WifiProfile profile{};
        profile.ssid = wifiSsid();
        profile.enterprise = wifiIsEnterprise();
        if (serialCommand == "wifi show") {
          Serial.printf("[wifi] SSID=%s auth=%s password=[hidden]\n", profile.ssid.c_str(), profile.enterprise ? "PEAP" : "PSK");
        } else if (serialCommand == "wifi save") {
          if (wifiProfileIsEmpty()) {
            Serial.println("[wifi] set a profile first");
          } else {
            if (profile.enterprise && profile.identity.isEmpty()) profile.identity = profile.username;
            setWifiProfile(profile);
            saveWifiProfile();
            Serial.println("[wifi] profile saved in NVS");
          }
        } else if (serialCommand == "wifi clear") {
          clearSavedWifiProfile();
          Serial.println("[wifi] saved profile removed; reboot to use build defaults");
        } else if (serialCommand == "wifi ca clear") {
          profile.caCert = "";
          setWifiProfile(profile);
          Serial.println("[wifi] CA removed from active profile; use 'wifi save' to persist");
        } else if (serialCommand.startsWith("wifi ca ")) {
          String certificate = serialCommand.substring(8);
          certificate.replace("\\n", "\n");
          if (!certificate.startsWith("-----BEGIN CERTIFICATE-----") || !certificate.endsWith("-----END CERTIFICATE-----")) {
            Serial.println("[wifi] invalid PEM certificate");
          } else if (certificate.length() > 5000) {
            Serial.println("[wifi] CA certificate too long (max 5000 chars)");
          } else {
            profile.caCert = certificate;
            setWifiProfile(profile);
            Serial.println("[wifi] CA set; use 'wifi save' to persist");
          }
        } else if (serialCommand.startsWith("wifi set eduroam ")) {
          String fields = serialCommand.substring(16);
          fields.trim();
          const int first = fields.indexOf(' ');
          if (first <= 0) {
            Serial.println("[wifi] usage: wifi set eduroam <username> <password> [identity]");
          } else {
            const int second = fields.indexOf(' ', first + 1);
            const String username = fields.substring(0, first);
            const String password = second < 0 ? fields.substring(first + 1) : fields.substring(first + 1, second);
            const String identity = second < 0 ? username : fields.substring(second + 1);
            if (username.isEmpty() || password.isEmpty() || password.length() > 63 || identity.isEmpty() || identity.indexOf(' ') >= 0) {
              Serial.println("[wifi] invalid fields; use single tokens without spaces");
            } else if (username.length() > 128 || identity.length() > 128) {
              Serial.println("[wifi] username/identity too long (max 128 chars)");
            } else {
              profile.ssid = "eduroam";
              profile.username = username;
              profile.password = password;
              profile.identity = identity;
              profile.enterprise = true;
              setWifiProfile(profile);
              Serial.println("[wifi] eduroam profile set (password hidden); use 'wifi save' then 'wifi apply'");
            }
          }
        } else if (serialCommand.startsWith("wifi set ")) {
          String fields = serialCommand.substring(9);
          fields.trim();
          const int split = fields.indexOf(' ');
          if (split <= 0 || split == static_cast<int>(fields.length()) - 1) {
            Serial.println("[wifi] usage: wifi set <ssid> <password>");
          } else {
            const String ssid = fields.substring(0, split);
            const String password = fields.substring(split + 1);
            if (ssid.length() > 32 || password.length() > 63 || ssid.indexOf(' ') >= 0 || ssid.equalsIgnoreCase("eduroam")) {
              Serial.println("[wifi] invalid PSK profile; SSID must be space-free (max 32) and not eduroam; password max 63");
            } else {
              profile.ssid = ssid;
              profile.password = password;
              profile.username = "";
              profile.identity = "";
              profile.caCert = "";
              profile.enterprise = profile.ssid.equalsIgnoreCase("eduroam");
              setWifiProfile(profile);
              Serial.println("[wifi] PSK profile set (password hidden); use 'wifi save' then 'wifi apply'");
            }
          }
        } else if (serialCommand == "wifi scan") {
          setWifiScanRequested();
          Serial.println("[wifi] scan queued");
        } else if (serialCommand == "wifi apply") {
          setWifiApplyRequested();
          Serial.println("[wifi] will reconnect with the new profile");
        } else {
          Serial.println("[serial] unknown command; use 'wifi help'");
        }
      }
      serialCommand = "";
    } else if (serialCommand.length() < 6000) {
      serialCommand += c;
    } else {
      serialCommand = "";
      Serial.println("[serial] command too long; discarded");
    }
  }
}
#endif

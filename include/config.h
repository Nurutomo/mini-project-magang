#pragma once

// Optional untracked local settings; keep Wi-Fi and broker secrets out of git.
#if __has_include("config.local.h")
#include "config.local.h"
#endif

#include <Arduino.h>

#if defined(DEVICE_ROLE_SENSOR) && DEVICE_ROLE_SENSOR && defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
#error "A gateway cannot also be a sensor node."
#elif (!defined(DEVICE_ROLE_SENSOR) || !DEVICE_ROLE_SENSOR) && \
      (!defined(DEVICE_ROLE_RELAY) || !DEVICE_ROLE_RELAY) && \
      (!defined(DEVICE_ROLE_GATEWAY) || !DEVICE_ROLE_GATEWAY)
#error "Build one of the gateway-* or sensor-relay-* environments for esp32c3 or esp32classic."
#endif

#define FW_VERSION "0.2.1"

#ifndef NODE_ID
#define NODE_ID ""
#endif

#ifndef ESPNOW_CHANNEL
#define ESPNOW_CHANNEL 1
#endif
#ifndef ESPNOW_TTL
#define ESPNOW_TTL 3
#endif
#ifndef ESPNOW_ENABLE_RELAY
#define ESPNOW_ENABLE_RELAY 1
#endif
#ifndef SAMPLE_INTERVAL_S
#define SAMPLE_INTERVAL_S 300
#endif
#ifndef SYNC_BEACON_INTERVAL_MS
#define SYNC_BEACON_INTERVAL_MS 1000
#endif
#ifndef NODE_ACTIVE_WINDOW_MS
#define NODE_ACTIVE_WINDOW_MS 3000
#endif
#ifndef NODE_DEEP_SLEEP
#define NODE_DEEP_SLEEP 1
#endif

// Many ESP32-C3 SuperMini boards wire an active-low onboard LED to GPIO8.
// I2C defaults to GPIO6/7 to avoid the LED conflict; verify the actual
// board revision and sensor wiring before use.
#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN 6
#endif
#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN 7
#endif
#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN 8
#endif
#ifndef STATUS_LED_ACTIVE_HIGH
#define STATUS_LED_ACTIVE_HIGH 0
#endif
#ifndef BH1750_I2C_ADDRESS
#define BH1750_I2C_ADDRESS 0x23
#endif
#ifndef OLED_I2C_ADDRESS
#define OLED_I2C_ADDRESS 0x3C
#endif

#ifndef NTC_ADC_PIN
#define NTC_ADC_PIN 0
#endif
#ifndef WIND_ADC_PIN
#define WIND_ADC_PIN 1
#endif
#ifndef BATTERY_ADC_PIN
#define BATTERY_ADC_PIN 3
#endif
#ifndef ADC_SAMPLES
#define ADC_SAMPLES 8
#endif
#ifndef ADC_REFERENCE_MV
#define ADC_REFERENCE_MV 3300.0f
#endif

// NTC assumptions: fixed resistor to 3V3, thermistor to GND.
#ifndef NTC_SERIES_OHM
#define NTC_SERIES_OHM 10000.0f
#endif
#ifndef NTC_NOMINAL_OHM
#define NTC_NOMINAL_OHM 10000.0f
#endif
#ifndef NTC_NOMINAL_TEMP_C
#define NTC_NOMINAL_TEMP_C 25.0f
#endif
#ifndef NTC_BETA
#define NTC_BETA 3950.0f
#endif

// Divider output only (not converted to wind speed without calibration).
#ifndef WIND_DIVIDER_RATIO
#define WIND_DIVIDER_RATIO 2.0f
#endif
#ifndef FEATURE_WIND_ADC
#define FEATURE_WIND_ADC 1
#endif
#ifndef FEATURE_BATTERY_SENSE
#define FEATURE_BATTERY_SENSE 1
#endif
#ifndef FEATURE_OLED
#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
#define FEATURE_OLED 1
#else
#define FEATURE_OLED 0
#endif
#endif
#ifndef BATTERY_DIVIDER_RATIO
#define BATTERY_DIVIDER_RATIO 2.0f
#endif

// Solar charger settings are not used to configure hardware. Values are
// reported only as design assumptions; verify TP4056/solar compatibility.
#ifndef SOLAR_PANEL_RATED_VOLTAGE
#define SOLAR_PANEL_RATED_VOLTAGE 6.0f
#endif
#ifndef SOLAR_PANEL_RATED_POWER_W
#define SOLAR_PANEL_RATED_POWER_W 1.0f
#endif

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif
#ifndef EDUROAM_USERNAME
#define EDUROAM_USERNAME ""
#endif
#ifndef EDUROAM_IDENTITY
#define EDUROAM_IDENTITY EDUROAM_USERNAME
#endif
#ifndef EDUROAM_PASSWORD
#define EDUROAM_PASSWORD ""
#endif
#ifndef EDUROAM_CA_CERT
#define EDUROAM_CA_CERT ""
#endif
#ifndef WIFI_CONFIG_NAMESPACE
#define WIFI_CONFIG_NAMESPACE "wifi"
#endif
#ifndef WIFI_CONFIG_SSID_KEY
#define WIFI_CONFIG_SSID_KEY "ssid"
#endif
#ifndef WIFI_CONFIG_PASSWORD_KEY
#define WIFI_CONFIG_PASSWORD_KEY "password"
#endif
#ifndef WIFI_CONFIG_USERNAME_KEY
#define WIFI_CONFIG_USERNAME_KEY "username"
#endif
#ifndef WIFI_CONFIG_IDENTITY_KEY
#define WIFI_CONFIG_IDENTITY_KEY "identity"
#endif
#ifndef WIFI_CONFIG_CA_CERT_KEY
#define WIFI_CONFIG_CA_CERT_KEY "ca_cert"
#endif
#ifndef RTC_TIME_VALID
#define RTC_TIME_VALID 1700000000LL
#endif
#ifndef TIME_SYNC_TIMEOUT_MS
#define TIME_SYNC_TIMEOUT_MS 30000UL
#endif
#ifndef MQTT_HOST
#define MQTT_HOST "test.mosquitto.org"
#endif
#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif
#ifndef MQTT_CONFIG_NAMESPACE
#define MQTT_CONFIG_NAMESPACE "mqtt"
#endif
#ifndef MQTT_CONFIG_HOST_KEY
#define MQTT_CONFIG_HOST_KEY "host"
#endif
#ifndef MQTT_CONFIG_PORT_KEY
#define MQTT_CONFIG_PORT_KEY "port"
#endif
#ifndef MQTT_CONFIG_TOPIC_KEY
#define MQTT_CONFIG_TOPIC_KEY "topic"
#endif
#ifndef MQTT_CONFIG_USER_KEY
#define MQTT_CONFIG_USER_KEY "user"
#endif
#ifndef MQTT_CONFIG_PASSWORD_KEY
#define MQTT_CONFIG_PASSWORD_KEY "password"
#endif
#ifndef MQTT_USER
#define MQTT_USER ""
#endif
#ifndef MQTT_PASSWORD
#define MQTT_PASSWORD ""
#endif
#ifndef MQTT_TOPIC_PREFIX
#define MQTT_TOPIC_PREFIX "unesa/ussc/sensor"
#endif
#ifndef MQTT_CLIENT_ID
#define MQTT_CLIENT_ID ""
#endif
#ifndef NTP_SERVER_1
#define NTP_SERVER_1 "pool.ntp.org"
#endif
#ifndef NTP_SERVER_2
#define NTP_SERVER_2 "time.google.com"
#endif
#ifndef GATEWAY_QUEUE_SIZE
#define GATEWAY_QUEUE_SIZE 16
#endif
#ifndef GATEWAY_DEDUP_SIZE
#define GATEWAY_DEDUP_SIZE 32
#endif
#ifndef RELAY_DEDUP_SIZE
#define RELAY_DEDUP_SIZE 32
#endif
#ifndef RELAY_JITTER_MS
#define RELAY_JITTER_MS 120
#endif

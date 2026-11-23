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
#error "Build env:sensor-node, env:relay-node, or env:gateway."
#endif

#define FW_VERSION "0.2.0"

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
#define SAMPLE_INTERVAL_S 60
#endif
#ifndef NODE_DEEP_SLEEP
#define NODE_DEEP_SLEEP 0
#endif

// Default ESP32-C3 SuperMini I2C wiring; confirm board revision before use.
#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN 8
#endif
#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN 9
#endif
#ifndef BH1750_I2C_ADDRESS
#define BH1750_I2C_ADDRESS 0x23
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
#ifndef MQTT_HOST
#define MQTT_HOST ""
#endif
#ifndef MQTT_PORT
#define MQTT_PORT 1883
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

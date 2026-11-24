#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "config.h"
#include "voltage-read.h"

uint16_t readAdcMillivolts(uint8_t pin) {
  analogRead(pin); // discard conversion after channel switch
  delay(4);
  uint32_t total = 0;
  uint8_t valid = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES; ++i) {
    const uint32_t millivolts = analogReadMilliVolts(pin);
    if (millivolts > 0) {
      total += millivolts;
      ++valid;
    }
    delay(2);
  }
  if (valid == 0) {
    static uint32_t lastWarnMs = 0;
    if (millis() - lastWarnMs >= 5000) {
      lastWarnMs = millis();
      Serial.printf("[sensor] GPIO%u ADC read failed; check wiring. On classic ESP32 use ADC1 "
                    "GPIO32-39 because ADC2 (GPIO0/2/4/12-15/25-27) does not work with Wi-Fi.\n",
                    pin);
    }
    return 0;
  }
  return static_cast<uint16_t>(total / valid);
}

bool readBh1750(float &lux) {
  Wire.beginTransmission(BH1750_I2C_ADDRESS);
  Wire.write(0x10); // continuous high-resolution mode, approx. 1 lx resolution
  if (Wire.endTransmission() != 0) return false;
  delay(180);
  if (Wire.requestFrom(static_cast<int>(BH1750_I2C_ADDRESS), 2) != 2) return false;
  const uint16_t raw = (static_cast<uint16_t>(Wire.read()) << 8) | Wire.read();
  lux = raw / 1.2f;
  return isfinite(lux) && lux >= 0.0f && lux <= 200000.0f;
}

float readNtcCelsius(bool &valid) {
  const float mv = readAdcMillivolts(NTC_ADC_PIN);
  const float vref = ADC_REFERENCE_MV;
  if (mv <= 1 || mv >= vref - 1 || NTC_BETA <= 0.0f) {
    valid = false;
    return NAN;
  }
  // 3V3 -> fixed resistor -> ADC -> NTC -> GND.
  const float resistance = NTC_SERIES_OHM * mv / (vref - mv);
  const float nominalK = NTC_NOMINAL_TEMP_C + 273.15f;
  const float invT = (1.0f / nominalK) + logf(resistance / NTC_NOMINAL_OHM) / NTC_BETA;
  if (!isfinite(invT) || invT <= 0.0f) {
    valid = false;
    return NAN;
  }
  const float celsius = (1.0f / invT) - 273.15f;
  valid = isfinite(celsius) && celsius > -40.0f && celsius < 150.0f;
  return valid ? celsius : NAN;
}

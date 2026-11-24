#pragma once
#include <Arduino.h>

uint16_t readAdcMillivolts(uint8_t pin);
bool readBh1750(float &lux);
float readNtcCelsius(bool &valid);

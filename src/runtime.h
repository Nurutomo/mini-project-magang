#pragma once
#include <Arduino.h>
#include "telemetry.h"

void initStatusLed();
void blinkStatusLed(uint8_t pulses = 1);
void serviceStatusLed();

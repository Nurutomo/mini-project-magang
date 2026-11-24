#include <Arduino.h>
#include "config.h"

#if STATUS_LED_PIN >= 0
namespace {
volatile uint8_t statusLedPendingPulses = 0;
portMUX_TYPE statusLedMux = portMUX_INITIALIZER_UNLOCKED;
bool statusLedOn = false;
bool statusLedGap = false;
uint32_t statusLedNextTransitionMs = 0;
}
#endif

void initStatusLed() {
#if STATUS_LED_PIN >= 0
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, STATUS_LED_ACTIVE_HIGH ? LOW : HIGH);
#endif
}

void blinkStatusLed(uint8_t pulses) {
#if STATUS_LED_PIN >= 0
  portENTER_CRITICAL(&statusLedMux);
  const uint16_t queued = static_cast<uint16_t>(statusLedPendingPulses) + pulses;
  statusLedPendingPulses = static_cast<uint8_t>(queued > 20 ? 20 : queued);
  portEXIT_CRITICAL(&statusLedMux);
#else
  (void)pulses;
#endif
}

void serviceStatusLed() {
#if STATUS_LED_PIN >= 0
  const uint32_t now = millis();
  if (statusLedOn && static_cast<int32_t>(now - statusLedNextTransitionMs) >= 0) {
    digitalWrite(STATUS_LED_PIN, STATUS_LED_ACTIVE_HIGH ? LOW : HIGH);
    statusLedOn = false;
    statusLedGap = true;
    statusLedNextTransitionMs = now + 45;
  }
  if (!statusLedOn && statusLedPendingPulses &&
      (!statusLedGap || static_cast<int32_t>(now - statusLedNextTransitionMs) >= 0)) {
    portENTER_CRITICAL(&statusLedMux);
    if (statusLedPendingPulses) --statusLedPendingPulses;
    portEXIT_CRITICAL(&statusLedMux);
    digitalWrite(STATUS_LED_PIN, STATUS_LED_ACTIVE_HIGH ? HIGH : LOW);
    statusLedOn = true;
    statusLedGap = false;
    statusLedNextTransitionMs = now + 70;
  }
#endif
}

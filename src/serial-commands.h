#pragma once
#include <Arduino.h>

#if defined(DEVICE_ROLE_GATEWAY) && DEVICE_ROLE_GATEWAY
void pollSerialCommands();
void setMeshCommandHandler(void (*handler)(String));
#endif

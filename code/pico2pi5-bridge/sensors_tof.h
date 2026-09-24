#pragma once
#include <stdint.h>

// Stable UART telemetry slots, independent of which sensors are fitted.
#define TOF_AUX    0
#define TOF_REAR   TOF_AUX
#define TOF_LEFT   1
#define TOF_RIGHT  2
#define TOF_FRONT  3
#define TOF_COUNT  4
#define TOF_INVALID_MM 9999

// Currently LEFT and RIGHT only. Set 0x0F when FRONT and REAR are fitted.
// Disabled slots remain in XSHUT reset and retain their telemetry positions.
#ifndef TOF_ENABLED_MASK
#define TOF_ENABLED_MASK 0x06
#endif

// Own these APIs on core 1; other cores consume g_state.tof_mm under STATE_LOCK.
void     tof_init();
void     tof_update();
// Invalid, disconnected, or older than 150 ms => TOF_INVALID_MM.
uint16_t tof_get(uint8_t idx);
// Configured and delivering frames; does not imply a valid optical return.
bool     tof_connected(uint8_t idx);

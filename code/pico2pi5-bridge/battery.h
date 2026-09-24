#pragma once
#include <stdint.h>

// Resistor divider values for BAT1 - adjust to your actual resistors!!
// Example: 100k + 68k divider - ratio = 68/(100+68) = 0.405
// Vbat = Vadc / ratio
// With 2S LiPo max ~8.4V - Vadc max = 8.4 * 0.405 = 3.4V (just within 3.3V!!)
#define BAT_DIVIDER_RATIO  0.7826f
#define ADC_SAMPLES        16       // average this many samples
#define ADC_VREF_MV        3300     // RP2350 ADC reference in mV
#define BAT_ADC_RESOLUTION 4096     // 12-bit

void     battery_init();
void     battery_update();          // call on Core 1, 100ms
uint16_t battery_get_mv();         // BAT1 in millivolts
uint8_t  battery_get_pct();        // rough % for 2S LiPo
bool     battery_low();            // true if below 6.8V (2S cutoff)

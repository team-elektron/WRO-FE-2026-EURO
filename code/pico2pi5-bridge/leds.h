#pragma once
#include <stdint.h>

enum LedMode : uint8_t {
    LED_IDLE      = 0,
    LED_DRIVING   = 1,
    LED_OBSTACLE  = 2,
    LED_LOW_BAT   = 3,
    LED_FAULT     = 4,
    LED_OFF       = 5,
};

void leds_init();
void leds_update();
void leds_set_mode(LedMode mode);
void leds_set_initialized();   // call from main once boot is complete
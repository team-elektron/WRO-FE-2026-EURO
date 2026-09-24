#include "leds.h"
#include "pins.h"
#include "shared_state.h"
#include "battery.h"
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

static Adafruit_NeoPixel strip(NUM_LEDS, PIN_LEDS, NEO_GRB + NEO_KHZ800);

static uint32_t g_lastUpdate  = 0;
static uint8_t  g_step        = 0;
static bool     g_initializing = true;

// helpers 

static void update_battery_leds() {
    uint8_t pct = battery_get_pct();
    uint8_t bars;
    uint32_t color;
    bool blink = false;

    if (pct >= 75) {
        bars  = 4;
        color = strip.Color(0, 180, 0);
        blink = (pct <= 80);        // blink 75-80%
    } else if (pct >= 50) {
        bars  = 3;
        color = strip.Color(180, 120, 0);
        blink = (pct <= 55);        // blink 50-55%
    } else if (pct >= 25) {
        bars  = 2;
        color = strip.Color(200, 80, 0);
        blink = (pct <= 30);        // blink 25-30%
    } else {
        bars  = 1;
        color = strip.Color(200, 0, 0);
        blink = (pct <= 5);         // blink 0-5%
    }

    bool on = (g_step & 0x08) != 0;   // ~3Hz blink

    for (int i = 0; i < 4; i++) {
        if (i < bars - 1) {
            // Lower bars always solid
            strip.setPixelColor(i, color);
        } else if (i == bars - 1) {
            // Top bar blinks if near threshold, solid otherwise
            strip.setPixelColor(i, (blink && !on) ? 0 : color);
        } else {
            strip.setPixelColor(i, 0);
        }
    }
}

static void update_movement_led() {
    // Pull cmd_dir from shared state
    STATE_LOCK();
    int8_t dir = g_state.cmd_dir;
    STATE_UNLOCK();

    if (dir == 0) {
        strip.setPixelColor(4, 0);                               // off = stopped
    } else if (dir > 0) {
        strip.setPixelColor(4, strip.Color(0, 0, 200));          // blue = forward
    } else {
        strip.setPixelColor(4, strip.Color(120, 0, 180));        // purple = backward
    }
}

static void update_status_led() {
    STATE_LOCK();
    bool fault = g_state.hbr_fault;
    STATE_UNLOCK();

    if (g_initializing) {
        float rad = (g_step * 2 / 255.0f) * 2.0f * M_PI;
        uint8_t b = (uint8_t)((sinf(rad) * 0.5f + 0.5f) * 200.0f);
        strip.setPixelColor(5, strip.Color(0, 0, b));
    } else if (fault) {
        bool on = (g_step & 0x04) != 0;
        strip.setPixelColor(5, on ? strip.Color(220, 0, 0) : 0);
    } else {
        strip.setPixelColor(5, strip.Color(0, 180, 0));          // green = all ok
    }
}

// public API

void leds_init() {
    strip.begin();
    strip.setBrightness(30);
    strip.clear();
    strip.show();
    g_initializing = true;
    Serial.println("[OK] LEDs init");
}

void leds_set_mode(LedMode mode) {
    // Only thing worth acting on now is fault override
    if (mode == LED_FAULT) {
        g_initializing = false;
    }
}

void leds_set_initialized() {
    g_initializing = false;
}

void leds_update() {
    uint32_t now = millis();
    if (now - g_lastUpdate < 20) return;
    g_lastUpdate = now;
    g_step++;

    strip.clear();
    update_battery_leds();   // LEDs 0-3
    update_movement_led();   // LED  4
    update_status_led();     // LED  5
    // LEDs 6-7 unused — dark
    strip.show();
}
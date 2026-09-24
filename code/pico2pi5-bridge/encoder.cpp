#include "encoder.h"
#include "pins.h"
#include <Arduino.h>

static volatile int32_t g_ticks = 0;
static int32_t g_last_ticks     = 0;

// Speed hall only - always increment.
// Direction comes from cmd_dir, not the encoder.
// The direction hall (PIN_ENC_B) is unreliable inside an ISR
// because it can lag the speed pulse by several microseconds.
static void enc_isr() {
    g_ticks++;
}

void encoder_init() {
    pinMode(PIN_ENC_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B, INPUT_PULLUP);   // still pulled up in case we use it later
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), enc_isr, RISING);
    Serial.println("[OK] Encoder init");
}

void encoder_reset() {
    noInterrupts();
    g_ticks      = 0;
    g_last_ticks = 0;
    interrupts();
}

int32_t encoder_get_ticks() {
    noInterrupts();
    int32_t t = g_ticks;
    interrupts();
    return t;
}

// Returns unsigned RPM magnitude - caller applies direction sign
float encoder_get_rpm(float dt_s) {
    noInterrupts();
    int32_t now = g_ticks;
    interrupts();

    int32_t delta = now - g_last_ticks;
    g_last_ticks  = now;

    if (dt_s <= 0.0f) return 0.0f;
    return (delta / (float)COUNTS_PER_REV) / dt_s * 60.0f;
}
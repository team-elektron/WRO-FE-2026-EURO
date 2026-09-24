#include "battery.h"
#include "pins.h"
#include <Arduino.h>
#include <hardware/adc.h>

#define BAT_MAX_MV  4150
#define BAT_MIN_MV  3200
#define BAT_LOW_MV  3700

#define EMA_ALPHA   0.1f   // lower = smoother, slower to react (0.0-1.0)

static float    g_battery_mv_f = 0.0f;   // EMA accumulator
static uint16_t g_battery_mv   = 0;

void battery_init() {
    adc_init();
    adc_gpio_init(PIN_BAT1_ADC);

    // Seed EMA with first real reading so it doesn't start from 0
    adc_select_input(3);
    uint32_t sum = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) sum += adc_read();
    float vadc_mv = ((sum / ADC_SAMPLES) / (float)BAT_ADC_RESOLUTION) * ADC_VREF_MV;
    g_battery_mv_f = vadc_mv / BAT_DIVIDER_RATIO;
    g_battery_mv   = (uint16_t)g_battery_mv_f;

    Serial.println("[OK] Battery ADC init");
}

void battery_update() {
    adc_select_input(3);

    uint32_t sum = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) sum += adc_read();
    float vadc_mv  = ((sum / ADC_SAMPLES) / (float)BAT_ADC_RESOLUTION) * ADC_VREF_MV;
    float raw_mv   = vadc_mv / BAT_DIVIDER_RATIO;

    // Exponential moving average across updates
    g_battery_mv_f = (EMA_ALPHA * raw_mv) + ((1.0f - EMA_ALPHA) * g_battery_mv_f);
    g_battery_mv   = (uint16_t)g_battery_mv_f;
}

uint16_t battery_get_mv() {
    return g_battery_mv;
}

uint8_t battery_get_pct() {
    if (g_battery_mv >= BAT_MAX_MV) return 100;
    if (g_battery_mv <= BAT_MIN_MV) return 0;
    return (uint8_t)((g_battery_mv - BAT_MIN_MV) * 100 / (BAT_MAX_MV - BAT_MIN_MV));
}

bool battery_low() {
    return g_battery_mv > 0 && g_battery_mv < BAT_LOW_MV;
}
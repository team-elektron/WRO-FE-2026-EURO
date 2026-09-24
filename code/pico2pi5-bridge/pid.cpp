#include "pid.h"
#include "encoder.h"
#include <Arduino.h>

static float  g_target_rpm = 0.0f;
static float  g_integral   = 0.0f;
static int8_t g_last_dir   = 0;

void pid_init() {
    g_target_rpm = 0.0f;
    g_integral   = 0.0f;
    g_last_dir   = 0;
}

void pid_set_target(float target_rpm) {
    int8_t dir = (target_rpm > 0.0f) ? 1 : (target_rpm < 0.0f) ? -1 : 0;
    if (dir != g_last_dir) {
        g_integral = 0.0f;
        g_last_dir = dir;
    }
    g_target_rpm = target_rpm;
}

void pid_reset() {
    g_integral   = 0.0f;
    g_last_dir   = 0;
    g_target_rpm = 0.0f;
}

// current_rpm: pass in magnitude only (always positive)
// returns signed PWM: positive = forward, negative = backward
int pid_compute(float current_rpm, float dt_s) {
    if (dt_s <= 0.0f)          return 0;
    if (g_target_rpm == 0.0f) return 0;

    float target_mag = fabsf(g_target_rpm);

    float error    = target_mag - current_rpm;
    g_integral    += error * dt_s;

    // Clamp integral the same way the working script does
    float max_i    = (float)(PWM_MAX - PWM_MIN) / max(PID_KI, 0.001f);
    g_integral     = constrain(g_integral, 0.0f, max_i);  // 0 minimum, not negative

    int pwm = PWM_MIN + (int)(PID_KP * error + PID_KI * g_integral);
    pwm = constrain(pwm, PWM_MIN, PWM_MAX);

    // Apply direction sign
    return (g_target_rpm > 0.0f) ? pwm : -pwm;
}
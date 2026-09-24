#pragma once
#include <stdint.h>

#define PID_KP          1.8f
#define PID_KI          0.5f
#define PWM_MIN         0       // additive output — no deadband needed
#define PWM_MAX         255

void  pid_init();
void  pid_set_target(float target_rpm);
void  pid_reset();
int   pid_compute(float current_rpm, float dt_s);  // returns signed PWM
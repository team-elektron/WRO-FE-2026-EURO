#pragma once
#include <stdint.h>

//Motor directions
#define MOT_STOP     0
#define MOT_FORWARD  1
#define MOT_BACKWARD 2

// Servo slew rate — degrees per 20ms update tick
// 2 = smooth (~900ms for 90°), 4 = faster (~450ms), 1 = very slow
#define SERVO_SLEW_RATE  5

void motors_init();
void motors_set(int8_t direction, uint8_t speed);
void motors_stop();
void servo_set(uint8_t angle);    // set target angle — movement is slewed
void servo_update();              // call every 20ms to step toward target
void motion_enable(bool en);
bool motion_fault();              // true if DRV8833 nFAULT asserted
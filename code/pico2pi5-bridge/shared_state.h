#pragma once
#include "pico/mutex.h"
#include <stdint.h>

// Shared state
typedef struct {
    // Commands from Pi
    int8_t   cmd_dir;
    uint8_t  cmd_speed;
    uint8_t  cmd_servo;
    uint8_t  cmd_led_mode;
    bool     cmd_fresh;

    // Telemetry
    uint16_t tof_mm[4];
    uint16_t battery_mv;
    int16_t  imu_yaw;
    int32_t  enc_ticks;
    bool     hbr_fault;
} RobotState;

extern RobotState g_state;
extern mutex_t    g_mutex;
extern mutex_t    g_wire_mutex;   // protects Wire across both cores

#define STATE_LOCK()   mutex_enter_blocking(&g_mutex)
#define STATE_UNLOCK() mutex_exit(&g_mutex)

#define WIRE_LOCK()    mutex_enter_blocking(&g_wire_mutex)
#define WIRE_UNLOCK()  mutex_exit(&g_wire_mutex)
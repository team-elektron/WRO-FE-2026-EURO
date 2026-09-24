//
//  Team Elektron - WRO Future Engineers 2026
//  RP2350 (Pico 2) dual-core firmware
//
//  Core 0: motors, servo, encoder, IMU, PID
//  Core 1: ToF sensors, UART, battery ADC, LEDs
//

#include "pico/multicore.h"
#include "pico/mutex.h"
#include <Wire.h>

#include "pins.h"
#include "shared_state.h"
#include "motors.h"
#include "encoder.h"
#include "pid.h"
#include "sensors_imu.h"
#include "sensors_tof.h"
#include "battery.h"
#include "leds.h"
#include "uart_comms.h"

// Shared state definition

RobotState g_state = {};
    mutex_t    g_mutex;
    mutex_t    g_wire_mutex;   

// Core 0 timing
static uint32_t t_pid   = 0;   // 10ms  — PID update
static uint32_t t_imu   = 0;   // 20ms  — IMU read
static uint32_t t_servo = 0;   // 20ms  — servo slew step
static uint32_t t_telem = 0;   // 100ms — write telem to shared state

//  CORE 1 — sensors, UART, LEDs, battery

void core1_main() {
    tof_init();
    battery_init();
    uart_init();
    leds_init();
    leds_set_initialized();
    multicore_fifo_push_blocking(1);

    uint32_t t_uart_rx  = 0;
    uint32_t t_uart_tx  = 0;
    uint32_t t_bat      = 0;
    uint32_t t_led      = 0;

    while (true) {
        uint32_t now = millis();

        tof_update();

        if (now - t_uart_rx >= 10) {
            t_uart_rx = now;
            uart_receive();
        }

        if (now - t_uart_tx >= 100) {
            t_uart_tx = now;
            uart_send_telemetry();
        }

        if (now - t_bat >= 1000) {
            t_bat = now;
            battery_update();

            uint16_t mv = battery_get_mv();
            STATE_LOCK();
            g_state.battery_mv = mv;
            STATE_UNLOCK();

            if (battery_low()) {
                leds_set_mode(LED_LOW_BAT);
            }
        }

        static uint32_t t_tof_share = 0;
        if (now - t_tof_share >= 10) {
            t_tof_share = now;
            STATE_LOCK();
            for (int i = 0; i < TOF_COUNT; i++) {
                g_state.tof_mm[i] = tof_get(i);
            }
            STATE_UNLOCK();
        }

        STATE_LOCK();
        bool    fresh     = g_state.cmd_fresh;
        uint8_t led_mode  = g_state.cmd_led_mode;
        bool    hbr_fault = g_state.hbr_fault;
        STATE_UNLOCK();

        if (hbr_fault) {
            leds_set_mode(LED_FAULT);
        } else if (!battery_low() && fresh) {
            leds_set_mode((LedMode)led_mode);
        }

        if (now - t_led >= 20) {
            t_led = now;
            leds_update();
        }
    }
}


//  SETUP — Core 0

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("\n=== WRO Robot booting ===");
    

    mutex_init(&g_mutex);
    mutex_init(&g_wire_mutex);

    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SCL);
    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);
    Wire.setTimeout(I2C_TIMEOUT_MS);

    motors_init();
    encoder_init();
    pid_init();

    // Initialize the IMU before core 1 starts using Wire.
    imu_init();
    for (int i = 0; i < TOF_COUNT; ++i) g_state.tof_mm[i] = TOF_INVALID_MM;

    multicore_launch_core1(core1_main);
    multicore_fifo_pop_blocking();

    motion_enable(true);

    Serial.println("=== Boot complete ===\n");

    t_pid   = millis();
    t_imu   = millis();
    t_servo = millis();
    t_telem = millis();
}

//  LOOP — Core 0

void loop() {
    uint32_t now = millis();
    if (now - t_telem >= 100) {
        Serial.printf("yaw=%.2f\n", imu_get_yaw());
    }

    // IMU update — 20ms 
    if (now - t_imu >= 20) {
        t_imu = now;
        imu_update();
    }

    // Servo slew — 20ms
    if (now - t_servo >= 20) {
        t_servo = now;
        servo_update();
    }

    // PID + motor update — 10ms
    if (now - t_pid >= 10) {
        float dt = (now - t_pid) / 1000.0f;
        t_pid = now;

        // Always read RPM exactly once - it consumes the delta
        float rpm_magnitude = encoder_get_rpm(dt);

        STATE_LOCK();
        int8_t  dir     = g_state.cmd_dir;
        uint8_t speed   = g_state.cmd_speed;
        uint8_t servo_a = g_state.cmd_servo;
        g_state.cmd_fresh = false;
        STATE_UNLOCK();

        // Always update servo target regardless of motor state
        servo_set(servo_a);

        bool fault = motion_fault();
        STATE_LOCK();
        g_state.hbr_fault = fault;
        STATE_UNLOCK();

        if (fault) {
            motors_stop();
            pid_reset();
        } else if (dir == MOT_STOP) {
            motors_stop();
            pid_reset();
        } else {
            float target_rpm = (speed / 255.0f) * 800.0f;
            if (dir == MOT_BACKWARD) target_rpm = -target_rpm;

            pid_set_target(target_rpm);

            float current_rpm = rpm_magnitude;
            int   pwm         = pid_compute(current_rpm, dt);

            if (pwm == 0) {
                motors_stop();
            } else if (pwm > 0) {
                motors_set(MOT_FORWARD,  (uint8_t) pwm);
            } else {
                motors_set(MOT_BACKWARD, (uint8_t)-pwm);
            }
        }
    }

    // Telemetry to shared state — 100ms 
    if (now - t_telem >= 100) {
        t_telem = now;

        int16_t yaw10 = (int16_t)(imu_get_yaw() * 10.0f);
        int32_t enc   = encoder_get_ticks();

        STATE_LOCK();
        g_state.imu_yaw   = yaw10;
        g_state.enc_ticks = enc;
        STATE_UNLOCK();
    }
}
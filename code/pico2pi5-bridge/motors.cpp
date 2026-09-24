#include "motors.h"
#include "pins.h"
#include <Arduino.h>
#include <Servo.h>
#include <hardware/pwm.h>
#include <hardware/clocks.h>

static Servo   g_servo;
static uint8_t g_servo_angle  = 90;   // current position
static uint8_t g_servo_target = 90;   // commanded position

// Set a single GPIO to 16kHz PWM via hardware API, bypassing the
// global analogWriteFreq so the Servo library's slice is never touched.
static void set_motor_pin_pwm(uint8_t pin) {
    uint slice = pwm_gpio_to_slice_num(pin);
    uint chan  = pwm_gpio_to_channel(pin);

    gpio_set_function(pin, GPIO_FUNC_PWM);

    // RP2350 default sys clock = 150MHz
    // 150,000,000 / (16000 * 256) ≈ 36.6
    uint32_t sys_hz = clock_get_hz(clk_sys);
    float    clkdiv = (float)sys_hz / (16000.0f * 256.0f);

    pwm_set_clkdiv(slice, clkdiv);
    pwm_set_wrap(slice, 255);
    pwm_set_chan_level(slice, chan, 0);
    pwm_set_enabled(slice, true);
}

static void motor_pwm_write(uint8_t pin, uint8_t value) {
    pwm_set_chan_level(pwm_gpio_to_slice_num(pin),
                       pwm_gpio_to_channel(pin),
                       value);
}

void motors_init() {
    pinMode(PIN_MOTION_EN, OUTPUT);
    pinMode(PIN_NFAULT, INPUT_PULLUP);

    motion_enable(false);

    set_motor_pin_pwm(PIN_MOT1_A);
    set_motor_pin_pwm(PIN_MOT1_B);
    set_motor_pin_pwm(PIN_MOT2_A);
    set_motor_pin_pwm(PIN_MOT2_B);

    motors_stop();

    // MG90S: 500–2400us covers full 180 degrees without binding
    g_servo.attach(PIN_SERVO, 500, 2400);
    g_servo.write(90);

    Serial.println("[OK] Motors + servo init");
}

void motion_enable(bool en) {
    // Controls DRV8833 nSLEEP AND servo power — same pin
    digitalWrite(PIN_MOTION_EN, en ? HIGH : LOW);
}

bool motion_fault() {
    return digitalRead(PIN_NFAULT) == LOW;
}

void motors_stop() {
    motor_pwm_write(PIN_MOT1_A, 0);
    motor_pwm_write(PIN_MOT1_B, 0);
    motor_pwm_write(PIN_MOT2_A, 0);
    motor_pwm_write(PIN_MOT2_B, 0);
    motion_enable(false);   // cuts power to DRV8833 and servo together
}

void motors_set(int8_t direction, uint8_t speed) {
    if (motion_fault()) {
        motors_stop();
        Serial.println("[WARN] DRV8833 fault! Motors stopped.");
        return;
    }

    switch (direction) {
        case MOT_FORWARD:
            motion_enable(true);
            motor_pwm_write(PIN_MOT1_A, speed);
            motor_pwm_write(PIN_MOT1_B, 0);
            motor_pwm_write(PIN_MOT2_A, speed);
            motor_pwm_write(PIN_MOT2_B, 0);
            break;

        case MOT_BACKWARD:
            motion_enable(true);
            motor_pwm_write(PIN_MOT1_A, 0);
            motor_pwm_write(PIN_MOT1_B, speed);
            motor_pwm_write(PIN_MOT2_A, 0);
            motor_pwm_write(PIN_MOT2_B, speed);
            break;

        case MOT_STOP:
        default:
            motors_stop();
            break;
    }
}

void servo_set(uint8_t angle) {
    g_servo_target = constrain(angle, 0, 180);
}

void servo_update() {
    if (g_servo_angle == g_servo_target) return;

    if (g_servo_angle < g_servo_target) {
        g_servo_angle = (uint8_t)min((int)g_servo_angle + SERVO_SLEW_RATE, (int)g_servo_target);
    } else {
        g_servo_angle = (uint8_t)max((int)g_servo_angle - SERVO_SLEW_RATE, (int)g_servo_target);
    }

    g_servo.write(g_servo_angle);
}
#pragma once

// ── UART (to/from Pi 5) 
#define PIN_UART_TX       0
#define PIN_UART_RX       1

// Encoder
#define PIN_ENC_A         2
#define PIN_ENC_B         3

// I2C (VL53L1X + MPU6050) 
#define PIN_I2C_SDA       4
#define PIN_I2C_SCL       5

// Shared bus: 400 kHz, bounded transactions (Arduino-Pico Wire).
#define I2C_CLOCK_HZ      400000
#define I2C_TIMEOUT_MS    5

// ToF XSHUT  
#define PIN_XSHUT_1       6 // FRONT
#define PIN_XSHUT_2   7   // LEFT
#define PIN_XSHUT_3   8   // RIGHT
#define PIN_XSHUT_4   9   // REAR (AUX telemetry slot)

// Motors (DRV8833) 
#define PIN_MOT1_A        10
#define PIN_MOT1_B        11
#define PIN_MOT2_A        12
#define PIN_MOT2_B        13

// Motion enable (nSLEEP + servo MOSFET)
#define PIN_MOTION_EN     14

// Servo 
#define PIN_SERVO         15

// WS2812 Indicator LEDs (8x)
#define PIN_LEDS          26
#define NUM_LEDS          8

// DRV8833 nFAULT (input) 
#define PIN_NFAULT        27

// Battery ADC 
#define PIN_BAT1_ADC      29
// PIN_BAT2_ADC (28) — wiring error, unused

#include "sensors_imu.h"
#include "shared_state.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

static Adafruit_MPU6050 mpu;
static bool     g_ready    = false;
static float    g_yaw      = 0.0f;
static float    g_offset   = 0.0f;
static float    g_bias_gz  = 0.0f;
static uint32_t g_lastTime = 0;

#define GYRO_DEADBAND_DEG  0.08f

void imu_init() {
    if (!mpu.begin()) {
        Serial.println("[FAIL] MPU6050 not found");
        return;
    }

    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Calibrate gyro bias — keep robot still for 2 seconds
    Serial.println("[IMU] Calibrating — keep still...");
    double sum = 0;
    for (int i = 0; i < 500; i++) {
        sensors_event_t a, g, t;
        WIRE_LOCK();
        mpu.getEvent(&a, &g, &t);
        WIRE_UNLOCK();
        sum += g.gyro.z;
        delay(4);
    }
    g_bias_gz = (float)(sum / 500.0);
    Serial.printf("[IMU] Bias gz=%.4f rad/s\n", g_bias_gz);

    g_lastTime = millis();
    g_ready    = true;
    Serial.println("[OK] IMU ready");
}

void imu_update() {
    if (!g_ready) return;

    uint32_t now = millis();
    float dt = (now - g_lastTime) / 1000.0f;
    if (dt <= 0.0f) return;
    g_lastTime = now;

    sensors_event_t a, gyro, t;
    WIRE_LOCK();
    mpu.getEvent(&a, &gyro, &t);
    WIRE_UNLOCK();

    // Subtract bias, convert to deg/s
    float gz = (gyro.gyro.z - g_bias_gz) * 180.0f / M_PI;

    // Deadband — kill noise when still
    if (fabsf(gz) < GYRO_DEADBAND_DEG) gz = 0.0f;

    g_yaw += gz * dt;

    // Normalise to -180..+180
    while (g_yaw >  180.0f) g_yaw -= 360.0f;
    while (g_yaw < -180.0f) g_yaw += 360.0f;
}

bool imu_ready() { return g_ready; }

float imu_get_yaw() {
    float y = g_yaw - g_offset;
    while (y >  180.0f) y -= 360.0f;
    while (y < -180.0f) y += 360.0f;
    return y;
}

void imu_reset_yaw() {
    g_offset = g_yaw;
    Serial.println("[IMU] Yaw zeroed");
}
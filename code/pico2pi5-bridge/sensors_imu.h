#pragma once
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


void  imu_init();
void  imu_update();
bool  imu_ready();
float imu_get_yaw();
void  imu_reset_yaw();

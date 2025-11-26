#pragma once
#include <Arduino.h>
#include <stdint.h>

// Initialize IMU (MPU6050 / MPU65xx clone) using raw I2C.
// Returns true if the device is present and configured.
bool initIMU();

// Returns true if IMU was successfully initialized.
bool imuIsOK();

// Read raw accelerometer and gyroscope data.
// ax, ay, az -> acceleration raw units
// gx, gy, gz -> gyro raw units
// Returns true if data was read successfully, false otherwise.
bool readIMURaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz);

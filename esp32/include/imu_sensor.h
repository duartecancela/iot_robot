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

// Estimate robot orientation (pitch and roll) using a complementary filter.
//
// pitchDeg -> rotation around X axis in degrees (forward/backward tilt)
// rollDeg  -> rotation around Y axis in degrees (left/right tilt)
//
// This function uses:
//  - Accelerometer to estimate the long-term inclination (gravity direction)
//  - Gyroscope to integrate fast angular changes between samples
//
// It maintains internal state (previous angles and last sample time),
// so it should be called periodically (e.g. every 20–50 ms) to get
// smooth and meaningful results.
//
// Returns true if estimation was updated successfully, false otherwise.
bool imuGetAngles(float &pitchDeg, float &rollDeg);


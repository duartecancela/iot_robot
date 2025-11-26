#include "imu_sensor.h"
#include <Wire.h>

// I2C address for MPU devices (6050 / 65xx clones)
static const uint8_t IMU_ADDR          = 0x68;
static const uint8_t REG_PWR_MGMT_1    = 0x6B;
static const uint8_t REG_WHO_AM_I      = 0x75;
static const uint8_t REG_ACCEL_XOUT_H  = 0x3B;
static const uint8_t REG_GYRO_XOUT_H   = 0x43;

static bool imuOK = false;

// ---- Low-level helpers ----
static uint8_t imuRead8(uint8_t reg) {
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(IMU_ADDR, (uint8_t)1);
    if (Wire.available() < 1) return 0;
    return Wire.read();
}

static void imuWrite8(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

static int16_t imuRead16(uint8_t reg) {
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(IMU_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return 0;
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    return (int16_t)((hi << 8) | lo);
}

// ---- Public API ----
bool initIMU() {
    // If Wire is not yet initialized, you can uncomment the next line.
    // In your project, Wire.begin(21,22) is already called in initBME().
    // Wire.begin(21, 22);

    // Check if device responds at IMU_ADDR
    Wire.beginTransmission(IMU_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("IMU: no response at 0x68");
        imuOK = false;
        return false;
    }

    // Read WHO_AM_I
    uint8_t who = imuRead8(REG_WHO_AM_I);
    Serial.print("IMU WHO_AM_I = 0x");
    Serial.println(who, HEX);

    // Accept known IDs:
    // 0x68 -> genuine MPU6050 / 6500
    // 0x70 -> common clone / variant (ICM / MPU65xx clone)
    if (who == 0x68 || who == 0x70) {
        Serial.println("IMU detected (MPU6050 / MPU65xx clone).");
    } else {
        Serial.println("IMU: unknown WHO_AM_I value, using raw mode anyway.");
    }

    // Wake up device (clear sleep bit)
    imuWrite8(REG_PWR_MGMT_1, 0x00);
    delay(100);

    imuOK = true;
    Serial.println("IMU initialized (raw I2C).");
    return true;
}

bool imuIsOK() {
    return imuOK;
}

bool readIMURaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
    if (!imuOK) return false;

    ax = imuRead16(REG_ACCEL_XOUT_H + 0); // ACCEL_XOUT_H/L
    ay = imuRead16(REG_ACCEL_XOUT_H + 2); // ACCEL_YOUT_H/L
    az = imuRead16(REG_ACCEL_XOUT_H + 4); // ACCEL_ZOUT_H/L

    gx = imuRead16(REG_GYRO_XOUT_H + 0);  // GYRO_XOUT_H/L
    gy = imuRead16(REG_GYRO_XOUT_H + 2);  // GYRO_YOUT_H/L
    gz = imuRead16(REG_GYRO_XOUT_H + 4);  // GYRO_ZOUT_H/L

    return true;
}

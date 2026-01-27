#include "imu_sensor.h"
#include <Wire.h>
#include <math.h>   // for atan2f, sqrtf, PI

// I2C address for MPU devices (6050 / 65xx clones)
static const uint8_t IMU_ADDR          = 0x68;
static const uint8_t REG_PWR_MGMT_1    = 0x6B;
static const uint8_t REG_WHO_AM_I      = 0x75;
static const uint8_t REG_ACCEL_XOUT_H  = 0x3B;
static const uint8_t REG_GYRO_XOUT_H   = 0x43;

static bool imuOK = false;

// ---- Angle estimation state (for complementary filter) ----

// These hold the current estimated angles in degrees.
// They are updated every time imuGetAngles() is called.
static float s_pitchDeg = 0.0f;
static float s_rollDeg  = 0.0f;

// Last time (in microseconds) we updated the IMU angles.
// Used to compute delta time (dt) between samples.
static unsigned long s_lastMicros = 0;

// Typical gyro sensitivity for +/-250 deg/s range:
// 1 LSB ~ 131 units per deg/s.
static const float GYRO_SENS     = 131.0f;

// Typical accel sensitivity for +/-2 g range:
// 1 g ~ 16384 LSB. We only use this indirectly when computing tilt.
static const float ACC_SENS      = 16384.0f;

// Complementary filter coefficient.
// alpha close to 1.0 means:
//   - fast changes from gyro
//   - long-term drift corrected slowly by accel
static const float COMP_ALPHA    = 0.98f;

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
    s_pitchDeg   = 0.0f;
    s_rollDeg    = 0.0f;
    s_lastMicros = 0;

    Serial.println("IMU initialized (raw I2C).");
    return true;
}

bool imuIsOK() {
    return imuOK;
}

bool readIMURaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
    if (!imuOK) return false;

    // Read accelerometer axes (X, Y, Z)
    ax = imuRead16(REG_ACCEL_XOUT_H + 0); // ACCEL_XOUT_H/L
    ay = imuRead16(REG_ACCEL_XOUT_H + 2); // ACCEL_YOUT_H/L
    az = imuRead16(REG_ACCEL_XOUT_H + 4); // ACCEL_ZOUT_H/L

    // Read gyroscope axes (X, Y, Z)
    gx = imuRead16(REG_GYRO_XOUT_H + 0);  // GYRO_XOUT_H/L
    gy = imuRead16(REG_GYRO_XOUT_H + 2);  // GYRO_YOUT_H/L
    gz = imuRead16(REG_GYRO_XOUT_H + 4);  // GYRO_ZOUT_H/L

    return true;
}

bool imuGetAngles(float &pitchDegOut, float &rollDegOut) {
    if (!imuOK) {
        return false;
    }

    int16_t axRaw, ayRaw, azRaw;
    int16_t gxRaw, gyRaw, gzRaw;

    if (!readIMURaw(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw)) {
        return false;
    }

    // ---- Compute delta time in seconds ----
    unsigned long nowMicros = micros();
    float dt = 0.0f;

    if (s_lastMicros == 0) {
        // First call: no previous timestamp available
        dt = 0.0f;
    } else {
        unsigned long diff = nowMicros - s_lastMicros;
        dt = (float)diff / 1e6f; // convert microseconds to seconds
    }
    s_lastMicros = nowMicros;

    // Convert raw accelerometer to float (we keep it in raw units).
    float ax = (float)axRaw;
    float ay = (float)ayRaw;
    float az = (float)azRaw;

    // ---- Compute angles from accelerometer only ----
    //
    // We assume the following axis convention:
    //  - X: forward/backward
    //  - Y: left/right
    //  - Z: up/down
    //
    // roll_acc  -> rotation around X axis, computed from Y and Z
    // pitch_acc -> rotation around Y axis, computed from X, Y, Z
    //
    // These formulas give us the inclination relative to gravity.
    float rollAccRad  = atan2f(ay, az);
    float pitchAccRad = atan2f(-ax, sqrtf(ay * ay + az * az));

    float rollAccDeg  = rollAccRad  * 180.0f / PI;
    float pitchAccDeg = pitchAccRad * 180.0f / PI;

    // ---- Convert gyro raw to deg/s ----
    //
    // Assuming +/-250 deg/s range, where 1 LSB ~ 131 units.
    // If you change the gyro range in the IMU configuration,
    // you must also adjust GYRO_SENS accordingly.
    float gxDegPerSec = (float)gxRaw / GYRO_SENS;
    float gyDegPerSec = (float)gyRaw / GYRO_SENS;
    // gzDegPerSec is currently not used for pitch/roll estimation
    // but can be used to estimate yaw rate if needed.
    // float gzDegPerSec = (float)gzRaw / GYRO_SENS;

    // ---- Complementary filter ----
    //
    // If dt is valid, we:
    //  1) Integrate gyro to predict new angle
    //  2) Blend with accelerometer angle to correct drift
    //
    // If dt is invalid (first call or big glitch), we initialize
    // angles directly from accelerometer to avoid nonsense.
    if (dt > 0.0f && dt < 0.1f) {
        // Predict angles using gyro integration
        float pitchPred = s_pitchDeg + gxDegPerSec * dt;
        float rollPred  = s_rollDeg  + gyDegPerSec * dt;

        // Blend predicted gyro angles with accel angles
        s_pitchDeg = COMP_ALPHA * pitchPred + (1.0f - COMP_ALPHA) * pitchAccDeg;
        s_rollDeg  = COMP_ALPHA * rollPred  + (1.0f - COMP_ALPHA) * rollAccDeg;
    } else {
        // First valid sample: initialize angles from accelerometer
        s_pitchDeg = pitchAccDeg;
        s_rollDeg  = rollAccDeg;
    }

    // Output the current estimated angles
    pitchDegOut = s_pitchDeg;
    rollDegOut  = s_rollDeg;

    return true;
}

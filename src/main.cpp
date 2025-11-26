#include <Arduino.h>
#include "motor_control.h"
#include "bluetooth_control.h"
#include "bme280_sensor.h"
#include "imu_sensor.h"

unsigned long lastBME = 0;
unsigned long lastIMU = 0;

const unsigned long BME_INTERVAL_MS = 2000;
const unsigned long IMU_INTERVAL_MS = 500;

void setup() {
    Serial.begin(115200);
    delay(300);

    initMotors();
    initBluetooth("ESP32_4WD_ROBOT");
    initBME();    // initializes I2C for BME (and IMU shares the same bus)
    initIMU();    // initialize IMU (will print WHO_AM_I, etc.)

    Serial.println("System ready.");
}

void loop() {
    // 1️⃣ Bluetooth motor control
    int left, right;
    if (getMotorCommand(left, right)) {
        drive(left, right);
        Serial.printf("BT CMD -> L=%d R=%d\n", left, right);
    }

    // 2️⃣ Periodic BME280 reading
    if (millis() - lastBME > BME_INTERVAL_MS) {
        lastBME = millis();
        float t, h, p;
        if (readBME(t, h, p)) {
            Serial.printf("BME280 -> T=%.2fC  H=%.2f%%  P=%.2fhPa\n",
                          t, h, p);
        }
    }

    // 3️⃣ Periodic IMU reading (raw ACC/GYRO)
    if (imuIsOK() && (millis() - lastIMU > IMU_INTERVAL_MS)) {
        lastIMU = millis();

        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        if (readIMURaw(ax, ay, az, gx, gy, gz)) {
            Serial.print("IMU -> ACC(");
            Serial.print(ax); Serial.print(", ");
            Serial.print(ay); Serial.print(", ");
            Serial.print(az); Serial.print(")  ");

            Serial.print("GYRO(");
            Serial.print(gx); Serial.print(", ");
            Serial.print(gy); Serial.print(", ");
            Serial.print(gz); Serial.println(")");
        }
    }

    delay(5);
}

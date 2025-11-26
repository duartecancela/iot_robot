#include <Arduino.h>
#include "motor_control.h"
#include "bluetooth_control.h"
#include "bme280_sensor.h"

unsigned long lastBME = 0;

void setup() {
    Serial.begin(115200);
    delay(300);

    initMotors();
    initBluetooth("ESP32_4WD_ROBOT");
    initBME();

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
    if (millis() - lastBME > 2000) {
        lastBME = millis();
        float t, h, p;
        if (readBME(t, h, p)) {
            Serial.printf("BME280 -> T=%.2fC  H=%.2f%%  P=%.2fhPa\n", t, h, p);
        }
    }

    delay(5);
}

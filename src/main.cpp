#include <Arduino.h>
#include "motor_control.h"
#include "bluetooth_control.h"
#include "bme280_sensor.h"
#include "imu_sensor.h"
#include "vl53l0x_sensor.h"

// Timers for periodic tasks
unsigned long lastBME = 0;
unsigned long lastIMU = 0;
unsigned long lastToF = 0;

const unsigned long BME_INTERVAL_MS = 2000;
const unsigned long IMU_INTERVAL_MS = 500;
const unsigned long TOF_INTERVAL_MS = 200;

void setup()
{
    Serial.begin(115200);
    delay(300);

    initMotors();
    initBluetooth("ESP32_4WD_ROBOT");

    // BME280 initializes I2C bus (SDA=21, SCL=22)
    initBME();   // initializes I2C for BME (and IMU + ToF share the same bus)
    initIMU();   // initialize IMU (will print WHO_AM_I, etc.)
    initToFSensors(); // initialize both VL53L0X sensors

    Serial.println("System ready.");
}

void loop()
{
    // 1️⃣ Bluetooth motor control
    int left, right;
    if (getMotorCommand(left, right))
    {
        drive(left, right);
        Serial.printf("BT CMD -> L=%d R=%d\n", left, right);
    }

    // 2️⃣ Periodic BME280 reading
    if (millis() - lastBME > BME_INTERVAL_MS)
    {
        lastBME = millis();
        float t, h, p;
        if (readBME(t, h, p))
        {
            Serial.printf("BME280 -> T=%.2fC  H=%.2f%%  P=%.2fhPa\n",
                          t, h, p);
        }
    }

    // 3️⃣ Periodic IMU reading (pitch/roll angles)
    if (imuIsOK() && (millis() - lastIMU > IMU_INTERVAL_MS))
    {
        lastIMU = millis();

        float pitch, roll;
        if (imuGetAngles(pitch, roll))
        {
            Serial.print("IMU -> pitch=");
            Serial.print(pitch, 2);
            Serial.print(" deg  roll=");
            Serial.print(roll, 2);
            Serial.println(" deg");
        }
    }

    // 4️⃣ Periodic ToF reading (two VL53L0X sensors)
    if (millis() - lastToF > TOF_INTERVAL_MS)
    {
        lastToF = millis();

        uint16_t frontMm, sideMm;
        if (readToFSensors(frontMm, sideMm))
        {
            // Interpret values >= 8000 as "no object"
            Serial.print("ToF -> FRONT: ");
            if (frontMm >= 8000 || frontMm == 0xFFFF)
            {
                Serial.print("NO OBJ");
            }
            else
            {
                Serial.print(frontMm);
                Serial.print(" mm");
            }

            Serial.print("   SIDE: ");
            if (sideMm >= 8000 || sideMm == 0xFFFF)
            {
                Serial.println("NO OBJ");
            }
            else
            {
                Serial.print(sideMm);
                Serial.println(" mm");
            }
        }
    }

    delay(5);
}

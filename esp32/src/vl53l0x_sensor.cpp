#include "vl53l0x_sensor.h"
#include <Wire.h>
#include <VL53L0X.h>

// I2C is already initialized in initBME() with Wire.begin(21, 22).

// XSHUT pins for each sensor (both are at the front: left + right)
static const int XSHUT_FRONT_LEFT_PIN  = 4;  // front-left sensor XSHUT
static const int XSHUT_FRONT_RIGHT_PIN = 5;  // front-right sensor XSHUT

// New I2C addresses for each sensor (must be different)
static const uint8_t ADDR_FRONT_LEFT  = 0x30;
static const uint8_t ADDR_FRONT_RIGHT = 0x31;

static VL53L0X s_frontLeft;
static VL53L0X s_frontRight;

static bool s_frontLeftOK  = false;
static bool s_frontRightOK = false;

bool initToFSensors() {
    // Configure XSHUT pins
    pinMode(XSHUT_FRONT_LEFT_PIN,  OUTPUT);
    pinMode(XSHUT_FRONT_RIGHT_PIN, OUTPUT);

    // Turn both sensors OFF
    digitalWrite(XSHUT_FRONT_LEFT_PIN,  LOW);
    digitalWrite(XSHUT_FRONT_RIGHT_PIN, LOW);
    delay(10);

    // -------- Front-left sensor setup --------
    digitalWrite(XSHUT_FRONT_LEFT_PIN, HIGH);   // enable front-left sensor only
    delay(10);

    if (!s_frontLeft.init()) {
        Serial.println("ERROR: VL53 front-left sensor init() failed!");
        s_frontLeftOK = false;
    } else {
        s_frontLeftOK = true;
        s_frontLeft.setTimeout(500);
        s_frontLeft.setAddress(ADDR_FRONT_LEFT);
        s_frontLeft.setMeasurementTimingBudget(50000); // 50 ms
        Serial.println("VL53 front-left sensor initialized (addr 0x30).");
    }

    // -------- Front-right sensor setup --------
    digitalWrite(XSHUT_FRONT_RIGHT_PIN, HIGH);    // enable front-right sensor
    delay(10);

    if (!s_frontRight.init()) {
        Serial.println("ERROR: VL53 front-right sensor init() failed!");
        s_frontRightOK = false;
    } else {
        s_frontRightOK = true;
        s_frontRight.setTimeout(500);
        s_frontRight.setAddress(ADDR_FRONT_RIGHT);
        s_frontRight.setMeasurementTimingBudget(50000); // 50 ms
        Serial.println("VL53 front-right sensor initialized (addr 0x31).");
    }

    if (!s_frontLeftOK && !s_frontRightOK) {
        Serial.println("WARNING: no VL53L0X sensor initialized.");
        return false;
    }

    return true;
}

bool readToFSensors(uint16_t &frontLeftMm, uint16_t &frontRightMm) {
    bool anyOK = false;

    // Default values if sensors are not OK
    frontLeftMm  = 0xFFFF;
    frontRightMm = 0xFFFF;

    if (s_frontLeftOK) {
        frontLeftMm = s_frontLeft.readRangeSingleMillimeters();
        anyOK = true;
    }

    if (s_frontRightOK) {
        frontRightMm = s_frontRight.readRangeSingleMillimeters();
        anyOK = true;
    }

    return anyOK;
}

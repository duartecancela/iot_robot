#include "vl53l0x_sensor.h"
#include <Wire.h>
#include <VL53L0X.h>

// I2C is already initialized in initBME() with Wire.begin(21, 22).

// XSHUT pins for each sensor
static const int XSHUT_FRONT_PIN = 4;  // front sensor
static const int XSHUT_SIDE_PIN  = 5;  // side sensor

// New I2C addresses for each sensor (must be different)
static const uint8_t ADDR_FRONT = 0x30;
static const uint8_t ADDR_SIDE  = 0x31;

static VL53L0X s_front;
static VL53L0X s_side;

static bool s_frontOK = false;
static bool s_sideOK  = false;

bool initToFSensors() {
    // Configure XSHUT pins
    pinMode(XSHUT_FRONT_PIN, OUTPUT);
    pinMode(XSHUT_SIDE_PIN,  OUTPUT);

    // Turn both sensors OFF
    digitalWrite(XSHUT_FRONT_PIN, LOW);
    digitalWrite(XSHUT_SIDE_PIN,  LOW);
    delay(10);

    // -------- Front sensor setup --------
    digitalWrite(XSHUT_FRONT_PIN, HIGH);   // enable front sensor only
    delay(10);

    if (!s_front.init()) {
        Serial.println("ERROR: VL53 front sensor init() failed!");
        s_frontOK = false;
    } else {
        s_frontOK = true;
        s_front.setTimeout(500);
        s_front.setAddress(ADDR_FRONT);
        s_front.setMeasurementTimingBudget(50000); // 50 ms
        Serial.println("VL53 front sensor initialized (addr 0x30).");
    }

    // -------- Side sensor setup --------
    digitalWrite(XSHUT_SIDE_PIN, HIGH);    // enable side sensor
    delay(10);

    if (!s_side.init()) {
        Serial.println("ERROR: VL53 side sensor init() failed!");
        s_sideOK = false;
    } else {
        s_sideOK = true;
        s_side.setTimeout(500);
        s_side.setAddress(ADDR_SIDE);
        s_side.setMeasurementTimingBudget(50000); // 50 ms
        Serial.println("VL53 side sensor initialized (addr 0x31).");
    }

    if (!s_frontOK && !s_sideOK) {
        Serial.println("WARNING: no VL53L0X sensor initialized.");
        return false;
    }

    return true;
}

bool readToFSensors(uint16_t &frontMm, uint16_t &sideMm) {
    bool anyOK = false;

    // Default values if sensors are not OK
    frontMm = 0xFFFF;
    sideMm  = 0xFFFF;

    if (s_frontOK) {
        frontMm = s_front.readRangeSingleMillimeters();
        anyOK = true;
    }

    if (s_sideOK) {
        sideMm = s_side.readRangeSingleMillimeters();
        anyOK = true;
    }

    return anyOK;
}

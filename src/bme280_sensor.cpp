#include "bme280_sensor.h"
#include <Wire.h>
#include <Adafruit_BME280.h>

static Adafruit_BME280 bme;
static bool bmeOK = false;

bool initBME() {
    Wire.begin(21, 22);
    if (bme.begin(0x76) || bme.begin(0x77)) {
        bmeOK = true;
        Serial.println("BME280 detected.");
    } else {
        Serial.println("WARNING: BME280 not found!");
    }
    return bmeOK;
}

bool readBME(float& t, float& h, float& p) {
    if (!bmeOK) return false;

    t = bme.readTemperature();
    h = bme.readHumidity();
    p = bme.readPressure() / 100.0F;
    return true;
}

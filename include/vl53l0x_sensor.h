#pragma once
#include <Arduino.h>
#include <stdint.h>

// Initialize both VL53L0X sensors on the shared I2C bus.
// Returns true if at least one sensor was initialized.
bool initToFSensors();

// Read both sensors.
// frontMm -> distance from front sensor (ADDR 0x30)
// sideMm  -> distance from side sensor  (ADDR 0x31)
//
// If a sensor is not OK, its value will be set to 0xFFFF.
// If the sensor returns an "out of range" value (>= 8000 mm),
// the raw distance is still returned and should be interpreted
// as "no object".
//
// Returns true if at least one sensor is OK.
bool readToFSensors(uint16_t &frontMm, uint16_t &sideMm);

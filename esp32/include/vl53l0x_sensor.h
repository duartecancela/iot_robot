#pragma once
#include <Arduino.h>
#include <stdint.h>

// Initialize both VL53L0X sensors on the shared I2C bus.
// Returns true if at least one sensor was initialized.
bool initToFSensors();

// Read both front sensors.
// frontLeftMm  -> distance from FRONT-LEFT sensor  (ADDR 0x30)
// frontRightMm -> distance from FRONT-RIGHT sensor (ADDR 0x31)
//
// If a sensor is not OK, its value will be set to 0xFFFF.
// If the sensor returns an "out of range" value (>= 8000 mm),
// the raw distance is still returned and should be interpreted
// as "no object".
//
// Returns true if at least one sensor is OK.
bool readToFSensors(uint16_t &frontLeftMm, uint16_t &frontRightMm);


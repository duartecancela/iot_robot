#pragma once
#include <Arduino.h>

// Initializes Bluetooth with a given device name
void initBluetooth(const char* deviceName);

// Returns true when a valid motor command "L,R" is received
bool getMotorCommand(int& left, int& right);

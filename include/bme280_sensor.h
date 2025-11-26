#pragma once
#include <Arduino.h>

bool initBME();
bool readBME(float& temperature, float& humidity, float& pressure);

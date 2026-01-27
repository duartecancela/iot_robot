#pragma once
#include <Arduino.h>

// Init PWM and attach pins
void initMotors();

// Individual motor control (raw PWM 0–255)
void leftNeutral();
void rightNeutral();
void leftForward(uint8_t duty);
void leftReverse(uint8_t duty);
void rightForward(uint8_t duty);
void rightReverse(uint8_t duty);
void stopAll();

// Drive both motors with signed PWM (-255..255)
// >0 = forward, <0 = reverse, 0 = stop
void drive(int16_t leftPwm, int16_t rightPwm);

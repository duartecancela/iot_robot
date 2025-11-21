#include <Arduino.h>
#include "motor_control.h"

// Simple test params
const uint16_t STEP_DELAY    = 5;
const uint16_t NEUTRAL_DELAY = 300;
const uint16_t PAUSE_DELAY   = 500;

void setup() {
  Serial.begin(115200);
  Serial.println("Init");
  initMotors();
}

void loop() {
  // Right motor only (forward ramp)
  for (int v = 0; v <= 255; v++) {
    drive(0, v);           // left=0, right=v
    delay(STEP_DELAY);
  }
  for (int v = 255; v >= 0; v--) {
    drive(0, v);
    delay(STEP_DELAY);
  }
  drive(0, 0);
  delay(NEUTRAL_DELAY);

  // Right motor only (reverse ramp)
  for (int v = 0; v <= 255; v++) {
    drive(0, -v);
    delay(STEP_DELAY);
  }
  for (int v = 255; v >= 0; v--) {
    drive(0, -v);
    delay(STEP_DELAY);
  }
  drive(0, 0);
  delay(PAUSE_DELAY);

  // Left motor only (forward ramp)
  for (int v = 0; v <= 255; v++) {
    drive(v, 0);           // left=v, right=0
    delay(STEP_DELAY);
  }
  for (int v = 255; v >= 0; v--) {
    drive(v, 0);
    delay(STEP_DELAY);
  }
  drive(0, 0);
  delay(NEUTRAL_DELAY);

  // Left motor only (reverse ramp)
  for (int v = 0; v <= 255; v++) {
    drive(-v, 0);
    delay(STEP_DELAY);
  }
  for (int v = 255; v >= 0; v--) {
    drive(-v, 0);
    delay(STEP_DELAY);
  }
  drive(0, 0);
  delay(PAUSE_DELAY);
}

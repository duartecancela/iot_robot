#include <Arduino.h>

/*
  RIGHT MOTOR DRIVER (BTS7960) — RIGHT SIDE ONLY (ESP32 classic)
  - EN pins are hard-wired to +5V (no GPIO enables).
  - Direction convention for BTS7960:
      * Forward  = R_PWM > 0, L_PWM = 0
      * Reverse  = L_PWM > 0, R_PWM = 0
  - SAFETY: Never drive L_PWM and R_PWM > 0 simultaneously.
*/

// ===== Safe pin mapping for ESP32 (RIGHT driver) =====
// Use non-strapping, LEDC-capable pins.
const int RIGHT_LPWM_PIN = 25;   // reverse (L_PWM)
const int RIGHT_RPWM_PIN = 26;   // forward (R_PWM)

// ===== LEDC (PWM) configuration =====
const uint32_t PWM_FREQ_HZ   = 20000; // 20 kHz (above audible range)
const uint8_t  PWM_RES_BITS  = 8;     // 0..255
const uint8_t  CH_RIGHT_LPWM = 2;     // channel for RIGHT L_PWM
const uint8_t  CH_RIGHT_RPWM = 3;     // channel for RIGHT R_PWM

// ===== Test profile =====
const uint8_t  MAX_DUTY      = 255;   // max duty for 8-bit resolution
const uint16_t RAMP_STEP_MS  = 5;     // ramp step delay
const uint16_t NEUTRAL_MS    = 300;   // neutral dwell
const uint16_t DEADTIME_MS   = 50;    // short dead-time before direction change

inline void rightNeutral() {
  // Immediately stop: both channels to 0 (coast)
  ledcWrite(CH_RIGHT_RPWM, 0);
  ledcWrite(CH_RIGHT_LPWM, 0);
}

inline void rightForward(uint8_t duty) {
  // Forward: R_PWM drives, L_PWM forced to 0
  ledcWrite(CH_RIGHT_LPWM, 0);
  ledcWrite(CH_RIGHT_RPWM, duty);
}

inline void rightReverse(uint8_t duty) {
  // Reverse: L_PWM drives, R_PWM forced to 0
  ledcWrite(CH_RIGHT_RPWM, 0);
  ledcWrite(CH_RIGHT_LPWM, duty);
}

void setup() {
  // Configure PWM channels
  ledcSetup(CH_RIGHT_RPWM, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(CH_RIGHT_LPWM, PWM_FREQ_HZ, PWM_RES_BITS);

  // Attach pins
  ledcAttachPin(RIGHT_RPWM_PIN, CH_RIGHT_RPWM);
  ledcAttachPin(RIGHT_LPWM_PIN, CH_RIGHT_LPWM);

  // Start neutral (both off)
  rightNeutral();
}

void loop() {
  // ===== Forward ramp (RIGHT side) =====
  rightNeutral();
  delay(DEADTIME_MS);

  for (int v = 0; v <= MAX_DUTY; ++v) { rightForward((uint8_t)v); delay(RAMP_STEP_MS); }
  for (int v = MAX_DUTY; v >= 0; --v) { rightForward((uint8_t)v); delay(RAMP_STEP_MS); }

  rightNeutral();
  delay(NEUTRAL_MS);

  // ===== Reverse ramp (RIGHT side) =====
  rightNeutral();
  delay(DEADTIME_MS);

  for (int v = 0; v <= MAX_DUTY; ++v) { rightReverse((uint8_t)v); delay(RAMP_STEP_MS); }
  for (int v = MAX_DUTY; v >= 0; --v) { rightReverse((uint8_t)v); delay(RAMP_STEP_MS); }

  rightNeutral();
  delay(500);
}

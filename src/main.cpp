#include <Arduino.h>

/*
  DUAL MOTOR DRIVER TEST — BTS7960 RIGHT & LEFT
  -------------------------------------------------------
  Each side uses two PWM lines:
    * R_PWM -> Forward
    * L_PWM -> Reverse
  EN pins are hard-wired to +5 V (always enabled).
  Safe logic: never drive R_PWM and L_PWM > 0 simultaneously.
*/

// ===== Pin mapping for RIGHT driver =====
const int RIGHT_LPWM_PIN = 25;   // reverse  (L_PWM)
const int RIGHT_RPWM_PIN = 26;   // forward  (R_PWM)

// ===== Pin mapping for LEFT driver =====
const int LEFT_LPWM_PIN  = 27;   // reverse  (L_PWM)
const int LEFT_RPWM_PIN  = 14;   // forward  (R_PWM)

// ===== LEDC (PWM) configuration =====
const uint32_t PWM_FREQ_HZ   = 20000; // 20 kHz to avoid audible noise
const uint8_t  PWM_RES_BITS  = 8;     // 0..255 duty range
// Each PWM pin needs a unique channel (0–7 on ESP32)
const uint8_t  CH_RIGHT_LPWM = 0;
const uint8_t  CH_RIGHT_RPWM = 1;
const uint8_t  CH_LEFT_LPWM  = 2;
const uint8_t  CH_LEFT_RPWM  = 3;

// ===== Test parameters =====
const uint8_t  MAX_DUTY      = 255;
const uint16_t RAMP_STEP_MS  = 5;
const uint16_t NEUTRAL_MS    = 300;
const uint16_t DEADTIME_MS   = 50;

// --- Helper functions for RIGHT motor ---
inline void rightNeutral() {
  ledcWrite(CH_RIGHT_RPWM, 0);
  ledcWrite(CH_RIGHT_LPWM, 0);
}

inline void rightForward(uint8_t duty) {
  ledcWrite(CH_RIGHT_LPWM, 0);
  ledcWrite(CH_RIGHT_RPWM, duty);
}

inline void rightReverse(uint8_t duty) {
  ledcWrite(CH_RIGHT_RPWM, 0);
  ledcWrite(CH_RIGHT_LPWM, duty);
}

// --- Helper functions for LEFT motor ---
inline void leftNeutral() {
  ledcWrite(CH_LEFT_RPWM, 0);
  ledcWrite(CH_LEFT_LPWM, 0);
}

inline void leftForward(uint8_t duty) {
  ledcWrite(CH_LEFT_LPWM, 0);
  ledcWrite(CH_LEFT_RPWM, duty);
}

inline void leftReverse(uint8_t duty) {
  ledcWrite(CH_LEFT_RPWM, 0);
  ledcWrite(CH_LEFT_LPWM, duty);
}

// =====================================================

void setup() {
  // Configure all PWM channels
  ledcSetup(CH_RIGHT_RPWM, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(CH_RIGHT_LPWM, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(CH_LEFT_RPWM,  PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(CH_LEFT_LPWM,  PWM_FREQ_HZ, PWM_RES_BITS);

  // Attach pins to channels
  ledcAttachPin(RIGHT_RPWM_PIN, CH_RIGHT_RPWM);
  ledcAttachPin(RIGHT_LPWM_PIN, CH_RIGHT_LPWM);
  ledcAttachPin(LEFT_RPWM_PIN,  CH_LEFT_RPWM);
  ledcAttachPin(LEFT_LPWM_PIN,  CH_LEFT_LPWM);

  // Start neutral (motors off)
  rightNeutral();
  leftNeutral();

  Serial.begin(115200);
  Serial.println("Starting dual BTS7960 motor test...");
}

// =====================================================

void loop() {
  // ===== RIGHT DRIVER TEST =====
  Serial.println("Testing RIGHT motor: forward");
  rightNeutral(); delay(DEADTIME_MS);
  for (int v = 0; v <= MAX_DUTY; ++v) { rightForward(v); delay(RAMP_STEP_MS); }
  for (int v = MAX_DUTY; v >= 0; --v) { rightForward(v); delay(RAMP_STEP_MS); }

  rightNeutral(); delay(NEUTRAL_MS);

  Serial.println("Testing RIGHT motor: reverse");
  rightNeutral(); delay(DEADTIME_MS);
  for (int v = 0; v <= MAX_DUTY; ++v) { rightReverse(v); delay(RAMP_STEP_MS); }
  for (int v = MAX_DUTY; v >= 0; --v) { rightReverse(v); delay(RAMP_STEP_MS); }

  rightNeutral(); delay(500);

  // ===== LEFT DRIVER TEST =====
  Serial.println("Testing LEFT motor: forward");
  leftNeutral(); delay(DEADTIME_MS);
  for (int v = 0; v <= MAX_DUTY; ++v) { leftForward(v); delay(RAMP_STEP_MS); }
  for (int v = MAX_DUTY; v >= 0; --v) { leftForward(v); delay(RAMP_STEP_MS); }

  leftNeutral(); delay(NEUTRAL_MS);

  Serial.println("Testing LEFT motor: reverse");
  leftNeutral(); delay(DEADTIME_MS);
  for (int v = 0; v <= MAX_DUTY; ++v) { leftReverse(v); delay(RAMP_STEP_MS); }
  for (int v = MAX_DUTY; v >= 0; --v) { leftReverse(v); delay(RAMP_STEP_MS); }

  leftNeutral(); delay(1000);
}

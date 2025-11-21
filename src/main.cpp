#include <Arduino.h>

/* BTS7960 direct PWM control (0–255)
   RPWM = forward, LPWM = reverse */

const int RIGHT_LPWM_PIN = 25;
const int RIGHT_RPWM_PIN = 26;
const int LEFT_LPWM_PIN  = 27;
const int LEFT_RPWM_PIN  = 14;

const uint32_t PWM_FREQ = 20000;
const uint8_t  PWM_RES  = 8;

const uint8_t CH_RIGHT_LPWM = 0;
const uint8_t CH_RIGHT_RPWM = 1;
const uint8_t CH_LEFT_LPWM  = 2;
const uint8_t CH_LEFT_RPWM  = 3;

// ---- basic motor controls ----

void leftNeutral() {
  Serial.println("L N"); // Left Neutral
  ledcWrite(CH_LEFT_LPWM, 0);
  ledcWrite(CH_LEFT_RPWM, 0);
}

void rightNeutral() {
  Serial.println("R N"); // Right Neutral
  ledcWrite(CH_RIGHT_LPWM, 0);
  ledcWrite(CH_RIGHT_RPWM, 0);
}

void leftForward(uint8_t duty) {
  Serial.print("L F "); Serial.println(duty);
  ledcWrite(CH_LEFT_LPWM, 0);
  ledcWrite(CH_LEFT_RPWM, duty);
}

void leftReverse(uint8_t duty) {
  Serial.print("L R "); Serial.println(duty);
  ledcWrite(CH_LEFT_RPWM, 0);
  ledcWrite(CH_LEFT_LPWM, duty);
}

void rightForward(uint8_t duty) {
  Serial.print("R F "); Serial.println(duty);
  ledcWrite(CH_RIGHT_LPWM, 0);
  ledcWrite(CH_RIGHT_RPWM, duty);
}

void rightReverse(uint8_t duty) {
  Serial.print("R R "); Serial.println(duty);
  ledcWrite(CH_RIGHT_RPWM, 0);
  ledcWrite(CH_RIGHT_LPWM, duty);
}

void stopAll() {
  Serial.println("STOP");
  leftNeutral();
  rightNeutral();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Init");

  ledcSetup(CH_RIGHT_RPWM, PWM_FREQ, PWM_RES);
  ledcSetup(CH_RIGHT_LPWM, PWM_FREQ, PWM_RES);
  ledcSetup(CH_LEFT_RPWM,  PWM_FREQ, PWM_RES);
  ledcSetup(CH_LEFT_LPWM,  PWM_FREQ, PWM_RES);

  ledcAttachPin(RIGHT_RPWM_PIN, CH_RIGHT_RPWM);
  ledcAttachPin(RIGHT_LPWM_PIN, CH_RIGHT_LPWM);
  ledcAttachPin(LEFT_RPWM_PIN,  CH_LEFT_RPWM);
  ledcAttachPin(LEFT_LPWM_PIN,  CH_LEFT_LPWM);

  stopAll();
}

void loop() {

  // Right forward ramp
  for (int v = 0; v <= 255; v++) { rightForward(v); delay(5); }
  for (int v = 255; v >= 0; v--) { rightForward(v); delay(5); }
  rightNeutral(); delay(300);

  // Right reverse ramp
  for (int v = 0; v <= 255; v++) { rightReverse(v); delay(5); }
  for (int v = 255; v >= 0; v--) { rightReverse(v); delay(5); }
  rightNeutral(); delay(300);

  // Left forward ramp
  for (int v = 0; v <= 255; v++) { leftForward(v); delay(5); }
  for (int v = 255; v >= 0; v--) { leftForward(v); delay(5); }
  leftNeutral(); delay(300);

  // Left reverse ramp
  for (int v = 0; v <= 255; v++) { leftReverse(v); delay(5); }
  for (int v = 255; v >= 0; v--) { leftReverse(v); delay(5); }
  leftNeutral(); delay(500);
}

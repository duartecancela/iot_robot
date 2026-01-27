#include <Arduino.h>
#include "motor_control.h"

/* BTS7960 direct PWM control
   RPWM = forward, LPWM = reverse */

static const int RIGHT_LPWM_PIN = 25;
static const int RIGHT_RPWM_PIN = 26;
static const int LEFT_LPWM_PIN  = 27;
static const int LEFT_RPWM_PIN  = 14;

static const uint32_t PWM_FREQ = 20000;
static const uint8_t  PWM_RES  = 8;

static const uint8_t CH_RIGHT_LPWM = 0;
static const uint8_t CH_RIGHT_RPWM = 1;
static const uint8_t CH_LEFT_LPWM  = 2;
static const uint8_t CH_LEFT_RPWM  = 3;

// Basic motor controls

void leftNeutral() {
  Serial.println("L N");
  ledcWrite(CH_LEFT_LPWM, 0);
  ledcWrite(CH_LEFT_RPWM, 0);
}

void rightNeutral() {
  Serial.println("R N");
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

// Signed drive: -255..255 → dir + duty
void drive(int16_t leftPwm, int16_t rightPwm) {
  if (leftPwm > 255)  leftPwm = 255;
  if (leftPwm < -255) leftPwm = -255;
  if (rightPwm > 255)  rightPwm = 255;
  if (rightPwm < -255) rightPwm = -255;

  // left
  if (leftPwm > 0) {
    leftForward((uint8_t)leftPwm);
  } else if (leftPwm < 0) {
    leftReverse((uint8_t)(-leftPwm));
  } else {
    leftNeutral();
  }

  // right
  if (rightPwm > 0) {
    rightForward((uint8_t)rightPwm);
  } else if (rightPwm < 0) {
    rightReverse((uint8_t)(-rightPwm));
  } else {
    rightNeutral();
  }
}

// PWM + pins init
void initMotors() {
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

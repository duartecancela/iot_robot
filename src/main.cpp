#include <Arduino.h>

#define RPWM_PIN 48   // pino PWM direita
#define LPWM_PIN 47   // pino PWM esquerda
#define R_EN 46       // enable direita
#define L_EN 45       // enable esquerda

#define FREQ 1000     // frequência PWM
#define RES 8         // resolução 8 bits (0–255)

void setup() {
  // configura PWM
  ledcSetup(0, FREQ, RES);  // canal 0 para RPWM
  ledcSetup(1, FREQ, RES);  // canal 1 para LPWM
  ledcAttachPin(RPWM_PIN, 0);
  ledcAttachPin(LPWM_PIN, 1);

  // ativa enable
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
}

void loop() {
  // Sentido 1
  for (int v = 0; v <= 255; v++) {
    ledcWrite(0, v);   // aumenta velocidade para frente
    ledcWrite(1, 0);
    delay(5);
  }

  for (int v = 255; v >= 0; v--) {
    ledcWrite(0, v);   // diminui velocidade
    ledcWrite(1, 0);
    delay(5);
  }

  delay(500);

  // Sentido 2
  for (int v = 0; v <= 255; v++) {
    ledcWrite(1, v);   // aumenta velocidade para trás
    ledcWrite(0, 0);
    delay(5);
  }

  for (int v = 255; v >= 0; v--) {
    ledcWrite(1, v);   // diminui velocidade
    ledcWrite(0, 0);
    delay(5);
  }

  delay(1000);
}

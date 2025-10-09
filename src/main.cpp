#include <Arduino.h>

#define RPWM_PIN 48   // PWM direita
#define LPWM_PIN 47   // PWM esquerda
#define R_EN 38       // enable direita
#define L_EN 39       // enable esquerda

#define FREQ 20000    // 20 kHz para reduzir zumbido (podes deixar 1000)
#define RES 8         // 0..255

void setup() {
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);

  ledcSetup(0, FREQ, RES);  // canal 0 -> RPWM
  ledcSetup(1, FREQ, RES);  // canal 1 -> LPWM
  ledcAttachPin(RPWM_PIN, 0);
  ledcAttachPin(LPWM_PIN, 1);

  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

void loop() {
  // ---- Sentido 1 (frente): ativa só R_EN ----
  digitalWrite(L_EN, LOW);
  digitalWrite(R_EN, HIGH);
  for (int v = 0; v <= 255; v++) { ledcWrite(0, v); ledcWrite(1, 0); delay(5); }
  for (int v = 255; v >= 0; v--) { ledcWrite(0, v); ledcWrite(1, 0); delay(5); }

  // Neutro (tudo a 0) antes de inverter
  ledcWrite(0, 0); ledcWrite(1, 0);
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
  delay(300);

  // ---- Sentido 2 (trás): ativa só L_EN ----
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, HIGH);
  for (int v = 0; v <= 255; v++) { ledcWrite(1, v); ledcWrite(0, 0); delay(5); }
  for (int v = 255; v >= 0; v--) { ledcWrite(1, v); ledcWrite(0, 0); delay(5); }

  // Neutro outra vez
  ledcWrite(0, 0); ledcWrite(1, 0);
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
  delay(500);
}

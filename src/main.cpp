#include <Arduino.h>
#include "motor_control.h"
#include "BluetoothSerial.h"

// Bluetooth serial instance (ESP32 classic Bluetooth)
BluetoothSerial SerialBT;

const char* DEVICE_NAME = "ESP32_4WD_ROBOT";

// Helper: clamp values to valid PWM range
int clampSpeed(int v) {
  if (v > 255) return 255;
  if (v < -255) return -255;
  return v;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("Initializing motors...");
  initMotors();

  Serial.println("Starting Bluetooth (numeric mode)...");
  SerialBT.begin(DEVICE_NAME);

  Serial.print("Bluetooth device name: ");
  Serial.println(DEVICE_NAME);

  Serial.println("Numeric control mode enabled.");
  Serial.println("Send commands as: L,R");
  Serial.println("Example: 150,150   (forward)");
  Serial.println("         -150,-150 (reverse)");
  Serial.println("         150,-150  (turn)");
  Serial.println("         0,0       (stop)");
}

void loop() {
  // Check if data is available from Bluetooth
  if (SerialBT.available()) {
    // Read one line until newline '\n'
    String line = SerialBT.readStringUntil('\n');
    line.trim();  // Remove spaces, \r, etc.

    if (line.length() == 0) {
      return;  // Ignore empty lines
    }

    // Expected format: "L,R"
    int commaIndex = line.indexOf(',');
    if (commaIndex < 0) {
      SerialBT.println("ERR: use format L,R  (ex: 150,-100)");
      Serial.print("Invalid command: ");
      Serial.println(line);
      return;
    }

    // Split into left and right parts
    String leftStr  = line.substring(0, commaIndex);
    String rightStr = line.substring(commaIndex + 1);

    int left  = clampSpeed(leftStr.toInt());
    int right = clampSpeed(rightStr.toInt());

    // Drive motors using signed PWM values
    drive(left, right);

    // Debug over USB serial
    Serial.print("BT CMD -> L: ");
    Serial.print(left);
    Serial.print("  R: ");
    Serial.println(right);

    // Feedback to Bluetooth terminal
    SerialBT.print("OK L=");
    SerialBT.print(left);
    SerialBT.print(" R=");
    SerialBT.println(right);
  }

  delay(5);  // small delay to avoid busy loop
}

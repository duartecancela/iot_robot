#include <Arduino.h>
#include "motor_control.h"
#include "BluetoothSerial.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// -------------------- Bluetooth --------------------
BluetoothSerial SerialBT;
const char* DEVICE_NAME = "ESP32_4WD_ROBOT";

// -------------------- BME280 --------------------
Adafruit_BME280 bme;
bool bmeOK = false;
const uint8_t BME_ADDR_1 = 0x76;
const uint8_t BME_ADDR_2 = 0x77;

// How often to read sensor (ms)
const unsigned long BME_INTERVAL_MS = 2000;
unsigned long lastBmeRead = 0;

// -------------------- Helpers --------------------
int clampSpeed(int v) {
  if (v > 255) return 255;
  if (v < -255) return -255;
  return v;
}

void printBmeValues() {
  if (!bmeOK) return;

  float t = bme.readTemperature();           // °C
  float h = bme.readHumidity();              // %
  float p = bme.readPressure() / 100.0F;     // hPa

  // ONLY print to USB Serial
  Serial.print("BME280 -> T=");
  Serial.print(t);
  Serial.print(" °C  H=");
  Serial.print(h);
  Serial.print(" %  P=");
  Serial.print(p);
  Serial.println(" hPa");
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("Initializing motors...");
  initMotors();

  // I2C for BME280 (ESP32 default pins: SDA=21, SCL=22)
  Wire.begin(21, 22);

  Serial.println("Initializing BME280...");
  if (bme.begin(BME_ADDR_1) || bme.begin(BME_ADDR_2)) {
    bmeOK = true;
    Serial.println("BME280 detected.");
  } else {
    bmeOK = false;
    Serial.println("WARNING: BME280 not found!");
  }

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

// -------------------- Loop --------------------
void loop() {
  // ---- Handle Bluetooth motor commands ----
  if (SerialBT.available()) {
    String line = SerialBT.readStringUntil('\n');
    line.trim();

    if (line.length() > 0) {
      int commaIndex = line.indexOf(',');
      if (commaIndex < 0) {
        SerialBT.println("ERR: use format L,R  (ex: 150,-100)");
        Serial.print("Invalid command: ");
        Serial.println(line);
      } else {
        String leftStr  = line.substring(0, commaIndex);
        String rightStr = line.substring(commaIndex + 1);

        int left  = clampSpeed(leftStr.toInt());
        int right = clampSpeed(rightStr.toInt());

        drive(left, right);

        Serial.print("BT CMD -> L: ");
        Serial.print(left);
        Serial.print("  R: ");
        Serial.println(right);

        // ONLY acknowledge command (no BME data!)
        SerialBT.print("OK L=");
        SerialBT.print(left);
        SerialBT.print(" R=");
        SerialBT.println(right);
      }
    }
  }

  // ---- Periodic BME280 reading ----
  unsigned long now = millis();
  if (bmeOK && (now - lastBmeRead >= BME_INTERVAL_MS)) {
    lastBmeRead = now;
    printBmeValues();   // prints only to USB Serial
  }

  delay(5);
}

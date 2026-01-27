#include "bluetooth_control.h"
#include <BluetoothSerial.h>

static BluetoothSerial SerialBT;

void initBluetooth(const char* deviceName) {
    SerialBT.begin(deviceName);
    Serial.print("Bluetooth started: ");
    Serial.println(deviceName);
}

bool getMotorCommand(int& left, int& right) {
    if (!SerialBT.available()) return false;

    String line = SerialBT.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return false;

    int commaIndex = line.indexOf(',');
    if (commaIndex < 0) {
        SerialBT.println("ERR: use format L,R");
        return false;
    }

    left  = constrain(line.substring(0, commaIndex).toInt(),  -255, 255);
    right = constrain(line.substring(commaIndex + 1).toInt(), -255, 255);

    SerialBT.print("OK L=");
    SerialBT.print(left);
    SerialBT.print(" R=");
    SerialBT.println(right);

    return true;
}

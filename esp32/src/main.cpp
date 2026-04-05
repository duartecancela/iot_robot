// main.cpp

#include <Arduino.h>
#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "motor_control.h"
#include "bluetooth_control.h"
#include "bme280_sensor.h"
#include "imu_sensor.h"
#include "vl53l0x_sensor.h"

// Timers for periodic tasks
unsigned long lastBME = 0;
unsigned long lastIMU = 0;
unsigned long lastToF = 0;

// Sensor read intervals
const unsigned long BME_INTERVAL_MS = 5000;
const unsigned long IMU_INTERVAL_MS = 100;
const unsigned long TOF_INTERVAL_MS = 100;

// MQTT publish intervals
unsigned long lastFastPub = 0;
unsigned long lastSlowPub = 0;
const unsigned long FAST_PUB_MS = 200;
const unsigned long SLOW_PUB_MAX_MS = 30000;

// BME publish thresholds
const float BME_DELTA_T = 0.2f;  // °C
const float BME_DELTA_H = 1.0f;  // %
const float BME_DELTA_P = 1.0f;  // hPa

// Cached latest sensor values
float lastT = NAN, lastH = NAN, lastP = NAN;
float lastPitch = NAN, lastRoll = NAN;
uint16_t lastFrontLeftMm = 0xFFFF, lastFrontRightMm = 0xFFFF;

// Last published BME values
float pubT = NAN, pubH = NAN, pubP = NAN;

// Last applied motor values (what the robot is effectively using right now)
static int appliedLeft = 0;
static int appliedRight = 0;

static inline bool isInvalidU16(uint16_t v) { return (v == 0xFFFF); }
static inline bool isNoObjToF(uint16_t v) { return (isInvalidU16(v) || v >= 8000); }
static inline float fAbs(float x) { return (x < 0) ? -x : x; }

static inline bool isRobotMoving()
{
    return (appliedLeft != 0 || appliedRight != 0);
}

static bool shouldPublishBME(float t, float h, float p, unsigned long now)
{
    if (isnan(pubT) || isnan(pubH) || isnan(pubP)) return true;

    if (fAbs(t - pubT) >= BME_DELTA_T) return true;
    if (fAbs(h - pubH) >= BME_DELTA_H) return true;
    if (fAbs(p - pubP) >= BME_DELTA_P) return true;

    if (now - lastSlowPub >= SLOW_PUB_MAX_MS) return true;

    return false;
}

// Publishes the current applied motor values so the dashboard can show "Applied"
static void publishAppliedDriveState()
{
    if (!mqttIsConnected()) return;

    char payload[160];
    unsigned long ts = millis();
    bool moving = isRobotMoving();

    snprintf(payload, sizeof(payload),
             "{\"left\":%d,\"right\":%d,\"moving\":%s,\"ts\":%lu}",
             appliedLeft,
             appliedRight,
             moving ? "true" : "false",
             ts);

    // Retained state: any client gets the latest applied motor state immediately
    mqttPublish("robot/state/drive", payload, true);
}

/*
 * MQTT -> motor command hook
 * - This function is called by mqtt_manager.cpp when a valid drive command arrives over MQTT.
 * - mqtt_manager.cpp already publishes ack for MQTT commands.
 * - Here we apply the motors, keep a local cache of what's applied,
 *   and publish the resulting applied state.
 */
void onMqttDriveCommand(int left, int right)
{
    drive(left, right);
    appliedLeft = left;
    appliedRight = right;
    Serial.printf("MQTT CMD -> L=%d R=%d\n", left, right);

    // Publish applied state so other components know whether the robot is moving
    publishAppliedDriveState();
}

void setup()
{
    Serial.begin(115200);

    // Add BOTH networks (work + home)
    addWiFiNetwork(WIFI_SSID_PRIMARY, WIFI_PASS_PRIMARY);
    addWiFiNetwork(WIFI_SSID_SECONDARY, WIFI_PASS_SECONDARY);

    initWiFiMulti();

    initMQTT(MQTT_HOST, MQTT_PORT);
    delay(300);

    initMotors();
    initBluetooth("ESP32_4WD_ROBOT");

    // BME280 initializes I2C bus (SDA=21, SCL=22)
    initBME();
    initIMU();
    initToFSensors();

    Serial.println("System ready.");
}

void loop()
{
    wifiTask();

    // Print connection info once (per boot)
    static bool printed = false;
    if (!printed && wifiIsConnected())
    {
        printed = true;
        Serial.print("WiFi connected. SSID=");
        Serial.print(wifiSsid());
        Serial.print(" IP=");
        Serial.println(wifiIp());
    }

    mqttTask();

    const unsigned long now = millis();

    // Bluetooth motor control
    int left, right;
    if (getMotorCommand(left, right))
    {
        drive(left, right);
        appliedLeft = left;
        appliedRight = right;

        Serial.printf("BT CMD -> L=%d R=%d\n", left, right);

        // Also publish applied state so the UI shows the real motor values even when using Bluetooth
        publishAppliedDriveState();
    }

    // Periodic BME280 reading
    if (now - lastBME >= BME_INTERVAL_MS)
    {
        lastBME = now;
        float t, h, p;
        if (readBME(t, h, p))
        {
            lastT = t;
            lastH = h;
            lastP = p;

            // Serial output for environmental telemetry
            Serial.printf("BME280 -> T=%.2fC  H=%.2f%%  P=%.2fhPa\n", t, h, p);
        }
    }

    // Periodic IMU reading
    if (imuIsOK() && (now - lastIMU >= IMU_INTERVAL_MS))
    {
        lastIMU = now;
        float pitch, roll;
        if (imuGetAngles(pitch, roll))
        {
            lastPitch = pitch;
            lastRoll = roll;

            // Serial output for IMU orientation telemetry
            Serial.print("IMU -> pitch=");
            Serial.print(pitch, 2);
            Serial.print(" deg  roll=");
            Serial.print(roll, 2);
            Serial.println(" deg");
        }
    }

    // Periodic ToF reading
    if (now - lastToF >= TOF_INTERVAL_MS)
    {
        lastToF = now;
        uint16_t fl, fr;
        if (readToFSensors(fl, fr))
        {
            lastFrontLeftMm = fl;
            lastFrontRightMm = fr;

            // Serial output for obstacle distance telemetry
            Serial.print("ToF -> FRONT-LEFT: ");
            if (isNoObjToF(fl))
            {
                Serial.print("NO OBJ");
            }
            else
            {
                Serial.print(fl);
                Serial.print(" mm");
            }

            Serial.print("   FRONT-RIGHT: ");
            if (isNoObjToF(fr))
            {
                Serial.println("NO OBJ");
            }
            else
            {
                Serial.print(fr);
                Serial.println(" mm");
            }
        }
    }

    // FAST telemetry topic: ToF and IMU snapshot at fixed publish rate
    if (mqttIsConnected() && (now - lastFastPub >= FAST_PUB_MS))
    {
        lastFastPub = now;

        char payload[200];

        int fl = isNoObjToF(lastFrontLeftMm) ? -1 : (int)lastFrontLeftMm;
        int fr = isNoObjToF(lastFrontRightMm) ? -1 : (int)lastFrontRightMm;

        if (isnan(lastPitch) || isnan(lastRoll))
        {
            snprintf(payload, sizeof(payload),
                     "{\"tof\":{\"fl\":%d,\"fr\":%d},\"imu\":null}",
                     fl, fr);
        }
        else
        {
            snprintf(payload, sizeof(payload),
                     "{\"tof\":{\"fl\":%d,\"fr\":%d},\"imu\":{\"pitch\":%.2f,\"roll\":%.2f}}",
                     fl, fr, lastPitch, lastRoll);
        }

        mqttPublish("robot/telemetry/fast", payload);
    }

    // SLOW telemetry topic: environmental data with change-based publishing
    if (mqttIsConnected() && !isnan(lastT) && !isnan(lastH) && !isnan(lastP))
    {
        if (shouldPublishBME(lastT, lastH, lastP, now))
        {
            lastSlowPub = now;
            pubT = lastT;
            pubH = lastH;
            pubP = lastP;

            char payload[160];
            snprintf(payload, sizeof(payload),
                     "{\"bme\":{\"t\":%.2f,\"h\":%.2f,\"p\":%.2f}}",
                     lastT, lastH, lastP);

            mqttPublish("robot/telemetry/slow", payload);
        }
    }

    // Cooperative yield for background networking tasks
    delay(1);
}
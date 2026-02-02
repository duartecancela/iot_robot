#include "mqtt_manager.h"
#include <WiFi.h>
#include <PubSubClient.h>

/*
 * IMPORTANT DESIGN NOTE
 * ---------------------
 * We intentionally avoid changing mqtt_manager.h to preserve the existing API
 * and avoid breaking other compilation units.
 *
 * The MQTT "drive command" is delivered to the application via this hook.
 * main.cpp must define:
 *   void onMqttDriveCommand(int left, int right);
 */
extern void onMqttDriveCommand(int left, int right);

static WiFiClient wifiClient;
static PubSubClient mqtt(wifiClient);

static const char* g_broker = nullptr;
static uint16_t g_port = 1883;

static unsigned long lastReconnect = 0;
static const unsigned long RECONNECT_INTERVAL = 3000;

// Topics (keep them stable early on)
static const char* TOPIC_CMD_DRIVE = "robot/cmd/drive";

/*
 * ACK topic:
 * - The ESP32 publishes a small confirmation message here every time it receives
 *   and parses a drive command.
 * - This is the fastest way to prove (from CLI / MQTT clients) that RX works.
 */
static const char* TOPIC_CMD_DRIVE_ACK = "robot/cmd/drive/ack";

/*
 * NEW: Applied drive state topic (retained)
 * - The ESP32 publishes the command it actually applied (after parsing + clamp)
 * - Retained so dashboards can immediately show the latest applied L/R
 */
static const char* TOPIC_STATE_DRIVE = "robot/state/drive";

static bool g_subscribed = false;

/*
 * Small helpers: robust parsing without extra libraries (no ArduinoJson needed).
 * Supported payload formats:
 *   1) JSON: {"left":120,"right":-80}
 *   2) CSV:  "120,-80"
 */

// Extract an integer value after a given JSON key, e.g. key="left" -> 120
// Returns true if found and parsed.
static bool jsonExtractInt(const char* payload, const char* key, int& out)
{
    if (!payload || !key) return false;

    // Build pattern: "left"
    char pattern[32];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);

    const char* p = strstr(payload, pattern);
    if (!p) return false;

    // Find ':' after the key
    p = strchr(p, ':');
    if (!p) return false;
    p++; // move past ':'

    // Skip whitespace
    while (*p == ' ' || *p == '\t') p++;

    // Parse integer (supports negative)
    char* endPtr = nullptr;
    long v = strtol(p, &endPtr, 10);
    if (endPtr == p) return false; // no conversion happened

    out = (int)v;
    return true;
}

static bool parseDriveCommand(const char* payload, int& left, int& right)
{
    if (!payload) return false;

    // 1) Try JSON first (best for future backend/frontend integration)
    int l = 0, r = 0;
    bool hasL = jsonExtractInt(payload, "left", l);
    bool hasR = jsonExtractInt(payload, "right", r);

    if (hasL && hasR)
    {
        left = l;
        right = r;
        return true;
    }

    // 2) Fallback: CSV "L,R" (handy for quick CLI tests)
    int ll = 0, rr = 0;
    if (sscanf(payload, " %d , %d ", &ll, &rr) == 2)
    {
        left = ll;
        right = rr;
        return true;
    }

    return false;
}

static inline int clamp255(int v)
{
    if (v > 255) return 255;
    if (v < -255) return -255;
    return v;
}

// Publish an ACK message to confirm reception/parsing of a command.
// Keeping it small avoids flooding.
static void publishDriveAck(bool ok, int left, int right, const char* err)
{
    if (!mqtt.connected()) return;

    char ack[128];

    if (ok)
    {
        snprintf(ack, sizeof(ack),
                 "{\"ok\":true,\"left\":%d,\"right\":%d}",
                 left, right);
    }
    else
    {
        // Keep error short; avoid sending big payloads
        snprintf(ack, sizeof(ack),
                 "{\"ok\":false,\"err\":\"%s\"}",
                 (err && err[0]) ? err : "unknown");
    }

    mqtt.publish(TOPIC_CMD_DRIVE_ACK, ack);
}

/*
 * NEW: Publish applied drive state (retained)
 * This is what the dashboard should display as "Applied".
 */
static void publishDriveState(int left, int right)
{
    if (!mqtt.connected()) return;

    char st[96];
    snprintf(st, sizeof(st),
             "{\"left\":%d,\"right\":%d}",
             left, right);

    // Retained publish: dashboards get the last known applied command immediately
    mqtt.publish(TOPIC_STATE_DRIVE, st, true);
}

// MQTT message callback
static void onMqttMessage(char* topic, byte* payload, unsigned int length)
{
    if (!topic || !payload || length == 0) return;

    // Copy payload into a null-terminated buffer
    // Commands should be small; keep a safe cap.
    static char msg[256];
    unsigned int n = (length >= (sizeof(msg) - 1)) ? (sizeof(msg) - 1) : length;
    memcpy(msg, payload, n);
    msg[n] = '\0';

    // Initial phase: log everything we receive.
    Serial.print("MQTT RX -> topic=");
    Serial.print(topic);
    Serial.print(" payload=");
    Serial.println(msg);

    // Handle drive command topic
    if (strcmp(topic, TOPIC_CMD_DRIVE) == 0)
    {
        int left = 0, right = 0;

        if (parseDriveCommand(msg, left, right))
        {
            left = clamp255(left);
            right = clamp255(right);

            // Deliver to the application (main.cpp)
            onMqttDriveCommand(left, right);

            // NEW: publish applied state (retained) for dashboards
            publishDriveState(left, right);

            // ACK back so you can verify with mosquitto_sub
            publishDriveAck(true, left, right, nullptr);
        }
        else
        {
            Serial.println("MQTT: invalid drive payload (expected JSON or 'L,R')");
            publishDriveAck(false, 0, 0, "invalid_payload");
        }

        return;
    }

    // Other topics can be handled here later (e.g. robot/cmd/mode, robot/cmd/limits, etc.)
}

void initMQTT(const char* brokerIp, uint16_t port)
{
    g_broker = brokerIp;
    g_port = port;

    mqtt.setServer(g_broker, g_port);
    mqtt.setCallback(onMqttMessage);

    g_subscribed = false;
}

static void subscribeTopicsOnce()
{
    if (!mqtt.connected()) return;
    if (g_subscribed) return;

    // Subscribe to command topic(s)
    if (mqtt.subscribe(TOPIC_CMD_DRIVE))
    {
        Serial.print("MQTT: subscribed -> ");
        Serial.println(TOPIC_CMD_DRIVE);
        g_subscribed = true;

        // Optional: publish a small "ready" ACK
        publishDriveAck(false, 0, 0, "ready");
    }
    else
    {
        Serial.print("MQTT: subscribe failed -> ");
        Serial.println(TOPIC_CMD_DRIVE);
    }
}

static void reconnectMQTT()
{
    if (mqtt.connected()) return;
    if (millis() - lastReconnect < RECONNECT_INTERVAL) return;

    lastReconnect = millis();

    String clientId = "ESP32_4WD_ROBOT_" + String((uint32_t)ESP.getEfuseMac(), HEX);

    Serial.println("MQTT: connecting...");
    if (mqtt.connect(clientId.c_str()))
    {
        Serial.println("MQTT: connected");
        g_subscribed = false; // force resubscribe after reconnect
        subscribeTopicsOnce();
    }
    else
    {
        Serial.print("MQTT: failed, rc=");
        Serial.println(mqtt.state());
    }
}

void mqttTask()
{
    reconnectMQTT();

    if (mqtt.connected())
    {
        subscribeTopicsOnce();
        mqtt.loop();
    }
}

bool mqttPublish(const char* topic, const char* payload)
{
    if (!mqtt.connected())
        return false;

    return mqtt.publish(topic, payload);
}

bool mqttIsConnected()
{
    return mqtt.connected();
}

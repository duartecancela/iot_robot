#include "mqtt_manager.h"
#include <WiFi.h>
#include <PubSubClient.h>

static WiFiClient wifiClient;
static PubSubClient mqtt(wifiClient);

static const char* g_broker = nullptr;
static uint16_t g_port = 1883;

static unsigned long lastReconnect = 0;
static const unsigned long RECONNECT_INTERVAL = 3000;

void initMQTT(const char* brokerIp, uint16_t port)
{
    g_broker = brokerIp;
    g_port = port;
    mqtt.setServer(g_broker, g_port);
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
        mqtt.loop();
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

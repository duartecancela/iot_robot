#pragma once
#include <Arduino.h>

// Initialize MQTT client
void initMQTT(const char* brokerIp, uint16_t port);

// Non-blocking MQTT task (call inside loop)
void mqttTask();

// Publish message
bool mqttPublish(const char* topic, const char* payload);
bool mqttPublish(const char* topic, const char* payload, bool retained);

// Check MQTT connection
bool mqttIsConnected();

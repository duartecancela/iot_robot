#pragma once
#include <Arduino.h>

// Initialize WiFi connection
void initWiFi(const char* ssid, const char* pass);

// Non-blocking WiFi task (call inside loop)
void wifiTask();

// Check if WiFi is connected
bool wifiIsConnected();

// Get local IP address as string
String wifiIp();


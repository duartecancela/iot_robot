#pragma once
#include <Arduino.h>

// Register multiple WiFi networks (SSID/PASS)
void addWiFiNetwork(const char* ssid, const char* pass);

// Initialize WiFiMulti (station mode + auto reconnect)
void initWiFiMulti();

// Non-blocking task: periodically tries to connect to any known network
void wifiTask();

bool wifiIsConnected();
String wifiIp();
String wifiSsid();



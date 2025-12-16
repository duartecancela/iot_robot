#include "wifi_manager.h"
#include <WiFi.h>

// Stored WiFi credentials
static const char* g_ssid = nullptr;
static const char* g_pass = nullptr;

// Reconnection timing
static unsigned long lastTry = 0;
static const unsigned long RETRY_MS = 5000;

void initWiFi(const char* ssid, const char* pass)
{
    g_ssid = ssid;
    g_pass = pass;

    // Station mode (client)
    WiFi.mode(WIFI_STA);

    // Enable automatic reconnection
    WiFi.setAutoReconnect(true);

    // Avoid writing WiFi credentials to flash repeatedly
    WiFi.persistent(false);

    Serial.printf("WiFi: connecting to '%s'...\n", g_ssid);
    WiFi.begin(g_ssid, g_pass);

    lastTry = millis();
}

void wifiTask()
{
    // Already connected
    if (WiFi.status() == WL_CONNECTED)
        return;

    // Retry connection periodically without blocking
    if (millis() - lastTry >= RETRY_MS)
    {
        lastTry = millis();
        Serial.println("WiFi: reconnecting...");
        WiFi.disconnect(false);
        WiFi.begin(g_ssid, g_pass);
    }
}

bool wifiIsConnected()
{
    return (WiFi.status() == WL_CONNECTED);
}

String wifiIp()
{
    if (!wifiIsConnected())
        return String("-");

    return WiFi.localIP().toString();
}

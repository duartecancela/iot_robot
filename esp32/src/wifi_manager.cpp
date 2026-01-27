#include "wifi_manager.h"
#include <WiFi.h>
#include <WiFiMulti.h>

static WiFiMulti wifiMulti;

// Run interval to avoid spamming connection attempts
static unsigned long lastRun = 0;
static const unsigned long RUN_MS = 2000;

void addWiFiNetwork(const char* ssid, const char* pass)
{
    // Add a known access point (SSID + password)
    wifiMulti.addAP(ssid, pass);
}

void initWiFiMulti()
{
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false); // avoid writing credentials to flash repeatedly

    Serial.println("WiFiMulti: started (will connect to any known network).");
}

void wifiTask()
{
    if (millis() - lastRun < RUN_MS) return;
    lastRun = millis();

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFiMulti: trying to connect...");
        wifiMulti.run(); // tries the best available known network
    }
}

bool wifiIsConnected()
{
    return (WiFi.status() == WL_CONNECTED);
}

String wifiIp()
{
    if (!wifiIsConnected()) return String("-");
    return WiFi.localIP().toString();
}

String wifiSsid()
{
    if (!wifiIsConnected()) return String("-");
    return WiFi.SSID();
}

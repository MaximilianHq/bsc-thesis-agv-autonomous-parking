#include <WiFi.h>
#include <ArduinoOTA.h>
#include "ota.h"

void ota_begin(const char *ssid, const char *password)
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    Serial.print("[OTA] Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.print("[OTA] Connected. IP: ");
    Serial.println(WiFi.localIP());

    ArduinoOTA.setHostname("esp32-agv");
    ArduinoOTA.begin();

    Serial.println("[OTA] Ready");
}

void ota_handle()
{
    ArduinoOTA.handle();
}
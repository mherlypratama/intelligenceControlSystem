#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

const char *ssid = "pertanian24";
const char *password = "luarbiasa";

AsyncWebServer server(80);

void setup(void)
{
    Serial.begin(9600);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.print("IP address: ");
    Serial.print(WiFi.localIP());

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/plain", "ESP32 OTA (Over the Air)."); });

    AsyncElegantOTA.begin(&server);
    server.begin();
}

void loop(void)
{
    AsyncElegantOTA.loop();
}
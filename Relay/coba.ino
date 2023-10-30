#include <WiFi.h>
#include "time.h"

const char *ssid = "Lab Telkom 2.4 GHz";
const char *password = "telekomunikasi";

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 21600;
const int daylightOffset_sec = 3600;

#define RELAY_PIN 16
#define RELAY_PIN2 17
#define RELAY_PIN3 4

int relay1, relay2, relay3;

void setup()
{
    Serial.begin(9600);
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(RELAY_PIN2, OUTPUT);
    pinMode(RELAY_PIN3, OUTPUT);

    // Connect to Wi-Fi
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected.");

    // Init and get the time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    printLocalTime();

    // disconnect WiFi as it's no longer needed
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
}

void loop()
{
    printLocalTime();
    relay1(); // Untuk Lampu UV
    relay2();
    relay3();
    delay(1000);
}

void relay1()
{
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        Serial.println("Failed to obtain time");
        return;
    }
    // Extract the hour (0-23) from the struct tm
    int jam = timeinfo.tm_hour;
    int minute = timeinfo.tm_min;

    // Kontrol RELAY_PIN2
    if (jam >= 18 && jam < 6)
    {
        digitalWrite(RELAY_PIN, LOW);
        relay1 = 1;
    }
    else
    {
        digitalWrite(RELAY_PIN, HIGH);
        relay1 = 0;
    }
}

void relay2()
{
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        Serial.println("Failed to obtain time");
        return;
    }
    // Extract the hour (0-23) from the struct tm
    int jam = timeinfo.tm_hour;
    int minute = timeinfo.tm_min;

    // Kontrol RELAY_PIN2
    if (jam == 23 && minute >= 25 && minute < 30)
    {
        digitalWrite(RELAY_PIN2, LOW);
        relay1 = 1;
    }
    else
    {
        digitalWrite(RELAY_PIN2, HIGH);
        relay1 = 0;
    }
}

void relay3()
{
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        Serial.println("Failed to obtain time");
        return;
    }
    // Extract the hour (0-23) from the struct tm
    int jam = timeinfo.tm_hour;
    int minute = timeinfo.tm_min;

    // Kontrol RELAY_PIN2
    if (jam == 23 && minute >= 25 && minute < 30)
    {
        digitalWrite(RELAY_PIN3, LOW);
        relay1 = 1;
    }
    else
    {
        digitalWrite(RELAY_PIN3, HIGH);
        relay1 = 0;
    }
}

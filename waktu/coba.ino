#include <WiFi.h>
#include "time.h"

const char *ssid = "Lab Telkom 2.4 GHz";
const char *password = "telekomunikasi";

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 21600;
const int daylightOffset_sec = 3600;

void setup()
{
    Serial.begin(9600);

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
    delay(1000);
    printLocalTime();
}

void printLocalTime()
{
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        Serial.println("Failed to obtain time");
        return;
    }

    tanggal = &timeinfo, "%A, %B %d %Y %H:%M:%S";
    Serial.println();

    char strftime_buf[64];

    // Format waktu sesuai keinginan Anda, contoh: "Sabtu, 06 November 2023 14:30:45"
    strftime(strftime_buf, sizeof(strftime_buf), "%A, %d %B %Y %H:%M:%S", &timeinfo);
    Serial.println(strftime_buf);
    Serial.println();
}
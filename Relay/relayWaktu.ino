#include <WiFi.h>
#include "time.h"

const char *ssid = "Lab Telkom 2.4 GHz";
const char *password = "telekomunikasi";

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 21600;
const int daylightOffset_sec = 3600;
#define RELAY_PIN 16
#define RELAY_PIN2 17

void setup()
{
    Serial.begin(9600);
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(RELAY_PIN2, OUTPUT);

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

    // Extract the hour (0-23) from the struct tm
    int jam = timeinfo.tm_hour;
    int minute = timeinfo.tm_min;

    // Print the extracted hour
    Serial.print("Hour: ");
    Serial.println(jam);

    // Add your conditions based on the 'jam' variable here
    if (jam >= 22 && minute >= 27 && minute < 28)
    {
        // Morning
        digitalWrite(RELAY_PIN, LOW);
    }

    else if (jam >= 22 && minute >= 28 && minute < 29)
    {
        digitalWrite(RELAY_PIN2, LOW);
        // Afternoon
    }

    else
    {
        // Evening
        digitalWrite(RELAY_PIN, HIGH);
    }

    // Print other time information
    Serial.print("Minute: ");
    Serial.println(timeinfo.tm_min);
    Serial.print("Second: ");
    Serial.println(timeinfo.tm_sec);
}

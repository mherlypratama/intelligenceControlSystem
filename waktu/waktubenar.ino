#include <WiFi.h>
#include "time.h"

const char *ssid = "Lab Telkom 2.4 GHz";
const char *password = "telekomunikasi";

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 21600;
const int daylightOffset_sec = 3600;

int jam, minute, second, tanggal, bulan, tahun;

void setup()
{
    Serial.begin(9600);

    // Init and get the time
    setwifi();
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    printLocalTime();

    // // disconnect WiFi as it's no longer needed
    // WiFi.disconnect(true);
    // WiFi.mode(WIFI_OFF);
}

void loop()
{
    delay(1000);
    printLocalTime();
}

void setwifi()
{
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
}

void printLocalTime()
{
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        Serial.println("Failed to obtain time");
        return;
    }

    jam = timeinfo.tm_hour;
    minute = timeinfo.tm_min;
    second = timeinfo.tm_sec;

    tanggal = timeinfo.tm_mday;
    bulan = timeinfo.tm_mon + 1;     // Bulan dimulai dari 0, sehingga Anda perlu menambahkan 1
    tahun = 1900 + timeinfo.tm_year; // Tahun dimulai dari 1900

    char strftime_buf[50]; // Buffer untuk menyimpan timestamp yang diformat
    strftime(strftime_buf, sizeof(strftime_buf), "%A, %d %B %Y %H:%M:%S", &timeinfo);
    Serial.println(strftime_buf);
}
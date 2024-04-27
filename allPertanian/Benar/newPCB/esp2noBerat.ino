#include <SoftwareSerial.h>
#include <DFRobot_BMP3XX.h>

#include <OneWire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "time.h"

#include <Wire.h>

// Konfigurasi jaringan Wi-Fi
const char *ssid = "pertanian24";
const char *password = "luarbiasa";

// Konfigurasi server MQTT di VPS Anda
const char *mqtt_server = "vps.isi-net.org";
const int mqtt_port = 1883;
const char *mqtt_user = "unila";
const char *mqtt_password = "pwdMQTT@123";

const char *topic_utama = "ics/gisting";

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 25200;
const int daylightOffset_sec = 3600;

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsgTime = 0;
const long interval = 5000; // Kirim data setiap 5 detik

int jam, minute, second, tanggal, bulan, tahun;

DFRobot_BMP388_I2C sensor(&Wire, sensor.eSDOVDD);
#define CALIBRATE_ABSOLUTE_DIFFERENCE
float temperaturebmp, Pressure, altitude;

#define RE 25
#define DE 33

const byte pyranometer[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
byte values[8];
SoftwareSerial mod(32, 26); // mod(Ro, DI)

int Solar_Radiation;

void setup()
{
    Serial.begin(9600);
    setupWiFi();
    client.setServer(mqtt_server, mqtt_port);
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    printLocalTime();
    setpyrano();
    setbmp();
}

void loop()
{
    if (!client.connected())
    {
        reconnectMQTT();
    }
    client.loop();
    printLocalTime();

    sensorpyrano();
    sensorbmp();
    nodered();
    delay(1000);
}

void nodered()
{

    // Buat objek JSON yang berisi data dari keempat sensor
    char utamaStr[1000]; // Buffer untuk menyimpan JSON
    snprintf(utamaStr, sizeof(utamaStr),
             "{"
             "\"TimeStamp\": \"%04d-%02d-%02dT%02d:%02d:%02d+07:00\","
             "\"temperaturebmp\": %.2f,"
             "\"pressurebmp\": %.2f,"
             "\"pyrano\": %d,"
             "\"relay1\": %d,"
             "\"relay2\": %d,"
             "\"relay3\": %d,"
             "\"berat2\": %.2f,"
             "\"berat3\": %.2f,"
             "\"berat4\": %.2f"
             "}",
             tahun, bulan, tanggal, jam, minute, second, temperaturebmp, Pressure, Solar_Radiation, relay1, relay2, relay3, a, b, c);

    client.publish(topic_utama, utamaStr); // Mengirim data suhu ke broker MQTT
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

void setupWiFi()
{
    Serial.print("Menghubungkan ke WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(5000);
        Serial.println("Menghubungkan ke WiFi...");
    }
    Serial.println("Terhubung ke WiFi");
}

void reconnectMQTT()
{
    while (!client.connected())
    {
        Serial.print("Menghubungkan ke broker MQTT...");
        if (client.connect("ESP32Client", mqtt_user, mqtt_password))
        {
            Serial.println("Terhubung ke broker MQTT");
        }
        else
        {
            Serial.print("Gagal, kode kesalahan = ");
            Serial.println(client.state());
            delay(5000);
        }
    }
}

void callback(char *topic, byte *payload, unsigned int length)
{
    // Implementasi callback jika diperlukan
}

// Lampu UV
void relay11()
{
    // Kontrol RELAY_PIN2
    if (jam >= 18 && jam < 23 || jam >= 0 && jam < 6)
    {
        digitalWrite(RELAY_PIN1, LOW);
        relay1 = 1;
        Serial.print("Relay 1: ");
        Serial.println(relay1);
    }
    else
    {
        digitalWrite(RELAY_PIN1, HIGH);
        relay1 = 0;
        Serial.println(relay1);
    }
}

// Nutrisi
void relay22()
{

    // Kontrol RELAY_PIN22
    if (jam == 16 && minute >= 1 && minute < 5 || jam == 6 && minute >= 1 && minute < 5 || jam == 10 && minute >= 1 && minute < 5 || jam == 13 && minute >= 1 && minute < 5)
    {
        digitalWrite(RELAY_PIN2, LOW);
        relay2 = 1;
        Serial.println(relay2);
    }
    else
    {
        digitalWrite(RELAY_PIN2, HIGH);
        relay2 = 0;
        Serial.println(relay2);
    }
}

// air pendingin
void relay33()
{

    // Kontrol RELAY_PIN2
    if (temperaturebmp > 35)
    {
        digitalWrite(RELAY_PIN3, LOW);
        relay3 = 1;
        Serial.println(relay3);
    }
    else
    {
        digitalWrite(RELAY_PIN3, HIGH);
        relay3 = 0;
        Serial.println(relay3);
    }
}

void setbmp()
{
    int rslt;
    while (ERR_OK != (rslt = sensor.begin()))
    {
        if (ERR_DATA_BUS == rslt)
        {
            Serial.println("Data bus error!!!");
        }
        else if (ERR_IC_VERSION == rslt)
        {
            Serial.println("Chip versions do not match!!!");
        }
        delay(3000);
    }
    Serial.println("Begin ok!");

    while (!sensor.setSamplingMode(sensor.eUltraPrecision))
    {
        Serial.println("Set samping mode fail, retrying....");
        delay(3000);
    }

    delay(100);
#ifdef CALIBRATE_ABSOLUTE_DIFFERENCE

    if (sensor.calibratedAbsoluteDifference(540.0))
    {
        Serial.println("Absolute difference base value set successfully!");
    }
#endif

    float sampingPeriodus = sensor.getSamplingPeriodUS();
    Serial.print("samping period : ");
    Serial.print(sampingPeriodus);
    Serial.println(" us");

    float sampingFrequencyHz = 1000000 / sampingPeriodus;
    Serial.print("samping frequency : ");
    Serial.print(sampingFrequencyHz);
    Serial.println(" Hz");

    Serial.println();
}

void sensorbmp()
{
    temperaturebmp = sensor.readTempC();
    Serial.print("temperaturebmp : ");
    Serial.print(temperaturebmp);
    Serial.println(" C");

    Pressure = sensor.readPressPa();
    Serial.print("Pressure : ");
    Serial.print(Pressure);
    Serial.println(" Pa");

    altitude = sensor.readAltitudeM();
    Serial.print("Altitude : ");
    Serial.print(altitude);
    Serial.println(" m");

    Serial.println();
    delay(1000);
}

void setpyrano()
{
    mod.begin(4800);
    pinMode(RE, OUTPUT);
    pinMode(DE, OUTPUT);
}

void sensorpyrano()
{
    // Transmit the request to the sensor
    digitalWrite(DE, HIGH);
    digitalWrite(RE, HIGH);
    delay(10);

    mod.write(pyranometer, sizeof(pyranometer));

    digitalWrite(DE, LOW);
    digitalWrite(RE, LOW);
    delay(10); // Give some time for the sensor to respond

    // Wait until we have the expected number of bytes or timeout
    unsigned long startTime = millis();
    while (mod.available() < 7 && millis() - startTime < 1000)
    {
        delay(1);
    }

    // Read the response
    byte index = 0;
    while (mod.available() && index < 8)
    {
        values[index] = mod.read();
        Serial.print(values[index], HEX);
        Serial.print(" ");
        index++;
    }
    Serial.println();

    // Parse the Solar Radiation value
    Solar_Radiation = int(values[3] << 8 | values[4]);
    Serial.print("Solar Radiation: ");
    Serial.print(Solar_Radiation);
    Serial.println(" W/m^2");

    delay(3000);
}
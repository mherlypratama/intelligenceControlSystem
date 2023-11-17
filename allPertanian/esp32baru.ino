#include <WiFi.h>
#include "time.h"
#include <OneWire.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <HX711_ADC.h>

#include <DFRobot_BMP3XX.h>

DFRobot_BMP388_I2C sensor(&Wire, sensor.eSDOVDD);
#define CALIBRATE_ABSOLUTE_DIFFERENCE

#define RELAY_PIN 14
#define RELAY_PIN2 12
#define RELAY_PIN3 13
const int HX711_dout_1 = 27;
const int HX711_sck_1 = 26;

const int HX711_dout_2 = 15; // Ganti dengan pin dout sensor berat kedua
const int HX711_sck_2 = 2;   // Ganti dengan pin sck sensor berat kedua

#define RE 33
#define DE 32

const byte pyranometer[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
byte values[8];
SoftwareSerial mod(25, 4);

HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1);
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2);

int Direction, jam, minute, second, tanggal, bulan, tahun;
int relay1, relay2, relay3;

float temperature, altitude, Pressure, weight_1, weight_2, weight_3, Solar_Radiation, humi;

const char *ssid = "pertanian24";
const char *password = "luarbiasa";

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 25200;
const int daylightOffset_sec = 3600;

// Konfigurasi server MQTT di VPS Anda
const char *mqtt_server = "vps.isi-net.org";
const int mqtt_port = 1883;
const char *mqtt_user = "unila";
const char *mqtt_password = "pwdMQTT@123";

const char *topic_ketiga = "ics/pertanian3";

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsgTime = 0;
const long interval = 5000;

void setup(void)
{
    Serial.begin(9600);
    // Connect to Wi-Fi
    Serial.print("Menghubungkan ke WiFi...");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected.");
    client.setServer(mqtt_server, mqtt_port);

    pinMode(RELAY_PIN, OUTPUT);
    pinMode(RELAY_PIN2, OUTPUT);
    pinMode(RELAY_PIN3, OUTPUT);
    mod.begin(4800);
    pinMode(RE, OUTPUT);
    pinMode(DE, OUTPUT);

    // Init and get the time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    setBMP();
    setBerat();

    delay(1000);
}

void loop()
{
    if (!client.connected())
    {
        reconnectMQTT();
    }
    client.loop();

    printLocalTime();
    sensorBMP();
    sensorBerat();
    sensorpyrano();
    relay11(); // Untuk Lampu UV
    relay22();
    relay33();
    nodered();
    delay(60000);
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

void nodered()
{

    // Buat objek JSON yang berisi data dari keempat sensor
    char utamaStr[1000]; // Buffer untuk menyimpan JSON
    snprintf(utamaStr, sizeof(utamaStr),
             "{"
             "\"TimeStamp\": \"%04d-%02d-%02dT%02d:%02d:%02d+07:00\","
             "\"pyrano\": %.2f,"
             "\"temperatureBMP\": %.2f,"
             "\"humidity\": %.2f,"
             "\"pressure\": %.2f,"
             "\"berat_2\": %.2f,"
             "\"berat_3\": %.2f,"
             "\"berat_4\": %.2f,"
             "\"lampu_uv\": %d,"
             "\"pompanutrisi\": %d,"
             "\"pompapendingin\": %d"
             "}",
             tahun, bulan, tanggal, jam, minute, second, Solar_Radiation, temperature, humi, Pressure, weight_1, weight_2, weight_3, relay1, relay2, relay3);

    client.publish(topic_ketiga, utamaStr); // Mengirim data suhu ke broker MQTT
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
}

void setBerat()
{
    LoadCell_1.begin();
    LoadCell_1.start(2000);
    LoadCell_1.setCalFactor(106.26);

    LoadCell_2.begin();
    LoadCell_2.start(2000);
    LoadCell_2.setCalFactor(106.26);
}

void sensorBerat()
{
    LoadCell_1.update();
    LoadCell_2.update();

    weight_1 = LoadCell_1.getData();
    weight_2 = LoadCell_2.getData();

    if (weight_1 < 0)
    {
        weight_1 = 0;
    }

    if (weight_2 < 0)
    {
        weight_2 = 0;
    }

    weight_3 = (weight_1 + weight_2) / 2;

    Serial.print("Weight Sensor 1 [g]: ");
    Serial.println(weight_1);

    Serial.print("Weight Sensor 2 [g]: ");
    Serial.println(weight_2);
}

void setBMP()
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

void sensorBMP()
{
    temperature = sensor.readTempC();
    Serial.print("temperature : ");
    Serial.print(temperature);
    Serial.println(" C");

    humi = 100 - temperature;

    Pressure = sensor.readPressPa();
    Serial.print("Pressure : ");
    Serial.print(Pressure);
    Serial.println(" Pa");

    altitude = sensor.readAltitudeM();
    Serial.print("Altitude : ");
    Serial.print(altitude);
    Serial.println(" m");

    Serial.println();
}

// Lampu UV
void relay11()
{
    // digitalWrite(RELAY_PIN, LOW);
    // relay1 = 0;
    // delay(1000);
    // digitalWrite(RELAY_PIN, HIGH);
    // relay1 = 1;

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        Serial.println("Failed to obtain time");
        return;
    }
    // Extract the hour (0-23) from the struct tm
    jam = timeinfo.tm_hour;
    minute = timeinfo.tm_min;

    // Kontrol RELAY_PIN2
    if (jam >= 18 && jam < 23 || jam >= 0 && jam < 6)
    {
        digitalWrite(RELAY_PIN, LOW);
        relay1 = 1;
        Serial.print("Relay 1: ");
        Serial.println(relay1);
    }
    else
    {
        digitalWrite(RELAY_PIN, HIGH);
        relay1 = 0;
        Serial.println(relay1);
    }
}

// Nutrisi
void relay22()
{
    // digitalWrite(RELAY_PIN2, LOW);
    // relay1 = 0;
    // delay(1000);
    // digitalWrite(RELAY_PIN2, HIGH);
    // relay1 = 1;
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        Serial.println("Failed to obtain time");
        return;
    }
    // Extract the hour (0-23) from the struct tm
    int jam = timeinfo.tm_hour;
    int minute = timeinfo.tm_min;

    // Kontrol RELAY_PIN22
    if (jam == 16 && minute >= 1 && minute < 16 || jam == 6 && minute >= 1 && minute < 16 || jam == 10 && minute >= 1 && minute < 16 || jam == 13 && minute >= 1 && minute < 16)
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
    // digitalWrite(RELAY_PIN3, LOW);
    // relay1 = 0;
    // delay(1000);
    // relay1 = 1;
    // digitalWrite(RELAY_PIN3, HIGH);
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
    if (temperature > 32)
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

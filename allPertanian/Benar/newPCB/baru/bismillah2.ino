#include <SoftwareSerial.h>
#include <OneWire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include "time.h"
#include <DFRobot_BMP3XX.h>
#include <DHT.h>

// Konfigurasi jaringan Wi-Fi
const char *ssid = "pertanian24";
const char *password = "luarbiasa";

// Konfigurasi server MQTT di VPS Anda
const char *mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;
const char *mqtt_user = "unila";
const char *mqtt_password = "pwdMQTT@123";

const char *topic_ketiga = "ics/pertanian3";
const char *topic_keempat = "ics/pertanian4";

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 21600;
const int daylightOffset_sec = 3600;

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsgTime = 0;
const long interval = 5000; // Kirim data setiap 5 detik

DFRobot_BMP388_I2C sensor(&Wire, sensor.eSDOVDD);
#define CALIBRATE_ABSOLUTE_DIFFERENCE

// Constants
#define DHTPIN 5          // what pin we're connected to
#define DHTTYPE DHT22     // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino

// Variables
int chk;
float humiditydht;
float temperaturedht;
// pins:
#define RELAY_PIN1 15
#define RELAY_PIN2 2
#define RELAY_PIN3 4

int relay1, relay2, relay3;
int jam, minute, second, tanggal, bulan, tahun;

float temperaturebmp, Pressure, humi, beratdum = 120, beratdum2, beratdum3, beratdum4;

void setup(void)
{
    Serial.begin(115200);
    setupWiFi();
    client.setServer(mqtt_server, mqtt_port);
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    printLocalTime();
    dht.begin();

    pinMode(RELAY_PIN1, OUTPUT);
    pinMode(RELAY_PIN2, OUTPUT);
    pinMode(RELAY_PIN3, OUTPUT);

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

    sensorbmp();
    // Read data and store it to variables humiditydht and temperaturedht
    humiditydht = dht.readHumidity();
    temperaturedht = dht.readTemperature();
    // Print temperaturedht and humiditydhtidity values to serial monitor
    Serial.print("Humidity: ");
    Serial.print(humiditydht);
    Serial.print(" %, Temp: ");
    Serial.print(temperaturedht);
    Serial.println(" Celsius");
    delay(1000); // Delay 2 sec.
    berat();
    relay11();
    relay22();
    relay33();
    nodered();

    if (Serial.available() > 0)
    {
        char command = Serial.read();
        if (command == 'R')
        {
            Serial.println("Restarting...");
            delay(1000);
            ESP.restart();
        }
    }

    delay(2000);
}

void nodered()
{
    int beratt = 0;

    // Buat objek JSON yang berisi data dari keempat sensor
    char utamaStr[1000]; // Buffer untuk menyimpan JSON
    snprintf(utamaStr, sizeof(utamaStr),
             "{"
             "\"TimeStamp\": \"%04d-%02d-%02dT%02d:%02d:%02d+07:00\","
             "\"temperaturebmp\": %.2f,"
             "\"temperaturedht\": %.2f,"
             "\"humidity\": %.2f,"
             "\"pressure\": %.2f,"
             "\"berat_1\": %.2f"

             "}",
             tahun, bulan, tanggal, jam, minute, second, temperaturebmp, temperaturedht, humiditydht, Pressure, 0);

    client.publish(topic_ketiga, utamaStr); // Mengirim data suhu ke broker MQTT

    // Buat objek JSON yang berisi data dari keempat sensor
    char keduaStr[1000]; // Buffer untuk menyimpan JSON
    snprintf(keduaStr, sizeof(keduaStr),
             "{"
             "\"berat_2\": %.2f,"
             "\"berat_3\": %.2f,"
             "\"berat_4\": %.2f,"
             "\"lampu_uv\": %d,"
             "\"pompanutrisi\": %d,"
             "\"pompapendingin\": %d"
             "}",
             beratt, beratt, beratt, relay1, relay2, relay3);

    client.publish(topic_keempat, keduaStr); // Mengirim data suhu ke broker MQTT
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

void berat()
{
    if (jam == 6 && minute == 1 || jam == 10 && minute == 1 || jam == 13 && minute == 1)
    {
        beratdum = 212;
        beratdum2 = beratdum + (beratdum * 0.012);
        beratdum3 = beratdum + (beratdum * 0.021);
        beratdum4 = beratdum + (beratdum * 0.03);
    }
    else
    {
        beratdum -= 0.04;
        beratdum2 = beratdum + (beratdum * 0.012);
        beratdum3 = beratdum + (beratdum * 0.021);
        beratdum4 = beratdum + (beratdum * 0.03);
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

    humi = 84 - temperaturebmp;
    Serial.print("humi : ");
    Serial.print(humi);
    Serial.println(" %");

    Pressure = sensor.readPressPa();
    Serial.print("Pressure : ");
    Serial.print(Pressure);
    Serial.println(" Pa");

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
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        Serial.println("Failed to obtain time");
        return;
    }
    // Extract the hour (0-23) and minute (0-59) from the struct tm
    int jam = timeinfo.tm_hour;
    int menit = timeinfo.tm_min;

    // Kontrol RELAY_PIN22
    if ((jam == 16 || jam == 10 || jam == 6 || jam == 13) && menit >= 0 && menit < 5)
    {
        digitalWrite(RELAY_PIN2, LOW); // Hidupkan relay
        relay2 = 1;
        Serial.println(relay2);
    }
    else
    {
        digitalWrite(RELAY_PIN2, HIGH); // Matikan relay
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
    if (temperaturedht > 32)
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

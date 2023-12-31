#include <OneWire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "time.h"
#include <Adafruit_ADS1X15.h>

#include <Wire.h>

// Konfigurasi jaringan Wi-Fi
const char *ssid = "Lab Telkom 2.4 GHz";
const char *password = "telekomunikasi";

// Konfigurasi server MQTT di VPS Anda
const char *mqtt_server = "vps.isi-net.org";
const int mqtt_port = 1883;
const char *mqtt_user = "unila";
const char *mqtt_password = "pwdMQTT@123";

const char *topic_utama = "ics/gisting2";

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 25200;
const int daylightOffset_sec = 3600;

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsgTime = 0;
const long Interval = 5000; // Kirim data setiap 5 detik

// ***************************SOIL*************************
Adafruit_ADS1115 ads;

// *************************WATER**************************
#define LED_BUILTIN 2

// Define pins for water flow sensors
#define SENSOR1 3
#define SENSOR2 25
#define SENSOR3 26
#define SENSOR4 0
#define SENSOR5 0
#define SENSOR6 0
#define SENSOR7 0
#define SENSOR8 0
#define SENSOR9 0
#define SENSOR10 0
#define SENSOR11 0
#define SENSOR12 0

long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;
boolean ledState = LOW;

// Calibration factors for each sensor (modify as needed)
float calibrationFactor1 = 4.5;
float calibrationFactor2 = 4.5;
float calibrationFactor3 = 4.5;
float calibrationFactor4 = 4.5;
float calibrationFactor5 = 4.5;
float calibrationFactor6 = 4.5;
float calibrationFactor7 = 4.5;
float calibrationFactor8 = 4.5;
float calibrationFactor9 = 4.5;
float calibrationFactor10 = 4.5;
float calibrationFactor11 = 4.5;
float calibrationFactor12 = 4.5;

volatile byte pulseCount1, pulseCount2, pulseCount3, pulseCount4, pulseCount5, pulseCount6, pulseCount7, pulseCount8, pulseCount9, pulseCount10, pulseCount11, pulseCount12;
byte pulse1Sec1, pulse1Sec2, pulse1Sec3, pulse1Sec4, pulse1Sec5, pulse1Sec6, pulse1Sec7, pulse1Sec8, pulse1Sec9, pulse1Sec10, pulse1Sec11, pulse1Sec12;
float flowRate1, flowRate2, flowRate3, flowRate4, flowRate5, flowRate6, flowRate7, flowRate8, flowRate9, flowRate10, flowRate11, flowRate12;
unsigned int flowMilliLitres1, flowMilliLitres2, flowMilliLitres3, flowMilliLitres4, flowMilliLitres5, flowMilliLitres6, flowMilliLitres7, flowMilliLitres8, flowMilliLitres9, flowMilliLitres10, flowMilliLitres11, flowMilliLitres12;
unsigned long totalMilliLitres1, totalMilliLitres2, totalMilliLitres3, totalMilliLitres4, totalMilliLitres5, totalMilliLitres6, totalMilliLitres7, totalMilliLitres8, totalMilliLitres9, totalMilliLitres10, totalMilliLitres11, totalMilliLitres12;

void IRAM_ATTR pulseCounter1()
{
    pulseCount1++;
}

void IRAM_ATTR pulseCounter2()
{
    pulseCount2++;
}

void IRAM_ATTR pulseCounter3()
{
    pulseCount3++;
}

void IRAM_ATTR pulseCounter4()
{
    pulseCount4++;
}
void IRAM_ATTR pulseCounter5()
{
    pulseCount5++;
}
void IRAM_ATTR pulseCounter6()
{
    pulseCount6++;
}
void IRAM_ATTR pulseCounter7()
{
    pulseCount7++;
}
void IRAM_ATTR pulseCounter8()
{
    pulseCount8++;
}
void IRAM_ATTR pulseCounter9()
{
    pulseCount9++;
}
void IRAM_ATTR pulseCounter10()
{
    pulseCount10++;
}
void IRAM_ATTR pulseCounter11()
{
    pulseCount11++;
}
void IRAM_ATTR pulseCounter12()
{
    pulseCount12++;
}

float fix0, fix1, fix2, fix3;

int jam, minute, second, tanggal, bulan, tahun;

void setup(void)
{
    Serial.begin(9600);
    setupWiFi();
    client.setServer(mqtt_server, mqtt_port);
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    printLocalTime();
    setsoil();
    setwater();
}

void loop()
{
    if (!client.connected())
    {
        reconnectMQTT();
    }
    client.loop();
    printLocalTime();
    sensorsoil();
    sensorwater();

    nodered();

    delay(60000);
}

void nodered()
{

    // Buat objek JSON yang berisi data dari keempat sensor
    char utamaStr[1000]; // Buffer untuk menyimpan JSON
    snprintf(utamaStr, sizeof(utamaStr),
             "{"
             "\"TimeStamp\": \"%04d-%02d-%02dT%02d:%02d:%02d+07:00\","
             "\"soil1\": %.2f,"
             "\"soil2\": %.2f,"
             "\"soil3\": %.2f,"
             "\"water1\": %.2f,"
             "\"water2\": %.2f,"
             "\"water3\": %.2f,"
             "\"water4\": %.2f,"
             "\"water5\": %.2f,"
             "\"water6\": %.2f,"
             "\"water7\": %.2f,"
             "\"water8\": %.2f,"
             "\"water9\": %.2f,"
             "\"water10\": %.2f,"
             "\"water11\": %.2f,"
             "\"water12\": %.2f,"
             "}",
             tahun, bulan, tanggal, jam, minute, second, fix0, fix1, fix2, flowRate1, flowRate2, flowRate3, flowRate4, flowRate5, flowRate6, flowRate7, flowRate8, flowRate9, flowRate10, flowRate11, flowRate12);

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

void setsoil()
{
    Serial.println("Getting single-ended readings from AIN0..3");
    Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

    ads.setGain(GAIN_TWOTHIRDS);

    if (!ads.begin())
    {
        Serial.println("Failed to initialize ADS.");
        while (1)
            ;
    }
}

void sensorsoil()
{
    int16_t adc0, adc1, adc2, adc3;
    float volts0, volts1, volts2, volts3;

    adc0 = ads.readADC_SingleEnded(0);
    adc1 = ads.readADC_SingleEnded(1);
    adc2 = ads.readADC_SingleEnded(2);
    adc3 = ads.readADC_SingleEnded(3);

    volts0 = ads.computeVolts(adc0);
    volts1 = ads.computeVolts(adc1);
    volts2 = ads.computeVolts(adc2);
    volts3 = ads.computeVolts(adc3);

    // Pastikan rentang nilai ADC yang diharapkan sesuai dengan sensor soil moisture
    float adcMin = 0.5;  // Ganti dengan nilai ADC terendah yang dihasilkan oleh sensor
    float adcMax = 2.53; // Ganti dengan nilai ADC tertinggi yang dihasilkan oleh sensor

    float datakonversi0 = (volts0 - adcMin) / (adcMax - adcMin) * 100;
    float datakonversi1 = (volts1 - adcMin) / (adcMax - adcMin) * 100;
    float datakonversi2 = (volts2 - adcMin) / (adcMax - adcMin) * 100;
    float datakonversi3 = (volts3 - adcMin) / (adcMax - adcMin) * 100;

    fix0 = 100 - datakonversi0;
    fix1 = 100 - datakonversi1;
    fix2 = 100 - datakonversi2;
    fix3 = 100 - datakonversi3;

    Serial.print("A0:");
    Serial.print(fix0);
    Serial.print("%,  ");
    Serial.print("A1:");
    Serial.print(fix1);
    Serial.print("%,  ");
    Serial.print("A2:");
    Serial.print(fix2);
    Serial.print("%,  ");
    Serial.print("A3:");
    Serial.print(fix3);
    Serial.println("%");

    delay(1000);
}

void setwater()
{
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(SENSOR1, INPUT_PULLUP);
    pinMode(SENSOR2, INPUT_PULLUP);
    pinMode(SENSOR3, INPUT_PULLUP);
    pinMode(SENSOR4, INPUT_PULLUP);
    pinMode(SENSOR5, INPUT_PULLUP);
    pinMode(SENSOR6, INPUT_PULLUP);
    pinMode(SENSOR7, INPUT_PULLUP);
    pinMode(SENSOR8, INPUT_PULLUP);
    pinMode(SENSOR9, INPUT_PULLUP);
    pinMode(SENSOR10, INPUT_PULLUP);
    pinMode(SENSOR11, INPUT_PULLUP);
    pinMode(SENSOR12, INPUT_PULLUP);

    pulseCount1 = 0;
    pulseCount2 = 0;
    pulseCount3 = 0;
    pulseCount4 = 0;
    pulseCount5 = 0;
    pulseCount6 = 0;
    pulseCount7 = 0;
    pulseCount8 = 0;
    pulseCount9 = 0;
    pulseCount10 = 0;
    pulseCount11 = 0;
    pulseCount12 = 0;

    flowRate1 = 0.0;
    flowRate2 = 0.0;
    flowRate3 = 0.0;
    flowRate4 = 0.0;
    flowRate5 = 0.0;
    flowRate6 = 0.0;
    flowRate7 = 0.0;
    flowRate8 = 0.0;
    flowRate9 = 0.0;
    flowRate10 = 0.0;
    flowRate11 = 0.0;
    flowRate12 = 0.0;

    flowMilliLitres1 = 0;
    flowMilliLitres2 = 0;
    flowMilliLitres3 = 0;
    flowMilliLitres4 = 0;
    flowMilliLitres5 = 0;
    flowMilliLitres6 = 0;
    flowMilliLitres7 = 0;
    flowMilliLitres8 = 0;
    flowMilliLitres9 = 0;
    flowMilliLitres10 = 0;
    flowMilliLitres11 = 0;
    flowMilliLitres12 = 0;

    totalMilliLitres1 = 0;
    totalMilliLitres2 = 0;
    totalMilliLitres3 = 0;
    totalMilliLitres4 = 0;
    totalMilliLitres5 = 0;
    totalMilliLitres6 = 0;
    totalMilliLitres7 = 0;
    totalMilliLitres8 = 0;
    totalMilliLitres9 = 0;
    totalMilliLitres10 = 0;
    totalMilliLitres11 = 0;
    totalMilliLitres12 = 0;

    attachInterrupt(digitalPinToInterrupt(SENSOR1), pulseCounter1, FALLING);
    attachInterrupt(digitalPinToInterrupt(SENSOR2), pulseCounter2, FALLING);
    attachInterrupt(digitalPinToInterrupt(SENSOR3), pulseCounter3, FALLING);
    attachInterrupt(digitalPinToInterrupt(SENSOR4), pulseCounter4, FALLING);
    attachInterrupt(digitalPinToInterrupt(SENSOR5), pulseCounter5, FALLING);
    attachInterrupt(digitalPinToInterrupt(SENSOR6), pulseCounter6, FALLING);
    attachInterrupt(digitalPinToInterrupt(SENSOR7), pulseCounter7, FALLING);
    attachInterrupt(digitalPinToInterrupt(SENSOR8), pulseCounter8, FALLING);
    attachInterrupt(digitalPinToInterrupt(SENSOR9), pulseCounter9, FALLING);
    attachInterrupt(digitalPinToInterrupt(SENSOR10), pulseCounter10, FALLING);
    attachInterrupt(digitalPinToInterrupt(SENSOR11), pulseCounter11, FALLING);
    attachInterrupt(digitalPinToInterrupt(SENSOR12), pulseCounter12, FALLING);
}

void sensorwater()
{
    currentMillis = millis();

    if (currentMillis - previousMillis > interval)
    {
        pulse1Sec1 = pulseCount1;
        pulseCount1 = 0;

        pulse1Sec2 = pulseCount2;
        pulseCount2 = 0;

        pulse1Sec3 = pulseCount3;
        pulseCount3 = 0;

        pulse1Sec4 = pulseCount4;
        pulseCount4 = 0;
        pulse1Sec5 = pulseCount5;
        pulseCount5 = 0;
        pulse1Sec6 = pulseCount6;
        pulseCount6 = 0;
        pulse1Sec7 = pulseCount7;
        pulseCount7 = 0;
        pulse1Sec8 = pulseCount8;
        pulseCount8 = 0;
        pulse1Sec9 = pulseCount9;
        pulseCount9 = 0;
        pulse1Sec10 = pulseCount10;
        pulseCount10 = 0;
        pulse1Sec11 = pulseCount11;
        pulseCount11 = 0;
        pulse1Sec12 = pulseCount12;
        pulseCount12 = 0;

        flowRate1 = ((1000.0 / (millis() - previousMillis)) * pulse1Sec1) / calibrationFactor1;
        flowRate2 = ((1000.0 / (millis() - previousMillis)) * pulse1Sec2) / calibrationFactor2;
        flowRate3 = ((1000.0 / (millis() - previousMillis)) * pulse1Sec3) / calibrationFactor3;
        flowRate4 = ((1000.0 / (millis() - previousMillis)) * pulse1Sec4) / calibrationFactor4;
        flowRate5 = ((1000.0 / (millis() - previousMillis)) * pulse1Sec5) / calibrationFactor5;
        flowRate6 = ((1000.0 / (millis() - previousMillis)) * pulse1Sec6) / calibrationFactor6;
        flowRate7 = ((1000.0 / (millis() - previousMillis)) * pulse1Sec7) / calibrationFactor7;
        flowRate8 = ((1000.0 / (millis() - previousMillis)) * pulse1Sec8) / calibrationFactor8;
        flowRate9 = ((1000.0 / (millis() - previousMillis)) * pulse1Sec9) / calibrationFactor9;
        flowRate10 = ((1000.0 / (millis() - previousMillis)) * pulse1Sec10) / calibrationFactor10;
        flowRate11 = ((1000.0 / (millis() - previousMillis)) * pulse1Sec11) / calibrationFactor11;
        flowRate12 = ((1000.0 / (millis() - previousMillis)) * pulse1Sec12) / calibrationFactor12;

        previousMillis = millis();

        flowMilliLitres1 = (flowRate1 / 60) * 1000;
        flowMilliLitres2 = (flowRate2 / 60) * 1000;
        flowMilliLitres3 = (flowRate3 / 60) * 1000;
        flowMilliLitres4 = (flowRate4 / 60) * 1000;
        flowMilliLitres5 = (flowRate5 / 60) * 1000;
        flowMilliLitres6 = (flowRate6 / 60) * 1000;
        flowMilliLitres7 = (flowRate7 / 60) * 1000;
        flowMilliLitres8 = (flowRate8 / 60) * 1000;
        flowMilliLitres9 = (flowRate9 / 60) * 1000;
        flowMilliLitres10 = (flowRate10 / 60) * 1000;
        flowMilliLitres11 = (flowRate11 / 60) * 1000;
        flowMilliLitres12 = (flowRate12 / 60) * 1000;

        totalMilliLitres1 += flowMilliLitres1;
        totalMilliLitres2 += flowMilliLitres2;
        totalMilliLitres3 += flowMilliLitres3;
        totalMilliLitres4 += flowMilliLitres4;
        totalMilliLitres5 += flowMilliLitres5;
        totalMilliLitres6 += flowMilliLitres6;
        totalMilliLitres7 += flowMilliLitres7;
        totalMilliLitres8 += flowMilliLitres8;
        totalMilliLitres9 += flowMilliLitres9;
        totalMilliLitres10 += flowMilliLitres10;
        totalMilliLitres11 += flowMilliLitres11;
        totalMilliLitres12 += flowMilliLitres12;

        Serial.print("Flow 1: ");
        Serial.print(int(flowRate1)); // Print the integer part of the variable
        Serial.print("L/min,   ");

        Serial.print("Flow 2: ");
        Serial.print(int(flowRate2)); // Print the integer part of the variable
        Serial.print("L/min,   ");

        Serial.print("Flow 3: ");
        Serial.print(int(flowRate3)); // Print the integer part of the variable
        Serial.print("L/min,   ");

        Serial.print("Flow 4: ");
        Serial.print(int(flowRate4)); // Print the integer part of the variable
        Serial.println("L/min");
        Serial.print("Flow 5: ");
        Serial.print(int(flowRate5)); // Print the integer part of the variable
        Serial.println("L/min");
        Serial.print("Flow 6: ");
        Serial.print(int(flowRate6)); // Print the integer part of the variable
        Serial.println("L/min");
        Serial.print("Flow 7: ");
        Serial.print(int(flowRate7)); // Print the integer part of the variable
        Serial.println("L/min");
        Serial.print("Flow 8: ");
        Serial.print(int(flowRate8)); // Print the integer part of the variable
        Serial.println("L/min");
        Serial.print("Flow 9: ");
        Serial.print(int(flowRate9)); // Print the integer part of the variable
        Serial.println("L/min");
        Serial.print("Flow 10: ");
        Serial.print(int(flowRate10)); // Print the integer part of the variable
        Serial.println("L/min");
        Serial.print("Flow 11: ");
        Serial.print(int(flowRate11)); // Print the integer part of the variable
        Serial.println("L/min");
        Serial.print("Flow 12: ");
        Serial.print(int(flowRate12)); // Print the integer part of the variable
        Serial.println("L/min");
    }
}

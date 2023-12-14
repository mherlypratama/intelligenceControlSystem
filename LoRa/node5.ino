#include "Wire.h"
#include "Adafruit_INA219.h"
#include <DHT.h>
#include <DHT_U.h>
#include <SPI.h>
#include <LoRa.h>
#include <RTClib.h>

#define AOUT_PIN 36
#define THRESHOLD 1500
#define DHTPIN 15
#define DHTTYPE DHT22
#define ss 5
#define rst 14
#define dio0 4

Adafruit_INA219 ina219;
DHT dht(DHTPIN, DHTTYPE);
RTC_DS3231 rtc;

char daysOfWeek[7][12] = {
    "Sunday",
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday"};

void setup()
{
    // Open serial communications and wait for port to open:
    delay(2000);
    Serial.begin(9600);
    while (!Serial)
    {
        ; // wait for serial port to connect. Needed for native USB port only
    }

    if (!ina219.begin())
    {
        Serial.println("Failed to find INA219 chip");
        while (1)
        {
            delay(500);
        }
    }

    Serial.println(F("DHTxx test!"));
    dht.begin();

    // SETUP RTC MODULE
    if (!rtc.begin())
    {
        Serial.println("RTC module is NOT found");
        Serial.flush();
        while (1)
            ;
    }

    rtc.adjust(DateTime(F(_DATE), F(__TIME_)));

    LoRa.setPins(ss, rst, dio0);
    while (!LoRa.begin(433E6))
    { // Ganti dengan frekuensi yang diinginkan, misalnya 915 MHz
        Serial.println(".");
        delay(500);
    }
    LoRa.setSyncWord(0xF3);
}

void loop()
{
    // INA219
    float shuntvoltage = ina219.getShuntVoltage_mV();
    float busvoltage = ina219.getBusVoltage_V();
    float current_mA = ina219.getCurrent_mA();
    float power_mW = ina219.getPower_mW();
    float loadvoltage = busvoltage + (shuntvoltage / 1000);

    // DHT
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    float f = dht.readTemperature(true);

    // LoRa
    float temp = dht.readTemperature();
    float hum = dht.readHumidity();
    float tempF = dht.readTemperature(true);

    // RTC
    DateTime now = rtc.now();

    // Soil Moisture Sensor
    int value = analogRead(AOUT_PIN);
    int percentage = map(value, 0, 4095, 0, 100);
    float soil = 122 - percentage;

    // Create a string to store the sensor readings
    char utamaStr[1000]; // Buffer untuk menyimpan JSON
    snprintf(utamaStr, sizeof(utamaStr),
             "{"
             "\"TimeStamp\": \"%04d-%02d-%02dT%02d:%02d:%02d+07:00\","
             "\"BusVoltage\": %.2f,"
             "\"ShuntVoltage\": %.2f,"
             "\"LoadVoltage\": %.2f,"
             "\"Current\": %.2f,"
             "\"Power\": %.2f,"
             "\"Humidity\": %.2f,"
             "\"Temperaturec\": %.2f,"
             "\"Temperaturef\": %.2f,"
             "\"Heatindex\": %.2f,"
             "\"soilmoisture\": %.2f,"
             "}",
             now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second(), busvoltage, shuntvoltage, loadvoltage, current_mA, power_mW, h, t, f, dht.computeHeatIndex(t, h, false), soil);

    // String sensorData = "INA219 - Bus Voltage: " + String(busvoltage) + "V, Shunt Voltage: " + String(shuntvoltage) + "mV, Load Voltage: " + String(loadvoltage) + "V, Current: " + String(current_mA) + "mA, Power: " + String(power_mW) + "mW\n";
    // sensorData += "DHT22 - Humidity: " + String(h) + "%, Temperature: " + String(t) + "C, " + String(f) + "F, Heat index: " + String(dht.computeHeatIndex(t, h, false)) + "C, " + String(dht.computeHeatIndex(f, h)) + "F\n";
    // sensorData += "RTC - Date Time: " + String(now.year(), DEC) + "/" + String(now.month(), DEC) + "/" + String(now.day(), DEC) + " (" + String(daysOfWeek[now.dayOfTheWeek()]) + ") " + String(now.hour(), DEC) + ":" + String(now.minute(), DEC) + ":" + String(now.second(), DEC) + "\n";
    // sensorData += "Soil Moisture - The soil is " + String(value > THRESHOLD ? "DRY" : "WET") + " (" + String(122 - percentage) + ")\n";
    String sensorData = utamaStr;
    // Print the sensor readings to Serial
    Serial.println(sensorData);

    // Send LoRa packet to receiver
    LoRa.beginPacket();
    LoRa.print("(Node 6) LoRa - Sending : ");
    LoRa.print(sensorData);
    LoRa.endPacket();

    delay(1000);
}
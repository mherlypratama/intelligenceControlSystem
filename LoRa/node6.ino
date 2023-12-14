#include <DHT.h>
#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_INA219.h>

#define DHT_SENSOR_PIN 15 // Pin sensor DHT
#define DHT_SENSOR_TYPE DHT22

#define SS_PIN 5   // Pin sensor LoRa
#define RST_PIN 14 // Pin sensor LoRa
#define DIO0_PIN 4 // Pin sensor LoRa

#define SOIL_SENSOR_PIN 36 // Pin sensor Soil Moisture

Adafruit_INA219 ina219;
DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
RTC_DS3231 rtc;

int moisture, sensor_analog;
float humidity, temperature, current_mA, power_mW, loadVoltage;

void setup()
{
    Serial.begin(9600);

    setDHT22();
    setSoil();
    setINA219();

    if (!rtc.begin())
    {
        Serial.println("Couldn't find RTC");
        while (1)
            ;
    }

    rtc.adjust(DateTime(F(_DATE), F(__TIME_)));

    ina219.begin();

    Serial.println("Measuring voltage and current with INA219 ...");

    while (!Serial)
    {
        delay(1);
    }

    Serial.println("LoRa Sender");
    LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);

    while (!LoRa.begin(433E6))
    {
        Serial.println(".");
        delay(2000);
    }

    LoRa.setSyncWord(0xF3);
    Serial.println("LoRa Sender");
}

void loop()
{
    DateTime now = rtc.now();
    readDHT22();
    readSoil();
    readINA219();

    char utamaStr[1000]; // Buffer untuk menyimpan JSON
    snprintf(utamaStr, sizeof(utamaStr),
             "{"
             "\"TimeStamp\": \"%04d-%02d-%02dT%02d:%02d:%02d+07:00\","
             "\"LoadVoltage\": %.2f,"
             "\"Current\": %.2f,"
             "\"Power\": %.2f,"
             "\"Humidity\": %.2f,"
             "\"Temperaturec\": %.2f,"
             "\"soilmoisture\": %.2f,"
             "}",
             now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second(), loadvoltage, current_mA, power_mW, humidity, temperature, moisture);
    String sensorData = utamaStr;
    LoRa.beginPacket();
    LoRa.print("(Node 6) LoRa - Sending : ");
    LoRa.print(sensorData);
    LoRa.endPacket();
    Serial.println("=============================================================");
    delay(2000);
}

void printDateTime(DateTime dt)
{
    Serial.print(dt.year(), DEC);
    Serial.print('/');
    Serial.print(dt.month(), DEC);
    Serial.print('/');
    Serial.print(dt.day(), DEC);
    Serial.print(" ");
    Serial.print(dt.hour(), DEC);
    Serial.print(':');
    Serial.print(dt.minute(), DEC);
    Serial.print(':');
    Serial.print(dt.second(), DEC);
}

void setDHT22()
{
    dht_sensor.begin();
}

void readDHT22()
{
    humidity = dht_sensor.readHumidity();
    temperature = dht_sensor.readTemperature();

    if (isnan(temperature) || isnan(humidity))
    {
        Serial.println("Failed to read from DHT sensor!");
    }
    else
    {

        Serial.print(" | Humidity: ");
        Serial.print(humidity);
        Serial.print("% | Temperature: ");
        Serial.print(temperature);
        Serial.println("°C");
    }
}

void setSoil()
{
    // No specific setup needed for soil moisture sensor
}

void readSoil()
{
    sensor_analog = analogRead(SOIL_SENSOR_PIN);

    if (sensor_analog > 2550)
    {
        moisture = 0;
    }
    else if (sensor_analog < 1450)
    {
        moisture = 100;
    }
    else
    {
        moisture = (1 - ((sensor_analog - 1450) / 1100)) * 100;
    }

    Serial.print(" | Moisture = ");
    Serial.print(moisture);
    Serial.println("%");
}

void setINA219()
{
    // No specific setup needed for INA219
}

void readINA219()
{
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    loadVoltage = ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV() / 1000);

    Serial.print(" | Load Voltage:  ");
    Serial.print(loadVoltage);
    Serial.println(" V");

    Serial.print(" | Current:       ");
    Serial.print(current_mA);
    Serial.println(" mA");

    Serial.print(" | Power:         ");
    Serial.print(power_mW);
    Serial.println(" mW");
    Serial.println("");
}

void printDateTime(DateTime dt)
{
    Serial.print(dt.year(), DEC);
    Serial.print('/');
    Serial.print(dt.month(), DEC);
    Serial.print('/');
    Serial.print(dt.day(), DEC);
    Serial.print(" ");
    Serial.print(dt.hour(), DEC);
    Serial.print(':');
    Serial.print(dt.minute(), DEC);
    Serial.print(':');
    Serial.print(dt.second(), DEC);
}
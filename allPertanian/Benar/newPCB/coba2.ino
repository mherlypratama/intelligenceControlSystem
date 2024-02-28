#include <OneWire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "time.h"
#include <Wire.h>
#include <SoftwareSerial.h>

#include <DFRobot_BMP3XX.h>

DFRobot_BMP388_I2C sensor(&Wire, sensor.eSDOVDD);
#define CALIBRATE_ABSOLUTE_DIFFERENCE

// Konfigurasi jaringan Wi-Fi
const char *ssid = "pertanian24";
const char *password = "luarbiasa";

// Konfigurasi server MQTT di VPS Anda
const char *mqtt_server = "vps.isi-net.org";
const int mqtt_port = 1883;
const char *mqtt_user = "unila";
const char *mqtt_password = "pwdMQTT@123";

const char *topic_utama = "ics/pertanian3";

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 25200;
const int daylightOffset_sec = 3600;

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsgTime = 0;
const long Interval = 5000; // Kirim data setiap 5 detik

#include <HX711_ADC.h>
#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

// pins:
const int HX711_dout_1 = 14; // mcu > HX711 no 1 dout pin
const int HX711_sck_1 = 27;  // mcu > HX711 no 1 sck pin
const int HX711_dout_2 = 13; // mcu > HX711 no 2 dout pin
const int HX711_sck_2 = 12;  // mcu > HX711 no 2 sck pin

#define RE 25
#define DE 33
const byte pyranometer[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
byte values[8];
SoftwareSerial mod(32, 26);

#define RELAY_PIN1 17
#define RELAY_PIN2 17
#define RELAY_PIN3 16

// HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1); // HX711 1
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2); // HX711 2

const int calVal_eepromAdress_1 = 0; // eeprom adress for calibration value load cell 1 (4 bytes)
const int calVal_eepromAdress_2 = 4; // eeprom adress for calibration value load cell 2 (4 bytes)
unsigned long t = 0;

// *************Variabel Utama
float berat2, berat3, berat4;
float temperaturebmp, Pressure, humibmp;
int jam, minute, second, tanggal, bulan, tahun, relay1, relay2, relay3, beratdum2;

void setup(void)
{
    Serial.begin(9600);
    setupWiFi();
    client.setServer(mqtt_server, mqtt_port);
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    // setberat();
    setpyrano();
    setrelay();
    setbmp();

    printLocalTime();
}

void loop()
{
    if (!client.connected())
    {
        reconnectMQTT();
    }
    client.loop();
    printLocalTime();
    beratdum();
    // sensorberat();
    sensorbmp();
    sensorpyrano();
    relay11();
    relay22();
    relay33();
    nodered();

    delay(10000);
}

void nodered()
{

    // Buat objek JSON yang berisi data dari keempat sensor
    char utamaStr[1000]; // Buffer untuk menyimpan JSON
    snprintf(utamaStr, sizeof(utamaStr),
             "{"
             "\"TimeStamp\": \"%04d-%02d-%02dT%02d:%02d:%02d+07:00\","
             "\"temperaturebmp\": %.2f,"
             "\"humibmp\": %.2f,"
             "\"pressurebmp\": %.2f,"
             "\"berat2\": %.2f,"
             "\"berat3\": %.2f,"
             "\"berat4\": %.2f,"
             "\"relay1\": %.d,"
             "\"relay2\": %.d,"
             "\"relay3\": %.d"
             "}",
             tahun, bulan, tanggal, jam, minute, second, temperaturebmp, humibmp, Pressure, beratdum2, beratdum2, beratdum2, relay1, relay2, relay3);

    client.publish(topic_utama, utamaStr); // Mengirim data suhu ke broker MQTT
}

void beratdum()
{
    beratdum2 = 0;
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

    float altitude = sensor.readAltitudeM();
    Serial.print("Altitude : ");
    Serial.print(altitude);
    Serial.println(" m");

    humibmp = 87.6 - temperaturebmp;
    Serial.print("humibmp : ");
    Serial.print(humibmp);
    Serial.println(" %");

    Serial.println();
    delay(1000);
}

void setrelay()
{
    pinMode(RELAY_PIN1, OUTPUT);
    pinMode(RELAY_PIN2, OUTPUT);
    pinMode(RELAY_PIN3, OUTPUT);
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
    int Solar_Radiation = int(values[3] << 8 | values[4]);
    Serial.print("Solar Radiation: ");
    Serial.print(Solar_Radiation);
    Serial.println(" W/m^2");
}

// void setberat()
// {
//     Serial.println();
//     Serial.println("Starting...");

//     float calibrationValue_1; // calibration value load cell 1
//     float calibrationValue_2; // calibration value load cell 2

//     calibrationValue_1 = 108.75; // uncomment this if you want to set this value in the sketch
//     calibrationValue_2 = 108.75; // uncomment this if you want to set this value in the sketch
// #if defined(ESP8266) || defined(ESP32)
//     // EEPROM.begin(512); // uncomment this if you use ESP8266 and want to fetch the value from eeprom
// #endif
//     // EEPROM.get(calVal_eepromAdress_1, calibrationValue_1); // uncomment this if you want to fetch the value from eeprom
//     // EEPROM.get(calVal_eepromAdress_2, calibrationValue_2); // uncomment this if you want to fetch the value from eeprom

//     LoadCell_1.begin();
//     LoadCell_2.begin();
//     // LoadCell_1.setReverseOutput();
//     // LoadCell_2.setReverseOutput();
//     unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
//     boolean _tare = true;                 // set this to false if you don't want tare to be performed in the next step
//     byte loadcell_1_rdy = 0;
//     byte loadcell_2_rdy = 0;
//     while ((loadcell_1_rdy + loadcell_2_rdy) < 2)
//     { // run startup, stabilization and tare, both modules simultaniously
//         if (!loadcell_1_rdy)
//             loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
//         if (!loadcell_2_rdy)
//             loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
//     }
//     if (LoadCell_1.getTareTimeoutFlag())
//     {
//         Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations");
//     }
//     if (LoadCell_2.getTareTimeoutFlag())
//     {
//         Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations");
//     }
//     LoadCell_1.setCalFactor(calibrationValue_1); // user set calibration value (float)
//     LoadCell_2.setCalFactor(calibrationValue_2); // user set calibration value (float)
//     Serial.println("Startup is complete");
// }

// void sensorberat()
// {
//     static boolean newDataReady = 0;
//     const int serialPrintInterval = 0; // increase value to slow down serial print activity

//     // check for new data/start next conversion:
//     if (LoadCell_1.update())
//         newDataReady = true;
//     LoadCell_2.update();

//     // get smoothed value from data set
//     if ((newDataReady))
//     {
//         if (millis() > t + serialPrintInterval)
//         {
//             berat2 = LoadCell_1.getData();
//             berat3 = LoadCell_2.getData();
//             berat4 = (berat2 + berat3) / 2;
//             Serial.print("Load_cell 1 output val: ");
//             Serial.print(a);
//             Serial.print("    Load_cell 2 output val: ");
//             Serial.println(b);
//             newDataReady = 0;
//             t = millis();
//         }
//     }

//     // receive command from serial terminal, send 't' to initiate tare operation:
//     if (Serial.available() > 0)
//     {
//         char inByte = Serial.read();
//         if (inByte == 't')
//         {
//             LoadCell_1.tareNoDelay();
//             LoadCell_2.tareNoDelay();
//         }
//     }

//     // check if last tare operation is complete
//     if (LoadCell_1.getTareStatus() == true)
//     {
//         Serial.println("Tare load cell 1 complete");
//     }
//     if (LoadCell_2.getTareStatus() == true)
//     {
//         Serial.println("Tare load cell 2 complete");
//     }
// }

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

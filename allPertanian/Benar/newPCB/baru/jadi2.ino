#include <OneWire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include "time.h"
#include <DFRobot_BMP3XX.h>
#include <HX711_ADC.h>
#include <DHT.h>

#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

// Konfigurasi jaringan Wi-Fi
const char *ssid = "pertanian24";
const char *password = "luarbiasa";

// Konfigurasi server MQTT di VPS Anda
const char *mqtt_server = "vps.isi-net.org";
const int mqtt_port = 1883;
const char *mqtt_user = "unila";
const char *mqtt_password = "pwdMQTT@123";

const char *topic_ketiga = "ics/pertanian3";

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 18000;
const int daylightOffset_sec = 3600;

WiFiClient espClient;
PubSubClient client(espClient);

const int HX711_dout_1 = 34; // mcu > HX711 no 1 dout pin
const int HX711_sck_1 = 35;  // mcu > HX711 no 1 sck pin
const int HX711_dout_2 = 13; // mcu > HX711 no 2 dout pin
const int HX711_sck_2 = 12;  // mcu > HX711 no 2 sck pin
const int HX711_dout_3 = 14; // mcu > HX711 no 3 dout pin
const int HX711_sck_3 = 27;  // mcu > HX711 no 3 sck pin
// HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1); // HX711 1
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2); // HX711 2
HX711_ADC LoadCell_3(HX711_dout_3, HX711_sck_3); // HX711 3

const int calVal_eepromAdress_1 = 0; // eeprom address for calibration value load cell 1 (4 bytes)
const int calVal_eepromAdress_2 = 4; // eeprom address for calibration value load cell 2 (4 bytes)
const int calVal_eepromAdress_3 = 8; // eeprom address for calibration value load cell 3 (4 bytes)
unsigned long t = 0;

float a, b, c, d;

// Constants
#define DHTPIN 5          // what pin we're connected to
#define DHTTYPE DHT22     // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino

// Variables
int chk;
float humiditydht;
float temperaturedht;

unsigned long lastMsgTime = 0;
const long interval = 5000; // Kirim data setiap 5 detik

DFRobot_BMP388_I2C sensor(&Wire, sensor.eSDOVDD);
#define CALIBRATE_ABSOLUTE_DIFFERENCE

// pins:
#define RELAY_PIN1 15
#define RELAY_PIN2 2
#define RELAY_PIN3 4

int Solar_Radiation, relay1, relay2, relay3;
int jam, minute, second, tanggal, bulan, tahun;

float temperaturebmp, Pressure, humi, beratdum = 7301, beratdum2, beratdum3, beratdum4;

void setup(void)
{
    Serial.begin(9600);
    setupWiFi();
    client.setServer(mqtt_server, mqtt_port);
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    printLocalTime();

    dht.begin();

    pinMode(RELAY_PIN1, OUTPUT);
    pinMode(RELAY_PIN2, OUTPUT);
    pinMode(RELAY_PIN3, OUTPUT);

    setbmp();
    setberat();
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
    sensorberat();
    relay11();
    relay22();
    relay33();
    delay(1000);
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

    delay(5000);
}

void nodered()
{

    // Buat objek JSON yang berisi data dari keempat sensor
    char utamaStr[1000]; // Buffer untuk menyimpan JSON
    snprintf(utamaStr, sizeof(utamaStr),
             "{"
             "\"TimeStamp\": \"%04d-%02d-%02dT%02d:%02d:%02d+07:00\","
             "\"temperaturebmp\": %.2f,"
             "\"temperaturedht\": %.2f,"
             "\"humidity\": %.2f,"
             "\"pressure\": %.2f,"
             "\"berat_1\": %.2f,"
             "\"berat_2\": %.2f,"
             "\"berat_3\": %.2f,"
             "\"berat_4\": %.2f,"
             "\"lampu_uv\": %d,"
             "\"pompanutrisi\": %d,"
             "\"pompapendingin\": %d"
             "}",
             tahun, bulan, tanggal, jam, minute, second, temperaturebmp, temperaturedht, humiditydht, Pressure, d, a, b, c, relay1, relay2, relay3);

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

void setberat()
{
    Serial.println();
    Serial.println("Starting...");

    float calibrationValue_1; // calibration value load cell 1
    float calibrationValue_2; // calibration value load cell 2
    float calibrationValue_3; // calibration value load cell 3

    calibrationValue_1 = 108.75; // uncomment this if you want to set this value in the sketch
    calibrationValue_2 = 108.75; // uncomment this if you want to set this value in the sketch
    calibrationValue_3 = 108.75; // uncomment this if you want to set this value in the sketch
#if defined(ESP8266) || defined(ESP32)
    // EEPROM.begin(512); // uncomment this if you use ESP8266 and want to fetch the value from eeprom
#endif
    // EEPROM.get(calVal_eepromAdress_1, calibrationValue_1); // uncomment this if you want to fetch the value from eeprom
    // EEPROM.get(calVal_eepromAdress_2, calibrationValue_2); // uncomment this if you want to fetch the value from eeprom
    // EEPROM.get(calVal_eepromAdress_3, calibrationValue_3); // uncomment this if you want to fetch the value from eeprom

    LoadCell_1.begin();
    LoadCell_2.begin();
    LoadCell_3.begin();

    unsigned long stabilizingtime = 2000;
    boolean _tare = true;
    byte loadcell_1_rdy = 0;
    byte loadcell_2_rdy = 0;
    byte loadcell_3_rdy = 0;

    while ((loadcell_1_rdy + loadcell_2_rdy + loadcell_3_rdy) < 3)
    {
        if (!loadcell_1_rdy)
            loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
        if (!loadcell_2_rdy)
            loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
        if (!loadcell_3_rdy)
            loadcell_3_rdy = LoadCell_3.startMultiple(stabilizingtime, _tare);
    }

    if (LoadCell_1.getTareTimeoutFlag())
    {
        Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations");
    }
    if (LoadCell_2.getTareTimeoutFlag())
    {
        Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations");
    }
    if (LoadCell_3.getTareTimeoutFlag())
    {
        Serial.println("Timeout, check MCU>HX711 no.3 wiring and pin designations");
    }

    LoadCell_1.setCalFactor(calibrationValue_1);
    LoadCell_2.setCalFactor(calibrationValue_2);
    LoadCell_3.setCalFactor(calibrationValue_3);

    Serial.println("Startup is complete");
}

void sensorberat()
{
    static boolean newDataReady = 0;
    const int serialPrintInterval = 0;

    if (LoadCell_1.update())
        newDataReady = true;
    LoadCell_2.update();
    LoadCell_3.update();

    if (newDataReady)
    {
        if (millis() > t + serialPrintInterval)
        {
            a = LoadCell_1.getData();
            b = LoadCell_2.getData();
            c = LoadCell_3.getData();
            d = (a + b + c) / 3;
            Serial.print("Load_cell 1 output val: ");
            Serial.print(a);
            Serial.print("    Load_cell 2 output val: ");
            Serial.print(b);
            Serial.print("    Load_cell 3 output val: ");
            Serial.println(c);
            newDataReady = 0;
            t = millis();
        }
    }

    if (Serial.available() > 0)
    {
        char inByte = Serial.read();
        if (inByte == 't')
        {
            LoadCell_1.tareNoDelay();
            LoadCell_2.tareNoDelay();
            LoadCell_3.tareNoDelay();
        }
    }

    if (LoadCell_1.getTareStatus() == true)
    {
        Serial.println("Tare load cell 1 complete");
    }
    if (LoadCell_2.getTareStatus() == true)
    {
        Serial.println("Tare load cell 2 complete");
    }
    if (LoadCell_3.getTareStatus() == true)
    {
        Serial.println("Tare load cell 3 complete");
    }
}
void berat()
{
    if (jam == 6 && minute == 1 || jam == 10 && minute == 1 || jam == 13 && minute == 1)
    {
        beratdum = 7341;
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
    if (jam == 16 && minute >= 1 && minute < 3 || jam == 6 && minute >= 1 && minute < 3 || jam == 10 && minute >= 1 && minute < 3 || jam == 13 && minute >= 1 && minute < 3)
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
    if (temperaturedht > 33)
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

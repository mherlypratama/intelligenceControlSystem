#include <WiFi.h>
#include "time.h"
#include <OneWire.h>
#include <PubSubClient.h>

#include <DFRobot_BMP3XX.h>

#include <HX711_ADC.h>
#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

// pins:
#define RELAY_PIN 26
#define RELAY_PIN2 0
#define RELAY_PIN3 14

const int HX711_dout_1 = 17; // mcu > HX711 no 1 dout pin
const int HX711_sck_1 = 16;  // mcu > HX711 no 1 sck pin
const int HX711_dout_2 = 12; // mcu > HX711 no 2 dout pin
const int HX711_sck_2 = 4;   // mcu > HX711 no 2 sck pin

// HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1); // HX711 1
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2); // HX711 2

const int calVal_eepromAdress_1 = 0; // eeprom adress for calibration value load cell 1 (4 bytes)
const int calVal_eepromAdress_2 = 4; // eeprom adress for calibration value load cell 2 (4 bytes)
unsigned long t = 0;

DFRobot_BMP388_I2C sensor(&Wire, sensor.eSDOVDD);
#define CALIBRATE_ABSOLUTE_DIFFERENCE

const char *ssid = "Lab Telkom 2.4 GHz";
const char *password = "telekomunikasi";

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 21600;
const int daylightOffset_sec = 3600;

// Konfigurasi server MQTT di VPS Anda
const char *mqtt_server = "vps.isi-net.org";
const int mqtt_port = 1883;
const char *mqtt_user = "unila";
const char *mqtt_password = "pwdMQTT@123";

const char *topic_ketiga = "ics/pertanian";

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsgTime = 0;
const long interval = 5000;

int relay1, relay2, relay3;

float temperature, Pressure, a, b, c;

void setup()
{
    Serial.begin(9600);
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(RELAY_PIN2, OUTPUT);
    pinMode(RELAY_PIN3, OUTPUT);
    setBmp();
    setBerat();

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
    client.setServer(mqtt_server, mqtt_port);

    // Init and get the time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    // disconnect WiFi as it's no longer needed
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
}

void loop()
{
    if (!client.connected())
    {
        reconnectMQTT();
    }
    client.loop();

    nodered();

    sensorBmp();
    sensorBerat();
    relay11(); // Untuk Lampu UV
    relay22();
    relay33();
    delay(1000);
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
             "\"temperatureBMP\": %.2f,"
             "\"pressure\": %.2f,"
             "\"berat_2\": %.2f,"
             "\"berat_3\": %.2f,"
             "\"berat_4\": %.2f,"
             "\"lampu_uv\": %.2f,"
             "\"pompanutrisi\": %.2f,"
             "\"pompapendingin\": %.2f"
             "}",
             temperature, Pressure, a, b, c, relay1, relay2, relay3);

    client.publish(topic_ketiga, utamaStr);

    // char phStr[10];
    // snprintf(phStr, sizeof(phStr), "%.2f", phValue); // Mengonversi nilai pH ke string
    // client.publish(topic_ph, phStr);                 // Mengirim data pH ke broker MQTT

    // // Kirim data TDS ke broker MQTT
    // char tdsStr[10];
    // snprintf(tdsStr, sizeof(tdsStr), "%d", (int)tdsValue); // Mengonversi nilai TDS ke string
    // client.publish(topic_tds, tdsStr);                     // Mengirim data TDS ke broker MQTT

    // // Kirim data sensor hujan ke broker MQTT
    // char rainStr[10];
    // snprintf(rainStr, sizeof(rainStr), "%.2f", rainAccumulated); // Mengonversi jumlah hujan ke string
    // client.publish(topic_rain, rainStr);                         // Mengirim data hujan ke broker MQTT

    // // Kirim data suhu ke broker MQTT
    // char tempStr[10];
    // snprintf(tempStr, sizeof(tempStr), "%.2f", tempe); // Mengonversi nilai suhu ke string
    // client.publish(topic_ds, tempStr);                 // Mengirim data suhu ke broker MQTT

    // // Kirim data suhu ke broker MQTT
    // char windStr[10];
    // snprintf(windStr, sizeof(windStr), "%.2f", Angle); // Mengonversi nilai suhu ke string
    // client.publish(topic_winddir, windStr);            // Mengirim data suhu ke broker MQTT

    // // Kirim data suhu ke broker MQTT
    // char anemStr[10];
    // snprintf(anemStr, sizeof(anemStr), "%.2f", readWindSpeed(Address0)); // Mengonversi nilai suhu ke string
    // client.publish(topic_wind, anemStr);                                 // Mengirim data suhu ke broker MQTT

    // // Kirim data suhu ke broker MQTT
    // char infraStr[10];
    // snprintf(infraStr, sizeof(infraStr), "%.2f", mlx.readObjectTempC()); // Mengonversi nilai suhu ke string
    // client.publish(topic_infra, infraStr);                               // Mengirim data suhu ke broker MQTT

    // // Buat objek JSON yang berisi data dari keempat sensor
    // char bmeStr[100]; // Buffer untuk menyimpan JSON
    // snprintf(bmeStr, sizeof(bmeStr), "{\"Temperature\": %.2f, \"Preassure\": %.2f, \"Altitude\": %.2f}", temp, Pressure, altitude);
    // client.publish(topic_bme, bmeStr);

    // // Buat objek JSON yang berisi data dari keempat sensor
    // char waterStr[100]; // Buffer untuk menyimpan JSON
    // snprintf(waterStr, sizeof(waterStr), "{\"Water_1\": %.2f, \"Water_2\": %.2f, \"Water_3\": %.2f, \"Water_4\": %.2f}", flowRate1, flowRate2, flowRate3, flowRate4);
    // client.publish(topic_water, waterStr);

    // // Kirim data suhu ke broker MQTT
    // char beratStr[10];
    // snprintf(beratStr, sizeof(beratStr), "%.2f", i); // Mengonversi nilai suhu ke string
    // client.publish(topic_weight, beratStr);          // Mengirim data suhu ke broker MQTT
}

void setBerat()
{
    Serial.println();
    Serial.println("Starting...");

    float calibrationValue_1; // calibration value load cell 1
    float calibrationValue_2; // calibration value load cell 2

    calibrationValue_1 = 108.75; // uncomment this if you want to set this value in the sketch
    calibrationValue_2 = 108.75; // uncomment this if you want to set this value in the sketch
#if defined(ESP8266) || defined(ESP32)
    // EEPROM.begin(512); // uncomment this if you use ESP8266 and want to fetch the value from eeprom
#endif
    // EEPROM.get(calVal_eepromAdress_1, calibrationValue_1); // uncomment this if you want to fetch the value from eeprom
    // EEPROM.get(calVal_eepromAdress_2, calibrationValue_2); // uncomment this if you want to fetch the value from eeprom

    LoadCell_1.begin();
    LoadCell_2.begin();
    // LoadCell_1.setReverseOutput();
    // LoadCell_2.setReverseOutput();
    unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
    boolean _tare = true;                 // set this to false if you don't want tare to be performed in the next step
    byte loadcell_1_rdy = 0;
    byte loadcell_2_rdy = 0;
    while ((loadcell_1_rdy + loadcell_2_rdy) < 2)
    { // run startup, stabilization and tare, both modules simultaniously
        if (!loadcell_1_rdy)
            loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
        if (!loadcell_2_rdy)
            loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
    }
    if (LoadCell_1.getTareTimeoutFlag())
    {
        Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations");
    }
    if (LoadCell_2.getTareTimeoutFlag())
    {
        Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations");
    }
    LoadCell_1.setCalFactor(calibrationValue_1); // user set calibration value (float)
    LoadCell_2.setCalFactor(calibrationValue_2); // user set calibration value (float)
    Serial.println("Startup is complete");
}

void sensorBerat()
{
    static boolean newDataReady = 0;
    const int serialPrintInterval = 0; // increase value to slow down serial print activity

    // check for new data/start next conversion:
    if (LoadCell_1.update())
        newDataReady = true;
    LoadCell_2.update();

    // get smoothed value from data set
    if ((newDataReady))
    {
        if (millis() > t + serialPrintInterval)
        {
            a = LoadCell_1.getData();
            b = LoadCell_2.getData();
            Serial.print("Load_cell 1 output val: ");
            Serial.print(a);
            Serial.print("    Load_cell 2 output val: ");
            Serial.println(b);
            c = b;
            newDataReady = 0;
            t = millis();
        }
    }

    // receive command from serial terminal, send 't' to initiate tare operation:
    if (Serial.available() > 0)
    {
        char inByte = Serial.read();
        if (inByte == 't')
        {
            LoadCell_1.tareNoDelay();
            LoadCell_2.tareNoDelay();
        }
    }

    // check if last tare operation is complete
    if (LoadCell_1.getTareStatus() == true)
    {
        Serial.println("Tare load cell 1 complete");
    }
    if (LoadCell_2.getTareStatus() == true)
    {
        Serial.println("Tare load cell 2 complete");
    }
}

void setBmp()
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

void sensorBmp()
{
    temperature = sensor.readTempC();
    Serial.print("temperature : ");
    Serial.print(temperature);
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
    if (jam >= 18 && jam < 6)
    {
        digitalWrite(RELAY_PIN, LOW);
        relay1 = 1;
    }
    else
    {
        digitalWrite(RELAY_PIN, HIGH);
        relay1 = 0;
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
    // Extract the hour (0-23) from the struct tm
    int jam = timeinfo.tm_hour;
    int minute = timeinfo.tm_min;

    // Kontrol RELAY_PIN2
    if (jam == 16 && minute >= 1 && minute < 16 || jam == 6 && minute >= 1 && minute < 16)
    {
        digitalWrite(RELAY_PIN2, LOW);
        relay2 = 1;
    }
    else
    {
        digitalWrite(RELAY_PIN2, HIGH);
        relay2 = 0;
    }
}

// air pendingin
void relay33()
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

    // Kontrol RELAY_PIN2
    if (temperature > 28)
    {
        digitalWrite(RELAY_PIN3, LOW);
        relay3 = 1;
    }
    else
    {
        digitalWrite(RELAY_PIN3, HIGH);
        relay3 = 0;
    }
}

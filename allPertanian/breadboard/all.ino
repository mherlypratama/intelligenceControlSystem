#include <SoftwareSerial.h>
#include <OneWire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include "time.h"
#include <DFRobot_BMP3XX.h>

#include "DFRobot_ESP_PH.h"
#include "EEPROM.h"

#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads;

float fix0, fix1, fix2, fix3;

#include <DHT.h>

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
const long gmtOffset_sec = 25200;
const int daylightOffset_sec = 3600;

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsgTime = 0;
const long interval = 5000; // Kirim data setiap 5 detik

DFRobot_BMP388_I2C sensor(&Wire, sensor.eSDOVDD);
#define CALIBRATE_ABSOLUTE_DIFFERENCE

// pins:
#define RELAY_PIN 26
#define RELAY_PIN2 25
#define RELAY_PIN3 33

#define RE 13
#define DE 12

const byte pyranometer[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
byte values[8];
SoftwareSerial mod(14, 27);

// pH
DFRobot_ESP_PH ph;
#define ESPADC 4096.0   // the esp Analog Digital Convertion value
#define ESPVOLTAGE 3300 // the esp voltage supply value
#define PH_PIN 32       // the esp gpio data pin number
float voltage, phValue, temperatureph = 25;

// DHT
#define DHTPIN 35     // Pin data sensor DHT22 terhubung ke pin 2 pada ESP32
#define DHTTYPE DHT22 // Jenis sensor, sesuaikan dengan sensor yang Anda gunakan

DHT dht(DHTPIN, DHTTYPE);

int Solar_Radiation, relay1, relay2, relay3;
int jam, minute, second, tanggal, bulan, tahun;

float temperature, Pressure, humi, beratdum = 7301, beratdum2, beratdum3, beratdum4, humidht, tempdht;

void setup(void)
{
    Serial.begin(9600);
    setupWiFi();
    client.setServer(mqtt_server, mqtt_port);
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    printLocalTime();
    dht.begin();

    pinMode(RELAY_PIN, OUTPUT);
    pinMode(RELAY_PIN2, OUTPUT);
    pinMode(RELAY_PIN3, OUTPUT);
    setph();
    setsoil();

    setbmp();
    setpyrano();
}

void loop()
{
    if (!client.connected())
    {
        reconnectMQTT();
    }
    client.loop();
    printLocalTime();
    sensorph();
    sensorbmp();
    sensorpyrano();
    berat();
    relay11();
    relay22();
    relay33();
    sensorsoil();
    sensorDHT();
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

    delay(60000);
}

void nodered()
{

    // Buat objek JSON yang berisi data dari keempat sensor
    char utamaStr[1000]; // Buffer untuk menyimpan JSON
    snprintf(utamaStr, sizeof(utamaStr),
             "{"
             "\"TimeStamp\": \"%04d-%02d-%02dT%02d:%02d:%02d+07:00\","
             "\"pyrano\": %d,"
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
             tahun, bulan, tanggal, jam, minute, second, Solar_Radiation, temperature, humi, Pressure, beratdum, beratdum2, beratdum3, relay1, relay2, relay3);

    client.publish(topic_ketiga, utamaStr); // Mengirim data suhu ke broker MQTT

    // Buat objek JSON yang berisi data dari keempat sensor
    char utamaStr[1000]; // Buffer untuk menyimpan JSON
    snprintf(utamaStr, sizeof(utamaStr),
             "{"
             "\"TimeStamp\": \"%04d-%02d-%02dT%02d:%02d:%02d+07:00\","
             "\"soil1\": %.2f,"
             "\"soil2\": %.2f,"
             "\"soil3\": %.2f,"
             "\"soil4\": %.2f,"
             "\"ph2\": %.2f,"
             "\"humidht\": %.2f,"
             "\"tempdht\": %.2f"
             "}",
             tahun, bulan, tanggal, jam, minute, second, fix0, fix1, fix2, fix3, phValue, humidht, tempdht);

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

void sensorDHT()
{
    // Baca data dari sensor DHT22
    humidht = dht.readHumidity();
    tempdht = dht.readTemperature();

    // Periksa apakah pembacaan sensor berhasil
    if (isnan(humidity) || isnan(temperature))
    {
        Serial.println("Gagal membaca data dari sensor DHT22!");
        return;
    }

    // Tampilkan nilai kelembaban dan suhu
    Serial.print("Kelembaban: ");
    Serial.print(humidity);
    Serial.print("%\t");
    Serial.print("Suhu: ");
    Serial.print(temperature);
    Serial.println("°C");
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
}

void setph()
{
    EEPROM.begin(32); // needed to permit storage of calibration value in eeprom
    ph.begin();
}

void sensorph()
{
    static unsigned long timepoint = millis();
    if (millis() - timepoint > 1000U) // time interval: 1s
    {
        timepoint = millis();
        // voltage = rawPinValue / esp32ADC * esp32Vin
        voltage = analogRead(PH_PIN) / ESPADC * ESPVOLTAGE; // read the voltage
        Serial.print("voltage:");
        Serial.println(voltage, 4);

        phValue = ph.readPH(voltage, temperatureph); // convert voltage to pH with temperatureph compensation
        Serial.print("pH:");
        Serial.println(phValue, 4);
    }
    ph.calibration(voltage, temperatureph);
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
    temperature = sensor.readTempC();
    Serial.print("temperature : ");
    Serial.print(temperature);
    Serial.println(" C");

    humi = 84 - temperature;
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

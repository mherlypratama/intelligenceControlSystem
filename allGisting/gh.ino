#include <OneWire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include "time.h"

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#include <DHT.h>

#include <SoftwareSerial.h>

#include <Adafruit_MLX90614.h>

// Konfigurasi jaringan Wi-Fi
const char *ssid = "Lab Telkom 2.4 GHz";
const char *password = "telekomunikasi";

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

// **************************SENSOR WIND*******************************
#define ADS1115_ADDRESS 0x48
const int windDirectionPin = 0; // Channel 0 pada ADS1115
Adafruit_ADS1115 ads;
// **************************END*******************************

// **************************SENSOR DHT*******************************
#define DHTPIN 3      // Pin data sensor DHT22 terhubung ke pin 2 pada ESP32
#define DHTTYPE DHT22 // Jenis sensor, sesuaikan dengan sensor yang Anda gunakan
DHT dht(DHTPIN, DHTTYPE);
// **************************END*******************************

// **************************SENSOR ANEMO*******************************
SoftwareSerial mySerial2(33, 32); // Define the soft serial port, port 3 is TX, port 2 is RX,
uint8_t Address0 = 0x10;
// **************************END*******************************

// **************************SENSOR INFRARED*******************************
// TCA9548A I2C Multiplexer Address
#define TCAADDR 0x70
Adafruit_MLX90614 mlx1 = Adafruit_MLX90614(); // Objek untuk sensor pertama
Adafruit_MLX90614 mlx2 = Adafruit_MLX90614(); // Objek untuk sensor kedua
// **************************END*******************************

int jam, minute, second, tanggal, bulan, tahun;
float humidity, temperature, degrees;

void selectTCAChannel(uint8_t channel)
{
    // Select the appropriate channel on TCA9548A
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

void setup(void)
{
    Serial.begin(9600);
    setupWiFi();
    client.setServer(mqtt_server, mqtt_port);
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    printLocalTime();
    setwind();
    setdht();
    setanemo();
    setinfra();
}

void loop()
{
    if (!client.connected())
    {
        reconnectMQTT();
    }
    client.loop();
    printLocalTime();
    sensordht();
    sensorwind();
    sensoranemo();
    sensorinfra();

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
             "\"temperature\": %.2f,"
             "\"humidity\": %.2f,"
             "\"infra1\": %.2f,"
             "\"infra2\": %.2f,"
             "\"windspeed\": %.2f,"
             "\"winddirection\": %.2f"
             "}",
             tahun, bulan, tanggal, jam, minute, second, temperature, humidity, mlx1.readObjectTempC(), mlx2.readObjectTempC(), Address0, degrees);

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

void setinfra()
{
    Wire.begin(); // Initialize the I2C communication

    // Configure TCA9548A channel (0 for the first sensor, 1 for the second sensor)
    selectTCAChannel(0); // Choose the channel for the first sensor

    if (!mlx1.begin())
    {
        Serial.println("Error connecting to MLX sensor 1. Check wiring.");
        while (1)
            ;
    }

    // Wait for a moment before switching to the second sensor
    delay(1000);

    // Configure TCA9548A channel for the second sensor
    selectTCAChannel(1); // Choose the channel for the second sensor

    if (!mlx2.begin())
    {
        Serial.println("Error connecting to MLX sensor 2. Check wiring.");
        while (1)
            ;
    }
}

void sensorinfra()
{
    selectTCAChannel(0); // Select channel 0 (first sensor)
    Serial.print("Sensor 1 - Ambient temperature = ");
    Serial.print(mlx1.readAmbientTempC());
    Serial.print("°C   ");
    Serial.print("Object temperature = ");
    Serial.print(mlx1.readObjectTempC());
    Serial.println("°C");

    selectTCAChannel(1); // Select channel 1 (second sensor)
    Serial.print("Sensor 2 - Ambient temperature = ");
    Serial.print(mlx2.readAmbientTempC());
    Serial.print("°C   ");
    Serial.print("Object temperature = ");
    Serial.print(mlx2.readObjectTempC());
    Serial.println("°C");

    Serial.println("-----------------------------------------------------------------");
}

void setanemo()
{
    mySerial2.begin(9600);
    ModifyAddress(0x00, Address0); // Modify device address0, please comment out this sentence after modifying the address0 and power on again.
}

void sensoranemo()
{
    Serial.print(readWindSpeed(Address0)); // Read wind speed
    Serial.println("m/s");
}

void setdht()
{
    dht.begin();
}

void sensordht()
{
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();

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

void setwind()
{
    // Mulai koneksi dengan modul ADS1115
    if (!ads.begin(ADS1115_ADDRESS))
    {
        Serial.println("Couldn't find ADS1115");
        while (1)
            ;
    }

    // Set range ke 4.096V (default adalah 6.144V)
    ads.setGain(GAIN_TWOTHIRDS);
}

void sensorwind()
{
    // Baca nilai analog dari sensor wind direction
    int16_t adcValue = ads.readADC_SingleEnded(windDirectionPin);

    // Konversi nilai analog menjadi sudut (0-360)
    float voltage = adcValue * (4.096 / 32767.0); // Konversi nilai ADC menjadi tegangan
    degrees = (voltage - 0.5) * 360.0 / 3.5;      // Konversi tegangan menjadi sudut

    // Pastikan nilai sudut berada dalam rentang 0-360
    if (degrees < 0)
    {
        degrees += 360;
    }

    // Tentukan keterangan arah mata angin berdasarkan sudut
    String windDirection;
    if (degrees >= 337.5 || degrees < 22.5)
    {
        windDirection = "Utara";
    }
    else if (degrees >= 22.5 && degrees < 67.5)
    {
        windDirection = "Timur Laut";
    }
    else if (degrees >= 67.5 && degrees < 112.5)
    {
        windDirection = "Timur";
    }
    else if (degrees >= 112.5 && degrees < 157.5)
    {
        windDirection = "Tenggara";
    }
    else if (degrees >= 157.5 && degrees < 202.5)
    {
        windDirection = "Selatan";
    }
    else if (degrees >= 202.5 && degrees < 247.5)
    {
        windDirection = "Barat Daya";
    }
    else if (degrees >= 247.5 && degrees < 292.5)
    {
        windDirection = "Barat";
    }
    else if (degrees >= 292.5 && degrees < 337.5)
    {
        windDirection = "Barat Laut";
    }

    // Tampilkan keterangan arah mata angin
    Serial.print("Sudut: ");
    Serial.print(degrees);
    Serial.print("°\tArah Angin: ");
    Serial.println(windDirection);
}

size_t readN(uint8_t *buf, size_t len)
{
    size_t offset = 0, left = len;
    int16_t Tineout = 1500;
    uint8_t *buffer = buf;
    long curr = millis();
    while (left)
    {
        if (mySerial2.available())
        {
            buffer[offset] = mySerial2.read();
            offset++;
            left--;
        }
        if (millis() - curr > Tineout)
        {
            break;
        }
    }
    return offset;
}

uint16_t CRC16_2(uint8_t *buf, int16_t len)
{
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--)
        {
            if ((crc & 0x0001) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    crc = ((crc & 0x00ff) << 8) | ((crc & 0xff00) >> 8);
    return crc;
}

void addedCRC(uint8_t *buf, int len)
{
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--)
        {
            if ((crc & 0x0001) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    buf[len] = crc % 0x100;
    buf[len + 1] = crc / 0x100;
}

float readWindSpeed(uint8_t Address0)
{
    uint8_t Data[7] = {0};                                             // Store the original data packet returned by the sensor
    uint8_t COM[8] = {0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00}; // Command for reading wind speed
    boolean ret = false;                                               // Wind speed acquisition success flag
    float WindSpeed = 0;
    long curr = millis();
    long curr1 = curr;
    uint8_t ch = 0;
    COM[0] = Address0;       // Add the complete command package with reference to the communication protocol.
    addedCRC(COM, 6);        // Add CRC_16 check for reading wind speed command packet
    mySerial2.write(COM, 8); // Send the command of reading the wind speed

    while (!ret)
    {
        if (millis() - curr > 1000)
        {
            WindSpeed = -1; // If the wind speed has not been read for more than 1000 milliseconds, it will be regarded as a timeout and return -1.
            break;
        }

        if (millis() - curr1 > 100)
        {
            mySerial2.write(COM, 8); // If the last command to read the wind speed is sent for more than 100 milliseconds and the return command has not been received, the command to read the wind speed will be re-sent
            curr1 = millis();
        }

        if (readN(&ch, 1) == 1)
        {
            if (ch == Address0)
            { // Read and judge the packet header.
                Data[0] = ch;
                if (readN(&ch, 1) == 1)
                {
                    if (ch == 0x03)
                    { // Read and judge the packet header.
                        Data[1] = ch;
                        if (readN(&ch, 1) == 1)
                        {
                            if (ch == 0x02)
                            { // Read and judge the packet header.
                                Data[2] = ch;
                                if (readN(&Data[3], 4) == 4)
                                {
                                    if (CRC16_2(Data, 5) == (Data[5] * 256 + Data[6]))
                                    { // Check data packet
                                        ret = true;
                                        WindSpeed = (Data[3] * 256 + Data[4]) / 10.00; // Calculate the wind speed
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return WindSpeed;
}

boolean ModifyAddress(uint8_t Address1, uint8_t Address2)
{
    uint8_t ModifyAddressCOM[11] = {0x00, 0x10, 0x10, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00};
    boolean ret = false;
    long curr = millis();
    long curr1 = curr;
    uint8_t ch = 0;
    ModifyAddressCOM[0] = Address1;
    ModifyAddressCOM[8] = Address2;
    addedCRC(ModifyAddressCOM, 9);
    mySerial2.write(ModifyAddressCOM, 11);
    while (!ret)
    {
        if (millis() - curr > 1000)
        {
            break;
        }

        if (millis() - curr1 > 100)
        {
            mySerial2.write(ModifyAddressCOM, 11);
            curr1 = millis();
        }

        if (readN(&ch, 1) == 1)
        {
            if (ch == Address1)
            {
                if (readN(&ch, 1) == 1)
                {
                    if (ch == 0x10)
                    {
                        if (readN(&ch, 1) == 1)
                        {
                            if (ch == 0x10)
                            {
                                if (readN(&ch, 1) == 1)
                                {
                                    if (ch == 0x00)
                                    {
                                        if (readN(&ch, 1) == 1)
                                        {
                                            if (ch == 0x00)
                                            {
                                                if (readN(&ch, 1) == 1)
                                                {
                                                    if (ch == 0x01)
                                                    {
                                                        // while (1) {
                                                        Serial.println("Please power on the sensor again.");
                                                        // delay(1000);
                                                        // }
                                                        ret = true;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return ret;
}

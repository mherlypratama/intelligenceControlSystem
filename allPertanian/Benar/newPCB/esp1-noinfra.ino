#include "DFRobot_ESP_PH.h"
#include "EEPROM.h"
#include <OneWire.h>
#include <SoftwareSerial.h>
#include "RS485_Wind_Direction_Transmitter_V2.h"
#include <HX711_ADC.h>
#include <Adafruit_ADS1X15.h>

#include <OneWire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "time.h"

#include <Wire.h>

// Konfigurasi jaringan Wi-Fi
const char *ssid = "pertanian24";
const char *password = "luarbiasa";

// Konfigurasi server MQTT di VPS Anda
const char *mqtt_server = "vps.isi-net.org";
const int mqtt_port = 1883;
const char *mqtt_user = "unila";
const char *mqtt_password = "pwdMQTT@123";

const char *topic_utama = "ics/pertanian";
const char *topic_kedua = "ics/pertanian2";

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 25200;
const int daylightOffset_sec = 3600;

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsgTime = 0;
const long Interval = 5000; // Kirim data setiap 5 detik

Adafruit_ADS1115 ads;

#if defined(ARDUINO_AVR_UNO) || defined(ESP8266) // Use softserial
SoftwareSerial softSerial(/*rx =*/13, /*tx =*/19);
RS485_Wind_Direction_Transmitter_V2 windDirection(/*softSerial =*/&softSerial);
#include <EEPROM.h>
#elif defined(ESP32) // Use the hardserial of remappable pin: Serial1
RS485_Wind_Direction_Transmitter_V2 windDirection(/*hardSerial =*/&Serial1, /*rx =*/13, /*tx =*/19);
#include <EEPROM.h>
#else // Use hardserial: Serial1
RS485_Wind_Direction_Transmitter_V2 windDirection(/*hardSerial =*/&Serial1);
#endif

uint8_t Address = 0x02;

const char *Orientation[17] = {
    "north", "north by northeast", "northeast", "east by northeast", "east", "east by southeast", "southeast", "south by southeast", "south",
    "south by southwest", "southwest", "west by southwest", "west", "west by northwest", "northwest", "north by northwest", "north"};

// ****************Berat**************
// HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
unsigned long t = 0;
volatile boolean newDataReady;

// *******************PIN************************
const int HX711_dout = 26;
const int HX711_sck = 25;
#define LED_BUILTIN 2
#define TdsSensorPin 39
#define PH_PIN 36
int DS18S20_Pin = 25;

SoftwareSerial mySerial2(14, 0);
uint8_t Address0 = 0x10;

// ************Water flow**************
#define SENSOR1 16
#define SENSOR2 4
#define SENSOR3 12
#define SENSOR4 18

// **********************Suhu Air*******************

OneWire ds(DS18S20_Pin);

const int rainSensorPin = 15;           // Pin GPIO yang terhubung ke sensor hujan
volatile unsigned long rainCounter = 0; // Variabel penghitung pulsa hujan
float rainAccumulated = 0.0;            // Variabel untuk menghitung hujan yang terakumulasi
unsigned long lastRainTime = 0;         // Waktu terakhir terdeteksi hujan
unsigned long noRainTimeout = 10000;    // Timeout dalam milidetik (misalnya, 10 menit)

// *******************TDS*********************
#define VREF 5.0  // analog reference voltage(Volt) of the ADC
#define SCOUNT 30 // sum of sample point

int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, temperaturetds = 25;

// ******************PH************************
DFRobot_ESP_PH ph;
#define ESPADC 4096.0   // the esp Analog Digital Convertion value
#define ESPVOLTAGE 3300 // the esp voltage supply value
float voltage, temperatureph = 25;

long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;
boolean ledState = LOW;

// Calibration factors for each sensor (modify as needed)
float calibrationFactor1 = 4.5;
float calibrationFactor2 = 4.5;
float calibrationFactor3 = 4.5;
float calibrationFactor4 = 4.5;

volatile byte pulseCount1, pulseCount2, pulseCount3, pulseCount4;
byte pulse1Sec1, pulse1Sec2, pulse1Sec3, pulse1Sec4;
unsigned int flowMilliLitres1, flowMilliLitres2, flowMilliLitres3, flowMilliLitres4;
unsigned long totalMilliLitres1, totalMilliLitres2, totalMilliLitres3, totalMilliLitres4;

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

// Variabel Utama

int jam, minute, second, tanggal, bulan, tahun;
float temperatureair, phValue, tdsValue = 0, Angle;
float flowRate1, flowRate2, flowRate3, flowRate4;
float fix0, fix1, fix2, fix3;

void setup()
{
    Serial.begin(9600);
    setnodered();
    setph();
    setsuhuair();
    settds();
    setwater();
    setanemo();
    setwind();
    setberat();
    setsoil();
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
    sensorsuhuair();
    sensortds();
    sensorwater();
    sensoranemo();
    sensorwind();
    sensorberat();
    sensorsoil();
    nodered();
    delay(60000);
}

void setnodered()
{
    setupWiFi();
    client.setServer(mqtt_server, mqtt_port);
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    printLocalTime();
}

void nodered()
{

    char utamaStr[1000]; // Buffer untuk menyimpan JSON
    snprintf(utamaStr, sizeof(utamaStr),
             "{"
             "\"TimeStamp\": \"%04d-%02d-%02dT%02d:%02d:%02d+07:00\","
             "\"ph\": %.2f,"
             "\"tds\": %.2f,"
             "\"tempDs\": %.2f,"
             "\"windDirection\": %.2f,"
             "\"anemo\": %.2f,"
             "\"infra1\": %.2f,"
             "\"infra2\": %.2f,"
             "\"Berat_1\": %.2f"
             "}",
             tahun, bulan, tanggal, jam, minute, second, phValue, tdsValue, temperatureair, Angle, readWindSpeed(Address0), mlx1.readObjectTempC(), mlx2.readObjectTempC(), berat1);
    client.publish(topic_utama, utamaStr);
    // mlx1.readObjectTempC(), mlx2.readObjectTempC(), mlx3.readObjectTempC()

    char keduaStr[1000]; // Buffer untuk menyimpan JSON
    snprintf(keduaStr, sizeof(keduaStr),
             "{"
             "\"TimeStamp\": \"%04d-%02d-%02dT%02d:%02d:%02d+07:00\","
             "\"Water_1\": %.2f,"
             "\"Water_2\": %.2f,"
             "\"Water_3\": %.2f,"
             "\"Water_4\": %.2f,"
             "\"Soil_1\": %.2f,"
             "\"Soil_2\": %.2f,"
             "\"Soil_3\": %.2f,"
             "\"Soil_4\": %.2f"
             "}",
             tahun, bulan, tanggal, jam, minute, second, flowRate1, flowRate2, flowRate3, flowRate4, fix0, fix1, fix2, fix3);

    client.publish(topic_kedua, keduaStr);
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

void setberat()
{
    delay(10);
    Serial.println();
    Serial.println("Starting...");

    float calibrationValue;    // calibration value
    calibrationValue = 108.71; // uncomment this if you want to set this value in the sketch
#if defined(ESP8266) || defined(ESP32)
    // EEPROM.begin(512); // uncomment this if you use ESP8266 and want to fetch the value from eeprom
#endif
    EEPROM.get(calVal_eepromAdress, calibrationValue); // uncomment this if you want to fetch the value from eeprom

    LoadCell.begin();
    // LoadCell.setReverseOutput();
    unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
    boolean _tare = true;                 // set this to false if you don't want tare to be performed in the next step
    LoadCell.start(stabilizingtime, _tare);
    if (LoadCell.getTareTimeoutFlag())
    {
        Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
        while (1)
            ;
    }
    else
    {
        LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
        Serial.println("Startup is complete");
    }

    attachInterrupt(digitalPinToInterrupt(HX711_dout), dataReadyISR, FALLING);
}

void sensorberat()
{
    const int serialPrintInterval = 0; // increase value to slow down serial print activity

    // get smoothed value from the dataset:
    if (newDataReady)
    {
        if (millis() > t + serialPrintInterval)
        {
            float i = LoadCell.getData();
            newDataReady = 0;
            Serial.print("Load_cell output val: ");
            Serial.print(i);
            Serial.println(" gram");

            // Serial.print("  ");
            // Serial.println(millis() - t);
            t = millis();
        }
    }

    // receive command from serial terminal, send 't' to initiate tare operation:
    if (Serial.available() > 0)
    {
        char inByte = Serial.read();
        if (inByte == 't')
            LoadCell.tareNoDelay();
    }

    // check if last tare operation is complete
    if (LoadCell.getTareStatus() == true)
    {
        Serial.println("Tare complete");
    }
}

// interrupt routine:
void dataReadyISR()
{
    if (LoadCell.update())
    {
        newDataReady = 1;
    }
}

void setwind()
{
    // Init the sensor
    while (!(windDirection.begin()))
    {
        Serial.println("Communication with device failed, please check connection");
        delay(3000);
    }
    Serial.println("Begin ok!");
}

void sensorwind()
{
    // Get 16 wind directions
    int Direction = windDirection.GetWindDirection(/*modbus slave address*/ Address);
    // Get 360° wind direction angle
    Angle = windDirection.GetWindAngle(/*modbus slave address*/ Address);
    Serial.println(Orientation[Direction]);
    Serial.print(Angle);
    Serial.println("°");
    Serial.println();
    delay(1000);
}

void setanemo()
{
    mySerial2.begin(9600);
    ModifyAddress(0x00, Address0);
}

void sensoranemo()
{
    Serial.print(readWindSpeed(Address0)); // Read wind speed
    Serial.println("m/s");
    delay(1000);
}

void setwater()
{

    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(SENSOR1, INPUT_PULLUP);
    pinMode(SENSOR2, INPUT_PULLUP);
    pinMode(SENSOR3, INPUT_PULLUP);
    pinMode(SENSOR4, INPUT_PULLUP);

    pulseCount1 = 0;
    pulseCount2 = 0;
    pulseCount3 = 0;
    pulseCount4 = 0;

    flowRate1 = 0.0;
    flowRate2 = 0.0;
    flowRate3 = 0.0;
    flowRate4 = 0.0;

    flowMilliLitres1 = 0;
    flowMilliLitres2 = 0;
    flowMilliLitres3 = 0;
    flowMilliLitres4 = 0;

    totalMilliLitres1 = 0;
    totalMilliLitres2 = 0;
    totalMilliLitres3 = 0;
    totalMilliLitres4 = 0;

    attachInterrupt(digitalPinToInterrupt(SENSOR1), pulseCounter1, FALLING);
    attachInterrupt(digitalPinToInterrupt(SENSOR2), pulseCounter2, FALLING);
    attachInterrupt(digitalPinToInterrupt(SENSOR3), pulseCounter3, FALLING);
    attachInterrupt(digitalPinToInterrupt(SENSOR4), pulseCounter4, FALLING);
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

        flowRate1 = ((1000.0 / (millis() - previousMillis)) * pulse1Sec1) / calibrationFactor1;
        flowRate2 = ((1000.0 / (millis() - previousMillis)) * pulse1Sec2) / calibrationFactor2;
        flowRate3 = ((1000.0 / (millis() - previousMillis)) * pulse1Sec3) / calibrationFactor3;
        flowRate4 = ((1000.0 / (millis() - previousMillis)) * pulse1Sec4) / calibrationFactor4;

        previousMillis = millis();

        flowMilliLitres1 = (flowRate1 / 60) * 1000;
        flowMilliLitres2 = (flowRate2 / 60) * 1000;
        flowMilliLitres3 = (flowRate3 / 60) * 1000;
        flowMilliLitres4 = (flowRate4 / 60) * 1000;

        totalMilliLitres1 += flowMilliLitres1;
        totalMilliLitres2 += flowMilliLitres2;
        totalMilliLitres3 += flowMilliLitres3;
        totalMilliLitres4 += flowMilliLitres4;

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
    }
}

void settds()
{
    pinMode(TdsSensorPin, INPUT);
}

void sensortds()
{
    static unsigned long analogSampleTimepoint = millis();
    if (millis() - analogSampleTimepoint > 40U) // every 40 milliseconds,read the analog value from the ADC
    {
        analogSampleTimepoint = millis();
        analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); // read the analog value and store into the buffer
        analogBufferIndex++;
        if (analogBufferIndex == SCOUNT)
            analogBufferIndex = 0;
    }
    static unsigned long printTimepoint = millis();
    if (millis() - printTimepoint > 800U)
    {
        printTimepoint = millis();
        for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
            analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
        averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;                                                                                                  // read the analog value more stable by the median filtering algorithm, and convert to voltage value
        float compensationCoefficient = 1.0 + 0.02 * (temperaturetds - 25.0);                                                                                                            // temperaturetds compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
        float compensationVolatge = averageVoltage / compensationCoefficient;                                                                                                            // temperaturetds compensation
        tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; // convert voltage value to tds value
        // Serial.print("voltage:");
        // Serial.print(averageVoltage,2);
        // Serial.print("V   ");
        Serial.print("TDS Value:");
        Serial.print(tdsValue, 0);
        Serial.println("ppm");
    }
}

void setsuhuair()
{
    pinMode(rainSensorPin, INPUT_PULLUP); // Mengatur pin sensor hujan sebagai input dengan pull-up
    attachInterrupt(digitalPinToInterrupt(rainSensorPin), rainInterrupt, FALLING);
}

void sensorsuhuair()
{
    unsigned long currentTime = millis();

    // Cek jika tidak ada pulsa hujan selama waktu tertentu (noRainTimeout)
    if (currentTime - lastRainTime >= noRainTimeout)
    {
        rainAccumulated = 0.0; // Reset jumlah hujan terakumulasi
    }

    // Mencetak jumlah hujan yang terakumulasi setiap beberapa detik
    delay(1000); // Misalnya, cetak setiap 5 detik
    Serial.print("Hujan Terakumulasi (mm): ");
    Serial.println(rainAccumulated, 2); // Menampilkan hingga 2 desimal

    // Read temperature and print
    temperatureair = getTemp();
    Serial.print("Temperature Air: ");
    Serial.println(temperatureair);
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

        // temperatureph = readTemperature();  // read your temperatureph sensor to execute temperatureph compensation
        Serial.print("temperatureph:");
        Serial.print(temperatureph, 1);
        Serial.println("^C");

        phValue = ph.readPH(voltage, temperatureph); // convert voltage to pH with temperatureph compensation
        Serial.print("pH:");
        Serial.println(phValue, 4);
    }
    ph.calibration(voltage, temperatureph); // calibration process by Serail CMD
}

void rainInterrupt()
{
    // Fungsi yang akan dipanggil ketika terjadi pulsa hujan
    rainCounter++;             // Menambah penghitung pulsa
    rainAccumulated += 0.2794; // Menambahkan jumlah hujan (silakan sesuaikan dengan spesifikasi sensor Anda)
    lastRainTime = millis();   // Memperbarui waktu terakhir terdeteksi hujan
}

float getTemp()
{
    // Returns the temperature from one DS18S20 in DEG Celsius

    byte data[12];
    byte addr[8];

    if (!ds.search(addr))
    {
        // No more sensors on the chain, reset search
        ds.reset_search();
        return -1000;
    }

    if (OneWire::crc8(addr, 7) != addr[7])
    {
        Serial.println("CRC is not valid!");
        return -1000;
    }

    if (addr[0] != 0x10 && addr[0] != 0x28)
    {
        Serial.print("Device is not recognized");
        return -1000;
    }

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1); // Start conversion, with parasite power on at the end

    delay(1000); // Wait for the conversion to complete (adjust as needed)

    ds.reset();
    ds.select(addr);
    ds.write(0xBE); // Read Scratchpad

    for (int i = 0; i < 9; i++)
    { // We need 9 bytes
        data[i] = ds.read();
    }

    ds.reset_search();

    byte MSB = data[1];
    byte LSB = data[0];

    float tempRead = ((MSB << 8) | LSB); // Using two's complement
    float TemperatureSum = tempRead / 16.0;

    return TemperatureSum;
}

int getMedianNum(int bArray[], int iFilterLen)
{
    int bTab[iFilterLen];
    for (byte i = 0; i < iFilterLen; i++)
        bTab[i] = bArray[i];
    int i, j, bTemp;
    for (j = 0; j < iFilterLen - 1; j++)
    {
        for (i = 0; i < iFilterLen - j - 1; i++)
        {
            if (bTab[i] > bTab[i + 1])
            {
                bTemp = bTab[i];
                bTab[i] = bTab[i + 1];
                bTab[i + 1] = bTemp;
            }
        }
    }
    if ((iFilterLen & 1) > 0)
        bTemp = bTab[(iFilterLen - 1) / 2];
    else
        bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
    return bTemp;
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

/**
   @brief 	Calculate CRC16_2 check value
   @param buf 		Packet for calculating the check value
   @param len 		Length of data that needs to check
   @return 		Return a 16-bit check result.
*/
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

/**
   @brief  Adds a CRC_16 check to the end of the packet
   @param buf 		Data packet that needs to add the check value
   @param len 		Length of data that needs to add check
   @return 		None
*/
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

/**
   @brief 	Read the wind speed
   @param Address0 	The read device address0
   @return 		Wind speed unit m/s, return -1 for read timeout
*/

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

/**
   @brief 	Modify the sensor device address
   @param Address1 	The address of the device before modification. Use the 0x00 address to set any address, after setting, you need to re-power on and restart the module.
   @param Address2 	The modified address of the device, the range is 0x00~0xFF,
   @return 		Returns true to indicate that the modification was successful, and returns false to indicate that the modification failed.
*/

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

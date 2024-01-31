#include "DFRobot_ESP_PH.h"
#include "EEPROM.h"
#include <OneWire.h>

// *******************PIN************************
#define TdsSensorPin 39
#define PH_PIN 36     // the esp gpio data pin number
int DS18S20_Pin = 25; // Choose any digital pin for DS18S20 Signal (e.g., GPIO 14)

// **********************Suhu Air*******************
// Temperature chip i/o
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

// Variabel Utama
float temperatureair, phValue, tdsValue = 0;

void setup()
{
    Serial.begin(9600);
    setph();
}

void loop()
{
    sensorph();
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
#include "DFRobot_ESP_PH.h"
#include "EEPROM.h"
#include <OneWire.h>

// *******************PIN************************
#define LED_BUILTIN 2

#define TdsSensorPin 39
#define PH_PIN 36     // the esp gpio data pin number
int DS18S20_Pin = 25; // Choose any digital pin for DS18S20 Signal (e.g., GPIO 14)

// ************Water flow**************
#define SENSOR1 16
#define SENSOR2 4
#define SENSOR3 12
#define SENSOR4 18

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
float temperatureair, phValue, tdsValue = 0;
float flowRate1, flowRate2, flowRate3, flowRate4;

void setup()
{
    Serial.begin(9600);
    setph();
    setsuhuair();
    settds();
    setwater();
}

void loop()
{
    sensorph();
    sensorsuhuair();
    sensortds();
    sensorwater();
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
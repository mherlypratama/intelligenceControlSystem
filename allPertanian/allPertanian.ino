// ### Analog

// 1. Sensor PH terhubung ke pin `35`
// 2. Sensor TDS terhubung ke pin `34`
// 3. Sensor Rain terhubung ke pin `15`

// ### Digital

// 1. Sensor Suhu Air terhubung ke pin `3`
// 2. Sensor Wind Direction terhubung ke pin `1(rx)` dan `25(tx)`
// 3. Sensor Anemometer terhubung ke pin `26` dan `0`
// 4. Sensor Waterflow terhubung ke pin `14`, `13`, `17`, dan `16`

// ### I2C Pin(Paralel)

// 1. Sensor BME-280
// 2. Sensor Soil Moisture > ADS115
// 3. Infrared Sensor > TCA9548A
#include <Arduino.h>
// I2C
#include <Wire.h>
#include <Adafruit_MLX90614.h>

#include <Wire.h>
#include <DFRobot_ADS1115.h>

#include "DFRobot_BME280.h"
#include "Wire.h"

#include <DHT.h>

// Constants
#define DHTPIN 23         // what pin we're connected to
#define DHTTYPE DHT22     // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino

// Variables
int chk;
float humiditydht;
float temperaturedht;

DFRobot_ADS1115 ads(&Wire);
// TCA9548A I2C Multiplexer Address
#define TCAADDR 0x70

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

typedef DFRobot_BME280_IIC BME; // *** use abbreviations instead of full names ***

BME bme(&Wire, 0x77); // select TwoWire peripheral and set sensor address

#define SEA_LEVEL_PRESSURE 1015.0f

// PH
#include <OneWire.h>
#include "DFRobot_ESP_PH.h"
#include "EEPROM.h"
#include <SoftwareSerial.h>

DFRobot_ESP_PH ph;
#define ESPADC 4096.0   // the esp Analog Digital Convertion value
#define ESPVOLTAGE 3300 // the esp voltage supply value
#define PH_PIN 36       // the esp gpio data pin number
float voltage, phValue;

// Wind
#include "RS485_Wind_Direction_Transmitter_V2.h"

#if defined(ARDUINO_AVR_UNO) || defined(ESP8266) // Use softserial
SoftwareSerial softSerial(/*rx =*/14, /*tx =*/0);
RS485_Wind_Direction_Transmitter_V2 windDirection(/*softSerial =*/&softSerial);
#elif defined(ESP32) // Use the hardserial of remappable pin: Serial1
RS485_Wind_Direction_Transmitter_V2 windDirection(/*hardSerial =*/&Serial1, /*rx =*/14, /*tx =*/0);
#else                // Use hardserial: Serial1
RS485_Wind_Direction_Transmitter_V2 windDirection(/*hardSerial =*/&Serial1);
#endif

uint8_t Address = 0x02;

const char *Orientation[17] = {
    "north", "north by northeast", "northeast", "east by northeast", "east", "east by southeast", "southeast", "south by southeast", "south",
    "south by southwest", "southwest", "west by southwest", "west", "west by northwest", "northwest", "north by northwest", "north"};

// Anemo
SoftwareSerial mySerial2(13, 19); // Define the soft serial port, port 3 is TX, port 2 is RX,

uint8_t Address0 = 0x10;

// TDS
#define TdsSensorPin 39   // sesuaikan dengan pin arduino
#define VREF 5.0          // analog reference voltage(Volt) of the ADC
#define SCOUNT 30         // sum of sample point
int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;

// DS18S20 dan Rain
int DS18S20_Pin = 18; // Choose any digital pin for DS18S20 Signal (e.g., GPIO 14)

// Temperature chip i/o
OneWire ds(DS18S20_Pin);

const int rainSensorPin = 15;           // Pin GPIO yang terhubung ke sensor hujan
volatile unsigned long rainCounter = 0; // Variabel penghitung pulsa hujan
float rainAccumulated = 0.0;            // Variabel untuk menghitung hujan yang terakumulasi
unsigned long lastRainTime = 0;         // Waktu terakhir terdeteksi hujan
unsigned long noRainTimeout = 10000;    // Timeout dalam milidetik (misalnya, 10 menit)

// Waterflow
#define LED_BUILTIN 2

// Define pins for water flow sensors
#define SENSOR1 12
#define SENSOR2 4
#define SENSOR3 16
#define SENSOR4 17

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
float flowRate1, flowRate2, flowRate3, flowRate4;
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

// show last sensor operate status
void printLastOperateStatus(BME::eStatus_t eStatus)
{
    switch (eStatus)
    {
    case BME::eStatusOK:
        Serial.println("everything ok");
        break;
    case BME::eStatusErr:
        Serial.println("unknow error");
        break;
    case BME::eStatusErrDeviceNotDetected:
        Serial.println("device not detected");
        break;
    case BME::eStatusErrParameter:
        Serial.println("parameter error");
        break;
    default:
        Serial.println("unknow status");
        break;
    }
}

void selectTCAChannel(uint8_t channel)
{
    // Select the appropriate channel on TCA9548A
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

void setup()
{
    pinMode(TdsSensorPin, INPUT);
    pinMode(rainSensorPin, INPUT_PULLUP);                                          // Mengatur pin sensor hujan sebagai input dengan pull-up
    attachInterrupt(digitalPinToInterrupt(rainSensorPin), rainInterrupt, FALLING); // Menghubungkan interrupt ke pin sensor hujan saat pulsa jatuh
    Serial.begin(9600);
    EEPROM.begin(32); // needed to permit storage of calibration value in eeprom
    ph.begin();
    setWind();
    mySerial2.begin(9600);
    ModifyAddress(0x00, Address0); // Modify device address0, please comment out this sentence after modifying the address0 and power on again.

    bme.reset();
    Serial.println("bme read data test");
    while (bme.begin() != BME::eStatusOK)
    {
        Serial.println("bme begin faild");
        printLastOperateStatus(bme.lastOperateStatus);
        delay(2000);
    }

    Wire.begin(); // Initialize the I2C communication

    // Atur pin TCA
    setInfra();

    ads.setAddr_ADS1115(ADS1115_IIC_ADDRESS1); // 0x48
    ads.setGain(eGAIN_TWOTHIRDS);              // 2/3x gain
    ads.setMode(eMODE_SINGLE);                 // single-shot mode
    ads.setRate(eRATE_128);                    // 128SPS (default)
    ads.setOSMode(eOSMODE_SINGLE);             // Set to start a single-conversion
    ads.init();
    setWater();

    Serial.println("Ready"); // Test the serial monitor
}
void loop()
{
    // Ambil Nilai PH
    unsigned long currentTime = millis();

    // Cek jika tidak ada pulsa hujan selama waktu tertentu (noRainTimeout)
    if (currentTime - lastRainTime >= noRainTimeout)
    {
        rainAccumulated = 0.0; // Reset jumlah hujan terakumulasi
    }

    delay(1000); // Misalnya, cetak setiap 5 detik
    Serial.print("Hujan Terakumulasi (mm): ");
    Serial.println(rainAccumulated, 2); // Menampilkan hingga 2 desimal

    // Read temperature and print
    float temperature = getTemp();
    Serial.print("Temperature: ");
    Serial.println(temperature);

    sensorPH();
    sensorTDS();
    sensorwind();
    waterFlow();

    Serial.print(readWindSpeed(Address0)); // Read wind speed
    Serial.println("m/s");

    Serial.println(" ");
    digitalWrite(13, HIGH);
    delay(800);
    digitalWrite(13, LOW);
    sensorInfra();
    Serial.println("-----------------------------------------------------------------");
    soilSensor();
    Serial.println("-----------------------------------------------------------------");
    sensorBME();
    Serial.println("-----------------------------------------------------------------");
    delay(1000);
}

void setdht()
{
}

void sensordht()
{
    // Read data and store it to variables humiditydht and temperaturedht
    humiditydht = dht.readHumidity();
    temperaturedht = dht.readTemperature();
    // Print temperaturedht and humiditydhtidity values to serial monitor
    Serial.print("Humidity: ");
    Serial.print(humiditydht);
    Serial.print(" %, Temp: ");
    Serial.print(temperaturedht);
    Serial.println(" Celsius");
}

void setWater()
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

void waterFlow()
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
        // Serial.print("Total Liquid Quantity: ");
        // Serial.print(totalMilliLitres1);
        // Serial.print("mL / ");
        // Serial.print(totalMilliLitres1 / 1000);
        // Serial.println("L");

        Serial.print("Flow 2: ");
        Serial.print(int(flowRate2)); // Print the integer part of the variable
        Serial.print("L/min,   ");
        // Serial.print("Total Liquid Quantity: ");
        // Serial.print(totalMilliLitres2);
        // Serial.print("mL / ");
        // Serial.print(totalMilliLitres2 / 1000);
        // Serial.println("L");

        Serial.print("Flow 3: ");
        Serial.print(int(flowRate3)); // Print the integer part of the variable
        Serial.print("L/min,   ");
        // Serial.print("Total Liquid Quantity: ");
        // Serial.print(totalMilliLitres3);
        // Serial.print("mL / ");
        // Serial.print(totalMilliLitres3 / 1000);
        // Serial.println("L");

        Serial.print("Flow 4: ");
        Serial.print(int(flowRate4)); // Print the integer part of the variable
        Serial.println("L/min");
        // Serial.print("Total Liquid Quantity: ");
        // Serial.print(totalMilliLitres4);
        // Serial.print("mL / ");
        // Serial.print(totalMilliLitres4 / 1000);
        // Serial.println("L");
    }
}

void sensorPH()
{
    static unsigned long timepoint = millis();
    if (millis() - timepoint > 1000U) // time interval: 1s
    {
        timepoint = millis();
        // voltage = rawPinValue / esp32ADC * esp32Vin
        voltage = analogRead(PH_PIN) / ESPADC * ESPVOLTAGE; // read the voltage
        // Serial.print("voltage:");
        // Serial.println(voltage, 4);

        phValue = ph.readPH(voltage, temperature); // convert voltage to pH with temperature compensation
        Serial.print("pH:");
        Serial.println(phValue, 4);
    }
    ph.calibration(voltage, temperature);
}

void sensorTDS()
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
        float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);                                                                                                               // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
        float compensationVolatge = averageVoltage / compensationCoefficient;                                                                                                            // temperature compensation
        tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; // convert voltage value to tds value
        // Serial.print("voltage:");
        // Serial.print(averageVoltage,2);
        // Serial.print("V   ");
        Serial.print("TDS Value:");
        Serial.print(tdsValue, 0);
        Serial.println("ppm");
    }
}

void sensorwind()
{
    // Get 16 wind directions
    int Direction = windDirection.GetWindDirection(/*modbus slave address*/ Address);
    // Get 360° wind direction angle
    float Angle = windDirection.GetWindAngle(/*modbus slave address*/ Address);
    Serial.println(Orientation[Direction]);
    Serial.print(Angle);
    Serial.println("°");
    Serial.println();
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

void setWind()
{
    // Init the sensor
    while (!(windDirection.begin()))
    {
        Serial.println("Communication with device failed, please check connection");
        delay(3000);
    }
    Serial.println("Begin ok!");
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

void setInfra()
{
    // Configure TCA9548A channel (0 for the first sensor, 1 for the second sensor)
    selectTCAChannel(0); // Choose the channel for the first sensor

    if (!mlx.begin())
    {
        Serial.println("Error connecting to MLX sensor 1. Check wiring.");
        while (1)
            ;
    }

    // Wait for a moment before switching to the second sensor
    delay(1000);

    // Configure TCA9548A channel for the second sensor
    selectTCAChannel(1); // Choose the channel for the second sensor

    if (!mlx.begin())
    {
        Serial.println("Error connecting to MLX sensor 2. Check wiring.");
        while (1)
            ;
    }

    // Wait for a moment before switching to the second sensor
    delay(1000);

    // Configure TCA9548A channel for the second sensor
    selectTCAChannel(2); // Choose the channel for the second sensor

    if (!mlx.begin())
    {
        Serial.println("Error connecting to MLX sensor 2. Check wiring.");
        while (1)
            ;
    }
}

void sensorInfra()
{
    selectTCAChannel(0); // Select channel 0 (first sensor)
    Serial.print("Sensor 1 - Ambient temperature = ");
    Serial.print(mlx.readAmbientTempC());
    Serial.print("°C   ");
    Serial.print("Object temperature = ");
    Serial.print(mlx.readObjectTempC());
    Serial.println("°C");

    selectTCAChannel(1); // Select channel 1 (second sensor)
    Serial.print("Sensor 2 - Ambient temperature = ");
    Serial.print(mlx.readAmbientTempC());
    Serial.print("°C   ");
    Serial.print("Object temperature = ");
    Serial.print(mlx.readObjectTempC());
    Serial.println("°C");

    selectTCAChannel(2); // Select channel 1 (second sensor)
    Serial.print("Sensor 3 - Ambient temperature = ");
    Serial.print(mlx.readAmbientTempC());
    Serial.print("°C   ");
    Serial.print("Object temperature = ");
    Serial.print(mlx.readObjectTempC());
    Serial.println("°C");
}

void soilSensor()
{
    if (ads.checkADS1115())
    {
        int16_t adc0, adc1, adc2, adc3;

        adc0 = ads.readVoltage(0);
        adc1 = ads.readVoltage(1);
        adc2 = ads.readVoltage(2);
        adc3 = ads.readVoltage(3);

        float datakonversi0 = ((adc0 - 10540) / (19941 - 10540)) * 100;
        float datakonversi1 = ((adc1 - 10540) / (19941 - 10540)) * 100;
        float datakonversi2 = ((adc2 - 10540) / (19941 - 10540)) * 100;
        float datakonversi3 = ((adc3 - 10540) / (19941 - 10540)) * 100;

        float fix0 = 100 - datakonversi0;
        float fix1 = 100 - datakonversi1;
        float fix2 = 100 - datakonversi2;
        float fix3 = 100 - datakonversi3;

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
    else
    {
        Serial.println("ADS1115 Disconnected!");
    }
}

void sensorBME()
{
    float temp = bme.getTemperature();
    uint32_t press = bme.getPressure();
    float alti = bme.calAltitude(SEA_LEVEL_PRESSURE, press);
    float humi = bme.getHumidity();

    Serial.println();
    Serial.println("======== start print ========");
    Serial.print("temperature (unit Celsius): ");
    Serial.println(temp);
    Serial.print("pressure (unit pa):         ");
    Serial.println(press);
    Serial.print("altitude (unit meter):      ");
    Serial.println(alti);
    Serial.print("humidity (unit percent):    ");
    Serial.println(humi);
    Serial.println("========  end print  ========");
}
#include <Wire.h>
#include <Adafruit_MLX90614.h>

#include <Wire.h>
#include <DFRobot_ADS1115.h>

#include "DFRobot_BME280.h"
#include "Wire.h"

DFRobot_ADS1115 ads(&Wire);
// TCA9548A I2C Multiplexer Address
#define TCAADDR 0x70

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

typedef DFRobot_BME280_IIC BME; // *** use abbreviations instead of full names ***

BME bme(&Wire, 0x77); // select TwoWire peripheral and set sensor address

#define SEA_LEVEL_PRESSURE 1015.0f

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
    Serial.begin(115200);
    bme.reset();
    Serial.println("bme read data test");
    while (bme.begin() != BME::eStatusOK)
    {
        Serial.println("bme begin faild");
        printLastOperateStatus(bme.lastOperateStatus);
        delay(2000);
    }
    while (!Serial)
        ;

    Wire.begin(); // Initialize the I2C communication

    // Atur pin TCA
    setInfra();

    ads.setAddr_ADS1115(ADS1115_IIC_ADDRESS1); // 0x48
    ads.setGain(eGAIN_TWOTHIRDS);              // 2/3x gain
    ads.setMode(eMODE_SINGLE);                 // single-shot mode
    ads.setRate(eRATE_128);                    // 128SPS (default)
    ads.setOSMode(eOSMODE_SINGLE);             // Set to start a single-conversion
    ads.init();
}

// ********************************UTAMA*********************************
void loop()
{
    sensorInfra();
    Serial.println("-----------------------------------------------------------------");
    soilSensor();
    Serial.println("-----------------------------------------------------------------");
    sensorBME();
    Serial.println("-----------------------------------------------------------------");
    delay(1000);
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
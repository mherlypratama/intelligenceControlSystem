#include "DFRobot_ESP_PH.h"
#include "EEPROM.h"

// *******************PIN************************
#define TdsSensorPin 39
#define PH_PIN 36 // the esp gpio data pin number

// *******************TDS*********************
#define VREF 5.0  // analog reference voltage(Volt) of the ADC
#define SCOUNT 30 // sum of sample point

int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperaturetds = 25;

// ******************PH************************
DFRobot_ESP_PH ph;
#define ESPADC 4096.0   // the esp Analog Digital Convertion value
#define ESPVOLTAGE 3300 // the esp voltage supply value
float voltage, phValue, temperatureph = 25;

void setup()
{
    Serial.begin(9600);
    setph();
}

void loop()
{
    sensorph();
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
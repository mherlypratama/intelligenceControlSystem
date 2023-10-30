#include <HX711_ADC.h>
#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

const int HX711_dout = 12; // mcu > HX711 dout pin, must be external interrupt capable!
const int HX711_sck = 4;   // mcu > HX711 sck pin

const int HX711_dout2 = 17; // mcu > HX711 dout pin, must be external interrupt capable!
const int HX711_sck2 = 16;  // mcu > HX711 sck pin

// HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);
HX711_ADC LoadCell2(HX711_dout2, HX711_sck2);

const int calVal_eepromAdress = 0;
unsigned long t = 0;
volatile boolean newDataReady;

const int calVal_eepromAdress2 = 0;
unsigned long t2 = 0;
volatile boolean newDataReady2;

void setup()
{
    Serial.begin(9600);
    delay(10);
    Serial.println();
    Serial.println("Starting...");

    float calibrationValue;    // calibration value
    calibrationValue = 108.71; // uncomment this if you want to set this value in the sketch
#if defined(ESP8266) || defined(ESP32)
    // EEPROM.begin(512); // uncomment this if you use ESP8266 and want to fetch the value from eeprom
#endif
    EEPROM.get(calVal_eepromAdress, calibrationValue);  // uncomment this if you want to fetch the value from eeprom
    EEPROM.get(calVal_eepromAdress2, calibrationValue); // uncomment this if you want to fetch the value from eeprom

    LoadCell.begin();
    LoadCell2.begin();
    // LoadCell.setReverseOutput();
    unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
    boolean _tare = true;                 // set this to false if you don't want tare to be performed in the next step
    LoadCell.start(stabilizingtime, _tare);

    unsigned long stabilizingtime2 = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
    boolean _tare2 = true;                 // set this to false if you don't want tare to be performed in the next step
    LoadCell2.start(stabilizingtime2, _tare2);

    if (LoadCell.getTareTimeoutFlag())
    {
        Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
        while (1)
            ;
    }
    else
    {
        LoadCell.setCalFactor(calibrationValue);  // set calibration value (float)
        LoadCell2.setCalFactor(calibrationValue); // set calibration value (float)
        Serial.println("Startup is complete");
    }

    attachInterrupt(digitalPinToInterrupt(HX711_dout), dataReadyISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(HX711_dout2), dataReadyISR2, FALLING);
}

// interrupt routine:
void dataReadyISR()
{
    if (LoadCell.update())
    {
        newDataReady = 1;
    }
}

// interrupt routine:
void dataReadyISR2()
{
    if (LoadCell2.update())
    {
        newDataReady2 = 1;
    }
}

void loop()
{
    berat1();
    berat2();
    delay(1000);
}

void berat1()
{
    const int serialPrintInterval = 0; // increase value to slow down serial print activity

    // get smoothed value from the dataset:
    if (newDataReady)
    {
        if (millis() > t + serialPrintInterval)
        {
            float i = LoadCell.getData();
            newDataReady = 0;
            Serial.print("Berat-1: ");
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

void berat2()
{
    const int serialPrintInterval2 = 0; // increase value to slow down serial print activity

    // get smoothed value from the dataset:
    if (newDataReady2)
    {
        if (millis() > t2 + serialPrintInterval2)
        {
            float i2 = LoadCell2.getData();
            newDataReady2 = 0;
            Serial.print("Berat-2: ");
            Serial.print(i2);
            Serial.println(" gram");

            // Serial.print("  ");
            // Serial.println(millis() - t);
            t2 = millis();
        }
    }

    // receive command from serial terminal, send 't' to initiate tare operation:
    if (Serial.available() > 0)
    {
        char inByte = Serial.read();
        if (inByte == 't')
            LoadCell2.tareNoDelay();
    }

    // check if last tare operation is complete
    if (LoadCell2.getTareStatus() == true)
    {
        Serial.println("Tare complete");
    }
}

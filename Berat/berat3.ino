#include <HX711_ADC.h>
#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

// pins:
const int HX711_dout_1 = 17; // mcu > HX711 no 1 dout pin
const int HX711_sck_1 = 16;  // mcu > HX711 no 1 sck pin
const int HX711_dout_2 = 4;  // mcu > HX711 no 2 dout pin
const int HX711_sck_2 = 12;  // mcu > HX711 no 2 sck pin
const int HX711_dout_3 = 5;  // mcu > HX711 no 3 dout pin
const int HX711_sck_3 = 18;  // mcu > HX711 no 3 sck pin

// HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1); // HX711 1
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2); // HX711 2
HX711_ADC LoadCell_3(HX711_dout_3, HX711_sck_3); // HX711 3

const int calVal_eepromAdress_1 = 0; // eeprom address for calibration value load cell 1 (4 bytes)
const int calVal_eepromAdress_2 = 4; // eeprom address for calibration value load cell 2 (4 bytes)
const int calVal_eepromAdress_3 = 8; // eeprom address for calibration value load cell 3 (4 bytes)
unsigned long t = 0;

float a, b, c;

void setup()
{
    Serial.begin(9600);
    delay(10);
    Serial.println();
    Serial.println("Starting...");

    float calibrationValue_1; // calibration value load cell 1
    float calibrationValue_2; // calibration value load cell 2
    float calibrationValue_3; // calibration value load cell 3

    calibrationValue_1 = 108.75; // uncomment this if you want to set this value in the sketch
    calibrationValue_2 = 108.75; // uncomment this if you want to set this value in the sketch
    calibrationValue_3 = 108.75; // uncomment this if you want to set this value in the sketch
#if defined(ESP8266) || defined(ESP32)
    // EEPROM.begin(512); // uncomment this if you use ESP8266 and want to fetch the value from eeprom
#endif
    // EEPROM.get(calVal_eepromAdress_1, calibrationValue_1); // uncomment this if you want to fetch the value from eeprom
    // EEPROM.get(calVal_eepromAdress_2, calibrationValue_2); // uncomment this if you want to fetch the value from eeprom
    // EEPROM.get(calVal_eepromAdress_3, calibrationValue_3); // uncomment this if you want to fetch the value from eeprom

    LoadCell_1.begin();
    LoadCell_2.begin();
    LoadCell_3.begin();

    unsigned long stabilizingtime = 2000;
    boolean _tare = true;
    byte loadcell_1_rdy = 0;
    byte loadcell_2_rdy = 0;
    byte loadcell_3_rdy = 0;

    while ((loadcell_1_rdy + loadcell_2_rdy + loadcell_3_rdy) < 3)
    {
        if (!loadcell_1_rdy)
            loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
        if (!loadcell_2_rdy)
            loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
        if (!loadcell_3_rdy)
            loadcell_3_rdy = LoadCell_3.startMultiple(stabilizingtime, _tare);
    }

    if (LoadCell_1.getTareTimeoutFlag())
    {
        Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations");
    }
    if (LoadCell_2.getTareTimeoutFlag())
    {
        Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations");
    }
    if (LoadCell_3.getTareTimeoutFlag())
    {
        Serial.println("Timeout, check MCU>HX711 no.3 wiring and pin designations");
    }

    LoadCell_1.setCalFactor(calibrationValue_1);
    LoadCell_2.setCalFactor(calibrationValue_2);
    LoadCell_3.setCalFactor(calibrationValue_3);

    Serial.println("Startup is complete");
}

void loop()
{
    static boolean newDataReady = 0;
    const int serialPrintInterval = 0;

    if (LoadCell_1.update())
        newDataReady = true;
    LoadCell_2.update();
    LoadCell_3.update();

    if (newDataReady)
    {
        if (millis() > t + serialPrintInterval)
        {
            a = LoadCell_1.getData();
            b = LoadCell_2.getData();
            c = LoadCell_3.getData();
            Serial.print("Load_cell 1 output val: ");
            Serial.print(a);
            Serial.print("    Load_cell 2 output val: ");
            Serial.print(b);
            Serial.print("    Load_cell 3 output val: ");
            Serial.println(c);
            newDataReady = 0;
            t = millis();
        }
    }

    if (Serial.available() > 0)
    {
        char inByte = Serial.read();
        if (inByte == 't')
        {
            LoadCell_1.tareNoDelay();
            LoadCell_2.tareNoDelay();
            LoadCell_3.tareNoDelay();
        }
    }

    if (LoadCell_1.getTareStatus() == true)
    {
        Serial.println("Tare load cell 1 complete");
    }
    if (LoadCell_2.getTareStatus() == true)
    {
        Serial.println("Tare load cell 2 complete");
    }
    if (LoadCell_3.getTareStatus() == true)
    {
        Serial.println("Tare load cell 3 complete");
    }
}

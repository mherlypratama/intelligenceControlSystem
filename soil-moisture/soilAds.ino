#include <Wire.h>
#include <DFRobot_ADS1115.h>

DFRobot_ADS1115 ads(&Wire);

void setup(void)
{
    Serial.begin(9600);
    ads.setAddr_ADS1115(ADS1115_IIC_ADDRESS1); // 0x48
    ads.setGain(eGAIN_TWOTHIRDS);              // 2/3x gain
    ads.setMode(eMODE_SINGLE);                 // single-shot mode
    ads.setRate(eRATE_128);                    // 128SPS (default)
    ads.setOSMode(eOSMODE_SINGLE);             // Set to start a single-conversion
    ads.init();
}

void loop(void)
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

    delay(1000);
}

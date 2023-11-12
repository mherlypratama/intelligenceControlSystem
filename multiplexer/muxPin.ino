#include "MUX74HC4067.h"
MUX74HC4067 DigitalMux(13, 12, 14, 27, 26);
int PIN;

void setup()
{
    DigitalMux.signalPin(25, INPUT_PULLUP, DIGITAL);
    Serial.begin(9600);
}

void loop()
{
    byte data;
    Serial.println("==0123456789012345==");
    Serial.print("  ");
    for (byte i = 0; i < 16; i++)
    {
        data = DigitalMux.read(i);
        if (data == HIGH)
        {
            Serial.print("1");
        }

        else
        {
            Serial.print("0");
        }
    }
    Serial.println();
}
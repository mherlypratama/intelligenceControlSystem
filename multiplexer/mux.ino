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
    PIN = DigitalMux.read(2);
    Serial.println(PIN);
}
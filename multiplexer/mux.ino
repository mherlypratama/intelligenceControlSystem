#include "MUX74HC4067.h"
MUX74HC4067 DigitalMux(11, 10, 9, 13, 12);
int PIN;

void setup()
{
    DigitalMux.signalPin(14, INPUT_PULLUP, DIGITAL);
    Serial.begin(9600);
}

void loop()
{
    PIN = DigitalMux.read(2);
    Serial.println(PIN);
}
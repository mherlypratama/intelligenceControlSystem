#include "RS485_Wind_Direction_Transmitter_V2.h"

#if defined(ARDUINO_AVR_UNO) || defined(ESP8266) // Use softserial
SoftwareSerial softSerial(/*rx =*/0, /*tx =*/26);
RS485_Wind_Direction_Transmitter_V2 windDirection(/*softSerial =*/&softSerial);
#elif defined(ESP32) // Use the hardserial of remappable pin: Serial1
RS485_Wind_Direction_Transmitter_V2 windDirection(/*hardSerial =*/&Serial1, /*rx =*/0, /*tx =*/26);
#else                // Use hardserial: Serial1
RS485_Wind_Direction_Transmitter_V2 windDirection(/*hardSerial =*/&Serial1);
#endif

uint8_t Address = 0x02;

const char *Orientation[17] = {
    "north", "north by northeast", "northeast", "east by northeast", "east", "east by southeast", "southeast", "south by southeast", "south",
    "south by southwest", "southwest", "west by southwest", "west", "west by northwest", "northwest", "north by northwest", "north"};

void setup()
{
    Serial.begin(9600);
    setwind();
}

void loop()
{
    sensorwind();
}

void setwind()
{

    // Init the sensor
    while (!(windDirection.begin()))
    {
        Serial.println("Communication with device failed, please check connection");
        delay(3000);
    }
    Serial.println("Begin ok!");
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
    delay(1000);
}
#include "RS485_Wind_Direction_Transmitter_V2.h"
#include <SoftwareSerial.h>
SoftwareSerial mySerial2(13, 19); // Define the soft serial port, port 3 is TX, port 2 is RX,

uint8_t Address0 = 0x10;

/**
   @brief RS485_Wind_Direction_Transmitter_V2 constructor
   @param serial - serial ports for communication, supporting hard and soft serial ports
   @param rx - UART Pin for receiving data
   @param tx - UART Pin for transmitting data
*/
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

void setup()
{
    Serial.begin(115200);

    // Init the sensor
    while (!(windDirection.begin()))
    {
        Serial.println("Communication with device failed, please check connection");
        delay(3000);
    }
    Serial.println("Begin ok!");
    mySerial2.begin(9600);
    ModifyAddress(0x00, Address0);
    // Revise Modbus slave address. You can use broadcast address 0x00 to revise if the current address is forgotten.
    //   windDirection.SetSlaveAddress(/*Current modbus slave address*/0x00, /*Revised address*/0x02);
    // Get modbus slave address
    //   Address = windDirection.GetSlaveAddress();
}

void loop()
{
    // Get 16 wind directions
    int Direction = windDirection.GetWindDirection(/*modbus slave address*/ Address);
    // Get 360° wind direction angle
    float Angle = windDirection.GetWindAngle(/*modbus slave address*/ Address);
    Serial.println(Orientation[Direction]);
    Serial.print(Angle);
    Serial.println("°");
    Serial.println();
    Serial.print(readWindSpeed(Address0)); // Read wind speed
    Serial.println("m/s");
    delay(1000);
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

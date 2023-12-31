#include <Wire.h>
#include <Adafruit_MLX90614.h>

// TCA9548A I2C Multiplexer Address
#define TCAADDR 0x70

Adafruit_MLX90614 mlx1 = Adafruit_MLX90614(); // Objek untuk sensor pertama
Adafruit_MLX90614 mlx2 = Adafruit_MLX90614(); // Objek untuk sensor kedua

void selectTCAChannel(uint8_t channel)
{
    // Select the appropriate channel on TCA9548A
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

void setup()
{
    Serial.begin(9600);

    Wire.begin(); // Initialize the I2C communication

    // Configure TCA9548A channel (0 for the first sensor, 1 for the second sensor)
    selectTCAChannel(0); // Choose the channel for the first sensor

    if (!mlx1.begin())
    {
        Serial.println("Error connecting to MLX sensor 1. Check wiring.");
        while (1)
            ;
    }

    // Wait for a moment before switching to the second sensor
    delay(1000);

    // Configure TCA9548A channel for the second sensor
    selectTCAChannel(1); // Choose the channel for the second sensor

    if (!mlx2.begin())
    {
        Serial.println("Error connecting to MLX sensor 2. Check wiring.");
        while (1)
            ;
    }
}

void loop()
{
    selectTCAChannel(0); // Select channel 0 (first sensor)
    Serial.print("Sensor 1 - Ambient temperature = ");
    Serial.print(mlx1.readAmbientTempC());
    Serial.print("°C   ");
    Serial.print("Object temperature = ");
    Serial.print(mlx1.readObjectTempC());
    Serial.println("°C");

    selectTCAChannel(1); // Select channel 1 (second sensor)
    Serial.print("Sensor 2 - Ambient temperature = ");
    Serial.print(mlx2.readAmbientTempC());
    Serial.print("°C   ");
    Serial.print("Object temperature = ");
    Serial.print(mlx2.readObjectTempC());
    Serial.println("°C");

    Serial.println("-----------------------------------------------------------------");
    delay(1000);
}

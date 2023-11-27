#include <SoftwareSerial.h>
#include <DFRobot_BMP3XX.h>

#define RE 33
#define DE 32

const byte pyranometer[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
byte values[8];
SoftwareSerial mod(25, 4);

int Solar_Radiation;
int jam, minute, second, tanggal, bulan, tahun;

float temperature, Pressure, humi;

DFRobot_BMP388_I2C sensor(&Wire, sensor.eSDOVDD);
#define CALIBRATE_ABSOLUTE_DIFFERENCE

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 25200;
const int daylightOffset_sec = 3600;

void setup()
{
    Serial.begin(9600);
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    printLocalTime();
    setbmp();
    setpyrano();

    delay(1000);
}

void loop()
{
    printLocalTime();

    sensorbmp();
    sensorpyrano();
    delay(3000);
}

void setpyrano()
{
    mod.begin(4800);
    pinMode(RE, OUTPUT);
    pinMode(DE, OUTPUT);
}

void sensorpyrano()
{
    // Transmit the request to the sensor
    digitalWrite(DE, HIGH);
    digitalWrite(RE, HIGH);
    delay(10);

    mod.write(pyranometer, sizeof(pyranometer));

    digitalWrite(DE, LOW);
    digitalWrite(RE, LOW);
    delay(10); // Give some time for the sensor to respond

    // Wait until we have the expected number of bytes or timeout
    unsigned long startTime = millis();
    while (mod.available() < 7 && millis() - startTime < 1000)
    {
        delay(1);
    }

    // Read the response
    byte index = 0;
    while (mod.available() && index < 8)
    {
        values[index] = mod.read();
        Serial.print(values[index], HEX);
        Serial.print(" ");
        index++;
    }
    Serial.println();

    // Parse the Solar Radiation value
    Solar_Radiation = int(values[3] << 8 | values[4]);
    Serial.print("Solar Radiation: ");
    Serial.print(Solar_Radiation);
    Serial.println(" W/m^2");
}

void setbmp()
{

    int rslt;
    while (ERR_OK != (rslt = sensor.begin()))
    {
        if (ERR_DATA_BUS == rslt)
        {
            Serial.println("Data bus error!!!");
        }
        else if (ERR_IC_VERSION == rslt)
        {
            Serial.println("Chip versions do not match!!!");
        }
        delay(3000);
    }
    Serial.println("Begin ok!");

    while (!sensor.setSamplingMode(sensor.eUltraPrecision))
    {
        Serial.println("Set samping mode fail, retrying....");
        delay(3000);
    }

    delay(100);
#ifdef CALIBRATE_ABSOLUTE_DIFFERENCE

    if (sensor.calibratedAbsoluteDifference(540.0))
    {
        Serial.println("Absolute difference base value set successfully!");
    }
#endif

    float sampingPeriodus = sensor.getSamplingPeriodUS();
    Serial.print("samping period : ");
    Serial.print(sampingPeriodus);
    Serial.println(" us");

    float sampingFrequencyHz = 1000000 / sampingPeriodus;
    Serial.print("samping frequency : ");
    Serial.print(sampingFrequencyHz);
    Serial.println(" Hz");

    Serial.println();
}

void sensorbmp()
{
    temperature = sensor.readTempC();
    Serial.print("temperature : ");
    Serial.print(temperature);
    Serial.println(" C");

    humi = 83.4 - temperature;
    Serial.print("humi : ");
    Serial.print(humi);
    Serial.println(" C");

    Pressure = sensor.readPressPa();
    Serial.print("Pressure : ");
    Serial.print(Pressure);
    Serial.println(" Pa");

    altitude = sensor.readAltitudeM();
    Serial.print("Altitude : ");
    Serial.print(altitude);
    Serial.println(" m");

    Serial.println();
}
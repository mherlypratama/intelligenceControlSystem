#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// Alamat I2C dari modul ADS1115
#define ADS1115_ADDRESS 0x48

// Pin untuk sensor wind direction
const int windDirectionPin = 0; // Channel 0 pada ADS1115

Adafruit_ADS1115 ads;

void setup(void)
{
    Serial.begin(115200);

    // Mulai koneksi dengan modul ADS1115
    if (!ads.begin(ADS1115_ADDRESS))
    {
        Serial.println("Couldn't find ADS1115");
        while (1)
            ;
    }

    // Set range ke 4.096V (default adalah 6.144V)
    ads.setGain(GAIN_TWOTHIRDS);
}

void loop(void)
{
    // Baca nilai analog dari sensor wind direction
    int16_t adcValue = ads.readADC_SingleEnded(windDirectionPin);

    // Konversi nilai analog menjadi sudut (0-360)
    float voltage = adcValue * (4.096 / 32767.0);  // Konversi nilai ADC menjadi tegangan
    float degrees = (voltage - 0.5) * 360.0 / 3.5; // Konversi tegangan menjadi sudut

    // Pastikan nilai sudut berada dalam rentang 0-360
    if (degrees < 0)
    {
        degrees += 360;
    }

    // Tentukan keterangan arah mata angin berdasarkan sudut
    String windDirection;
    if (degrees >= 337.5 || degrees < 22.5)
    {
        windDirection = "Utara";
    }
    else if (degrees >= 22.5 && degrees < 67.5)
    {
        windDirection = "Timur Laut";
    }
    else if (degrees >= 67.5 && degrees < 112.5)
    {
        windDirection = "Timur";
    }
    else if (degrees >= 112.5 && degrees < 157.5)
    {
        windDirection = "Tenggara";
    }
    else if (degrees >= 157.5 && degrees < 202.5)
    {
        windDirection = "Selatan";
    }
    else if (degrees >= 202.5 && degrees < 247.5)
    {
        windDirection = "Barat Daya";
    }
    else if (degrees >= 247.5 && degrees < 292.5)
    {
        windDirection = "Barat";
    }
    else if (degrees >= 292.5 && degrees < 337.5)
    {
        windDirection = "Barat Laut";
    }

    // Tampilkan keterangan arah mata angin
    Serial.print("Sudut: ");
    Serial.print(degrees);
    Serial.print("°\tArah Angin: ");
    Serial.println(windDirection);

    delay(1000); // Tunggu 1 detik sebelum membaca lagi
}
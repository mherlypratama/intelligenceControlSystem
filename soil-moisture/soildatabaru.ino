#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads;

float fix0, fix1, fix2, fix3;

void setup(void)
{
    Serial.begin(115200);

    Serial.println("Getting single-ended readings from AIN0..3");
    Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

    ads.setGain(GAIN_TWOTHIRDS);

    if (!ads.begin())
    {
        Serial.println("Failed to initialize ADS.");
        while (1)
            ;
    }
}

void loop(void)
{
    int16_t adc0, adc1, adc2, adc3;
    float volts0, volts1, volts2, volts3;

    adc0 = ads.readADC_SingleEnded(0);
    adc1 = ads.readADC_SingleEnded(1);
    adc2 = ads.readADC_SingleEnded(2);
    adc3 = ads.readADC_SingleEnded(3);

    volts0 = ads.computeVolts(adc0);
    volts1 = ads.computeVolts(adc1);
    volts2 = ads.computeVolts(adc2);
    volts3 = ads.computeVolts(adc3);

    // Pastikan rentang nilai ADC yang diharapkan sesuai dengan sensor soil moisture
    float adcMin = 0.5;  // Ganti dengan nilai ADC terendah yang dihasilkan oleh sensor
    float adcMax = 2.53; // Ganti dengan nilai ADC tertinggi yang dihasilkan oleh sensor

    float datakonversi0 = (volts0 - adcMin) / (adcMax - adcMin) * 100;
    float datakonversi1 = (volts1 - adcMin) / (adcMax - adcMin) * 100;
    float datakonversi2 = (volts2 - adcMin) / (adcMax - adcMin) * 100;
    float datakonversi3 = (volts3 - adcMin) / (adcMax - adcMin) * 100;

    fix0 = 100 - datakonversi0;
    fix1 = 100 - datakonversi1;
    fix2 = 100 - datakonversi2;
    fix3 = 100 - datakonversi3;

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

    delay(1000);
}
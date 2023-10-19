// Include library
#include <Arduino.h>

// Define the digital pins to which the water flow sensors are connected
const int waterFlowPin1 = 2; // Ganti dengan pin yang sesuai
const int waterFlowPin2 = 3; // Ganti dengan pin yang sesuai
const int waterFlowPin3 = 4; // Ganti dengan pin yang sesuai
const int waterFlowPin4 = 5; // Ganti dengan pin yang sesuai

unsigned int flowRate1 = 0; // Laju aliran air sensor 1 (liter/menit)
unsigned int flowRate2 = 0; // Laju aliran air sensor 2 (liter/menit)
unsigned int flowRate3 = 0; // Laju aliran air sensor 3 (liter/menit)
unsigned int flowRate4 = 0; // Laju aliran air sensor 4 (liter/menit)

unsigned long totalMilliliters1 = 0; // Total air yang telah mengalir sensor 1 (mililiter)
unsigned long totalMilliliters2 = 0; // Total air yang telah mengalir sensor 2 (mililiter)
unsigned long totalMilliliters3 = 0; // Total air yang telah mengalir sensor 3 (mililiter)
unsigned long totalMilliliters4 = 0; // Total air yang telah mengalir sensor 4 (mililiter)

unsigned long previousMillis = 0;   // Waktu sebelumnya
const unsigned long interval = 500; // Interval waktu (dalam milidetik) untuk membaca sensor

void setup()
{
    // Inisialisasi Serial Monitor
    Serial.begin(115200);

    // Set pin water flow sensors sebagai input
    pinMode(waterFlowPin1, INPUT);
    pinMode(waterFlowPin2, INPUT);
    pinMode(waterFlowPin3, INPUT);
    pinMode(waterFlowPin4, INPUT);
}

void loop()
{
    unsigned long currentMillis = millis();

    // Baca sensor water flow 1
    int sensorValue1 = digitalRead(waterFlowPin1);
    if (sensorValue1 == LOW)
    {
        flowRate1++;
        totalMilliliters1 += 50; // Sesuaikan faktor konversi sesuai spesifikasi sensor

        // Baca sensor water flow 2
        int sensorValue2 = digitalRead(waterFlowPin2);
        if (sensorValue2 == LOW)
        {
            flowRate2++;
            totalMilliliters2 += 50; // Sesuaikan faktor konversi sesuai spesifikasi sensor

            // Baca sensor water flow 3
            int sensorValue3 = digitalRead(waterFlowPin3);
            if (sensorValue3 == LOW)
            {
                flowRate3++;
                totalMilliliters3 += 50; // Sesuaikan faktor konversi sesuai spesifikasi sensor

                // Baca sensor water flow 4
                int sensorValue4 = digitalRead(waterFlowPin4);
                if (sensorValue4 == LOW)
                {
                    flowRate4++;
                    totalMilliliters4 += 50; // Sesuaikan faktor konversi sesuai spesifikasi sensor

                    // Cek apakah sudah waktunya untuk menampilkan hasil
                    if (currentMillis - previousMillis >= interval)
                    {
                        // Tampilkan hasil
                        Serial.print("Sensor 1 - Laju Aliran Air: ");
                        Serial.print(flowRate1);
                        Serial.println(" L/min");
                        Serial.print("Sensor 1 - Total Mililiter Air: ");
                        Serial.print(totalMilliliters1);
                        Serial.println(" mL");

                        Serial.print("Sensor 2 - Laju Aliran Air: ");
                        Serial.print(flowRate2);
                        Serial.println(" L/min");
                        Serial.print("Sensor 2 - Total Mililiter Air: ");
                        Serial.print(totalMilliliters2);
                        Serial.println(" mL");

                        Serial.print("Sensor 3 - Laju Aliran Air: ");
                        Serial.print(flowRate3);
                        Serial.println(" L/min");
                        Serial.print("Sensor 3 - Total Mililiter Air: ");
                        Serial.print(totalMilliliters3);
                        Serial.println(" mL");

                        Serial.print("Sensor 4 - Laju Aliran Air: ");
                        Serial.print(flowRate4);
                        Serial.println(" L/min");
                        Serial.print("Sensor 4 - Total Mililiter Air: ");
                        Serial.print(totalMilliliters4);
                        Serial.println(" mL");

                        // Reset flowRate dan update previousMillis
                        flowRate1 = 0;
                        flowRate2 = 0;
                        flowRate3 = 0;
                        flowRate4 = 0;
                        previousMillis = currentMillis;
                    }
                }
            }
        }
    }
}

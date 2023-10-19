// Include library
#include <Arduino.h>

// Define the digital pin to which the water flow sensor is connected
const int waterFlowPin = 2; // Ganti dengan pin yang sesuai

unsigned int flowRate = 0;          // Variable untuk laju aliran air (dalam liter/menit)
unsigned long totalMilliliters = 0; // Variable untuk menghitung total air yang telah mengalir (dalam mililiter)
unsigned long previousMillis = 0;   // Variable untuk menyimpan waktu sebelumnya
const unsigned long interval = 500; // Interval waktu (dalam milidetik) untuk membaca sensor

void setup()
{
    // Inisialisasi Serial Monitor
    Serial.begin(115200);

    // Set pin water flow sensor sebagai input
    pinMode(waterFlowPin, INPUT);
}

void loop()
{
    unsigned long currentMillis = millis();

    // Baca sensor water flow
    int sensorValue = digitalRead(waterFlowPin);

    // Jika sensor mendeteksi aliran air (sensor LOW), tambahkan jumlah liter
    if (sensorValue == LOW)
    {
        flowRate++;             // Increment laju aliran air
        totalMilliliters += 50; // Tambahkan 50 mL ke total (kustom sesuai dengan spesifikasi sensor Anda)
    }

    // Cek apakah sudah waktunya untuk menampilkan hasil
    if (currentMillis - previousMillis >= interval)
    {
        // Tampilkan hasil
        Serial.print("Laju Aliran Air: ");
        Serial.print(flowRate);
        Serial.println(" L/min");
        Serial.print("Total Mililiter Air: ");
        Serial.print(totalMilliliters);
        Serial.println(" mL");

        // Reset flowRate dan update previousMillis
        flowRate = 0;
        previousMillis = currentMillis;
    }
}

#include <SoftwareSerial.h>

SoftwareSerial Hujan(2, 3); // RX (2) dan TX (3) untuk komunikasi dengan ESP32

void setup()
{
    Serial.begin(9600); // Inisialisasi komunikasi serial dengan komputer (untuk debugging)
    Hujan.begin(9600);  // Inisialisasi komunikasi serial dengan komputer (untuk debugging)
}

void loop()
{
    String DataB = "tidur yuk";
    Serial.println(DataB);
    Hujan.println(DataB);
    delay(1000);
}

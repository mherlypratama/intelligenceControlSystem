#include <SoftwareSerial.h>

SoftwareSerial Hujan(16, 17); // RX (2) dan TX (3) untuk komunikasi dengan ESP32
 
void setup()
{
    Serial.begin(9600); // Inisialisasi komunikasi serial dengan komputer (untuk debugging)
    Hujan.begin(9600);  // Inisialisasi komunikasi serial dengan komputer (untuk debugging)
}

void loop()
{
    String data = "";
    while (Hujan.available() > 0)
    {
        data = char(Hujan.read());
    }
    Serial.println(data);
    delay(1000);
}

// if(data == "ON"){
// digitalWrite(relayPin, HIGH); // Nyalakan relay

// }else if (data == "OFF"){
//   digitalWrite(relayPin, LOW);
// }
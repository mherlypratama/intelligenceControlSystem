#define RELAY_PIN 26 // ESP32 pin GPIO16 connected to the IN pin of relay

// the setup function runs once when you press reset or power the board
void setup()
{
    // initialize digital pin as an output.
    pinMode(RELAY_PIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop()
{
    digitalWrite(RELAY_PIN, HIGH);
    delay(3000);
    digitalWrite(RELAY_PIN, LOW);
    delay(5000);
}

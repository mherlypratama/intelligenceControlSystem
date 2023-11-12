// Contoh untuk Arduino (menggunakan PubSubClient)
#include <PubSubClient.h>
#include <WiFi.h>

const char *ssid = "Lab Telkom 2.4 GHz";
const char *password = "telekomunikasi";
const char *mqttBroker = "alamat.broker.mqtt";
const int mqttPort = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

void setup()
{
    // Setup koneksi WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }

    // Setup koneksi MQTT
    client.setServer(mqttBroker, mqttPort);
    // Berlangganan ke topik "sensor/temperatur"
    client.subscribe("sensor/temperatur");
    client.setCallback(callback);
}

void loop()
{
    // Proses koneksi MQTT dan panggil fungsi callback jika ada pesan yang diterima
    client.loop();
}

void callback(char *topic, byte *payload, unsigned int length)
{
    Serial.print("Message received on topic: ");
    Serial.println(topic);

    Serial.print("Payload: ");
    for (int i = 0; i < length; i++)
    {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}

#include <OneWire.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Konfigurasi jaringan Wi-Fi
const char *ssid = "Modal";
const char *password = "12345678";

// Konfigurasi server MQTT di VPS Anda
const char *mqtt_server = "vps.isi-net.org";
const int mqtt_port = 1883;
const char *mqtt_user = "unila";
const char *mqtt_password = "pwdMQTT@123";

const char *topic_bme = "bme/pertanian";
const char *topic_soil = "soil/pertanian";
const char *topic_weight = "weight/pertanian";
const char *topic_infra = "infra/pertanian";
const char *topic_water = "water/pertanian";
const char *topic_rain = "rain/pertanian";
const char *topic_tds = "tds/pertanian";
const char *topic_ds = "ds/pertanian";
const char *topic_ph "ph/pertanian";
const char *topic_wind = "wind/pertanian";
const char *topic_winddir = "winddir/pertanian";

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsgTime = 0;
const long interval = 5000; // Kirim data setiap 5 detik

void setup()
{

    setupWiFi();
    client.setServer(mqtt_server, mqtt_port);
}

void loop()
{
    if (!client.connected())
    {
        reconnectMQTT();
    }
    client.loop();

    nodered();
}

void nodered()
{
    char phStr[10];
    snprintf(phStr, sizeof(phStr), "%.2f", phValue); // Mengonversi nilai pH ke string
    client.publish(topic_ph, phStr);                 // Mengirim data pH ke broker MQTT

    // Kirim data TDS ke broker MQTT
    char tdsStr[10];
    snprintf(tdsStr, sizeof(tdsStr), "%d", (int)tdsValue); // Mengonversi nilai TDS ke string
    client.publish(topic_tds, tdsStr);                     // Mengirim data TDS ke broker MQTT

    // Kirim data sensor hujan ke broker MQTT
    char rainStr[10];
    snprintf(rainStr, sizeof(rainStr), "%.2f", rainAccumulated); // Mengonversi jumlah hujan ke string
    client.publish(topic_rain, rainStr);                         // Mengirim data hujan ke broker MQTT

    // Kirim data suhu ke broker MQTT
    char tempStr[10];
    snprintf(tempStr, sizeof(tempStr), "%.2f", temperature); // Mengonversi nilai suhu ke string
    client.publish(topic_ds, tempStr);                       // Mengirim data suhu ke broker MQTT
    delay(5000);
}

void setupWiFi()
{
    Serial.print("Menghubungkan ke WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(5000);
        Serial.println("Menghubungkan ke WiFi...");
    }
    Serial.println("Terhubung ke WiFi");
}

void reconnectMQTT()
{
    while (!client.connected())
    {
        Serial.print("Menghubungkan ke broker MQTT...");
        if (client.connect("ESP32Client", mqtt_user, mqtt_password))
        {
            Serial.println("Terhubung ke broker MQTT");
        }
        else
        {
            Serial.print("Gagal, kode kesalahan = ");
            Serial.println(client.state());
            delay(5000);
        }
    }
}

void callback(char *topic, byte *payload, unsigned int length)
{
    // Implementasi callback jika diperlukan
}
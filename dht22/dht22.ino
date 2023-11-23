#include <DHT.h>

#define DHTPIN 2      // Pin data sensor DHT22 terhubung ke pin 2 pada ESP32
#define DHTTYPE DHT22 // Jenis sensor, sesuaikan dengan sensor yang Anda gunakan

DHT dht(DHTPIN, DHTTYPE);

void setup()
{
    Serial.begin(9600);
    dht.begin();
}

void loop()
{
    // Baca data dari sensor DHT22
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    // Periksa apakah pembacaan sensor berhasil
    if (isnan(humidity) || isnan(temperature))
    {
        Serial.println("Gagal membaca data dari sensor DHT22!");
        return;
    }

    // Tampilkan nilai kelembaban dan suhu
    Serial.print("Kelembaban: ");
    Serial.print(humidity);
    Serial.print("%\t");
    Serial.print("Suhu: ");
    Serial.print(temperature);
    Serial.println("°C");

    delay(2000); // Tunggu 2 detik sebelum membaca ulang
}
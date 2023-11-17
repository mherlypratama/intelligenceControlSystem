#define SOIL_MOISTURE_PIN 27 // Ganti dengan pin yang sesuai

void setup()
{
  Serial.begin(9600);
  pinMode(SOIL_MOISTURE_PIN, INPUT);
}

void loop()
{
  // Baca nilai analog dari sensor soil moisture
  int soilMoistureValue = analogRead(SOIL_MOISTURE_PIN);

  // Konversi nilai analog menjadi persentase kelembaban tanah (sesuaikan dengan kalibrasi Anda)
  float soilMoisturePercentage = map(soilMoistureValue, 900, 3000, 0, 100);

  // Tampilkan nilai kelembaban tanah
  Serial.print("Kelembaban Tanah: ");
  Serial.print(soilMoisturePercentage);
  Serial.println("%");

  delay(1000); // Baca data setiap 1 detik
}
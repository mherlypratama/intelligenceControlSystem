import bme280
import smbus2
from time import sleep

port = 1
address = 0x76  # Alamat sensor BME280. Alamat yang berbeda mungkin diperlukan untuk sensor lainnya
bus = smbus2.SMBus(port)

bme280.load_calibration_params(bus, address=0x77)

while True:
    bme280_data = bme280.sample(bus, address=0x77)
    humidity = bme280_data.humidity
    pressure = bme280_data.pressure
    ambient_temperature = bme280_data.temperature

    # Menghitung ketinggian berdasarkan tekanan
    sea_level_pressure = 1013.25  # Tekanan udara di permukaan laut dalam hPa
    altitude = 44330.0 * (1.0 - (pressure / sea_level_pressure) ** 0.1903)

    # Menampilkan data dengan format yang lebih baik
    print("Kelembaban: {:.2f}%".format(humidity))
    print("Tekanan: {:.2f} hPa".format(pressure))
    print("Suhu: {:.2f} C".format(ambient_temperature))
    print("Ketinggian: {:.2f} meters".format(altitude))  # Menampilkan ketinggian
    print("\n")

    sleep(1)

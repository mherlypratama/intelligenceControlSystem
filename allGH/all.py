import bme280
import smbus2
from time import sleep

import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn


# Inisialisasi antarmuka I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Inisialisasi ADC (ADS1115)
ads = ADS.ADS1115(i2c, address=0x49)


def soil():
    # Baca nilai analog dari sensor soil moisture
    # Konfigurasi kanal analog (A0 pada ADS1115)
    chan = AnalogIn(ads, ADS.P0)
    soil_moisture = chan.value
    datakonversi = ((soil_moisture - 10540) / (19941 - 10540)) * 100
    fix = 100 - datakonversi
    return fix


def bme_280():
    port = 1
    address = 0x76  # Alamat sensor BME280. Alamat yang berbeda mungkin diperlukan untuk sensor lainnya
    bus = smbus2.SMBus(port)

    bme280.load_calibration_params(bus, address=0x77)
    bme280_data = bme280.sample(bus, address=0x77)
    humidity = bme280_data.humidity
    pressure = bme280_data.pressure
    ambient_temperature = bme280_data.temperature

    # Menghitung ketinggian berdasarkan tekanan
    sea_level_pressure = 1013.25  # Tekanan udara di permukaan laut dalam hPa
    altitude = 44330.0 * (1.0 - (pressure / sea_level_pressure) ** 0.1903)

    return humidity, pressure, ambient_temperature, altitude


def main():
    while True:
        humidity, pressure, ambient_temperature, altitude = bme_280()
        fix = soil()

        # Menampilkan data dengan format yang lebih baik
        print("Kelembaban: {:.2f}%".format(humidity))
        print("Tekanan: {:.2f} hPa".format(pressure))
        print("Suhu: {:.2f} C".format(ambient_temperature))
        print("Ketinggian: {:.2f} meters".format(altitude))  # Menampilkan ketinggian
        print("Nilai Soil Moisture:", fix, "%")
        print("\n")

        sleep(1)


if __name__ == "__main__":
    main()

import board
import busio
import adafruit_tca9548a
import bme280
import smbus2
from time import sleep

# Inisialisasi objek I2C pada Raspberry Pi
i2c = busio.I2C(board.SCL, board.SDA)

# Inisialisasi objek TCA9548A dengan alamat I2C 0x70
tca = adafruit_tca9548a.TCA9548A(i2c, address=0x70)

# Pilih channel 0 (untuk BME280)
tca[0].enable()

# Inisialisasi objek BME280
port = 1
address = 0x76  # Alamat sensor BME280. Alamat yang berbeda mungkin diperlukan untuk sensor lainnya
bus = smbus2.SMBus(port)

bme280.load_calibration_params(bus, address=0x70)
bme280_data = bme280.sample(bus, address=0x70)

# Baca data dari BME280
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

# Kemudian, jika Anda ingin mengakses perangkat yang terhubung ke channel 1 (misalnya, ADS)
tca.switch_to_channel(1)
print("ini switch di channel 1")

# Sekarang Anda dapat mengakses perangkat yang terhubung ke channel 1 (misalnya, ADS)
# Lakukan bacaan atau operasi lainnya sesuai dengan perangkat tersebut

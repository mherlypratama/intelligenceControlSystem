import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import time

# Inisialisasi antarmuka I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Inisialisasi ADC (ADS1115)
ads = ADS.ADS1115(i2c)

# Konfigurasi kanal analog (A0 pada ADS1115)
chan = AnalogIn(ads, ADS.P0)

try:
    while True:
        # Baca nilai analog dari sensor soil moisture
        adc = chan.value
        tds = (adc - 30) / (5107 - 30) * 417
        print("Nilai TDS Meter:", tds, "ppm")

        # Tunggu sebentar sebelum membaca lagi
        time.sleep(1)

except KeyboardInterrupt:
    pass

import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import time

# Inisialisasi antarmuka I2C
i2c = busio.I2C(3, 2)

# Inisialisasi ADC (ADS1115)
ads = ADS.ADS1115(i2c, address=0x48)

# Konfigurasi kanal analog (A0 pada ADS1115)
chan = AnalogIn(ads, ADS.P0)
chan1 = AnalogIn(ads, ADS.P1)
chan2 = AnalogIn(ads, ADS.P2)

try:
    while True:
        # Baca nilai analog dari sensor soil moisture
        soil_moisture1 = chan.value
        soil_moisture2 = chan1.value
        soil_moisture3 = chan2.value

        datakonversi1 = ((soil_moisture1 - 10540) / (19941 - 10540)) * 100
        datakonversi2 = ((soil_moisture2 - 10540) / (19941 - 10540)) * 100
        datakonversi3 = ((soil_moisture3 - 10540) / (19941 - 10540)) * 100

        fix1 = 100 - datakonversi1
        fix2 = 100 - datakonversi2
        fix3 = 100 - datakonversi3

        print("Soil-1: ", fix1, "%")
        print("Soil-2: ", fix2, "%")
        print("Soil-3: ", fix3, "%")

        # Tunggu sebentar sebelum membaca lagi
        time.sleep(1)

except KeyboardInterrupt:
    pass

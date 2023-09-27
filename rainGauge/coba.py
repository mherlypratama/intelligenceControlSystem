import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import time
import RPi.GPIO as GPIO

# ****************START rain Gauge*************
bucketPin = 23  # Pin yang terhubung ke sensor ember
rainCount = 0
totalRainComulative = 0  # Jumlah hujan dalam interval 1 menit
interval = 1  # Interval pengukuran dalam detik
daysInterval = 60  # Interval 1 menit dalam detik

GPIO.setmode(GPIO.BCM)
GPIO.setup(bucketPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)


def bucket_tipped(channel):
    global rainCount
    rainCount += 1


GPIO.add_event_detect(bucketPin, GPIO.FALLING, callback=bucket_tipped, bouncetime=100)

# **************END RAIN GAUGE*************


# Inisialisasi I2C dengan alamat 0x49
i2c = busio.I2C(
    board.SCL, board.SDA, frequency=100000
)  # Sesuaikan alamat dan frekuensi I2C

# Inisialisasi ADC (ADS1115)
ads = ADS.ADS1115(i2c, address=0x49)  # Tentukan alamat I2C yang benar (0x49)

# Konfigurasi kanal analog (A0 pada ADS1115)
chan = AnalogIn(ads, ADS.P0)
chan2 = AnalogIn(ads, ADS.P1)

try:
    while True:
        # Baca nilai analog dari sensor soil moisture
        time.sleep(interval)

        # *****************START RAIN*****************
        mm = 0.3 * rainCount  # Menghitung volume hujan dalam mm
        totalRainComulative += mm
        rainCount = 0  # Reset hitungan setelah mengukur

        if time.time() % daysInterval < interval:
            # print("Total Rain in Last Minute:", totalRainMinute, "mm")
            # print(totalRainMinute)
            totalRainComulative = 0

        # ****************END RAIN****************
        soil_moisture = chan.value
        tds = chan2.value
        datakonversi = ((soil_moisture - 10540) / (19941 - 10540)) * 100
        fix = 100 - datakonversi
        tds_nilai = (tds - 30) / (5107 - 30) * 417
        # print(round(mm, 3), round(totalRainComulative, 3))
        print(
            "Soil Moisture:",
            fix,
            "%",
            "||",
            "Tds: ",
            tds_nilai,
            "ppm",
            "||",
            "Volume Hujan: ",
            round(mm, 3),
            "||",
            "Hujan(1 menit): ",
            round(totalRainComulative, 3),
        )

        # Tunggu sebentar sebelum membaca lagi
        time.sleep(1)

except KeyboardInterrupt:
    pass

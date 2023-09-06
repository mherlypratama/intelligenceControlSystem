import RPi.GPIO as GPIO
import time

bucketPin = 23  # Pin yang terhubung ke sensor ember
rainCount = 0
totalRainComulative = 0  # Jumlah hujan dalam interval 1 menit
interval = 1  # Interval pengukuran dalam detik
daysInterval = 60  # Interval 1 menit dalam detik

GPIO.setmode(GPIO.BCM)
GPIO.setup(bucketPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# GPIO.setwarnings(False)


def bucket_tipped(channel):
    global rainCount
    rainCount += 1


def main():
    global rainCount, totalRainComulative

    GPIO.add_event_detect(
        bucketPin, GPIO.FALLING, callback=bucket_tipped, bouncetime=100
    )

    try:
        while True:
            time.sleep(interval)

            mm = 0.3 * rainCount  # Menghitung volume hujan dalam mm
            totalRainComulative += mm
            rainCount = 0  # Reset hitungan setelah mengukur
            print(round(mm, 3), round(totalRainComulative, 3))

            if time.time() % daysInterval < interval:
                # print("Total Rain in Last Minute:", totalRainMinute, "mm")
                # print(totalRainMinute)
                totalRainComulative = 0

            # print("Rainfall Measurement:", mm, "mm")

    except KeyboardInterrupt:
        GPIO.cleanup()


if __name__ == "__main__":
    time.sleep(6)
    print(0, 0)
    main()

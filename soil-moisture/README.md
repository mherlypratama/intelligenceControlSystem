# SOIL MOISTURE

## Prepare

### 1. Wiring

Hubungkan sensor soil moisture ke ADC, Pada project ini saya menggunakan ADC yang outputnya menuju pin I2C(SDA dan SCL). Selanjutnya hubungkan dari ADC ke raspberrypi,

### 2. Preparing

#### 1. Update dan Upgrade Raspi

Lakukan update dan upgrade pada raspberrypi:

```
$ sudo apt update
$ sudo apt upgrade
$ sudo apt-get install python3-pip
$ sudo pip3 install --upgrade setuptools
```

#### 2. Instalasi Blinka

Selanjutnya adalah Install Blinka untuk inisasi Pin

```
cd ~
sudo pip3 install --upgrade adafruit-python-shell
wget https://raw.githubusercontent.com/adafruit/Raspberry-Pi-Installer-Scripts/master/raspi-blinka.py
sudo python3 raspi-blinka.py
```

#### 3. Periksa Koneksi I2C dan SPI

Cek I2C dan SPI

```
ls /dev/i2c* /dev/spi*
```

responsenya akan seperti berikut ini:
`/dev/i2c-1 /dev/spidev0.0 /dev/spidev0.1`

Selanjutnya adalah melakukan `Enabling Second SPI` :

```
sudo nano /boot/config.txt
```

Tambahkan `dtoverlay=spi1-3cs` pada baris paling bawah

#### 4. Blinka Test

buat file `blinkatest.py` dengan cara

```
sudo nano blinkatest.py
```

Masukkan kode berikut ini:

```
import board
import digitalio
import busio

print("Hello blinka!")

# Try to great a Digital input
pin = digitalio.DigitalInOut(board.D4)
print("Digital IO ok!")

# Try to create an I2C device
i2c = busio.I2C(board.SCL, board.SDA)
print("I2C ok!")

# Try to create an SPI device
spi = busio.SPI(board.SCLK, board.MOSI, board.MISO)
print("SPI ok!")

print("done!")
```

#### JIKA TERJADI ERROR

Jika terjadi Error seperti berikut:

```
AttributeError: module 'board' has no attribute 'D4'
```

atau

```
Traceback (most recent call last):
   File "servo_test.py", line 8, in <module>
      i2c = busio.I2C(board.SCL, board.SDA)
AttributeError: module 'board' has no attribute 'SCL'
```

Lakukan Langkah berikut ini:

```
pip3 uninstall board
pip3 install adafruit-blinka
```

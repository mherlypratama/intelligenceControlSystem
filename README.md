# Greenhouse Pertanian

## Hardware

1. Esp32 Wroom DFRobot
2. Wifi gak bisa 5 GHz

## Pin Sensor

### Analog

1. Sensor PH terhubung ke pin `35`
2. Sensor TDS terhubung ke pin `34`
3. Sensor Rain terhubung ke pin `15`

### Digital

1. Sensor Suhu Air terhubung ke pin `23`
2. Sensor Wind Direction terhubung ke pin `0` dan `26`
3. Sensor Anemometer terhubung ke pin `25` dan `21--19`
4. Sensor Waterflow terhubung ke pin `14`, `13`, `17`, dan `16`
5. Sensor Berat terhubung ke pin `12`(sck) dan `4` (dt)
6. Relay terhubung ke pin `19`, `23`, dan `18`

### I2C Pin(Paralel)

1. Sensor BME-280
2. Sensor Soil Moisture > ADS115
3. Infrared Sensor > TCA9548A

# intelligenceControlSystem

## ESP32 Greenhouse

### Digital

1. Anemometer terhubung ke pin `25` dan `26`
2. Wind Direction terhubung ke pin `0` dan `14`
3. Weight Sensor terhubung ke pin (`22`, `19`), (`23`, `18`), (`0`, `0`)
4. Pyranometer terhubung ke pin (4 pin)

### I2c

1. Infrared Sensor > TCA9548A

## RaspberryPI

1. Waterflow = 12 Buah
2. BME
3. DHT22
4. Relay Lampu UV
5. Soil Moisture

## Pyranometer

*Kabel biru ----- B
*Kabel Hijau----- A

*R0= 25/D2 atau 12/D11
*D1= 26/D3 atau 4/D12
*DE = 13/D7 atau 16
*RE = 14/D6 atau 17

## Anemometer

*RX= 16 atau 13
*TX= 17 atau 14

## Wind

*Rx = 26
*TX = 25

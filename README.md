# Greenhouse Pertanian

## Hardware

1. Esp32 Wroom DFRobot

## Pin Sensor

### Analog

1. Sensor PH terhubung ke pin `35`
2. Sensor TDS terhubung ke pin `34`
3. Sensor Rain terhubung ke pin `15`

### Digital

1. Sensor Suhu Air terhubung ke pin `3`
2. Sensor Wind Direction terhubung ke pin `1` dan `25`
3. Sensor Anemometer terhubung ke pin `26` dan `0`
4. Sensor Waterflow terhubung ke pin `14`, `13`, `17`, dan `16`
5. Sensor Berat terhubung ke pin `12`(sck) dan `4` (dt)

### I2C Pin(Paralel)

1. Sensor BME-280
2. Sensor Soil Moisture > ADS115
3. Infrared Sensor > TCA9548A

# intelligenceControlSystem

## ESP32 Greenhouse

### Digital

1. Wind Direction terhubung ke pin `3` dan `1`
2. Anemometer terhubung ke pin `25` dan `26`
3. Waterflow terhubung ke pin `0` dan `14`
4. Weight Sensor terhubung ke pin (`17`, `16`), (`4`, `12`)

### I2c

1. Infrared Sensor > TCA9548A

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

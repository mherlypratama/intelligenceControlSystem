# TDS METER

## Wiring

Koneksi wiring sama dengan Soil Moisture

## Kalibrasi nilai awal

- Baca nilai tegangan (misalnya, dalam volt) dari sensor Anda saat berada dalam larutan referensi dengan konsentrasi nol (biasanya air murni). Ini adalah nilai "V0."
- Baca nilai tegangan dari sensor Anda saat berada dalam larutan referensi dengan konsentrasi yang diketahui (misalnya, 1000 ppm). Ini adalah nilai "V1000."

## Perhitungan PPM

Hitung perbedaan tegangan antara nilai saat ini dari sensor Anda (V_sensor) dengan nilai tegangan saat larutan dalam kondisi nol (V0): ΔV = V_sensor - V0.

Hitung perbedaan tegangan antara nilai saat ini dengan nilai tegangan saat larutan dalam kondisi 1000 ppm (V1000 - V0): ΔV_reference = V1000 - V0.

Hitung nilai ppm dengan menggunakan rumus berikut:
`PPM = (ΔV/ΔV_reference)x 1000 ppm`

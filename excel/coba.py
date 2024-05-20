import pandas as pd
import numpy as np
from datetime import datetime, timedelta

# Baca file CSV data BMKG
bmkg_data = pd.read_csv(
    "D:/kuliah/kodingan/github/intelligenceControlSystem/excel/bmkg.csv",
    on_bad_lines="skip",
)

# Konversi kolom YEAR, MO, DY, dan HR ke string untuk membentuk timestamp
bmkg_data["YEAR"] = bmkg_data["YEAR"].astype(str)
bmkg_data["MO"] = bmkg_data["MO"].astype(str).str.zfill(2)  # Tambahkan leading zero
bmkg_data["DY"] = bmkg_data["DY"].astype(str).str.zfill(2)  # Tambahkan leading zero
bmkg_data["HR"] = bmkg_data["HR"].astype(str).str.zfill(2)  # Tambahkan leading zero

# Membuat kolom timestamp di BMKG data
bmkg_data["timestamp"] = pd.to_datetime(
    bmkg_data[["YEAR", "MO", "DY", "HR"]].agg("-".join, axis=1), format="%Y-%m-%d-%H"
)

# Buat DataFrame untuk timestamp dari 1 Januari 2024 sampai 5 Januari 2024 setiap menit
start_date = datetime(2024, 1, 1)
end_date = datetime(2024, 1, 6)
timestamps = pd.date_range(start=start_date, end=end_date, freq="T")[:-1]
df = pd.DataFrame(timestamps, columns=["timestamp"])

# Mengisi nilai default untuk kolom-kolom lainnya
df["ph"] = 7.0  # Nilai awal pH
df["tds"] = 0.0  # Nilai awal TDS
df["suhu_air"] = 0.0  # Nilai awal suhu_air
df["winddirection"] = 0.0
df["kecepatan_angin"] = 0.0
df["infrared1"] = 0.0
df["infrared2"] = 0.0
df["infrared3"] = 0.0
df["infrared4"] = 0.0
df["berat1"] = 6975.77
df["berat2"] = 7101.29
df["berat3"] = 7563.98
df["berat4"] = 7531.27
df["waterflow1"] = 0.0
df["waterflow2"] = 0.0
df["waterflow3"] = 0.0
df["waterflow4"] = 0.0
df["soilmoisture1"] = 0.0
df["soilmoisture2"] = 0.0
df["soilmoisture3"] = 0.0
df["soilmoisture4"] = 0.0
df["Suhu"] = 0.0
df["tekanan_udara"] = 0.0
df["pompanutrisi"] = 0
df["pompaair"] = 0
df["lampuuv"] = 0
df["svp"] = 0.0
df["avp"] = 0.0
df["vpd"] = 0.0
df["temperature_dht"] = 0.0

# Mengisi data pyrano, winddirection, kecepatan angin, suhu, dan humidity dari file BMKG
bmkg_data = (
    bmkg_data.set_index("timestamp")
    .reindex(df["timestamp"], method="ffill")
    .reset_index()
)
df = df.merge(bmkg_data, on="timestamp", how="left")

# Mengisi nilai pH, suhu_air, dan tds berdasarkan kondisi yang dijelaskan
for index, row in df.iterrows():
    current_time = row["timestamp"]
    hour = current_time.hour
    minute = current_time.minute

    if 8 <= hour < 12:
        df.at[index, "ph"] = 7.0 + (row["suhu_air"] - 25) * 0.1  # Contoh formula pH
        df.at[index, "suhu_air"] = row["Suhu"] - 2  # Contoh formula suhu_air
    else:
        df.at[index, "ph"] = 7.0
        df.at[index, "suhu_air"] = (
            row["Suhu"] - 5
        )  # Suhu air lebih rendah dari suhu ruang

    if 9 <= hour < 13:
        df.at[index, "tds"] = 300
    else:
        df.at[index, "tds"] = 200

# Mengisi kolom soil moisture, pompa nutrisi, dan pompa air berdasarkan kondisi
for index, row in df.iterrows():
    current_time = row["timestamp"]
    hour = current_time.hour
    minute = current_time.minute

    if (hour == 6 or hour == 10 or hour == 13 or hour == 16) and 0 <= minute < 15:
        df.at[index, "pompanutrisi"] = 1
        df.at[index, "waterflow1"] = 2.0
        df.at[index, "soilmoisture1"] += 0.1  # Contoh perubahan soil moisture

    if row["temperature_dht"] > 32:
        df.at[index, "pompaair"] = 1

# Tulis DataFrame ke file CSV
output_file = "greenhouse_data.csv"
df.to_csv(output_file, index=False)

print(f"File CSV '{output_file}' berhasil dibuat.")

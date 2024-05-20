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
# df["PS"] = np.nan  # Nilai awal tekanan
df["humidity"] = np.nan  # Nilai awal kelembapan
df["suhu"] = np.nan  # Nilai awal suhu
df["wind"] = np.nan  # Nilai awal wind direction

# Mengisi data PS, humidity, suhu, dan wind dari file BMKG
# Interpolasi linier untuk PS, humidity, dan suhu
df = df.merge(
    bmkg_data[["timestamp", "humidity", "suhu", "wind"]],
    on="timestamp",
    how="left",
)
# df["PS"] = df["PS"].interpolate()
df["humidity"] = df["humidity"].interpolate()
df["suhu"] = df["suhu"].interpolate()

# Mengisi nilai wind direction dengan nilai yang sama untuk setiap menit dalam satu jam
df["wind"] = df["wind"].ffill()

# Tulis DataFrame ke file Excel
output_file = "greenhouse_data_per_minute.xlsx"
df.to_excel(output_file, index=False)

print(f"File Excel '{output_file}' berhasil dibuat.")

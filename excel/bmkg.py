import pandas as pd
import numpy as np

# Membaca file CSV data BMKG
bmkg_data = pd.read_csv(
    "D:/kuliah/kodingan/github/intelligenceControlSystem/excel/bmkg.csv",
    on_bad_lines="skip",
)

wind_values = bmkg_data.loc[384:407, "wind"].tolist()
pyrano_values = bmkg_data.loc[384:407, "pyrano"].tolist()
suhu_values = bmkg_data.loc[384:407, "suhu"].tolist()
humidity_values = bmkg_data.loc[384:407, "humidity"].tolist()
pressure_values = bmkg_data.loc[384:407, "PS"].tolist()

# Buat DataFrame untuk data setiap jam
df_hourly = pd.DataFrame(
    {
        "timestamp": pd.date_range(start="2024-01-01 00:00:00", periods=24, freq="H"),
        "pyrano": pyrano_values,
        "wind": wind_values,
        "suhu": suhu_values,
        "humidity": humidity_values,
        "pressure": pressure_values,
    }
)

# Buat DataFrame untuk data setiap menit
df_minutely = pd.DataFrame(
    {
        "timestamp": pd.date_range(
            start="2024-01-01 00:00:00", end="2024-01-01 23:59:00", freq="T"
        )
    }
)

# Menggabungkan dan menginterpolasi data
df_minutely = df_minutely.merge(df_hourly, on="timestamp", how="left").interpolate(
    method="linear"
)

# Simpan data ke file CSV
output_file = "semua_data.csv"
df_minutely.to_csv(output_file, index=False)

print(f"File CSV '{output_file}' berhasil dibuat.")

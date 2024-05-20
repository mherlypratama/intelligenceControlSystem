import pandas as pd
import numpy as np
import xlsxwriter

# Data pyrano setiap jam dari jam 00 sampai jam 23
pyrano_hourly = [
    0,
    0,
    0,
    0,
    0,
    0,
    26.65,
    92.58,
    176.1,
    201.62,
    253.67,
    285.33,
    268.62,
    247.17,
    196.83,
    143.02,
    88.42,
    32.6,
    0,
    0,
    0,
    0,
    0,
    0,
]

# Buat DataFrame untuk data setiap jam
df_hourly = pd.DataFrame(
    {
        "timestamp": pd.date_range(start="2023-11-29 00:00:00", periods=24, freq="H"),
        "pyrano": pyrano_hourly,
    }
)

# Buat DataFrame untuk data setiap menit
df_minutely = pd.DataFrame(
    {
        "timestamp": pd.date_range(
            start="2023-11-29 00:00:00", end="2023-11-29 23:59:00", freq="T"
        )
    }
)

# Interpolasi data pyrano untuk mendapatkan nilai setiap menit
df_minutely = df_minutely.merge(df_hourly, on="timestamp", how="left").interpolate()

# Buat workbook dan worksheet
workbook = xlsxwriter.Workbook("pyrano_data.xlsx")
worksheet = workbook.add_worksheet()

# Tulis judul kolom
worksheet.write("A1", "timestamp")
worksheet.write("B1", "pyrano")

# Tulis data ke worksheet
for i, row in df_minutely.iterrows():
    worksheet.write(i + 1, 0, row["timestamp"].strftime("%Y-%m-%d %H:%M:%S"))
    worksheet.write(i + 1, 1, row["pyrano"])

# Tutup workbook
workbook.close()

# import xlsxwriter module
import xlsxwriter
import random

# Workbook() takes one, non-optional, argument
# which is the filename that we want to create.
workbook = xlsxwriter.Workbook("coba.xlsx")

# The workbook object is then used to add new
# worksheet via the add_worksheet() method.
worksheet = workbook.add_worksheet()

# Use the worksheet object to write
# data via the write() method.
detik = list(range(60))
menit = list(range(60))
jam = list(range(24))
tanggal = list(range(1, 32))

for b in jam:
    for c in menit:
        time_str = f"2023-11-29 {jam[b]}:{menit[c]}:{random.choice(detik)}"
        ph = 8 if b >= 8 and b < 12 else 7
        tds = 300 if b >= 9 and b < 13 else 200

        worksheet.write(f"A{b * 60 + c + 1}", time_str)
        worksheet.write(f"B{b * 60 + c + 1}", ph)
        worksheet.write(f"C{b * 60 + c + 1}", tds)

# Finally, close the Excel file
# via the close() method.
workbook.close()

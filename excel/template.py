import xlsxwriter
import random
from datetime import datetime

# Workbook() takes one, non-optional, argument
# which is the filename that we want to create.
workbook = xlsxwriter.Workbook("jan-1.xlsx")

# The workbook object is then used to add new
# worksheet via the add_worksheet() method.
worksheet = workbook.add_worksheet()

# Use the worksheet object to write
# data via the write() method.
tanggal = list(range(1, 32))

for b in tanggal:
    for c in range(24):
        for d in range(60):
            current_time = datetime(2023, 1, 1, c, d, random.choice(range(60)))
            timestamp = current_time.strftime("%Y-%m-%d %H:%M:%S")
            worksheet.write(f"A{(b - 1) * 1440 + c * 60 + d + 1}", timestamp)


# Finally, close the Excel file
# via the close() method.
workbook.close()

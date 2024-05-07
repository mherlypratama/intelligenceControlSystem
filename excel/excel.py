# import xlsxwriter module
import xlsxwriter
import random

# Workbook() takes one, non-optional, argument
# which is the filename that we want to create.
workbook = xlsxwriter.Workbook("Nov-30.xlsx")

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
        worksheet.write(
            f"A{b * 60 + c + 1}",
            f"2023-11-30 {jam[b]}:{menit[c]}:{random.choice(detik)}",
        )


# Finally, close the Excel file
# via the close() method.
workbook.close()

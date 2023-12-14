a = 0
soill = 65
while a <= 239:
    soill -= 0.04
    soill2 = soill + (soill * 0.012)
    soill3 = soill + (soill * 0.021)
    soill4 = soill + (soill * 0.03)
    a += 1

print("nilai a = ", a)
print("nilai soill = ", soill)
print("nilai soill2 = ", soill2)
print("nilai soill3 = ", soill3)
print("nilai soill4 = ", soill4)

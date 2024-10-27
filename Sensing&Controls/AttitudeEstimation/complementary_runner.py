import csv
import complementary
import numpy as np
import os

# Filter still needs to be tuned
filter = complementary.Complementary(0.5)
filter.initialize(0, np.array([0,0,1]))
attitudes = []

with open('monocopter_data.csv', mode ='r') as file:
    csvFile = csv.reader(file)
    firstLine = True
    for lines in csvFile:
        if (firstLine):
            firstLine = False
        else:
            update_arr = np.empty(10)
            for i in range(len(lines)):
                if (i < 1 or i > 3):
                    if (i > 3):
                        update_arr[i - 3] = float(lines[i])
                    else:
                        update_arr[i] = float(lines[i])
            attitude = filter.update(update_arr)
            attitudes.append(attitude.tolist())

with open('complementary_output.csv', 'w', newline='') as file:
    csvwriter = csv.writer(file, delimiter= ',')
    csvwriter.writerows(attitudes)
    
    file.seek(0, os.SEEK_END)
    file.seek(file.tell()-2, os.SEEK_SET)
    file.truncate()

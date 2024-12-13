import csv
import mahony
import numpy as np
import os

# Initialize Mahony filter with tunable gains
filter = mahony.Mahony(Kp=1.0, Ki=0.3)
filter.initialize(0, np.array([0,0,1]))
attitudes = []

with open('noisy_monocopter_data.csv', mode ='r') as file:
    csvFile = csv.reader(file)
    firstLine = True
    for lines in csvFile:
        if (firstLine):
            firstLine = False
        else:
            update_arr = np.empty(10)
            for i in range(len(lines)):
                update_arr[i] = float(lines[i])
            attitude = filter.update(update_arr)
            attitudes.append(attitude.tolist())

with open('mahony_output.csv', 'w', newline='') as file:
    csvwriter = csv.writer(file, delimiter=',')
    csvwriter.writerows(attitudes)
    
    file.seek(0, os.SEEK_END)
    file.seek(file.tell()-2, os.SEEK_SET)
    file.truncate()

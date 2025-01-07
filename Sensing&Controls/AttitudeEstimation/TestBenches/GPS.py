import csv
import numpy
import os

gps_noise_std_dev = 0.0000005
lidar_noise_std_dev = 0.2
# lat lng of middle of tech tower lawn
start_lat = 33.771831
start_lng = -84.394391

with open('position_data.csv', mode = 'r') as file:
    csvFile = csv.reader(file)
    eulers = [[],[],[]]
    for lines in csvFile:
        for i in range(1, len(lines)):
          eulers[i - 1].append(float(lines[i]))

for i in range(len(eulers[0])):
    gps_rand = numpy.random.normal(0, gps_noise_std_dev, size=(2))
    lidar_rand = numpy.random.normal(0, lidar_noise_std_dev, size=(1))
    eulers[0][i] = '%.6f'%(eulers[0][i] + gps_rand[0] + start_lat)
    eulers[1][i] = '%.6f'%(eulers[1][i] + gps_rand[1] + start_lng)
    eulers[2][i] = '%.8f'%(eulers[2][i] + lidar_rand[0])

noisy_gps_data = []
for i in range(len(eulers[0])):
    append = numpy.empty(2)
    for j in range(2):
        append[j] = eulers[j][i]    
    noisy_gps_data.append(append.tolist())

noisy_lidar_data = []
for i in range(len(eulers[2])):
    noisy_lidar_data.append({eulers[2][i]})
    
with open('noisy_gps_data.csv', 'w', newline='') as file:
    csvwriter = csv.writer(file, delimiter= ',')
    csvwriter.writerows(noisy_gps_data)
    
    file.seek(0, os.SEEK_END)
    file.seek(file.tell()-2, os.SEEK_SET)
    file.truncate()

with open('noisy_lidar_data.csv', 'w', newline='') as file:
    csvwriter = csv.writer(file, delimiter= ',')
    csvwriter.writerows(noisy_lidar_data)
    
    file.seek(0, os.SEEK_END)
    file.seek(file.tell()-2, os.SEEK_SET)
    file.truncate()
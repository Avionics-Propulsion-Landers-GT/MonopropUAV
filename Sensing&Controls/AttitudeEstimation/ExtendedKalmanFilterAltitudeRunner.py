import ExtendedKalmanFilterAltitude
import numpy as np
import csv
import os
import pandas as pd
import matplotlib.pyplot as plt

ekf = ExtendedKalmanFilterAltitude.ExtendedKalmanFilterAltitude(np.array([0]), np.zeros(1), 0.001, 0.2, 0.5 * 1000000, 100)
attitudes = []

with open('noisy_lidar_data.csv', mode ='r') as file:
    csvFile = csv.reader(file)
    firstLine = True
    timer = 0
    for lines in csvFile:
        if (firstLine):
            firstLine = False
        else:
            update_arr = np.empty(2)
            for i in range(len(lines)):
                update_arr[i+1] = float(lines[i])
            update_arr[0] = timer
            timer += 0.001

            ekf.predict()
            ekf.update(update_arr)
            attitude = ekf.state
            attitudes.append(attitude.tolist())

with open('extended_kalman_filter_altitude_output.csv', 'w', newline='') as file:
    csvwriter = csv.writer(file, delimiter= ',')
    csvwriter.writerows(attitudes)
    
    file.seek(0, os.SEEK_END)
    file.seek(file.tell()-2, os.SEEK_SET)
    file.truncate()


measurements = pd.read_csv('extended_kalman_filter_altitude_output.csv', header=None)

validation_df = pd.read_csv('position_data.csv')
validation_df = validation_df.iloc[: , 1:] # Remove the first column


# Plot x values
plt.figure()
plt.plot(measurements[measurements.columns[0]], label='Predicted x')
plt.plot(validation_df[validation_df.columns[0]], label='Actual x')
plt.xlabel('Time step')
plt.ylabel('x value')
plt.legend()
plt.title('Comparison of x values')
plt.show()
import numpy as np    
import pandas as pd
import matplotlib.pyplot as plt

def create_lidar_rest_data(rest_lidar = 0.5, lidar_noise_sd = 0.0045, num_samples = 1000):
    data = []
    delta_time = 0.001
    time = 0
    data.append([
        time,
        np.random.normal(rest_lidar, lidar_noise_sd)
    ])

    for _ in range(num_samples - 1):
        time += delta_time
        
        data.append([
            time,
            np.random.normal(rest_lidar, lidar_noise_sd)
        ])
    
    return data

def save_imu_data_to_file(data, filename):
    headers = ['time', 'lidar']
    with open(filename, 'w') as f:
        f.write(','.join(headers) + '\n')
    with open(filename, 'a') as f:
        for entry in data:
            # Flatten the nested lists for gyro, accel, mag into a single row
            row = [entry[0], entry[1]]
            f.write(','.join(map(str, row)) + '\n')

def graph_data(filename):

    data = pd.read_csv(filename)
    plt.figure(figsize=(12, 6))
    
    plt.subplot(1, 1, 1)
    plt.plot(data['time'], data['lidar'], label='Lidar')
    plt.title('Lidar Data')
    plt.xlabel('Time (s)')
    plt.ylabel('Lidar (m)')
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    lidar_data = create_lidar_rest_data()
    save_imu_data_to_file(lidar_data, 'lidar_at_rest.csv')
    print("Lidar data at rest saved to 'lidar_at_rest.csv'")
    graph_data('lidar_at_rest.csv')
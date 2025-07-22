import numpy as np    
import pandas as pd
import matplotlib.pyplot as plt

def create_imu_rest_data(rest_gyro = [0, 0, 0], rest_accel = [0, 0, -9.81], rest_mag = [1, 1, 0], gyro_noise_sd = 0.001, accel_noise_sd = 0.0001, mag_noise_sd = 0.0000005, gyro_drift = 0.01, gyro_drift_sd = 0.0001, num_samples = 10000):
    data = []
    delta_time = 0.001
    current_gyro_drift = np.random.choice([-1, 1], size=3).tolist()
    gyro_drift_direction = [1 if d >= 0 else -1 for d in current_gyro_drift]
    current_gyro_drift = [
        direction * np.random.normal(gyro_drift * delta_time, gyro_drift_sd * delta_time)
        for direction in gyro_drift_direction
    ]
    time = 0
    data.append([
        time,
        np.random.normal(rest_gyro, gyro_noise_sd).tolist(),
        np.random.normal(rest_accel, accel_noise_sd).tolist(),
        np.random.normal(rest_mag, mag_noise_sd).tolist()
    ])

    for _ in range(num_samples - 1):
        time += delta_time
        gyro_reading = np.random.normal(current_gyro_drift, gyro_noise_sd)
        accel_reading = np.random.normal(rest_accel, accel_noise_sd)
        mag_reading = np.random.normal(rest_mag, mag_noise_sd)
        
        data.append([
            time,
            gyro_reading.tolist(),
            accel_reading.tolist(),
            mag_reading.tolist()
        ])

        current_gyro_drift = [
            d + gyro_drift_direction[i] * np.random.normal(gyro_drift * delta_time, gyro_drift_sd * delta_time)
            for i, d in enumerate(current_gyro_drift)
        ]
    
    return data

def save_imu_data_to_file(data, filename):
    headers = ['time', 'gyro_x', 'gyro_y', 'gyro_z', 'accel_x', 'accel_y', 'accel_z', 'mag_x', 'mag_y', 'mag_z']
    with open(filename, 'w') as f:
        f.write(','.join(headers) + '\n')
    with open(filename, 'a') as f:
        for entry in data:
            # Flatten the nested lists for gyro, accel, mag into a single row
            row = [entry[0]] + entry[1] + entry[2] + entry[3]
            f.write(','.join(map(str, row)) + '\n')

def graph_data(filename):

    data = pd.read_csv(filename)
    plt.figure(figsize=(12, 6))
    
    plt.subplot(3, 1, 1)
    plt.plot(data['time'], data['gyro_x'], label='Gyro X')
    plt.plot(data['time'], data['gyro_y'], label='Gyro Y')
    plt.plot(data['time'], data['gyro_z'], label='Gyro Z')
    plt.title('Gyroscope Data')
    plt.xlabel('Time (s)')
    plt.ylabel('Gyro (rad/s)')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(data['time'], data['accel_x'], label='Accel X')
    plt.plot(data['time'], data['accel_y'], label='Accel Y')
    plt.plot(data['time'], data['accel_z'], label='Accel Z')
    plt.title('Accelerometer Data')
    plt.xlabel('Time (s)')
    plt.ylabel('Accel (m/s²)')
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(data['time'], data['mag_x'], label='Mag X')
    plt.plot(data['time'], data['mag_y'], label='Mag Y')
    plt.plot(data['time'], data['mag_z'], label='Mag Z')
    plt.title('Magnetometer Data')
    plt.xlabel('Time (s)')
    plt.ylabel('Mag (T)')
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    imu_data = create_imu_rest_data()
    save_imu_data_to_file(imu_data, 'imu_at_rest.csv')
    print("IMU data at rest saved to 'imu_at_rest.csv'")
    graph_data('imu_at_rest.csv')
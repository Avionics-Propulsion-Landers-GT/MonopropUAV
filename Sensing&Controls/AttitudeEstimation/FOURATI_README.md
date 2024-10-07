# Fourati Filter Attitude Estimation - README

## Overview

This project implements an attitude estimation algorithm based on the Fourati filter. The filter estimates the 3D orientation of a moving object using sensor data from an accelerometer, gyroscope, and magnetometer. The orientation is represented using quaternions, which allow for smooth 3D rotations.

### Key Features:
- Estimates **3D orientation** using quaternion-based calculations.
- Combines **accelerometer**, **gyroscope**, and **magnetometer** data for accurate orientation tracking.
- Applies filtering to reduce sensor noise and correct errors.
- Outputs a list of **quaternions** representing the object's orientation over time.

## How It Works

1. **Input Data**: The program processes a list of sensor data (`SensorData`), which contains:
   - 3-axis accelerometer readings
   - 3-axis gyroscope readings
   - 3-axis magnetometer readings
   - Timestamps for each data point

2. **Main Function**:
   - The `fouratiFilter` function takes the sensor data and computes the estimated orientation for each timestamp.
   - It integrates gyroscope data over time to update the orientation and applies corrections using the accelerometer and magnetometer to ensure accuracy.
   - The function returns a list of quaternions representing the estimated orientation at each time step.

3. **Helper Functions**:
   - Several helper functions are used within the `fouratiFilter` function to handle quaternion operations, sensor filtering, and error correction, ensuring the sensor data is properly processed.

## How to Use

1. **Prepare Sensor Data**: Create a list of `SensorData` structs containing accelerometer, gyroscope, and magnetometer readings, along with timestamps.
   
2. **Run the Filter**: 
   Call the `fouratiFilter` function, passing in the sensor data. The function will return the estimated orientation (quaternions) for each time step.

## Example

```
#include <iostream>
#include <vector>

int main() {
    vector<SensorData> sensorData = {
    {0.0, {0.0, 0.0, 9.81}, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}},
    {0.1, {0.1, 0.0, 9.7}, {0.0, 0.0, 0.1}, {1.0, 0.0, 0.0}},
    {0.2, {0.0, 0.1, 9.8}, {0.0, 0.1, 0.0}, {1.0, 0.0, 0.0}},
    {0.3, {-0.1, 0.0, 9.6}, {0.1, 0.0, 0.0}, {1.0, 0.0, 0.0}},
    {0.4, {0.0, -0.1, 9.8}, {0.0, 0.0, -0.1}, {0.5, 0.5, 0.0}},
    {0.5, {0.0, 0.0, 9.81}, {0.0, 0.0, 0.2}, {1.0, 0.0, 0.0}}
    };

    vector<vector<double>> quaternions = fouratiFilter(sensorData);

    for (const auto& q : quaternions) {
        cout << "Quaternion: [" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << "]" << endl;
    }

    return 0;
}

```

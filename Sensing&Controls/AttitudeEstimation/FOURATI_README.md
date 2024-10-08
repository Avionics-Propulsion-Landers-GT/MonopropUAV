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
   - Timestamps for each data point (in seconds)
   - 3-axis accelerometer readings (X, Y, Z)
   - 3-axis gyroscope readings (roll, pitch, yaw)
   - 3-axis magnetometer readings (X, Y, Z)


2. **Main Function**:

   The core function of the program is the `fouratiFilter`:
   - **Sensor Integration**: The gyroscope data is used to update the orientation by integrating angular velocity over time.
   - **Error Correction**: The accelerometer and magnetometer readings are used to correct drift in the orientation estimate and a weighted combination of errors.
   - **Filter Application**: Low-pass and high-pass filters are applied to smooth the input data.
   - **Quaternion Output**: The filter outputs quaternions representing the object's orientation over time.


4. **Helper Functions**:
   - **Quaternion Multiplication and Normalization**: Basic quaternion operations are used to combine and normalize rotations.
   -  **Error Calculation**: The program calculates the difference between expected and measured sensor values to apply corrections.
   -  **Filtering**: Low-pass and high-pass filters remove noise and emphasize relevant sensor data.


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

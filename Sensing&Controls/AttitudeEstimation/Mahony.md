# Mahony Filter Implementation Explanation

## Overview

The provided code implements the Mahony filter, an algorithm used for estimating orientation based on data from Inertial Measurement Units (IMUs). This filter fuses data from accelerometers, gyroscopes, and optionally magnetometers to provide a stable orientation estimate.

## Key Components

### 1. Main Function: `mahony_filter`

```python
def mahony_filter(data, frequency=100.0, Kp=1.0, Ki=0.3):
    # ...
```

This is the primary function that implements the Mahony filter. It takes the following parameters:

- `data`: A list of lists representing an n x 10 matrix containing sensor data.
- `frequency`: The sampling frequency in Hz (default: 100.0).
- `Kp`: The proportional gain for the filter (default: 1.0).
- `Ki`: The integral gain for the filter (default: 0.3).

The function returns a list of lists representing an n x 4 matrix containing estimated quaternions for each time step.

### 2. Helper Functions

Several helper functions are defined within `mahony_filter`:

#### a. `normalize(v)`

```python
def normalize(v):
    mag = sum(x*x for x in v) ** 0.5
    return [x/mag for x in v] if mag != 0 else v
```

This function normalizes a vector, ensuring it has a magnitude of 1. This is crucial for maintaining the integrity of direction vectors and quaternions.

#### b. `quaternion_multiply(a, b)`

```python
def quaternion_multiply(a, b):
    # ...
```

This function multiplies two quaternions, which is a fundamental operation in quaternion-based orientation calculations.

#### c. `quaternion_conjugate(q)`

```python
def quaternion_conjugate(q):
    return [q[0], -q[1], -q[2], -q[3]]
```

This function computes the conjugate of a quaternion, which is used in rotating vectors by quaternions.

#### d. `rotate_vector(v, q)`

```python
def rotate_vector(v, q):
    # ...
```

This function rotates a vector by a quaternion, which is used to transform vectors between different reference frames.

### 3. Main Algorithm Loop

The core of the Mahony filter is implemented in the main loop of the `mahony_filter` function. For each set of sensor measurements, it performs the following steps:

1. Normalize accelerometer and magnetometer measurements.
2. Estimate the direction of gravity and magnetic field in the body frame.
3. Compute the error between estimated and measured directions.
4. Apply PI (Proportional-Integral) feedback to correct gyroscope measurements and update bias estimate.
5. Compute the rate of change of the quaternion based on corrected gyroscope data.
6. Integrate to get the new quaternion estimate.
7. Normalize the quaternion to ensure it represents a valid rotation.

## Important Concepts

### Quaternions

Quaternions are a mathematical notation used to represent orientations and rotations in 3D space. They consist of four components: one real part and three imaginary parts. In the context of this implementation, quaternions are represented as lists of four float values.

Quaternions offer several advantages over other representations like Euler angles:
- They avoid the problem of "gimbal lock".
- They provide a more compact representation than rotation matrices.
- Quaternion operations are generally more computationally efficient.

### Sensor Fusion

The Mahony filter performs sensor fusion, combining data from multiple sensors to produce a more accurate and stable orientation estimate. In this implementation, it fuses data from:
- Accelerometers: Measure linear acceleration, including gravity.
- Gyroscopes: Measure angular velocity.
- Magnetometers: Measure the direction of the Earth's magnetic field.

### PI Controller

The filter uses a Proportional-Integral (PI) controller to correct errors in the orientation estimate:
- The proportional term (`Kp`) provides immediate correction.
- The integral term (`Ki`) helps to eliminate steady-state errors.

These terms can be adjusted to optimize the filter's performance for different sensors and applications.

### Gyroscope Bias Estimation

The filter estimates and corrects for gyroscope bias, which is a common source of error in inertial measurement units. This is done by integrating the error term and using it to adjust the gyroscope readings.

## Test Function

The code includes a test function `test_mahony_filter()` that demonstrates how to use the Mahony filter:

1. It generates sample data simulating 1 second of sensor readings at 100Hz, with the device starting at rest and then rotating around the z-axis.
2. It runs the Mahony filter on this sample data.
3. It prints the initial and final quaternions to show the estimated change in orientation.

## Usage

To use this implementation:

1. Prepare your sensor data in the required format: each row should contain [timestamp, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z].
2. Call the `mahony_filter` function with your data and desired parameters.
3. The function will return a list of quaternions representing the estimated orientation at each time step.

Example:
```python
data = [
    [0, 0, 0, 9.81, 0, 0, 0, 0.2, 0, 0.4],
    [0.01, 0, 0, 9.81, 0.1, 0, 0, 0.2, 0, 0.4],
    # ... more data rows ...
]
result = mahony_filter(data)
for q in result:
    print(f"Estimated orientation (quaternion): {q}")
```

## Conclusion

The Mahony filter is a powerful tool for orientation estimation in various applications, including robotics, drones, and virtual reality systems. This implementation provides a flexible and efficient way to apply the Mahony filter to sensor data, with options to adjust key parameters for optimal performance in different scenarios.

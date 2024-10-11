# Madgwick Filter Implementation Explanation
## Filter Overview

The provided code implements the Madgwick filter, an algorithm that calculates the object orientation accurately from data from accelerometer, gyroscope, 
and magnetometer (Optional).
The objective of Madgwick filter is to minimize the error created by raw gyroscope data, which always include high frequency of noise.
Therefore, accelerometer and magnetometer used to compute the gyroscope measurement error and correct it via gradient-descent algorithm.

## Main Components
### 1. Main Function: `madgwick_filter`

```python
def madgwick_update(q, gyro, accel, beta, dt):
    # ...
```

This is the primary function that implements the Madgwick filter. It takes the following parameters:

- `q`: Current orientation quaternion
- `gyro`: Gyroscope readings
- `accel`: Accelerometer readings
- `beta`: Filter gain for feedback correction 
- `dt`: Time step for integration

The function returns an updated orientation quanternion for the current time step.

### 2. Helper Functions

Several helper functions are utilized.

#### a. `normalize(vector)`
```python
def normalize(vector):
    return vector / np.linalg.norm(vector)
```

This function normalizes a vector, setting the magnitude to 1. Important for maintaining overall direction of vectors inputed. 

#### b. `quaternion_multiply(q1,q2)`
```python
def quaternion_multiply(q1,q2):
    # ...
```

This function multiplies two quaternions.

#### c. `integrate_gyro(q,gyro,dt)`
```python
def integrate_gyro(q, gyro, dt):
    # ...
```

This function approximates the quaternion derivative and adjusts the orientation quaternion accordingly.

### 3. Algorithm Loop

The main loop of the Madgwick Filter runs without a function being called. For each set of sensor measurement in a time step, it performs the following:

1) Read sensor data to obtain sensor measurements.                                       
2) Compute oritentation increment from accelerometer measurements (gradient step).
3) Compute orientation from gyroscope measurements (numeriacal integration).                                     
4) Fuse the measurements from both the accelerometer and gyroscope to obtain estimated altitude(roll, pitch, yaw).
    - Gyroscope drift is corrected using accelerometer data to maintain accurate orientation estimates. 

## To Use

To use this program:

1) Prepare accelerometer and gyroscope to have sensor data read.
2) Run script
3) The program will print estimated orientations at each time step. 

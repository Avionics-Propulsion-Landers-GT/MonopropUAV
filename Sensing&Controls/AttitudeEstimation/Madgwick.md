# Madgwick Filter Implementation Explanation
## Filter Overview

The provided code implements the Madgwick filter, an algorithm that calculates the object orientation accurately from data from accelerometer, gyroscope, and magnetometer.
The objective of Madgwick filter is to minimize the error created by raw gyroscope data, which may include a high frequency of noise.
Therefore, accelerometer and magnetometer are used to compute the gyroscope measurement error and correct it via gradient-descent algorithm.

## Functions
### `__init__(self, gain, beta)`
```python
    self.gain = gain
    self.beta = beta
    self.last_update_time = -1
    self.attitude_estimation = np.zeros(3)
    self.q = np.array([1.0, 0.0, 0.0, 0.0])
```
This defines the constructor of the `Madgwick` class with a `gain` and `beta` input. The `gain` determines the magnitude of gyroscope compensation and impacts responsiveness to the gyroscope's readings. The `beta` determines how much we trust is given to the accelerometer and magnetometer estimation versus the gyroscope estimation. The `last_update_time`, `attitude_estimation`, and `q`(quaternion) variables are set to appropriate default values.

### `initialize(self, time initial_attitude_estimation)`
```python
def initialize(self, time, initial_attitude_estimation):
        self.last_update_time = time
        if(np.linalg.norm(initial_attitude_estimation) != 0):
            self.attitude_estimation = initial_attitude_estimation / np.linalg.norm(initial_attitude_estimation)
```
This initializes the `last_update_time` and `attitude_estimation` variables to actual values that will be used before the `update()` function is run. `time` needs to be the absolute time in seconds, and `initial_attitude_estimation` needs to be an Euler angle.

### `update(self, update_arr)`
```python
def update(self, update_arr):
    # ...
```
This runs an update of the madgwick filter and returns an attitude estimation is returned in Euler angles. The filter requires the `initialize()` method to run beforehand, so `time` and `attitude_estimation` are properly defined. This function runs the `madgwick_update()` function, performs quaternion to euler conversions, and returns the attitude estimation for the run.


### `madgwick_update(self, q, gyro, accel, mag, dt)`

```python
def madgwick_update(self, q, gyro, accel, mag, dt):
    # ...
```

This is the primary function that implements the Madgwick filter. It takes the following parameters:

- `q`: Current orientation quaternion
- `gyro`: Gyroscope readings
- `accel`: Accelerometer readings
- `mag`: Magnetometer readings
- `dt`: Time step for integration

First, we normalize the read accelerometer and magnetometer vectors and compute the gradient descent step utilizing `compute_gradient()`. We then integrate the gyroscope to help minimize gyroscope compensation drift using the `gain` value defined for the class. We compute the average rate of change between our previous quaternion and our gyroscope using the `beta` value defined for the class. This change is used to adjust the current quaternion to yield the attitude calculation. The function returns an updated, normalized orientation quaternion for the current time step.

### `compute_gradient(self, q, accel, mag)`
```python
def compute_gradient(self, q, accel, mag):
    # ...
```
This function computes the gradient descent step utilizing the magnetometer and accelerometer as well as predefined Objective and Jacobian functions. `h` and `b` represent the Earth's magnetic field and are used in the calculation of the predefined functions. The two matrices are multiplied to compute and return the gradient descent. You can read more on the Objective and Jacobian functions and the compute_gradient function here: https://medium.com/@k66115704/imu-madgwick-filter-explanation-556fbe7f02e3.

### 2. Helper Functions

Several helper functions are utilized.

#### a. `normalize(vector)`
```python
def normalize(vector):
    return vector / np.linalg.norm(vector)
```

This function normalizes a vector, setting the magnitude to 1. Important for maintaining the overall direction of vectors input. 

#### b. `quaternion_multiply(q1,q2)`
```python
def quaternion_multiply(q1,q2):
    # ...
```

This function multiplies two quaternions.

## To Use

To use this program: Create an instance of the `Madgwick` class with appropriate gain and beta parameters. Call `initialize()` with the system start time and attitude estimation. Then, call `update()` with the input array, which should be a 1 x 10 vector with the time stamp first, then the 3 accelerometer axes (x, y, then z), the 3 gyroscope axes, and finally the 3 magnetometer axes. The estimation is returned both from `update()` itself and the `get_estimate()` method.

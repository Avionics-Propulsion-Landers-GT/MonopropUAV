# Complementary Filter Attitude Estimation README


## Overview

The complementary filter works by taking an average of two estimates from the IMU's sensors. The estimate from the gyroscope simply integrates the current gyroscope readings and adds them onto the previous attitude estimation. The accelerometer and magnetometer estimation first takes the accelerometer data and calculates the x and y tilt angles. Then, these tilt angles are used to adjust the magnetometer data, which is used for a yaw (z) angle estimation. All that put together gives an attitude estimation. Then, using a tuned gain, the filter takes an average of the two attitude estimation and puts them together to make one final estimation. This works in Euler angles.

## Functions

#### `__init__(self, gain)`
``` python
def __init__(self, gain):
        self.gain = gain
        self.last_update_time = -1
        self.attitude_estimation = np.zeros(3)
```
This creates a new instance of the `Complementary` class with a gain input. The gain determines how much we trust the accelerometer and magnetometer estimation or the gyroscope estimation. The `last_update_time` and `attitude_estimation` variables are set to some default value.

#### `initialize(self, time, initial_attitude_estimation)`
``` python
def initialize(self, time, initial_attitude_estimation):
        self.last_update_time = time
        self.attitude_estimation = initial_attitude_estimation
```
This initializes the `last_update_time` and `attitude_estimation` variables to actual values that we will use. This needs to be run before the `update()` function. `time` needs to be absolute time in seconds, and `initial_attitude_estimation` needs to be an Euler angle.

#### `update(self, update_arr)`
``` python
def update(self, update_arr):
        # ...
```
This runs an update of the complementary filter. The filter requires the `initialize()` method to be run beforehand, or else bad things happen. First, the tilt angles are calculated from the accelerometer data. These can be found by taking the arctan of the different parts of the accelerometer data. Then, the tilt angles are use to correct the magnetometer data. Then, we can use arctan on the corrected magnetometer data to find the yaw (z) angle. The gyroscope estimation is found by simply integrating the current gyroscope data and adding it to the previous attitude estimation. Then, a weighted average using the tuned gain is used to find the final attitude estimation, which is then normalized.

#### `get_estimate(self)`
``` python
    def get_estimate(self):
        return self.attitude_estimation
```
This returns the current ttitude estimation.

## Use

Call `initialize()` with the system start time and attitude estimation. Then, call `update()` with the input array, which should be a 1 x 10 vector with the time stamp first, then the 3 accelerometer axes (x, y, then z), the 3 gyroscope axes, and finally the 3 magnetometer axes. The estimation is returned both from `update()` itself and the `get_estimate()` method.
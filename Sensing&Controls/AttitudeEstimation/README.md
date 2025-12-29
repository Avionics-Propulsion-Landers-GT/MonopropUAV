# Attitude Estimation Folder
This folder contains many previous files on various methods of attitude estimation from sensor fusion of data from an IMU. This will merely cover the purpose of each of the program files contained in the `Sensing&Controls/AttitudeEstimation/` folder itself. Explanation of the `ekf` and `rust-ekf` folders within the main directory will be left to their resctive README files, while the `TestBenches` folder should be considered deprecated and unnecessary for understanding the larger project within this folder.

## Folder Structure

```
Sensing&Controls/AttitudeEstimation/
├── complementary_runner.py                 # Handles instantiating and running the complementary filter found in complementary.py
├── Complementary.md                        # Contains documentation regarding the use of the Python complementary filter
├── complementary.py                        # Contains an implmentation of a complementary filter in Python
├── ExtendedKalmanFilter.py                 # Contains an implmeentation of an Extended Kalman Filter (EKF) for attitude in Python
├── ExtendedKalmanFilterAltitude.py         # Contains an EKF designed for altitude
├── ExtendedKalmanFilterAltitudeRunner.py   # Handles instantiating and running the altitude EKF in ExtendedKalmanFilterAltitude.py
├── ExtendedKalmanFilterGeneral.cpp         # Contains the boilerplate class and methods for EKF implementations in C++
├── ExtendedKalmanFilterGeneral.h           # Contains the class definition and other header information for the C++ EKF boilerplate
├── ExtendedKalmanFilterRunner.py           # Handles instantiating and running the attitude EKF in ExtendedKalmanFilter.py
├── ExtendedKalmanFilterXY.py               # Contains an EKF designed for horizontal positioning
├── ExtendedKalmanFilterXYRunner.py         # Handles instantiating and running the XY position EKF in ExtendedKalmanFilterXY.py
├── FOURATI_README.md                       # Contains documentation regarding the use of the C++ Fourati's algorithm filter
├── Fouratis.cpp                            # Contains an implementation of the Fourati's algorithm filter for attitude, as well as supporting classes and methods and the running of the algorithm.
├── Madgwick.cpp                            # Contains an implementation of the Madgwick filter in C++
├── Madgwick.md                             # Contains documentation regarding the use of the Python Madgwick filter
├── madgwick.py                             # Contains an implementation of the Madgwick filter in Python
├── mahony_runner.py                        # Handles instantiating and running the Mahony filter in mahony.py
├── Mahony.md                               # Contains documentation regarding the use of the Python Mahony filter
├── mahony.py                               # Contains an implementation of the Mahony filter in Python
├── mathhelper.cpp                          # Contains methods handling Euler angle, quaternion, and rotation matrix conversions in C++.
├── mathhelper.h                            # Contains the class definition and header informaation of mathhelper.cpp
├── Matrix.cpp                              # An implementation of a matrix in C++ containing basic useful operations
├── Matrix.h                                # Contains class and method definitions and header information for Matrix.cpp
├── MatrixTester.cpp                        # Runs several tests to determine the validity of the Matrix.cpp imeplementation
├── Quaternion.cpp                          # An implementation of a quaternion in C++ containing basic useful operations
├── Quaternion.h                            # Contains class and method definitions and header information for Quaternion.cpp
├── Vector.cpp                              # An implementation of a vector in C++ containing basic useful operations
└── Vector.h                                # Contains class and method definitions and header information for Vector.cpp
```

## Project Descriptions

### Complementary Filter

This is a basic filter that works by returning a weighted average of the inputs. For attitude estimation, that means creating two attitude estimates based off of the integration of the gyroscope and the accelerometer vector and then finding a weighted average of the two resuts. This is the lightest weight algorithm in terms of computation, but it also does poorly in compensating for gyroscope drift and sensor noise.

### Extended Kalman Filter (EKF)

This is a more advanced filter that operates off of a mathematical model of the sensor. The filter must first be given estimates of the covariances of the sensor and error, as well as state and measurement transition functions that inform the model of how it can expect the state and sensor information to behave. This tells the filter what it can expect to happen to the data, as well as how accurate it can expect its predictions to be. From there, the filter runs a prediction step and generates an estimate of what it thinks the next state will be, and therefore what the sensors should be telling it. From there, the actual sensor data is given to the filter and corrections are made to its estimations. This repeats, converging the filter to the ground truth. This algorithm is more computationally expensive, but it gains much more in accuracy and gyroscope drift mitigation. Specifically, an Extended Kalman Filter differs from a typical Kalman Filter by using a Jacobian to update estimates to allow for estimates of nonlinear systems, increasing accuracy in dynamic conditions like ours.

### Fourati's Algorithm

Fourati's algorithm works by first applying low/high pass filters to reduce the input sensor noise. Then, an estimation of the attitude is produced by quaternion integration. That estimation is then corrected against the accelerometer and magentmeter estimations by a gain factor through some quaternion and matrix multiplication. This is also relatively computationally expensive, like the EKF, but it works differently to acheive the same goal. It has a decent accuracy.


### Madgwick Filter

The Madgwick filter operates somewhat similarly to Fourati's algorithm. It aims to minimize the error from the gyroscope estimation of the attitude. It achieves this by calculating an error estimation from the accelerometer and magnetometer data, and then correcting this via a gradient-descent approach. This is also relatively computationally expensive due to the gradient descent, but it has good accuracy and can compensate for gyroscope drift for the most part.

### Mahony FIlter

The Mahony filter operates by using a Proportional-Integral (PI) controller to minimize the estimated attitude error. First, the error of the accelerometer and magnetometer estimates is computed. This error is then applied to a PI controller to obtain a gain to correct the gyroscope measurements and update the bias estimate. The angular velocity is then computed based on the corrected gyroscope data and that is integrated to form a final quaternion estimate, which is normalized. This is less computationally expensive than the Madgwick filter since a PI controller is a very lightweight algorithm compared to gradient descent. However, its accuracy suffers a bit more compared to the Madgwick filter as a result.

### Supporting C++ Classes

These classes contain implementations of a Matrix, Qauternion, and Vector in C++. They are used in some of the other C++ implementations to make the handling of math operations both easier to read as well as program. They contain basic methods like addition, multiplication, inverses, raising to exponents, conversions, and more. They do not contain key information to the sensor fusion aspect of this folder, but can be useful to refer to.
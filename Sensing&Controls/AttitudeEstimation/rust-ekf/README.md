# Rust Extended Kalman Filter (EKF) Library

A Rust implementation of the Extended Kalman Filter. This library provides a generic EKF implementation along with specific models for attitude, altitude, and position estimation.

## Features

- **Generic EKF Implementation**: A flexible, trait-based EKF that can be extended for various state estimation problems. This is the core of the EKF implementation handling prediction and update logic.
- **Pre-built Models**: Includes models for common aerospace estimation tasks:
  - **Altitude Estimation**: For vertical position and velocity estimation
  - **Attitude Estimation**: For orientation estimation
  - **XY Position**: For 2D position tracking

## Project Structure

```
src/
├── lib.rs              # Library root and public API
├── ekf/
│   ├── mod.rs         # EKF module exports
│   ├── filter.rs      # Core EKF implementation
│   └── model.rs       # EKFModel trait definition
└── models/            # Pre-built EKF models
    ├── mod.rs         # Model exports and type aliases
    ├── altitude.rs    # Altitude estimation model
    ├── attitude.rs    # Attitude estimation model
    └── xy_position.rs # 2D position tracking model
```

## Core Components

### 1. ExtendedKalmanFilter

The main EKF implementation that works with any model implementing the `EKFModel` trait. Key methods:

- `new()`: Create a new EKF instance
- `predict()`: Predict the next state using the process model
- `update()`: Update the state estimate with new measurements
- `get_state()`: Get the current state estimate
- `get_covariance()`: Get the current state covariance

### 2. EKFModel Trait

Implement this trait to create custom EKF models. Required methods:

- `parse_data()`: Convert raw sensor data to a measurement vector
- `state_transition_function()`: Define the process model
- `state_transition_jacobian()`: Compute the process model Jacobian
- `measurement_prediction_function()`: Define the measurement model
- `measurement_prediction_jacobian()`: Compute the measurement model Jacobian

## Pre-built Models

### 1. AltitudeModel

Estimates vertical position and velocity using altitude measurements from sensors like barometers or GPS.

**State Vector**: `[altitude]`

**Input Data Format**: `[timestamp, lidar_measurement] `

**Example Usage**:
```rust
use rust_ekf::{AltitudeModel, AltitudeEKF};
use ndarray::array;

// Initialize with 10ms timestep
let model = AltitudeModel::new(0.01);
let initial_state = array![0.0];      // Starting altitude (meters)
let initial_measurement = array![0.0];

// Create EKF instance
let mut ekf = AltitudeEKF::new(
    initial_state,
    initial_measurement,
    0.01,  // dt (seconds)
    0.1,   // Process noise (Q)
    0.01,  // Measurement noise (R)
    1.0,   // Initial state covariance
    model,
);

// In your main loop:
loop {
    // 1. Get sensor data (time in seconds, altitude in meters)
    let sensor_data = [current_time, altitude_measurement];
    
    // 2. Prediction step
    ekf.predict();
    
    // 3. Update with new measurement
    ekf.update(&sensor_data);
    
    // 4. Get current state estimate
    let state = ekf.get_state();
    let altitude = state[0];
}
```

### 2. AttitudeModel

Estimates 3D orientation (roll, pitch, yaw) using IMU measurements from gyroscope, accelerometer, and magnetometer.

**State Vector**: `[roll, pitch, yaw]` (radians)

**Input Data Format**: `[timestamp, gx, gy, gz, ax, ay, az, mx, my, mz]`
- `gx,gy,gz`: Gyroscope measurements (rad/s)
- `ax,ay,az`: Accelerometer measurements (m/s²)
- `mx,my,mz`: Magnetometer measurements (normalized)

**Example Usage**:
```rust
use rust_ekf::{AttitudeModel, AttitudeEKF};
use ndarray::array;
use std::f64::consts::PI;

// Initialize with 10ms timestep
let model = AttitudeModel::new(0.01);
let initial_state = array![0.0, 0.0, 0.0];  // Initial orientation (roll, pitch, yaw)
let initial_measurement = array![0.0; 9];   // Initial sensor readings

let mut ekf = AttitudeEKF::new(
    initial_state,
    initial_measurement,
    0.01,  // dt
    0.1,   // Process noise
    0.01,  // Measurement noise
    1.0,   // Initial covariance
    model,
);

// In your main loop:
loop {
    // Get IMU data (time, gyro, accel, mag)
    let imu_data = [
        current_time,
        gx, gy, gz,   // Gyroscope (rad/s)
        ax, ay, az,   // Accelerometer (m/s²)
        mx, my, mz    // Magnetometer (normalized)
    ];
    
    ekf.predict();
    ekf.update(&imu_data);
    
    let state = ekf.get_state();
    let (roll, pitch, yaw) = (state[0], state[1], state[2]);
}
```

### 3. XYPositionModel

Tracks 2D position and velocity in the horizontal plane using GPS or other positioning data.

**State Vector**: `[x, y, vx, vy]`
- `x, y`: Position in meters (relative to initial position)
- `vx, vy`: Velocity in m/s

**Input Data Format**: `[longitude, latitude]` (degrees)

**Features**:
- Automatically handles conversion from lat/lon to local tangent plane coordinates
- Maintains velocity estimates for position prediction
- Uses a constant velocity motion model

**Example Usage**:
```rust
use rust_ekf::{XYPositionModel, XYPositionEKF};
use ndarray::array;

// Initialize with 1Hz update rate
let model = XYPositionModel::new(1.0);
let initial_state = array![0.0, 0.0, 0.0, 0.0];  // x, y, vx, vy
let initial_measurement = array![0.0, 0.0];      // Initial position

let mut ekf = XYPositionEKF::new(
    initial_state,
    initial_measurement,
    1.0,    // dt (1 second between updates)
    0.1,    // Process noise
    1.0,    // Measurement noise (higher for GPS)
    10.0,   // Initial covariance
    model,
);

// In your main loop:
loop {
    // Get position data (longitude, latitude in degrees)
    let position_data = [longitude, latitude];
    
    ekf.predict();
    ekf.update(&position_data);
    
    let state = ekf.get_state();
    let (x, y, vx, vy) = (state[0], state[1], state[2], state[3]);
}
```

## Model Tuning Tips

1. **Process Noise (Q)**:
   - Higher values make the filter more responsive to measurements
   - Lower values make the filter trust the model more
   - Typical range: 0.01 to 1.0

2. **Measurement Noise (R)**:
   - Should match your sensor's expected error characteristics
   - Lower values indicate more trust in the measurements
   - Typical range: 0.001 to 1.0

3. **Initial Covariance (P)**:
   - Represents uncertainty in initial state
   - Larger values allow faster initial convergence
   - Typical range: 0.1 to 10.0

## Adding a Custom Model

1. Create a new struct implementing the `EKFModel` trait
2. Define your state transition and measurement models
3. Create a type alias for convenience

Example:
```rust
use ndarray::{Array1, Array2};
use rust_ekf::ekf::model::EKFModel;

struct MyCustomModel {
    // Model parameters
}

impl EKFModel for MyCustomModel {
    // Implement required trait methods
}

// Type alias for convenience
pub type MyCustomEKF = crate::ekf::filter::ExtendedKalmanFilter<MyCustomModel>;
```
## References

- [Wikipedia: Extended Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter)
- [Math Behind EKF](https://www.alanzucconi.com/2022/07/24/extended-kalman-filter/)
- [EKF for Attitude Estimation](https://ahrs.readthedocs.io/en/latest/filters/ekf.html)

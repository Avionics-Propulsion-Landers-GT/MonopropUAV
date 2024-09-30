# Mahony Filter Implementation

# The Mahony filter is an algorithm used for estimating orientation based on data from 
# Inertial Measurement Units (IMUs). It fuses data from accelerometers, gyroscopes, and 
# optionally magnetometers to provide a stable orientation estimate.

# This implementation takes input data in the form of a n x 10 matrix, where each row contains:
# [timestamp, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z]

def mahony_filter(data, frequency=100.0, Kp=1.0, Ki=0.3):
    """
    Implements the Mahony filter for orientation estimation.

    Parameters:
    data (list of lists): n x 10 matrix containing sensor data
    frequency (float): Sampling frequency in Hz (default: 100.0)
    Kp (float): Proportional gain (default: 1.0)
    Ki (float): Integral gain (default: 0.3)

    Returns:
    list of lists: n x 4 matrix containing estimated quaternions for each time step
    """

    def normalize(v):
        """Normalize a vector."""
        mag = sum(x*x for x in v) ** 0.5
        return [x/mag for x in v] if mag != 0 else v

    def quaternion_multiply(a, b):
        """Multiply two quaternions."""
        return [
            a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
            a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
            a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
            a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
        ]

    def quaternion_conjugate(q):
        """Compute the conjugate of a quaternion."""
        return [q[0], -q[1], -q[2], -q[3]]

    def rotate_vector(v, q):
        """Rotate a vector by a quaternion."""
        p = [0] + v
        rotated = quaternion_multiply(quaternion_multiply(q, p), quaternion_conjugate(q))
        return rotated[1:]

    dt = 1.0 / frequency
    q = [1, 0, 0, 0]  # Initial quaternion
    gyro_bias = [0, 0, 0]  # Initial gyroscope bias estimate
    integral_error = [0, 0, 0]  # Integral of the error for PI control

    quaternions = []

    for row in data:
        timestamp, ax, ay, az, gx, gy, gz, mx, my, mz = row

        # Normalize accelerometer and magnetometer measurements
        acc = normalize([ax, ay, az])
        mag = normalize([mx, my, mz])

        # Estimated direction of gravity and magnetic field
        v = rotate_vector([0, 0, 1], q)  # Gravity in body frame
        b = rotate_vector(mag, q)  # Magnetic field in body frame

        # Error is sum of cross product between estimated and measured direction of gravity
        error = [
            acc[1]*v[2] - acc[2]*v[1] + mag[1]*b[2] - mag[2]*b[1],
            acc[2]*v[0] - acc[0]*v[2] + mag[2]*b[0] - mag[0]*b[2],
            acc[0]*v[1] - acc[1]*v[0] + mag[0]*b[1] - mag[1]*b[0]
        ]

        # Apply PI feedback terms
        for i in range(3):
            integral_error[i] += error[i] * Ki * dt
            gyro_bias[i] += integral_error[i]
            gx, gy, gz = gx - gyro_bias[0], gy - gyro_bias[1], gz - gyro_bias[2]

        # Compute rate of change of quaternion
        qDot = [
            0.5 * (-q[1]*gx - q[2]*gy - q[3]*gz),
            0.5 * (q[0]*gx + q[2]*gz - q[3]*gy),
            0.5 * (q[0]*gy - q[1]*gz + q[3]*gx),
            0.5 * (q[0]*gz + q[1]*gy - q[2]*gx)
        ]

        # Integrate to yield quaternion
        q = [q[i] + qDot[i]*dt for i in range(4)]
        q = normalize(q)

        quaternions.append(q)

    return quaternions

# Example usage:
# data = [
#     [0, 0, 0, 9.81, 0, 0, 0, 0.2, 0, 0.4],
#     [0.01, 0, 0, 9.81, 0.1, 0, 0, 0.2, 0, 0.4],
#     # ... more data rows ...
# ]
# result = mahony_filter(data)
# for q in result:
#     print(f"Estimated orientation (quaternion): {q}")

# Explanation of the Mahony filter:
# The Mahony filter is an algorithm used to estimate the orientation of an object in 3D space
# using data from inertial and magnetic sensors. It's particularly useful in situations where
# you need to track the orientation of a device, such as in drones, robots, or virtual reality systems.

# Key components of the Mahony filter:
# 1. Sensor Fusion: It combines data from accelerometers (which measure linear acceleration),
#    gyroscopes (which measure angular velocity), and optionally magnetometers (which measure
#    magnetic field direction) to provide a more accurate and stable orientation estimate.

# 2. Quaternion Representation: The filter uses quaternions to represent orientation. Quaternions
#    are a mathematical notation that can represent rotations in 3D space without the problem of
#    "gimbal lock" that can occur with Euler angles.

# 3. Proportional-Integral (PI) Controller: The filter uses a PI controller to correct errors
#    in the orientation estimate. The proportional term (Kp) provides immediate correction,
#    while the integral term (Ki) helps to eliminate steady-state errors.

# 4. Gyroscope Bias Estimation: The filter estimates and corrects for gyroscope bias, which
#    is a common source of error in inertial measurement units.

# How the algorithm works:
# 1. Initialize quaternion and gyroscope bias estimates.
# 2. For each set of sensor measurements:
#    a. Normalize accelerometer and magnetometer data.
#    b. Estimate the direction of gravity and magnetic field in the body frame.
#    c. Compute the error between estimated and measured directions.
#    d. Apply PI feedback to correct gyroscope measurements and update bias estimate.
#    e. Compute the rate of change of the quaternion based on corrected gyroscope data.
#    f. Integrate to get the new quaternion estimate.
#    g. Normalize the quaternion to ensure it represents a valid rotation.

# This implementation is scalable as it processes each row of input data independently,
# allowing it to handle datasets of any size. The filter parameters (frequency, Kp, Ki)
# can be adjusted to optimize performance for different sensors and applications.

# Test function and some sample data

def test_mahony_filter():
    # Generate some sample data
    # This data simulates 1 second of sensor readings at 100Hz
    # with the device starting at rest and then rotating around the z-axis
    sample_data = []
    for i in range(100):
        t = i / 100.0
        ax, ay, az = 0, 0, -9.81  # Constant gravity
        gx, gy, gz = 0, 0, 1.0  # Constant rotation around z-axis
        mx, my, mz = 0.2, 0.4, 0.1  # Constant magnetic field
        sample_data.append([t, ax, ay, az, gx, gy, gz, mx, my, mz])

    # Run the Mahony filter on the sample data
    result = mahony_filter(sample_data)

    # Print the first and last quaternions
    print("Initial quaternion:", result[0])
    print("Final quaternion:", result[-1])

    # In a perfect scenario, we'd expect to see rotation around the z-axis
    # This would be represented by changes in the x and y components of the quaternion

# Run the test
if __name__ == "__main__":
    test_mahony_filter()
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import pandas as pd


def quat_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def quat_normalize(q):
    return q / np.linalg.norm(q)

def quat_to_rotmat(q):
    return R.from_quat([q[0], q[1], q[2], q[3]], scalar_first = True).as_matrix()

# --- EKF functions ---
def state_transition(state, omega, dt):
    q = state[:4]
    b = state[4:]
    omega_corr = omega - b
    omega_mag = np.linalg.norm(omega_corr)
    if omega_mag < 1e-6:
        dq = np.array([1.0, 0.0, 0.0, 0.0])
    else:
        theta = omega_mag * dt
        dq = np.concatenate([[np.cos(theta/2)], np.sin(theta/2) * omega_corr / omega_mag])
    q_new = quat_mult(dq, q)
    q_new = quat_normalize(q_new)
    return np.concatenate([q_new, b])

def calc_delta_q_bias(omega_i, omega_j, OMEGA, kronecker_delta, dt):
    half_theta = OMEGA * dt / 2.0
    s = np.sin(half_theta)
    c = np.cos(half_theta)
    if kronecker_delta not in (0.0, 1.0):
        raise ValueError("Invalid Kronecker delta. Must be 0 or 1.")
    if OMEGA < 1e-6:
        return -0.5 * dt * kronecker_delta
    return (
        -dt * 0.5 * c * (omega_i / OMEGA) * (omega_j / OMEGA)
        - s * ((OMEGA**2 * kronecker_delta - omega_i * omega_j) / (OMEGA**3))
    )

def state_transition_jacobian(q, omega_corr, dt):
    # Implements the full jacobian as in the C++ code
    w, x, y, z = q # quaternion has the form [w, x, y, z]
    wx, wy, wz = omega_corr
    OMEGA = np.linalg.norm(omega_corr)
    half_theta = OMEGA * dt / 2.0
    s = np.sin(half_theta)
    c = np.cos(half_theta)

    if OMEGA < 1e-6:
        delta_w = c
        delta_x = 0.0
        delta_y = 0.0
        delta_z = 0.0
    else:
        delta_w = c
        delta_x = wx * s / OMEGA
        delta_y = wy * s / OMEGA
        delta_z = wz * s / OMEGA

    F = np.zeros((7, 7))
    # ∂q_new / ∂q
    F[0, 0] =  delta_w;  F[0, 1] = -delta_x;  F[0, 2] = -delta_y;  F[0, 3] = -delta_z
    F[1, 0] =  delta_x;  F[1, 1] =  delta_w;  F[1, 2] = -delta_z;  F[1, 3] =  delta_y
    F[2, 0] =  delta_y;  F[2, 1] =  delta_z;  F[2, 2] =  delta_w;  F[2, 3] = -delta_x
    F[3, 0] =  delta_z;  F[3, 1] = -delta_y;  F[3, 2] =  delta_x;  F[3, 3] =  delta_w

    # ∂Δq/∂bias
    # d_delta_qw_d_bi
    d_delta_qw_d_bx = dt * 0.5 * (x * s / OMEGA) if OMEGA >= 1e-6 else 0.0
    d_delta_qw_d_by = dt * 0.5 * (y * s / OMEGA) if OMEGA >= 1e-6 else 0.0
    d_delta_qw_d_bz = dt * 0.5 * (z * s / OMEGA) if OMEGA >= 1e-6 else 0.0

    # d_delta_qx_d_bj, etc.
    d_delta_qx_d_bx = calc_delta_q_bias(wx, wx, OMEGA, 1.0, dt)
    d_delta_qx_d_by = calc_delta_q_bias(wx, wy, OMEGA, 0.0, dt)
    d_delta_qx_d_bz = calc_delta_q_bias(wx, wz, OMEGA, 0.0, dt)
    d_delta_qy_d_bx = calc_delta_q_bias(wy, wx, OMEGA, 0.0, dt)
    d_delta_qy_d_by = calc_delta_q_bias(wy, wy, OMEGA, 1.0, dt)
    d_delta_qy_d_bz = calc_delta_q_bias(wy, wz, OMEGA, 0.0, dt)
    d_delta_qz_d_bx = calc_delta_q_bias(wz, wx, OMEGA, 0.0, dt)
    d_delta_qz_d_by = calc_delta_q_bias(wz, wy, OMEGA, 0.0, dt)
    d_delta_qz_d_bz = calc_delta_q_bias(wz, wz, OMEGA, 1.0, dt)

    # ∂(Δq ⊗ q)/∂b
    # ∂qw'/∂b
    d_qw_d_bx = w * d_delta_qw_d_bx - x * d_delta_qx_d_bx - y * d_delta_qy_d_bx - z * d_delta_qz_d_bx
    d_qw_d_by = w * d_delta_qw_d_by - x * d_delta_qx_d_by - y * d_delta_qy_d_by - z * d_delta_qz_d_by
    d_qw_d_bz = w * d_delta_qw_d_bz - x * d_delta_qx_d_bz - y * d_delta_qy_d_bz - z * d_delta_qz_d_bz
    # ∂qx'/∂b
    d_qx_d_bx = x * d_delta_qw_d_bx + w * d_delta_qx_d_bx + z * d_delta_qy_d_bx - y * d_delta_qz_d_bx
    d_qx_d_by = x * d_delta_qw_d_by + w * d_delta_qx_d_by + z * d_delta_qy_d_by - y * d_delta_qz_d_by
    d_qx_d_bz = x * d_delta_qw_d_bz + w * d_delta_qx_d_bz + z * d_delta_qy_d_bz - y * d_delta_qz_d_bz
    # ∂qy'/∂b
    d_qy_d_bx = y * d_delta_qw_d_bx - z * d_delta_qx_d_bx + w * d_delta_qy_d_bx + x * d_delta_qz_d_bx
    d_qy_d_by = y * d_delta_qw_d_by - z * d_delta_qx_d_by + w * d_delta_qy_d_by + x * d_delta_qz_d_by
    d_qy_d_bz = y * d_delta_qw_d_bz - z * d_delta_qx_d_bz + w * d_delta_qy_d_bz + x * d_delta_qz_d_bz
    # ∂qz'/∂b
    d_qz_d_bx = z * d_delta_qw_d_bx + y * d_delta_qx_d_bx - x * d_delta_qy_d_bx + w * d_delta_qz_d_bx
    d_qz_d_by = z * d_delta_qw_d_by + y * d_delta_qx_d_by - x * d_delta_qy_d_by + w * d_delta_qz_d_by
    d_qz_d_bz = z * d_delta_qw_d_bz + y * d_delta_qx_d_bz - x * d_delta_qy_d_bz + w * d_delta_qz_d_bz

    # Fill in bias columns
    F[0, 4] = d_qw_d_bx
    F[0, 5] = d_qw_d_by
    F[0, 6] = d_qw_d_bz

    F[1, 4] = d_qx_d_bx
    F[1, 5] = d_qx_d_by
    F[1, 6] = d_qx_d_bz

    F[2, 4] = d_qy_d_bx
    F[2, 5] = d_qy_d_by
    F[2, 6] = d_qy_d_bz

    F[3, 4] = d_qz_d_bx
    F[3, 5] = d_qz_d_by
    F[3, 6] = d_qz_d_bz

    # Bias block (bias is constant)
    F[4, 4] = 1.0
    F[5, 5] = 1.0
    F[6, 6] = 1.0

    return F

def measurement_function(state):
    q = state[:4]
    Rwb = quat_to_rotmat(q)
    accel_pred = Rwb.T @ g_ref
    mag_pred = Rwb.T @ m_ref
    return np.concatenate([accel_pred, mag_pred])

def measurement_jacobian(state):
    # Finite difference for quaternion part (first 4 elements)
    eps = 1e-6
    H = np.zeros((6, 7))
    y0 = measurement_function(state)
    for i in range(4):
        dstate = np.zeros(7)
        dstate[i] = eps
        y1 = measurement_function(state + dstate)
        H[:, i] = (y1 - y0) / eps
    # Bias does not affect accel/mag directly
    return H

def measurement_jacobian2(state):
    # Analytical Jacobian of measurement function w.r.t. quaternion (first 4 states)
    q = state[:4]
    qw, qx, qy, qz = q
    # Use the same gravity and mag vectors as in the simulation
    g = g_ref
    m = m_ref

    def dRqdq(q, v):
        qw, qx, qy, qz = q
        vx, vy, vz = v
        J = np.zeros((3, 4))
    
        # Row 0: dfx/dq (all seems correct)
        J[0, 0] = 2 * (-vz*qy + vy*qz)
        J[0, 1] = 2 * (vy*qy + vz*qz)
        J[0, 2] = 2 * (-2*vx*qy + vy*qx - vz*qw)
        J[0, 3] = 2 * (-2*vx*qz + vy*qw + vz*qx)

        # Row 1: dfy/dq 
        J[1, 0] = 2 * (vz*qx - vx*qz)
        J[1, 1] = 2 * (-2*vy*qx + vx*qy + vz*qw)
        J[1, 2] = 2 * (vx*qx + vz*qz)
        J[1, 3] = 2 * (-vx*qw + vz*qy - 2*vy*qz)
        
        # Row 2: dfz/dq
        J[2, 0] = 2 * (-vy*qx + vx*qy)
        J[2, 1] = 2 * (vx*qz - vy*qw - 2*vz*qx)
        J[2, 2] = 2 * (vx*qw + vy*qz - 2*vz*qy)
        J[2, 3] = 2 * (vx*qx + vy*qy)
        return J

    # The measurement function uses Rwb.T @ g_ref and Rwb.T @ m_ref
    # d(R(q).T v)/dq 
    Jg = dRqdq(q, g)
    Jm = dRqdq(q, m)

    H = np.zeros((6, 7))
    H[0:3, 0:4] = Jg
    H[3:6, 0:4] = Jm
    # Bias does not affect accel/mag directly so last 3 columns are zero
    return H

# --- EKF parameters ---
q_scalar = 1e-4
r_scalar = 1e-4
initial_p = 1e-3

state = np.zeros(7)

# Initialize quaternion state to a random orientation
rand_quat = R.random().as_quat()  # returns [x, y, z, w]
# Convert to [w, x, y, z] order
state[0] = rand_quat[3]
state[1] = rand_quat[0]
state[2] = rand_quat[1]
state[3] = rand_quat[2]

state[4:] = 0.0

P = np.eye(7) * initial_p
Q = np.eye(7) * q_scalar
Q[0:4, 0:4] *= 1
Q[4:, 4:] *= 3

R_meas = np.eye(6) * r_scalar



data = pd.read_csv('imu_at_rest.csv', header=0)  
# Assumes columns: time, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z

data = data.apply(pd.to_numeric, errors='coerce').dropna()

time = data.iloc[:, 0].values.astype(float)
measured_gyro = data.iloc[:, [1, 2, 3]].values.astype(float)
measured_accel = data.iloc[:, [4, 5, 6]].values.astype(float)
measured_mag = data.iloc[:, [7, 8, 9]].values.astype(float)

N = len(time)
est_quat = np.zeros((N, 4))
est_bias = np.zeros((N, 3))

dt = 0.001

# Reference vectors (gravity and magnetic field in world frame)
g_ref = np.array([0, 0, -9.81])
m_ref = np.array([1, 1, 0]) # magnetic field depends on location

# --- Simulate measured orientation ---
measured_quat = np.zeros((N, 4))
measured_quat[0] = state[0:4]  # Start with the same initial orientation
for k in range(1, N):
    omega = measured_gyro[k-1]
    theta = np.linalg.norm(omega) * dt
    if theta < 1e-6:
        dq = np.array([1.0, 0.0, 0.0, 0.0])
    else:
        dq = np.concatenate([[np.cos(theta/2)], np.sin(theta/2) * omega / np.linalg.norm(omega)])
    measured_quat[k] = quat_normalize(quat_mult(dq, measured_quat[k-1]))


for k in range(N):
    # --- Prediction ---
    omega = measured_gyro[k]

    omega_corr = omega - state[4:]
    F = state_transition_jacobian(state[:4], omega_corr, dt)
    state = state_transition(state, omega, dt)
    P = F @ P @ F.T + Q

    # --- Measurement update (accel + mag) ---
    z = np.concatenate([measured_accel[k], measured_mag[k]])
    z_pred = measurement_function(state)
    H = measurement_jacobian2(state)
    y = z - z_pred
    S = H @ P @ H.T + R_meas
    K = P @ H.T @ np.linalg.inv(S)
    state = state + K @ y
    state[:4] = quat_normalize(state[:4])
    P = (np.eye(7) - K @ H) @ P
    est_quat[k] = state[:4]
    est_bias[k] = state[4:]

# --- Convert estimated quaternions to Euler angles ---

# Predicted euler angle orientation
euler_ekf = R.from_quat(est_quat, scalar_first = True).as_euler('xyz', degrees=True)
euler_ekf = np.unwrap(np.deg2rad(euler_ekf), axis=0)
euler_ekf = np.rad2deg(euler_ekf)

# Unfiltered euler angle orientation
euler_unfiltered = R.from_quat(measured_quat, scalar_first = True).as_euler('xyz', degrees=True)
euler_unfiltered = np.unwrap(np.deg2rad(euler_unfiltered), axis=0)
euler_unfiltered = np.rad2deg(euler_unfiltered)


# --- Plotting results ---
plt.figure(figsize=(12, 8))
plt.subplot(4, 1, 1)
plt.plot(time, euler_ekf[:, 0], label='Roll (EKF)')
plt.plot(time, euler_ekf[:, 1], label='Pitch (EKF)')
plt.plot(time, euler_ekf[:, 2], label='Yaw (EKF)')
plt.xlabel('Time (s)')
plt.ylabel('Angle (degrees)')
plt.legend()
plt.grid()

plt.subplot(4, 1, 2)
plt.plot(time, euler_unfiltered[:, 0], label='Roll (Unfiltered)')
plt.plot(time, euler_unfiltered[:, 1], label='Pitch (Unfiltered)')
plt.plot(time, euler_unfiltered[:, 2], label='Yaw (Unfiltered)')
plt.xlabel('Time (s)')
plt.ylabel('Angle (degrees)')
plt.legend()
plt.grid()

# plotting quaternion orientation
plt.subplot(4, 1, 3)
plt.plot(time, est_quat[:, 0], label='q_w (EKF)')
plt.plot(time, est_quat[:, 1], label='q_x (EKF)')
plt.plot(time, est_quat[:, 2], label='q_y (EKF)')
plt.plot(time, est_quat[:, 3], label='q_z (EKF)')
plt.xlabel('Time (s)')
plt.ylabel('Quaternion Components')
plt.legend()
plt.grid()

plt.subplot(4, 1, 4)
plt.plot(time, measured_quat[:, 0], label='q_w (Measured)')
plt.plot(time, measured_quat[:, 1], label='q_x (Measured)')
plt.plot(time, measured_quat[:, 2], label='q_y (Measured)')
plt.plot(time, measured_quat[:, 3], label='q_z (Measured)')
plt.xlabel('Time (s)')
plt.ylabel('Quaternion Components')
plt.legend()
plt.grid()

plt.show()
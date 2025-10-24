import numpy as np

# Internal dynamics of the rocket
def dynamics(x, u):
    # 3D TVC Rocket Dynamics with Quaternion Attitude
    # x = [x, y, z, qx, qy, qz, qw, x_dot, y_dot, z_dot, wx, wy, wz]
    # u = [gimbal_theta, gimbal_phi, thrust]
    m = 1      # mass (kg)
    g = 9.81     # gravity (m/s^2)
    Ixx = 0.2    # moment of inertia x (kg*m^2)
    Iyy = 0.2    # moment of inertia y (kg*m^2)
    Izz = 0.4    # moment of inertia z (kg*m^2)
    I = np.diag([Ixx, Iyy, Izz])
    I_inv = np.linalg.inv(I)
    dt = 0.1    # integration step

    gimbal_inertia = 0.1  # gimbal inertia (kg*m^2)

    # Unpack state
    x_pos, y_pos, z_pos, qx, qy, qz, qw, x_dot, y_dot, z_dot, wx, wy, wz, gimbal_phi, gimbal_theta, gimbal_phi_rate, gimbal_theta_rate = x
    gimbal_theta_cmd, gimbal_phi_cmd, thrust = u

    # Clamp thrust to be non-negative
    thrust = np.maximum(thrust, 0.0)

    # Compute thrust direction in body frame (gimbal angles)
    # theta: pitch (up/down), phi: yaw (left/right)
    # Thrust vector in body frame
    # Note that the decomposition of angles to cartesian coordinates is an approximation (based on small angle approximation, will most likely change to be more accurate in the future)
    tx = np.sin(gimbal_theta) * np.cos(gimbal_phi)
    ty = np.sin(gimbal_phi) * np.cos(gimbal_theta)
    tz = np.cos(gimbal_theta) * np.cos(gimbal_phi)
    thrust_b = thrust * np.array([tx, ty, tz])

    # Quaternion to rotation matrix
    q = np.array([qx, qy, qz, qw])
    q_norm = np.linalg.norm(q)
    if q_norm < 1e-8:
        q = np.array([0, 0, 0, 1])
    else:
        q = q / q_norm
    qx, qy, qz, qw = q
    R = np.array([
        [1 - 2*qy**2 - 2*qz**2,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw,     1 - 2*qx**2 - 2*qz**2,     2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw,         2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
    ])

    # Acceleration in world frame
    # difficult to model aerodynamic forces because we don't know direction and it depends on weather conditions. 
    acc = (R @ thrust_b) / m - np.array([0, 0, g])
    x_ddot, y_ddot, z_ddot = acc

    # Assume thrust vector offset from center of mass by lever arm d along z_b (for TVC torque)
    d = 1.0  # lever arm (meters)
    r_cp = np.array([0, 0, -d])
    torque_b = np.cross(r_cp, thrust_b)


    # model servos as PD controllers
    Kp = 5.0
    Kd = 1.0
    gimbal_phi_ddot = (Kp * (- gimbal_phi + gimbal_phi_cmd) - Kd * gimbal_phi_rate)/gimbal_inertia
    gimbal_theta_ddot = (Kp * (- gimbal_theta + gimbal_theta_cmd) - Kd * gimbal_theta_rate)/gimbal_inertia
    #print(gimbal_phi_ddot, gimbal_theta_ddot)

    # torque_b += np.array([-gimbal_theta_ddot * gimbal_inertia, -gimbal_phi_ddot * gimbal_inertia, 0.0]) #reaction torques

    # Angular velocity
    omega = np.array([wx, wy, wz])
    omega_dot = I_inv @ (torque_b - np.cross(omega, I @ omega))

    # Quaternion derivative
    # dq/dt = 0.5 * quat_mult(q, [wx, wy, wz, 0])
    omega_quat = np.array([wx, wy, wz, 0.0])
    dqdt = 0.5 * np.array([
        qw * wx + qy * wz - qz * wy,
        qw * wy + qz * wx - qx * wz,
        qw * wz + qx * wy - qy * wx,
        -qx * wx - qy * wy - qz * wz
    ])

    # Euler integration
    x_pos_new = x_pos + dt * x_dot
    y_pos_new = y_pos + dt * y_dot
    z_pos_new = z_pos + dt * z_dot
    q_new = q + dt * dqdt
    q_new = q_new / np.linalg.norm(q_new)  # normalize quaternion
    x_dot_new = x_dot + dt * x_ddot
    y_dot_new = y_dot + dt * y_ddot
    z_dot_new = z_dot + dt * z_ddot
    wx_new = wx + dt * omega_dot[0]
    wy_new = wy + dt * omega_dot[1]
    wz_new = wz + dt * omega_dot[2]

    # Update gimbal rates
    gimbal_phi_rate += gimbal_phi_ddot * dt
    gimbal_theta_rate += gimbal_theta_ddot * dt
    # Update gimbal angles
    gimbal_phi += gimbal_phi_rate * dt
    gimbal_theta += gimbal_theta_rate * dt

    x_new = np.array([
        x_pos_new, y_pos_new, z_pos_new,
        q_new[0], q_new[1], q_new[2], q_new[3],
        x_dot_new, y_dot_new, z_dot_new,
        wx_new, wy_new, wz_new,
        gimbal_phi, gimbal_theta, gimbal_phi_rate, gimbal_theta_rate
    ])

    return x_new
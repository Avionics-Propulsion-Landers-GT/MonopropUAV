// WORK IN PROGRESS. Most likely not necessary for tethered hover

// A crate for realistic dynamics of lander
// Hansel Zhang

// TO DO: how the inertia tensor of the entire lander changes as the engine gimbals
// How the orientation changes as the engine gimbals (conservation of angular momentum)

use ndarray::{Array1, Array2};
use ndarray::s;
use ndarray_linalg::Inverse;
use ndarray_linalg::Norm;
use ndarray::ArrayView1;



// a more accurate dynamics function ffor simulation. 
// intentionally not used in mpc for speed, and is intentionally slightly different from mpc dynamics
pub fn real_dynamics(x: &Array1<f64>, u: &Array1<f64>) -> Array1<f64> {
    // let mut x_next = Array1::<f64>::zeros(x.len());
    // Constants
    let m = 70.0;
    let g = 9.81;
    let ixx = 25.0;
    let iyy = 22.0;
    let izz = 11.0;
    let dt = 0.1;

    // Inertia matrix and inverse
    let i = Array2::from_diag(&Array1::from(vec![ixx, iyy, izz]));
    let i_inv = i.clone().inv().unwrap();

    // Unpack state
    let x_pos = x[0];
    let y_pos = x[1];
    let z_pos = x[2];
    let qx = x[3];
    let qy = x[4];
    let qz = x[5];
    let qw = x[6];
    let x_dot = x[7];
    let y_dot = x[8];
    let z_dot = x[9];
    let wx = x[10];
    let wy = x[11];
    let wz = x[12];

    // Unpack control
    let gimbal_theta = u[0];
    let gimbal_phi = u[1];
    let mut thrust = u[2];
    if thrust < 0.0 { thrust = 0.0; }

    // Thrust direction in body frame
    let tx = gimbal_theta.sin() * gimbal_phi.cos();
    let ty = gimbal_phi.sin() * gimbal_theta.cos();
    let tz = gimbal_theta.cos() * gimbal_phi.cos();
    let thrust_b = Array1::from(vec![tx * thrust, ty * thrust, tz * thrust]);

    // Quaternion normalization
    let mut q = Array1::from(vec![qx, qy, qz, qw]);
    let q_norm = q.dot(&q).sqrt();
    if q_norm < 1e-8 {
        q = Array1::from(vec![0.0, 0.0, 0.0, 1.0]);
    } else {
        q = q.mapv(|v| v / q_norm);
    }
    let qx = q[0];
    let qy = q[1];
    let qz = q[2];
    let qw = q[3];

    // Rotation matrix from quaternion
    let r = Array2::from_shape_vec((3, 3), vec![
        1.0 - 2.0 * qy * qy - 2.0 * qz * qz, 2.0 * qx * qy - 2.0 * qz * qw, 2.0 * qx * qz + 2.0 * qy * qw,
        2.0 * qx * qy + 2.0 * qz * qw, 1.0 - 2.0 * qx * qx - 2.0 * qz * qz, 2.0 * qy * qz - 2.0 * qx * qw,
        2.0 * qx * qz - 2.0 * qy * qw, 2.0 * qy * qz + 2.0 * qx * qw, 1.0 - 2.0 * qx * qx - 2.0 * qy * qy,
    ]).unwrap();

    // Acceleration in world frame
    let acc = r.dot(&thrust_b) / m - Array1::from(vec![0.0, 0.0, g]);
    let x_ddot = acc[0];
    let y_ddot = acc[1];
    let z_ddot = acc[2];

    // TVC torque
    let d = 1.0;
    let r_cp = Array1::from(vec![0.0, 0.0, -d]);
    let torque_b = Array1::from(vec![
        r_cp[1] * thrust_b[2] - r_cp[2] * thrust_b[1],
        r_cp[2] * thrust_b[0] - r_cp[0] * thrust_b[2],
        r_cp[0] * thrust_b[1] - r_cp[1] * thrust_b[0],
    ]);

    // Angular velocity
    let omega = Array1::from(vec![wx, wy, wz]);
    let omega_cross_i_omega = Array1::from(vec![
        omega[1] * (iyy * omega[2]) - omega[2] * (izz * omega[1]),
        omega[2] * (izz * omega[0]) - omega[0] * (ixx * omega[2]),
        omega[0] * (ixx * omega[1]) - omega[1] * (iyy * omega[0]),
    ]);
    let omega_dot = i_inv.dot(&(torque_b - omega_cross_i_omega));

    // Quaternion derivative
    let dqdt = Array1::from(vec![
        qw * wx + qy * wz - qz * wy,
        qw * wy + qz * wx - qx * wz,
        qw * wz + qx * wy - qy * wx,
        -qx * wx - qy * wy - qz * wz,
    ]).mapv(|v| 0.5 * v);

    // Euler integration
    let x_pos_new = x_pos + dt * x_dot;
    let y_pos_new = y_pos + dt * y_dot;
    let z_pos_new = z_pos + dt * z_dot;
    let mut q_new = q.clone() + dqdt.mapv(|v| dt * v);
    let q_new_norm = q_new.dot(&q_new).sqrt();
    q_new = q_new.mapv(|v| v / q_new_norm);
    let x_dot_new = x_dot + dt * x_ddot;
    let y_dot_new = y_dot + dt * y_ddot;
    let z_dot_new = z_dot + dt * z_ddot;
    let wx_new = wx + dt * omega_dot[0];
    let wy_new = wy + dt * omega_dot[1];
    let wz_new = wz + dt * omega_dot[2];

    let x_next = Array1::from(vec![
        x_pos_new, y_pos_new, z_pos_new,
        q_new[0], q_new[1], q_new[2], q_new[3],
        x_dot_new, y_dot_new, z_dot_new,
        wx_new, wy_new, wz_new,
    ]);

    x_next
}

pub fn realer_dynamics(x: &Array1<f64>, u: &Array1<f64>, x2: &Array1<f64>) -> (Array1<f64>, Array1<f64>) { //returns new state and new actuator state
    // Physical parameters of the lander
    let m = 75.0; // mass in kg (without the engine)
    let m_engine = 10.0; // mass of engine in kg
    let g = 9.81; // gravity in m/s^2
    let dt = 0.1; // time step in seconds

    // the inertia tensor in the body frame (WITHOUT the engine)
    let I_main_tensor = Array2::from_shape_vec((3, 3), vec![
        22.0, 0.4, 0.2,
        0.4, 24.0, 0.3,
        0.2, 0.3, 12.0,
    ]).unwrap();

    let I_engine_tensor = Array2::from_shape_vec((3, 3), vec![
        1.0, 0.0, 0.0,
        0.0, 1.2, 0.0,
        0.0, 0.0, 0.8,
    ]).unwrap();

    let I_main_inv = I_main_tensor.inv().unwrap();

    // Defining key geometry
    // The tippy top of the lander (in the body frame) will be the origin [0.0, 0.0, 0.0]

    // TODO: Account for changing center of Mass
    let centerOfMass = Array1::from(vec![0.0, 0.0, -1.2]); // in body frame (NOT accounting for engine mass, that is a separate body)

    // the joint at which the engine is gimbaled. 
    let thrustPoint = Array1::from(vec![0.0, 0.0, -1.5]); // in body frame

    // the top joints at which the linear actuators connect to the lander. These are FIXED in the body frame
    let gimbalPhiTop = Array1::from(vec![0.0, 0.2, -1.4]); // in body frame. When the gimbal phi actuator extends, it causes rotation about the local body x axis.
    let gimbalThetaTop = Array1::from(vec![0.2, 0.0, -1.4]); // in body frame. When the gimbal theta actuator extends, it causes rotation about the local body y axis.

    // the bottom joints at which the linear actuators connect to the engine when the engine is pointed directly down. These are NOT fixed in the body frame. 
    let gimbalPhiBottom = Array1::from(vec![0.0, 0.2, -1.6]);
    let gimbalThetaBottom = Array1::from(vec![0.2, 0.0, -1.6]);

    // Modeling linear actuators for gimbal control as first order springs (hella stiff springs)
    let m_act = 0.4; // mass of each actuator in kg (assume insignificant compared to the rest of the lander)
    let k = 5000.0; // spring constant
    let c = 1000.0; // damping constant
    let rest_length = gimbalPhiBottom[2] - gimbalPhiTop[2]; // rest length of the actuator when no gimbal angle is applied

    // Unpack state
    let x_pos = x[0];
    let y_pos = x[1];
    let z_pos = x[2];
    let qx = x[3];
    let qy = x[4];
    let qz = x[5];
    let qw = x[6];
    let x_dot = x[7];
    let y_dot = x[8];
    let z_dot = x[9];
    let wx = x[10];
    let wy = x[11];
    let wz = x[12];

    // Unpack actuator state (includes length of actuator, velocity of actuator extension)
    let a_phi_length = x2[0];
    let a_phi_velocity = x2[1];
    let a_theta_length = x2[2];
    let a_theta_velocity = x2[3];
     // Unpack actuator state (includes length of actuator, velocity of actuator extension)
    let a_phi_length = x2[0];
    let a_phi_velocity = x2[1];
    let a_theta_length = x2[2];
    let a_theta_velocity = x2[3];

    // Unpack control
    let gimbal_theta = u[0];
    let gimbal_phi = u[1];
    let mut thrust = u[2];
    if thrust < 0.0 { thrust = 0.0; }

    // Thrust direction in body frame 
    // note that the engine is gimbaled using linear actuators that push and pull on the engine. 
    let tx = gimbal_theta.sin() * gimbal_phi.cos();
    let ty = gimbal_phi.sin() * gimbal_theta.cos();
    let tz = gimbal_theta.cos() * gimbal_phi.cos();

    // Thrust in body frame
    let thrust_b = Array1::from(vec![tx * thrust, ty * thrust, tz * thrust]);

    // Quaternion normalization
    let mut q = Array1::from(vec![qx, qy, qz, qw]);
    let q_norm = q.dot(&q).sqrt();
    if q_norm < 1e-8 {
        q = Array1::from(vec![0.0, 0.0, 0.0, 1.0]);
    } else {
        q = q.mapv(|v| v / q_norm);
    }
    let qx = q[0];
    let qy = q[1];
    let qz = q[2];
    let qw = q[3];

    // Rotation matrix from quaternion
    let r = Array2::from_shape_vec((3, 3), vec![
        1.0 - 2.0 * qy * qy - 2.0 * qz * qz, 2.0 * qx * qy - 2.0 * qz * qw, 2.0 * qx * qz + 2.0 * qy * qw,
        2.0 * qx * qy + 2.0 * qz * qw, 1.0 - 2.0 * qx * qx - 2.0 * qz * qz, 2.0 * qy * qz - 2.0 * qx * qw,
        2.0 * qx * qz - 2.0 * qy * qw, 2.0 * qy * qz + 2.0 * qx * qw, 1.0 - 2.0 * qx * qx - 2.0 * qy * qy,
    ]).unwrap();

    // Acceleration in world frame
    let acc = r.dot(&thrust_b) / m - Array1::from(vec![0.0, 0.0, g]);
    let x_ddot = acc[0];
    let y_ddot = acc[1];
    let z_ddot = acc[2];

    // Calculate the lengths at which the actuators must be to obtain a certain gimbal defined by the control inputs
    // Note the physical constraints where the top of the actuator is FIXED relative to the thrust point on the lander body, while the bottom of the actuator moves with the engine.
    // The distance between the thrust point and the bottom of each actuator is FIXED.
    // When the phi actuator it extends the bottom of the engine points in the negative y direction
    // When the theta actuator it extends the bottom of the engine points in the negative x direction

    let b = (thrustPoint - gimbalPhiBottom).mapv(|v| v.powi(2)).sum().sqrt(); // fixed distance between thrust point and bottom of phi actuator
    let a = rest_length; // note that at rest, we assume the linear actuator points vertically downward in the body frame. (this can be easily changed if the actual design is different)
    let d = (thrustPoint - gimbalPhiTop).mapv(|v| v.powi(2)).sum().sqrt(); // distance between thrust point and top of phi actuator (top is fixed in body frame)

    // Let thrust point be O, top of actuator be T and bottom of actuator be B.
    // We know OT = d, OB = b, TB = current length of actuator = a

    // calculate angle BOT
    let cos_angle_bot = (a.powi(2) + b.powi(2) - d.powi(2)) / (2.0 * a * b);
    let angle_bot_phi = cos_angle_bot.acos();
    let angle_bot_theta = angle_bot_phi; // the very same angle

    // ideal angle BOT. If the gimbal phi is positive, we extend the actuator such that the bottom of the engine moves in the negative y direction.
    // The ideal angle is then BOT + gimbal_phi
    let ideal_angle_bot_phi = angle_bot_phi + gimbal_phi;
    let ideal_angle_bot_theta = angle_bot_theta + gimbal_theta;

    // calculate the ideal length of the actuator using the law of cosines
    let ideal_a_phi = (b.powi(2) + d.powi(2) - 2.0 * b * d * ideal_angle_bot_phi.cos()).sqrt();
    let ideal_a_theta = (b.powi(2) + d.powi(2) - 2.0 * b * d * ideal_angle_bot_theta.cos()).sqrt();

    // calculate spring force from actuator length error
    let error_phi = ideal_a_phi - a_phi_length;
    let error_theta = ideal_a_theta - a_theta_length;
    let phi_velocity = error_phi/dt;
    let theta_velocity = error_theta/dt;

    let force_phi = k * error_phi - c * a_phi_velocity;
    let force_theta = k * error_theta - c * a_theta_velocity;

    // TVC torque
    let r_cp = centerOfMass - thrustPoint;
    let torque_b = Array1::from(vec![
        r_cp[1] * thrust_b[2] - r_cp[2] * thrust_b[1],
        r_cp[2] * thrust_b[0] - r_cp[0] * thrust_b[2],
        r_cp[0] * thrust_b[1] - r_cp[1] * thrust_b[0],
    ]);

    // Angular velocity
    let omega = Array1::from(vec![wx, wy, wz]);
    let omega_cross_i_omega = Array1::from(vec![
        omega[1] * (iyy * omega[2]) - omega[2] * (izz * omega[1]),
        omega[2] * (izz * omega[0]) - omega[0] * (ixx * omega[2]),
        omega[0] * (ixx * omega[1]) - omega[1] * (iyy * omega[0]),
    ]);
    let omega_dot = i_inv.dot(&(torque_b - omega_cross_i_omega));

    // Quaternion derivative
    let dqdt = Array1::from(vec![
        qw * wx + qy * wz - qz * wy,
        qw * wy + qz * wx - qx * wz,
        qw * wz + qx * wy - qy * wx,
        -qx * wx - qy * wy - qz * wz,
    ]).mapv(|v| 0.5 * v);

    // Euler integration
    let x_pos_new = x_pos + dt * x_dot;
    let y_pos_new = y_pos + dt * y_dot;
    let z_pos_new = z_pos + dt * z_dot;
    let mut q_new = q.clone() + dqdt.mapv(|v| dt * v);
    let q_new_norm = q_new.dot(&q_new).sqrt();
    q_new = q_new.mapv(|v| v / q_new_norm);
    let x_dot_new = x_dot + dt * x_ddot;
    let y_dot_new = y_dot + dt * y_ddot;
    let z_dot_new = z_dot + dt * z_ddot;
    let wx_new = wx + dt * omega_dot[0];
    let wy_new = wy + dt * omega_dot[1];
    let wz_new = wz + dt * omega_dot[2];

    let x_next = Array1::from(vec![
        x_pos_new, y_pos_new, z_pos_new,
        q_new[0], q_new[1], q_new[2], q_new[3],
        x_dot_new, y_dot_new, z_dot_new,
        wx_new, wy_new, wz_new,
    ]);

    let x2_next = Array1::from(vec![
        a_phi_length + dt * a_phi_velocity,
        a_phi_velocity + dt * (force_phi / m_act),
        a_theta_length + dt * a_theta_velocity,
        a_theta_velocity + dt * (force_theta / m_act),
    ]);

    x_next, x2_next
}
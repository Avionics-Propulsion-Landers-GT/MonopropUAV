// A crate for realistic dynamics of lander
// Hansel Zhang

// TO DO: how the inertia tensor of the entire lander changes as the engine gimbals
// How the orientation changes as the engine gimbals (conservation of angular momentum)

use ndarray::{Array1, Array2};
use ndarray::s;
use ndarray_linalg::Inverse;
use ndarray_linalg::Norm;
use ndarray::ArrayView1;

// Realistic dynamics function for the lander
// intentionally not used in mpc for speed, and is intentionally slightly different from mpc dynamics
pub fn real_dynamics(x: &Array1<f64>, u: &Array1<f64>) -> Array1<f64> {
    // Constants
    let m = 70.0; // mass of lander in kg WITHOUT engine
    let m_engine = 10.0; // mass of engine in kg
    let total_mass = m + m_engine;
    let g = 9.81;
    let dt = 0.1;
    
    // Inertia tensors (about their respective COMs)
    let i_lander_com = Array2::from_shape_vec((3, 3), vec![
        20.0, 0.3, 0.2,
        0.3, 20.0, 0.3,
        0.2, 0.3, 10.0,
    ]).unwrap();
    
    let i_engine_com = Array2::from_shape_vec((3, 3), vec![
        3.0, 0.0, 0.0,
        0.0, 3.0, 0.0,
        0.0, 0.0, 1.8,
    ]).unwrap();

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
    thrust = thrust.max(0.0); // Clamp to non-negative

    // Thrust direction in engine frame (spherical coordinates)
    let tx = gimbal_theta.sin() * gimbal_phi.cos();
    let ty = gimbal_phi.sin() * gimbal_theta.cos();
    let tz = gimbal_theta.cos() * gimbal_phi.cos();
    
    // Normalize thrust direction (safety)
    let norm = (tx * tx + ty * ty + tz * tz).sqrt().max(1e-8);
    let thrust_b = Array1::from(vec![
        tx * thrust / norm,
        ty * thrust / norm,
        tz * thrust / norm,
    ]);

    let thrust_b_normalized = {
        let n = thrust_b.dot(&thrust_b).sqrt().max(1e-8);
        thrust_b.mapv(|v| v / n)
    };

    // Quaternion normalization
    let q_norm_sq = qx*qx + qy*qy + qz*qz + qw*qw;
    let (qx, qy, qz, qw) = if q_norm_sq > 1e-8 {
        let q_norm = q_norm_sq.sqrt();
        (qx/q_norm, qy/q_norm, qz/q_norm, qw/q_norm)
    } else {
        (0.0, 0.0, 0.0, 1.0) // Default identity quaternion
    };

    // Rotation matrix from quaternion
    let r = Array2::from_shape_vec((3, 3), vec![
        1.0 - 2.0 * (qy*qy + qz*qz), 2.0 * (qx*qy - qz*qw),       2.0 * (qx*qz + qy*qw),
        2.0 * (qx*qy + qz*qw),       1.0 - 2.0 * (qx*qx + qz*qz), 2.0 * (qy*qz - qx*qw),
        2.0 * (qx*qz - qy*qw),       2.0 * (qy*qz + qx*qw),       1.0 - 2.0 * (qx*qx + qy*qy),
    ]).unwrap();

    // Acceleration in world frame
    let thrust_world = r.dot(&thrust_b);
    let x_ddot = thrust_world[0] / total_mass;
    let y_ddot = thrust_world[1] / total_mass;
    let z_ddot = thrust_world[2] / total_mass - g;

    // Define geometry in body frame
    let lander_com_body = Array1::from(vec![0.0, 0.0, -0.5]); // lander COM
    let thrust_point_body = Array1::from(vec![0.0, 0.0, -1.5]); // gimbal joint in body frame
    
    // Engine geometry - COM is 0.2m from gimbal joint along thrust direction
    let engine_com_offset = 0.2;
    let engine_com_body = Array1::from(vec![
        thrust_point_body[0] + thrust_b_normalized[0] * engine_com_offset,
        thrust_point_body[1] + thrust_b_normalized[1] * engine_com_offset,
        thrust_point_body[2] + thrust_b_normalized[2] * engine_com_offset,
    ]);

    // System COM
    let system_com_body = Array1::from(vec![
        (m * lander_com_body[0] + m_engine * engine_com_body[0]) / total_mass,
        (m * lander_com_body[1] + m_engine * engine_com_body[1]) / total_mass,
        (m * lander_com_body[2] + m_engine * engine_com_body[2]) / total_mass,
    ]);

    // Torque from thrust about system COM
    // Vector from system COM to thrust point in body frame
    let r_com_to_thrust = &thrust_point_body - &system_com_body;
    //print r_com_to_thrust
    println!("r_com_to_thrust = {:?}", r_com_to_thrust);
    let torque_b = Array1::from(vec![
        r_com_to_thrust[1] * thrust_b[2] - r_com_to_thrust[2] * thrust_b[1],
        r_com_to_thrust[2] * thrust_b[0] - r_com_to_thrust[0] * thrust_b[2],
        r_com_to_thrust[0] * thrust_b[1] - r_com_to_thrust[1] * thrust_b[0],
    ]);

    // Compute total inertia tensor about system COM
    let eye = Array2::eye(3);
    
    // Translate lander inertia to system COM
    let r_lander = &lander_com_body - &system_com_body;
    let r_lander_sq = r_lander.dot(&r_lander);
    let r_lander_outer = Array2::from_shape_vec((3, 3), vec![
        r_lander[0]*r_lander[0], r_lander[0]*r_lander[1], r_lander[0]*r_lander[2],
        r_lander[1]*r_lander[0], r_lander[1]*r_lander[1], r_lander[1]*r_lander[2],
        r_lander[2]*r_lander[0], r_lander[2]*r_lander[1], r_lander[2]*r_lander[2],
    ]).unwrap();
    
    let i_lander_system = &i_lander_com 
        + &(&eye * (m * r_lander_sq)) 
        - &(&r_lander_outer * m);

    // Engine inertia in engine frame needs rotation to main body frame
    // Rotation from engine z-axis to thrust direction
    let ez = Array1::from(vec![0.0, 0.0, 1.0]);
    let mut t = thrust_b.clone();

    //normalize t
    let t_norm = t.dot(&t).sqrt().max(1e-8);
    t = t.mapv(|v| v / t_norm);

    // Compute as ez × t whose magnitude is sin(theta)
    let v = Array1::from(vec![
        ez[1]*t[2] - ez[2]*t[1],
        ez[2]*t[0] - ez[0]*t[2],
        ez[0]*t[1] - ez[1]*t[0],
    ]);
    
    // compute the magnitude of v which is sin(theta)
    let s = v.dot(&v).sqrt();

    // compute cos(theta)
    let c = ez.dot(&t); // 

    
    let r_engine_to_body = if s < 1e-8 {
        if c > 0.0 {
            eye.clone() // super small angle so we assume no rotation.
        } else {
            // 180 degree rotation (arbitrary axis, use x-axis)
            Array2::from_shape_vec((3, 3), vec![
                1.0, 0.0, 0.0,
                0.0, -1.0, 0.0,
                0.0, 0.0, -1.0,
            ]).unwrap()
        }
    } else {
        let k = Array2::from_shape_vec((3, 3), vec![
            0.0, -v[2], v[1],
            v[2], 0.0, -v[0],
            -v[1], v[0], 0.0,
        ]).unwrap();
        let k2 = k.dot(&k);
        &eye + &k + &k2 * ((1.0 - c) / (s * s))
    };

    // Engine inertia in body frame (about engine COM)
    let i_engine_body = r_engine_to_body.dot(&i_engine_com).dot(&r_engine_to_body.t());

    // Translate engine inertia to system COM
    let r_engine = &engine_com_body - &system_com_body;
    let r_engine_sq = r_engine.dot(&r_engine);
    let r_engine_outer = Array2::from_shape_vec((3, 3), vec![
        r_engine[0]*r_engine[0], r_engine[0]*r_engine[1], r_engine[0]*r_engine[2],
        r_engine[1]*r_engine[0], r_engine[1]*r_engine[1], r_engine[1]*r_engine[2],
        r_engine[2]*r_engine[0], r_engine[2]*r_engine[1], r_engine[2]*r_engine[2],
    ]).unwrap();
    
    let i_engine_system = &i_engine_body 
        + &(&eye * (m_engine * r_engine_sq)) 
        - &(&r_engine_outer * m_engine);

    // Total inertia about system COM
    let i_total = &i_lander_system + &i_engine_system;
    
    // Angular dynamics
    let omega = Array1::from(vec![wx, wy, wz]);
    let i_total_inv = i_total.inv().unwrap_or_else(|_| eye.clone());
    
    // Coriolis term: ω × (Iω)
    let i_omega = i_total.dot(&omega);
    let omega_cross_i_omega = Array1::from(vec![
        omega[1] * i_omega[2] - omega[2] * i_omega[1],
        omega[2] * i_omega[0] - omega[0] * i_omega[2],
        omega[0] * i_omega[1] - omega[1] * i_omega[0],
    ]);
    
    let omega_dot = i_total_inv.dot(&(&torque_b - &omega_cross_i_omega));

    // Quaternion derivative
    let dqdt = Array1::from(vec![
        0.5 * (qw*wx + qy*wz - qz*wy),
        0.5 * (qw*wy + qz*wx - qx*wz),
        0.5 * (qw*wz + qx*wy - qy*wx),
        0.5 * (-qx*wx - qy*wy - qz*wz),
    ]);

    // Euler integration
    let x_next = Array1::from(vec![
        x_pos + dt * x_dot,
        y_pos + dt * y_dot,
        z_pos + dt * z_dot,
        qx + dt * dqdt[0],
        qy + dt * dqdt[1],
        qz + dt * dqdt[2],
        qw + dt * dqdt[3],
        x_dot + dt * x_ddot,
        y_dot + dt * y_ddot,
        z_dot + dt * z_ddot,
        wx + dt * omega_dot[0],
        wy + dt * omega_dot[1],
        wz + dt * omega_dot[2],
    ]);

    // Renormalize quaternion
    let q_idx = s![3..7];
    let mut q_slice = x_next.slice(q_idx).to_owned();
    let q_norm = q_slice.dot(&q_slice).sqrt().max(1e-8);
    q_slice.mapv_inplace(|v| v / q_norm);
    
    let mut result = x_next;
    result.slice_mut(q_idx).assign(&q_slice);
    
    result
}
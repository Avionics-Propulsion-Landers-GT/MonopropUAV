// A crate for realistic dynamics of lander
// Hansel Zhang

use ndarray::{Array1, Array2};
use ndarray::s;
use ndarray_linalg::{Inverse, Norm};
use ndarray::ArrayView1;
use rand::Rng;


// basic dynamics function
pub fn real_dynamics(x: &Array1<f64>, u: &Array1<f64>) -> Array1<f64> {
    // let mut x_next = Array1::<f64>::zeros(x.len());
    // Constants
    let m = 70.0;
    let g = 9.81;
    let ixx = 20.0;
    let iyy = 20.0;
    let izz = 10.0;
    let dt = 0.05;

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
    let gimbal_theta = x[13];
    let gimbal_phi = x[14];

    // Unpack control
    let gimbal_theta_rate = u[0];
    let gimbal_phi_rate = u[1];
    let mut thrust = u[2];
    if thrust < 0.0 { thrust = 0.0; }

    // Thrust direction in body frame
    let tx = gimbal_theta.sin() * gimbal_phi.cos();
    let ty = gimbal_phi.sin() * gimbal_theta.cos();
    let tz = gimbal_theta.cos() * gimbal_phi.cos();

    // normalize thrust vector
    let norm = (tx * tx + ty * ty + tz * tz).sqrt().max(1e-8);
    let thrust_b = Array1::from(vec![tx * thrust / norm, ty * thrust / norm, tz * thrust / norm]);

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

    // First compute Iω
    let i_omega = Array1::from(vec![
        ixx * omega[0],
        iyy * omega[1],
        izz * omega[2],
    ]);

    // Then compute ω × (Iω)
    let omega_cross_i_omega = Array1::from(vec![
        omega[1] * i_omega[2] - omega[2] * i_omega[1],
        omega[2] * i_omega[0] - omega[0] * i_omega[2],
        omega[0] * i_omega[1] - omega[1] * i_omega[0],
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
    let gimbal_theta_new = gimbal_theta + dt * gimbal_theta_rate;
    let gimbal_phi_new = gimbal_phi + dt * gimbal_phi_rate;

    let x_next = Array1::from(vec![
        x_pos_new, y_pos_new, z_pos_new,
        q_new[0], q_new[1], q_new[2], q_new[3],
        x_dot_new, y_dot_new, z_dot_new,
        wx_new, wy_new, wz_new,
        gimbal_theta_new, gimbal_phi_new,
    ]);

    x_next
}

/*
// more accurate dynamics function
pub fn realer_dynamics(x: &Array1<f64>, u: &Array1<f64>) -> Array1<f64> {

    // let mut x_next = Array1::<f64>::zeros(x.len());
    // Constants
    let m = 70.0; // mass of lander in kg WITHOUT engine
    let m_engine = 10.0; // mass of engine in kg
    let total_mass = m + m_engine;
    let g = 9.81;
    let dt = 0.05;

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
    let gimbal_theta = x[13];
    let gimbal_phi = x[14];

    // add uniformly distributed noise to control, sampled over a range
    let gimbal_rate_range = 0.01_f64.to_radians(); // +/- 3 degrees per second
    let thrust_range = 0.01; // +/- 50.0 N
    let mut rng = rand::thread_rng();
    let noise_control: Array1<f64> = Array1::from(vec![
        rng.gen_range(-gimbal_rate_range..gimbal_rate_range),
        rng.gen_range(-gimbal_rate_range..gimbal_rate_range),
        rng.gen_range(-thrust_range..thrust_range),
    ]);

    let u_noisy = u + &noise_control;

    // Unpack control
    let gimbal_theta_rate = u_noisy[0];
    let gimbal_phi_rate = u_noisy[1];
    let mut thrust = u_noisy[2];
    if thrust < 0.0 { thrust = 0.0; }

    // ======================================================
    // UPDATE GIMBAL ANGLES
    // ======================================================
    let gimbal_theta_new = gimbal_theta + gimbal_theta_rate * dt;
    let gimbal_phi_new = gimbal_phi + gimbal_phi_rate * dt;
    
    // Apply gimbal angle limits
    let max_gimbal_angle = 15.0_f64.to_radians();
    let gimbal_theta_clamped = gimbal_theta_new.max(-max_gimbal_angle).min(max_gimbal_angle);
    let gimbal_phi_clamped = gimbal_phi_new.max(-max_gimbal_angle).min(max_gimbal_angle);

    // check for limits, if limit reached then set rate to zero
    if gimbal_theta_new > max_gimbal_angle || gimbal_theta_new < -max_gimbal_angle {
        gimbal_theta_rate = 0.0;
    }
    if gimbal_phi_new > max_gimbal_angle || gimbal_phi_new < -max_gimbal_angle {
        gimbal_phi_rate = 0.0;
    }

    // Thrust direction in body frame (using UPDATED gimbal angles)
    let tx = gimbal_phi_clamped.sin() * gimbal_theta_clamped.cos();
    let ty = gimbal_phi_clamped.sin() * gimbal_theta_clamped.sin();
    let tz = gimbal_phi_clamped.cos() * gimbal_theta_clamped.cos();

    // normalize thrust vector
    let norm = (tx * tx + ty * ty + tz * tz).sqrt().max(1e-8);
    let thrust_b = Array1::from(vec![
        tx * thrust / norm, 
        ty * thrust / norm, 
        tz * thrust / norm
    ]);

    let thrust_b_normalized = {
        let n = thrust_b.dot(&thrust_b).sqrt().max(1e-8);
        thrust_b.mapv(|v| v / n)
    };

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
    let thrust_world = r.dot(&thrust_b);
    let x_ddot = thrust_world[0] / total_mass;
    let y_ddot = thrust_world[1] / total_mass;
    let z_ddot = thrust_world[2] / total_mass - g;

    // Define geometry in body frame
    let lander_com_body = Array1::from(vec![0.0, 0.0, -0.5]); // lander COM
    let thrust_point_body = Array1::from(vec![0.0, 0.0, -1.5]); // gimbal joint in body frame
    
    // Engine geometry - assume that COM is 0.2m from gimbal joint along thrust direction. This assumes the engine is axisymmetric.
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
    let r_com_to_thrust = &thrust_point_body - &system_com_body;
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

    // Normalize t
    let t_norm = t.dot(&t).sqrt().max(1e-8);
    t = t.mapv(|v| v / t_norm);

    // Compute rotation from engine z-axis to thrust direction
    let v = Array1::from(vec![
        ez[1]*t[2] - ez[2]*t[1],
        ez[2]*t[0] - ez[0]*t[2],
        ez[0]*t[1] - ez[1]*t[0],
    ]);
    
    let s = v.dot(&v).sqrt();
    let c = ez.dot(&t);
    
    let r_engine_to_body = if s < 1e-8 {
        if c > 0.0 {
            eye.clone()
        } else {
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
    
    // Conservation of angular momentum
    
    let omega = Array1::from(vec![wx, wy, wz]);
    let i_total_inv = i_total.inv().unwrap_or_else(|_| eye.clone());
    
        // Relative angular velocity derived from thrust direction kinematics (body frame)
        // Build raw direction u and its derivative u̇
        let ux = gimbal_phi_clamped.sin() * gimbal_theta_clamped.cos();
        let uy = gimbal_phi_clamped.sin() * gimbal_theta_clamped.sin();
        let uz = gimbal_phi_clamped.cos() * gimbal_theta_clamped.cos();
        let u = Array1::from(vec![ux, uy, uz]);
        let u_norm = u.dot(&u).sqrt().max(1e-8);
        let t_unit = u.mapv(|v| v / u_norm);

        let dphi = gimbal_phi_rate;
        let dtheta = gimbal_theta_rate;
        let dux = (gimbal_phi_clamped.cos() * gimbal_theta_clamped.cos()) * dphi
            + (-gimbal_phi_clamped.sin() * gimbal_theta_clamped.sin()) * dtheta;
        let duy = (gimbal_phi_clamped.cos() * gimbal_theta_clamped.sin()) * dphi
            + ( gimbal_phi_clamped.sin() * gimbal_theta_clamped.cos()) * dtheta;
        let duz = (-gimbal_phi_clamped.sin() * gimbal_theta_clamped.cos()) * dphi
            + (-gimbal_phi_clamped.cos() * gimbal_theta_clamped.sin()) * dtheta;
        let u_dot = Array1::from(vec![dux, duy, duz]);

        // Derivative of normalized vector: ṫ = (u̇ − t (t · u̇)) / ‖u‖
        let t_dot_num = &u_dot - &(t_unit.clone() * t_unit.dot(&u_dot));
        let t_dot = t_dot_num / u_norm;

        // ω_rel in body frame via ṫ = ω × t  => minimal ω = t × ṫ
        let omega_rel_body = Array1::from(vec![
        t_unit[1]*t_dot[2] - t_unit[2]*t_dot[1],
        t_unit[2]*t_dot[0] - t_unit[0]*t_dot[2],
        t_unit[0]*t_dot[1] - t_unit[1]*t_dot[0],
        ]);

        // Optional: engine-frame relative angular rate (for diagnostics)
        let omega_rel_engine = r_engine_to_body.t().dot(&omega_rel_body);
    
    // Current total angular momentum in body frame
    let h_current = i_total.dot(&omega) + i_engine_body.dot(&omega_rel_body);
    
    // Angular momentum after dt (conserved except for external torque)
    let h_next = h_current.clone() + &torque_b * dt;
    
    // Solve for new angular velocity that conserves angular momentum
    // h_next = I_total * ω_next + I_engine * ω_rel_body_next
    
    // Compute ω_rel at next time step (using same rates for small dt)
    // First-order: reuse omega_rel_body as approximation
    let omega_rel_body_next = omega_rel_body.clone();
    
    // Solve for ω_next that satisfies conservation
    let omega_next = i_total_inv.dot(&(&h_next - i_engine_body.dot(&omega_rel_body_next)));

    // Diagnostics: validate angular momentum conservation
    let h_reconstructed = i_total.dot(&omega_next) + i_engine_body.dot(&omega_rel_body_next);
    let h_residual = &h_next - &h_reconstructed;
    println!(
        "\n[AM Check] phi_dot={:.6}, theta_dot={:.6}",
        gimbal_phi_rate, gimbal_theta_rate
    );
    println!(
        "[AM Check] omega_rel_engine = {:?}",
        omega_rel_engine.to_vec()
    );
    println!(
        "[AM Check] omega_rel_body   = {:?}",
        omega_rel_body.to_vec()
    );
    println!(
        "[AM Check] H_current ||H||={:.6}",
        h_current.norm()
    );
    println!(
        "[AM Check] H_next    ||H||={:.6}",
        h_next.norm()
    );
    println!(
        "[AM Check] H_recon   ||H||={:.6}",
        h_reconstructed.norm()
    );
    println!(
        "[AM Check] Residual  ||H_next - H_recon||={:.6}",
        h_residual.norm()
    );
    println!(
        "[AM Check] torque_b = {:?}, dt={:.6}",
        torque_b.to_vec(), dt
    );
    
    // Angular acceleration (approximate)
    let omega_dot = (&omega_next - &omega) / dt;
    
    // Add Coriolis term (conservative dynamics)
    let i_omega = i_total.dot(&omega);
    let omega_cross_i_omega = Array1::from(vec![
        omega[1] * i_omega[2] - omega[2] * i_omega[1],
        omega[2] * i_omega[0] - omega[0] * i_omega[2],
        omega[0] * i_omega[1] - omega[1] * i_omega[0],
    ]);
    
    // Final angular acceleration including Coriolis
    let omega_dot_final = omega_dot - i_total_inv.dot(&omega_cross_i_omega);

    // Quaternion derivative
    let dqdt = Array1::from(vec![
        0.5 * (qw*wx + qy*wz - qz*wy),
        0.5 * (qw*wy + qz*wx - qx*wz),
        0.5 * (qw*wz + qx*wy - qy*wx),
        0.5 * (-qx*wx - qy*wy - qz*wz),
    ]);

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
    let wx_new = wx + dt * omega_dot_final[0];
    let wy_new = wy + dt * omega_dot_final[1];
    let wz_new = wz + dt * omega_dot_final[2];
    let gimbal_theta_new = gimbal_theta_clamped;
    let gimbal_phi_new = gimbal_phi_clamped;

    let x_next = Array1::from(vec![
        x_pos_new, y_pos_new, z_pos_new,
        q_new[0], q_new[1], q_new[2], q_new[3],
        x_dot_new, y_dot_new, z_dot_new,
        wx_new, wy_new, wz_new,
        gimbal_theta_new, gimbal_phi_new,
    ]);

    x_next
}
*/


#[path="../../../MPC/src/mpc_crate.rs"]
mod mpc_crate;
use ndarray::{Array1, Array2};
use std::f64::consts::PI;

#[derive(Debug, Clone)]
pub struct MPC {
    n: usize, // state dimension [x, y, z, qx, qy, qz, qw, x_dot, y_dot, z_dot, wx, wy, wz]
    m: usize, // control dimension [gimbal_theta, gimbal_phi, thrust]
    n_steps: usize, // MPC horizon steps
    dt: f64, // time step
    integral_gains: (f64, f64, f64), // integral gains for x, y, z
    q: Array2<f64>, // state cost matrix
    r: Array2<f64>, // control cost matrix
    qn: Array2<f64>, // terminal state cost matrix
    smoothing_weight: Array1<f64>, // weight for exponential smoothing
    panoc_cache_tolerance: f64, // tolerance for OpEn solver
    panoc_cache_lbfgs_memory: usize, // memory for L-BFGS in OpEn solver
}

impl MPC {
    pub fn new(n: usize, m: usize, n_steps: usize, dt: f64, integral_gains: (f64, f64, f64), q: Array2<f64>, r: Array2<f64>, qn: Array2<f64>, smoothing_weight: Array1<f64>, panoc_cache_tolerance: f64, panoc_cache_lbfgs_memory: usize) -> Self {
        Self {
            n,
            m,
            n_steps,
            dt,
            integral_gains,
            q,
            r,
            qn,
            smoothing_weight,
            panoc_cache_tolerance,
            panoc_cache_lbfgs_memory
        }
    }

    pub fn solve(&mut self, xref_traj: &Vec<Array1<f64>>, u_warm: &Vec<Array1<f64>>) -> () {
        let mut 
        let mut u_prev: Array1<f64>, // previous control input for smoothing
        let mut error_integral: (f64, f64, f64) = (0.0, 0.0, 0.0); // integral of errors for x, y, z
        let mut panoc_cache = optimization_engine::panoc::PANOCCache::new(self.m * self.n_steps, self.panoc_cache_tolerance, self.panoc_cache_lbfgs_memory);

        for k in 0..self.n_steps {
            if k as f64 * dt <= 10.0 {
                // change reference point after 5 seconds
                let f_iters = iters as f64;
                let f_k = k as f64;
                xref[0] = -0.0 * (f_k*dt)/10.0;
                xref[1] = 0.0 * (f_k*dt)/10.0;
                xref[2] = 25.0 * (f_k*dt)/10.0 + 3.0;

                xref_traj = Array2::from_shape_fn((n_steps + 1, n), |(_, j)| xref[j]);
                xref_traj_vec = xref_traj.axis_iter(ndarray::Axis(0)).map(|row| row.to_owned()).collect();
                
            } else if k as f64 * dt > 15.0 && k as f64 * dt <= 25.0 {
                // change reference point after 10 seconds
                let f_iters = iters as f64;
                let f_k = k as f64;
                xref[0] = 0.0 * ((f_k*dt)-15.0)/10.0;
                xref[1] = 0.0 * ((f_k*dt)-15.0)/10.0;
                xref[2] = 25.0 - 25.0 * ((f_k*dt)-15.0)/10.0;

                xref_traj = Array2::from_shape_fn((n_steps + 1, n), |(_, j)| xref[j]);
                xref_traj_vec = xref_traj.axis_iter(ndarray::Axis(0)).map(|row| row.to_owned()).collect();
            } 

            
            xref[0] = 0.0;
            xref[1] = 0.0;
            xref[2] = 5.0;

            let xref = xref_traj[k];

            let z_error = xref[2] - x[2];
            z_integral += z_error * dt;
            let mut x_ref_mod = xref.clone();
            x_ref_mod[2] += ki_z * z_integral; // modify z reference with integral term
            for i in 0..xref_traj_vec.len() {
                xref_traj_vec[i][2] = x_ref_mod[2];
            }
            let x_error = xref[0] - x[0];
            x_integral += x_error * dt;
            x_ref_mod[0] += ki_x * x_integral; // modify x reference with integral term
            for i in 0..xref_traj_vec.len() {
                xref_traj_vec[i][0] = x_ref_mod[0];
            }
            let y_error = xref[1] - x[1];
            y_integral += y_error * dt;
            x_ref_mod[1] += ki_y * y_integral; // modify y reference with integral term
            for i in 0..xref_traj_vec.len() {
                xref_traj_vec[i][1] = x_ref_mod[1];
            }

            // Solve MPC to get optimal control sequence
            // solve using OpEn
            let (mut u_apply, u_warm) = mpc_crate::OpEnSolve(&x, &u_warm.axis_iter(ndarray::Axis(0)).map(|row| row.to_owned()).collect(), &xref_traj_vec, &q, &r, &qn, &smoothing_weight, &mut panoc_cache, min_thrust, max_thrust, gimbal_limit);

            // exponential filter
            if k >= 1 {
                let alpha = 0.4; // smoothing factor
                u_apply = alpha * &u_apply + (1.0 - alpha) * &u_prev;
            }
            u_prev = u_apply.clone();
        }
    }
}
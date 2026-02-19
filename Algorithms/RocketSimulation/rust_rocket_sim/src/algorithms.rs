#[path="../../../MPC/src/mpc_crate.rs"]
mod mpc_crate;
use ndarray::{Array1, Array2};
use std::f64::consts::PI;

#[derive(Debug, Clone)]
pub struct MPC {
    pub n: usize, // state dimension [x, y, z, qx, qy, qz, qw, x_dot, y_dot, z_dot, wx, wy, wz]
    pub m: usize, // control dimension [gimbal_theta, gimbal_phi, thrust]
    pub n_steps: usize, // MPC horizon steps
    pub dt: f64, // time step
    pub integral_gains: (f64, f64, f64), // integral gains for x, y, z
    pub q: Array2<f64>, // state cost matrix
    pub r: Array2<f64>, // control cost matrix
    pub qn: Array2<f64>, // terminal state cost matrix
    pub smoothing_weight: Array1<f64>, // weight for exponential smoothing
    pub panoc_cache_tolerance: f64, // tolerance for OpEn solver
    pub panoc_cache_lbfgs_memory: usize, // memory for L-BFGS in OpEn solver
    pub min_thrust: f64, // minimum thrust
    pub max_thrust: f64, // maximum thrust
    pub gimbal_limit: f64, // maximum gimbal angle in radians
}

impl MPC {
    pub fn new(n: usize, m: usize, n_steps: usize, dt: f64, integral_gains: (f64, f64, f64), q: Array2<f64>, r: Array2<f64>, qn: Array2<f64>, smoothing_weight: Array1<f64>, panoc_cache_tolerance: f64, panoc_cache_lbfgs_memory: usize, min_thrust: f64, max_thrust: f64, gimbal_limit: f64) -> Self {
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
            panoc_cache_lbfgs_memory,
            min_thrust,
            max_thrust,
            gimbal_limit,
        }
    }

    pub fn solve(&mut self, x0: &Array1<f64>, xref_traj: &Vec<Array1<f64>>, u_warm: &Vec<Array1<f64>>) -> Vec<Array1<f64>> {
        let mut x = x0.clone(); // initial state
        let mut xref_traj = xref_traj.clone(); // reference trajectory
        let mut u_warm = u_warm.clone(); // warm start control sequence
        let mut u_prev: Array1<f64> = Array1::zeros(self.m); // previous control input for smoothing
        let mut error_integral: (f64, f64, f64) = (0.0, 0.0, 0.0); // integral of errors for x, y, z
        let mut panoc_cache = optimization_engine::panoc::PANOCCache::new(self.m * self.n_steps, self.panoc_cache_tolerance, self.panoc_cache_lbfgs_memory);

        let mut x_integral = 0.0;
        let mut y_integral = 0.0;
        let mut z_integral = 0.0;
        let mut control_sequence: Vec<Array1<f64>> = Vec::new();

        for k in 0..self.n_steps {
            let mut xref = xref_traj[k].clone();

            x_integral += self.integral_gains.0 * (xref[0] - x[0]) * self.dt;
            y_integral += self.integral_gains.1 * (xref[1] - x[1]) * self.dt;
            z_integral += self.integral_gains.2 * (xref[2] - x[2]) * self.dt;

            xref[0] += x_integral; // modify x reference with integral term
            xref[1] += y_integral; // modify y reference with integral term
            xref[2] += z_integral; // modify z reference with integral term

            for i in 0..xref_traj.len() {
                xref_traj[i][0] += x_integral;
                xref_traj[i][1] += y_integral;
                xref_traj[i][2] += z_integral;
            }

            // Solve MPC to get optimal control sequence
            // solve using OpEn
            let (mut u_apply, u_warm) = mpc_crate::OpEnSolve(&x, &u_warm, &xref_traj, &self.q, &self.r, &self.qn, &self.smoothing_weight, &mut panoc_cache, self.min_thrust, self.max_thrust, self.gimbal_limit);

            // exponential filter
            if k >= 1 {
                let alpha = 0.4; // smoothing factor
                u_apply = alpha * &u_apply + (1.0 - alpha) * &u_prev;
            }
            u_prev = u_apply.clone();

            control_sequence.push(u_apply.clone());
        }

        return control_sequence;
    }
}
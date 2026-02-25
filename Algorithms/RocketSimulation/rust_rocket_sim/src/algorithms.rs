#[path="../../../MPC/src/mpc_crate.rs"]
mod mpc_crate;
#[path="../../../LosslessConvexification/rust_lossless/src/lossless.rs"]
mod lossless;
use ndarray::{Array1, Array2};
use std::f64::consts::PI;
use clarabel::algebra::*;
use clarabel::solver::*;
use std::time::Instant;

#[derive(Debug, Clone)]
pub struct MPC {
    pub n: usize, // state dimension [x, y, z, qx, qy, qz, qw, x_dot, y_dot, z_dot, wx, wy, wz]
    pub m: usize, // control dimension [gimbal_theta, gimbal_phi, thrust]
    pub n_steps: usize, // MPC horizon steps
    pub dt: f64, // time step
    pub integral_gains: (f64, f64, f64), // integral gains for x, y, z
    pub integral_values: (f64, f64, f64), // integral values for x, y, z
    pub q: Array2<f64>, // state cost matrix
    pub r: Array2<f64>, // control cost matrix
    pub qn: Array2<f64>, // terminal state cost matrix
    pub smoothing_weight: Array1<f64>, // weight for exponential smoothing
    pub panoc_cache_tolerance: f64, // tolerance for OpEn solver
    pub panoc_cache_lbfgs_memory: usize, // memory for L-BFGS in OpEn solver
    pub min_thrust: f64, // minimum thrust
    pub max_thrust: f64, // maximum thrust
    pub gimbal_limit: f64, // maximum gimbal angle in radians
    pub system_time: f64, // internal time tracking for MPC updates
    pub update_rate: f64, // rate at which MPC updates (e.g., 10 Hz)
    previous_control: Vec<Array1<f64>>, // previous control input for smoothing
    pub last_solve_time: f64,
}

impl MPC {
    pub fn new(n: usize, m: usize, n_steps: usize, dt: f64, integral_gains: (f64, f64, f64), q: Array2<f64>, r: Array2<f64>, qn: Array2<f64>, smoothing_weight: Array1<f64>, panoc_cache_tolerance: f64, panoc_cache_lbfgs_memory: usize, min_thrust: f64, max_thrust: f64, gimbal_limit: f64, system_time: f64, update_rate: f64) -> Self {
        Self {
            n,
            m,
            n_steps,
            dt,
            integral_gains,
            integral_values: (0.0, 0.0, 0.0),
            q,
            r,
            qn,
            smoothing_weight,
            panoc_cache_tolerance,
            panoc_cache_lbfgs_memory,
            min_thrust,
            max_thrust,
            gimbal_limit,
            system_time: 0.0,
            update_rate,
            previous_control: vec![Array1::zeros(m); n_steps],
            last_solve_time: 0.0,
        }
    }

    pub fn update(&mut self, x0: &Array1<f64>, xref_traj: &Vec<Array1<f64>>, u_warm: &Vec<Array1<f64>>, mass: f64, system_time: f64) -> Vec<Array1<f64>> {
        let elapsed_time = system_time - self.system_time;
        if elapsed_time < 1.0 / self.update_rate {
            // Not time to update yet, return previous control sequence
            return self.previous_control.clone();
        } else {
            self.system_time = system_time;
            self.last_solve_time = system_time;

            let control_sequence = self.solve(x0, xref_traj, u_warm, mass);
            self.previous_control = control_sequence.clone();
            return control_sequence;
        }

    }

    pub fn solve(&mut self, x0: &Array1<f64>, xref_traj: &Vec<Array1<f64>>, u_warm: &Vec<Array1<f64>>, mass: f64) -> Vec<Array1<f64>> {
        let mut x = x0.clone(); // initial state
        let mut xref_traj = xref_traj.clone(); // reference trajectory
        let mut u_warm = u_warm.clone(); // warm start control sequence
        let mut u_prev: Array1<f64> = u_warm[0].clone(); // previous control input for smoothing
        let mut error_integral: (f64, f64, f64) = (0.0, 0.0, 0.0); // integral of errors for x, y, z
        let mut panoc_cache = optimization_engine::panoc::PANOCCache::new(self.m * self.n_steps, self.panoc_cache_tolerance, self.panoc_cache_lbfgs_memory);

        let mut control_sequence: Vec<Array1<f64>> = Vec::new();

        for k in 0..self.n_steps {
            let mut xref = xref_traj[k].clone();

            self.integral_values.0 += self.integral_gains.0 * (xref[0] - x[0]) * self.dt;
            self.integral_values.1 += self.integral_gains.1 * (xref[1] - x[1]) * self.dt;
            self.integral_values.2 += self.integral_gains.2 * (xref[2] - x[2]) * self.dt;

            xref[0] += self.integral_values.0; // modify x reference with integral term
            xref[1] += self.integral_values.1; // modify y reference with integral term
            xref[2] += self.integral_values.2; // modify z reference with integral term

            let mut xref_traj_k = xref_traj.clone();
            for i in k..self.n_steps {
                xref_traj_k[i][0] += self.integral_values.0; // modify future x references with integral term
                xref_traj_k[i][1] += self.integral_values.1; // modify future y references with integral term
                xref_traj_k[i][2] += self.integral_values.2; // modify future z references with integral term
            }

            // println!("Integral Values: {:?}", self.integral_values);

            // Solve MPC to get optimal control sequence
            // solve using OpEn
            let (mut u_apply, u_warm) = mpc_crate::OpEnSolve(&x, &u_warm, &xref_traj_k, &self.q, &self.r, &self.qn, &self.smoothing_weight, &mut panoc_cache, mass, self.min_thrust, self.max_thrust, self.gimbal_limit);

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

#[derive(Debug, Clone)]
pub struct Lossless {
    pub glide_slope: f64,
    pub use_glide_slope: bool,
    pub max_velocity: f64,
    pub dry_mass: f64,
    pub alpha: f64,
    pub lower_thrust_bound: f64,
    pub upper_thrust_bound: f64,
    pub tvc_range_rad: f64,
    pub coarse_delta_t: f64,
    pub fine_delta_t: f64,
    pub pointing_direction: [f64; 3],
    pub system_time: f64,
    pub update_rate: f64,
    last_solution: lossless::TrajectoryResult, // Store the last solution for use when not updating
    pub last_solve_time: f64,
}

impl Lossless {
    pub fn new(glide_slope: f64, use_glide_slope: bool, max_velocity: f64, dry_mass: f64, alpha: f64, lower_thrust_bound: f64, upper_thrust_bound: f64, tvc_range_rad: f64, coarse_delta_t: f64, fine_delta_t: f64, pointing_direction: [f64; 3], system_time: f64, update_rate: f64) -> Self {
        Self {
            glide_slope,
            use_glide_slope,
            max_velocity,
            dry_mass,
            alpha,
            lower_thrust_bound,
            upper_thrust_bound,
            tvc_range_rad,
            coarse_delta_t,
            fine_delta_t,
            pointing_direction,
            system_time,
            update_rate,
            last_solution: None,
            last_solve_time: 0.0,
        }
    }

    pub fn update(&mut self, current_position: [f64; 3], current_velocity: [f64; 3], target_position: [f64; 3], propellant_mass: f64, system_time: f64) -> lossless::TrajectoryResult {
        let elapsed_time = system_time - self.system_time;
        if elapsed_time < 1.0 / self.update_rate {
            // Not time to update yet, return previous control input (could be stored in the struct if needed)
            return self.last_solution; // Return the last solution if not time to update
        } else {
            self.system_time = system_time;
            self.last_solve_time = system_time;

            // Call the lossless convexification solver to get the optimal control input
            let control_input = lossless::solve(current_position, current_velocity, target_position, mass, self.glide_slope, self.use_glide_slope, self.max_velocity, self.dry_mass, self.alpha, self.lower_thrust_bound, self.upper_thrust_bound, self.tvc_range_rad, self.coarse_delta_t, self.fine_delta_t);

            return control_input;
        }

    }
}
use clarabel::algebra::*;
use clarabel::solver::*;

const GRAVITY: [f64; 3] = [0.0, 0.0, -9.81];

pub struct LosslessSolver {
    pub landing_point: [f64; 3],
    pub initial_position: [f64; 3],
    pub initial_velocity: [f64; 3],
    pub glide_slope: f64,
    pub use_glide_slope: bool,
    pub max_velocity: f64,
    pub dry_mass: f64,
    pub fuel_mass: f64,
    pub alpha: f64,
    pub lower_thrust_bound: f64,
    pub upper_thrust_bound: f64,
    pub tvc_range_rad: f64,
    pub delta_t: f64,
    pub pointing_direction: [f64; 3],
    pub N: i64,
}

pub struct TrajectoryResult {
    pub positions: Vec<[f64; 3]>,
    pub velocities: Vec<[f64; 3]>,
    pub masses: Vec<f64>,
    pub thrusts: Vec<[f64; 3]>,
    pub sigmas: Vec<f64>,
}

impl Default for LosslessSolver {
    fn default() -> Self {
        LosslessSolver {
            landing_point: [0.0, 0.0, 0.0],
            initial_position: [0.0, 0.0, 0.0],
            initial_velocity: [0.0, 0.0, 0.0],
            glide_slope: 0.0,
            use_glide_slope: false,
            max_velocity: 100.0,
            dry_mass: 0.0,
            fuel_mass: 0.0,
            alpha: 0.0,
            lower_thrust_bound: 0.0,
            upper_thrust_bound: 0.0,
            tvc_range_rad: (15.0_f64).to_radians(),
            delta_t: 1.0,
            pointing_direction: [0.0, 0.0, 1.0],
            N: 1
        }
    }
}

impl LosslessSolver {
    pub fn new() -> Self {
        LosslessSolver::default()
    }

    pub fn solve_at_current_time(&mut self) -> Result<clarabel::solver::DefaultSolution<f64>, String> {
        let m0 = self.dry_mass + self.fuel_mass;
        
        let n_vars = 3 * (self.N + 1) // x
            + 3 * (self.N + 1) // v
            + (self.N + 1) // w
            + 3 * self.N // u
            + self.N; // sigma (thrust slack variable)

        let idx_x = 0;
        let idx_v = idx_x + 3 * (self.N + 1);
        let idx_w = idx_v + 3 * (self.N + 1);
        let idx_u = idx_w + (self.N + 1);
        let idx_sigma = idx_u + 3 * self.N;

        /*
        Equality constraints (Ax = b)
        */

        let mut rows: Vec<usize> = Vec::new();
        let mut cols: Vec<usize> = Vec::new();
        let mut vals = Vec::new();
        let mut b = Vec::new();
        let mut row_counter = 0;

        
        // Setting initial conditions

        // x0 = initial_position
        for i in 0..3 {
            rows.push((row_counter + i) as usize);
            cols.push((idx_x + i) as usize);
            vals.push(1.0);
            b.push(self.initial_position[i as usize]);
        }
        row_counter += 3;

        // v0 = initial_velocity
        for i in 0..3 {
            rows.push((row_counter + i) as usize);
            cols.push((idx_v + i) as usize); // v0 indices
            vals.push(1.0);
            b.push(self.initial_velocity[i as usize]);
        }
        row_counter += 3;

        // w0 = ln(m0)
        rows.push(row_counter as usize);
        cols.push(idx_w as usize);
        vals.push(1.0);
        b.push(m0.ln());
        row_counter += 1;

        
        // vehicle kinematics constraints
        for k in 0..self.N {
            let curr_x_offset = idx_x + 3 * k;
            let next_x_offset = idx_x + 3 * (k + 1);
            let curr_v_offset = idx_v + 3 * k;
            let next_v_offset = idx_v + 3 * (k + 1);
            let curr_w_offset = idx_w + k;
            let next_w_offset = idx_w + (k + 1);
            let curr_u_offset = idx_u + 3 * k;
            let curr_sigma_offset = idx_sigma + k;

            // x_{k+1} - x_k - dt * v_k = 0
            for i in 0..3 {
                rows.push((row_counter + i) as usize);
                cols.push((next_x_offset + i) as usize);
                vals.push(1.0);

                rows.push((row_counter + i) as usize);
                cols.push((curr_x_offset + i) as usize);
                vals.push(-1.0);

                rows.push((row_counter + i) as usize);
                cols.push((curr_v_offset + i) as usize);
                vals.push(-self.delta_t);

                b.push(0.0);
            }
            row_counter += 3;

            // v_{k+1} - v_k - dt * u_k = dt * g
            for i in 0..3 {
                rows.push((row_counter + i) as usize);
                cols.push((next_v_offset + i) as usize);
                vals.push(1.0);

                rows.push((row_counter + i) as usize);
                cols.push((next_v_offset + i) as usize);
                vals.push(-1.0);

                rows.push((row_counter + i) as usize);
                cols.push((next_v_offset + i) as usize);
                vals.push(-self.delta_t);

                // b = dt * g
                b.push(self.delta_t * GRAVITY[i as usize]);
            }
            row_counter += 3;

            // w_{k+1} - w_k + dt * alpha * sigma_k = 0
            rows.push(row_counter as usize);
            cols.push(next_w_offset as usize);
            vals.push(1.0);

            rows.push(row_counter as usize);
            cols.push(curr_w_offset as usize);
            vals.push(-1.0);

            rows.push(row_counter as usize);
            cols.push(curr_sigma_offset as usize);
            vals.push(self.delta_t * self.alpha);

            b.push(0.0);
            row_counter += 1;
        }

        // End constraints

        
        // x_N = landing_point
        for i in 0..3 {
            rows.push(row_counter as usize);
            cols.push((idx_x + 3 * self.N + i) as usize);
            vals.push(1.0);
            b.push(self.landing_point[i as usize]);
            row_counter += 1;
        }

        // v_N = 0
        for i in 0..3 {
            rows.push(row_counter as usize);
            cols.push((idx_v + 3 * self.N + i) as usize);
            vals.push(1.0);
            b.push(0.0);
            row_counter += 1;
        }

        let A = Self::csc_from_triplets(row_counter as usize, n_vars as usize, &rows, &cols, &vals);

        /*
        SOCP constraints
        */
        let mut cones = Vec::new();
        let mut socp_rows: Vec<usize> = Vec::new();
        let mut socp_cols: Vec<usize> = Vec::new();
        let mut socp_vals = Vec::new();
        let mut h = Vec::new();
        let mut socp_row_counter = 0;

        let sin_theta = self.tvc_range_rad.sin();

        for k in 0..self.N {
            let w_offset = idx_w + k;
            let u_offset = idx_u + 3 * k;
            let sigma_offset = idx_sigma + k;

            // One change I did make is to make the lower bound only a linear Taylor approximation instead of a quadratic one.
            let z_0 = (m0 + self.alpha * self.upper_thrust_bound * self.delta_t * (k as f64)).ln();
            let z_0_exp = z_0.exp();
            let sigma_min_coefficient = self.lower_thrust_bound * z_0_exp;
            let sigma_min_h_val = sigma_min_coefficient * (1.0 + z_0);
            let sigma_max_coefficient = self.upper_thrust_bound * z_0_exp;
            let sigma_max_h_val = sigma_max_coefficient * (1.0 + z_0);

            // thrust bouunds
            cones.push(SupportedConeT::SecondOrderConeT(1));
            socp_rows.push(socp_row_counter as usize); socp_cols.push(sigma_offset as usize); socp_vals.push(1.0); // sigma_k - sigma_min
            socp_rows.push(socp_row_counter as usize); socp_cols.push(w_offset as usize); vals.push(-sigma_min_coefficient);
            h.push(sigma_min_h_val);
            socp_row_counter += 1;
            
            cones.push(SupportedConeT::SecondOrderConeT(1));
            socp_rows.push(socp_row_counter as usize); socp_cols.push(sigma_offset as usize); socp_vals.push(-1.0); // -sigma_k + sigma_max
            socp_rows.push(socp_row_counter as usize); socp_cols.push(w_offset as usize); vals.push(sigma_max_coefficient);
            h.push(-sigma_max_h_val);
            socp_row_counter += 1;

            // ---------- Thrust magnitude SOC: ||u_k|| <= sigma_k ----------
            socp_rows.push((socp_row_counter as usize) + 0); socp_cols.push(sigma_offset as usize); socp_vals.push(-1.0); // -sigma_k
            socp_rows.push((socp_row_counter as usize) + 1); socp_cols.push(u_offset as usize);     socp_vals.push(-1.0); // -u0
            socp_rows.push((socp_row_counter as usize) + 2); socp_cols.push((u_offset as usize) + 1);   socp_vals.push(-1.0); // -u1
            socp_rows.push((socp_row_counter as usize) + 3); socp_cols.push((u_offset as usize) + 2);   socp_vals.push(-1.0); // -u2
            cones.push(SupportedConeT::SecondOrderConeT(4));
            h.extend_from_slice(&[0.0; 4]);
            socp_row_counter += 4;

            // ---------- Thrust pointing SOC: ||u_k - sigma_k*d|| <= sigma_k*sin(theta) ----------
            socp_rows.push((socp_row_counter as usize) + 0); socp_cols.push(sigma_offset as usize); socp_vals.push(0.0); // top of SOC
            socp_rows.push((socp_row_counter as usize) + 1); socp_cols.push(u_offset as usize);     socp_vals.push(1.0);
            socp_rows.push((socp_row_counter as usize) + 2); socp_cols.push((u_offset as usize) + 1);   socp_vals.push(1.0);
            socp_rows.push((socp_row_counter as usize) + 3); socp_cols.push((u_offset as usize) + 2);   socp_vals.push(1.0);
            cones.push(SupportedConeT::SecondOrderConeT(4));
            // h = [sigma*sin(theta), sigma*d0, sigma*d1, sigma*d2]
            h.push(sin_theta);
            h.push(-self.pointing_direction[0]); // Clarabel enforces s - h ∈ SOC
            h.push(-self.pointing_direction[1]);
            h.push(-self.pointing_direction[2]);
            socp_row_counter += 4;

            // TODO: add glide slope constraints (ts gpt rn)
            // if (self.use_glide_slope) {
            //     // ---------- Glide slope SOC: ||x_k[:2]|| <= x_k[2] / glide_slope ----------
            //     let x_offset = idx_x + 3 * k;
            //     socp_rows.push((socp_row_counter as usize) + 0); socp_cols.push((x_offset as usize) + 2); socp_vals.push(1.0); // x_k[2]
            //     socp_rows.push((socp_row_counter as usize) + 1); socp_cols.push((x_offset as usize) + 0); socp_vals.push(-1.0); // -x_k[0]
            //     socp_rows.push((socp_row_counter as usize) + 2); socp_cols.push((x_offset as usize) + 1); socp_vals.push(-1.0); // -x_k[1]
            //     cones.push(SupportedConeT::SecondOrderConeT(3));
            //     h.push(0.0);
            //     h.push(0.0);
            //     h.push(0.0);
            //     socp_row_counter += 3;
            // }
        }

        // ---------- End mass SOC: w_N >= log(m_dry) ----------
        socp_rows.push(socp_row_counter as usize);
        socp_cols.push((idx_w + self.N) as usize);
        socp_vals.push(1.0);
        cones.push(SupportedConeT::SecondOrderConeT(1));
        h.push(self.dry_mass.ln());
        socp_row_counter += 1;

        // let G = Self::csc_from_triplets(socp_row_counter as usize, n_vars as usize, &socp_rows, &socp_cols, &socp_vals);
        
        /*
        Objective: minimize sum of sigma_k * delta_t
        */

        let mut c = vec![0.0; n_vars as usize];
        // sigma variables start at idx_sigma
        for k in 0..self.N {
            c[(idx_sigma + k) as usize] = self.delta_t; // weight = delta_t for discretization
        }

        // Create a zero matrix for P (linear problem)
        let P = CscMatrix::zeros((n_vars as usize, n_vars as usize)); // note the tuple

        let settings = DefaultSettings::default();

        // Build the solver
        let mut solver = DefaultSolver::new(
            &P,       // P matrix (or zero for linear problem)
            &c,       // cost vector
            &A,       // equality constraint matrix
            &b,       // equality RHS
            &cones,   // vector of SupportedConeT
            settings, // settings
        );

        solver.solve();

        
    if solver.solution.status == SolverStatus::Solved || solver.solution.status == SolverStatus::AlmostSolved {
        Ok(solver.solution) // <- no semicolon here
    } else {
        Err(format!("Solver failed with status {:?}", solver.solution.status)) // <- also no semicolon here
    }
    }

    pub fn csc_from_triplets(nrows: usize, ncols: usize, rows: &[usize], cols: &[usize], vals: &[f64]) -> CscMatrix<f64> {
        let mut triplets: Vec<(usize, usize, f64)> = rows.iter().zip(cols).zip(vals)
            .map(|((r, c), v)| (*r, *c, *v))
            .collect();
        triplets.sort_by_key(|&(r, c, _)| (c, r));

        let mut colptr = vec![0; ncols + 1];
        let mut rowval = Vec::with_capacity(vals.len());
        let mut nzval = Vec::with_capacity(vals.len());

        let mut current_col = 0;
        let mut count = 0;

        for &(r, c, v) in &triplets {
            while current_col < c {
                colptr[current_col + 1] = count;
                current_col += 1;
            }
            rowval.push(r);
            nzval.push(v);
            count += 1;
        }
        while current_col < ncols {
            colptr[current_col + 1] = count;
            current_col += 1;
        }

        CscMatrix::new(nrows, ncols, colptr, rowval, nzval)
    }

    fn extract_result(&self, result: &DefaultSolution<f64>) -> TrajectoryResult {
        let x = &result.x;

        let idx_x = 0;
        let idx_v = idx_x + 3 * (self.N + 1);
        let idx_w = idx_v + 3 * (self.N + 1);
        let idx_u = idx_w + (self.N + 1);
        let idx_sigma = idx_u + 3 * self.N;

        let mut positions = Vec::new();
        let mut velocities = Vec::new();
        let mut masses = Vec::new();
        let mut thrusts = Vec::new();
        let mut sigmas = Vec::new();

        // positions (x_k)
        for k in 0..=self.N {
            let start: usize = (idx_x + 3 * k) as usize;
            positions.push([
                x[start],
                x[start + 1],
                x[start + 2],
            ]);
        }

        // velocities (v_k)
        for k in 0..=self.N {
            let start: usize = (idx_v + 3 * k) as usize;
            velocities.push([
                x[start],
                x[start + 1],
                x[start + 2],
            ]);
        }

        // log-masses (w_k)
        for k in 0..=self.N {
            masses.push(x[(idx_w + k) as usize].exp()); // convert ln(m) → m
        }

        // thrusts (u_k)
        for k in 0..self.N {
            let start: usize = (idx_u + 3 * k) as usize;
            thrusts.push([
                x[start],
                x[start + 1],
                x[start + 2],
            ]);
        }

        // sigmas (σ_k)
        for k in 0..self.N {
            sigmas.push(x[(idx_sigma + k) as usize]);
        }

        TrajectoryResult {
            positions,
            velocities,
            masses,
            thrusts,
            sigmas,
        }
    }

    pub fn solve(&mut self) -> Option<TrajectoryResult> {
        let vel_norm = self.initial_velocity.iter()
            .map(|v| v * v)
            .sum::<f64>()
            .sqrt();
        
        let t_min = self.dry_mass * vel_norm / self.upper_thrust_bound;
        let t_max = self.fuel_mass / (self.alpha * self.lower_thrust_bound);

        let n_min = (t_min / self.delta_t).ceil() as i64;
        let n_max = (t_max / self.delta_t).floor() as i64;

        let mut result: Option<DefaultSolution<f64>> = None;

        for k in n_min..n_max {
            println!("Solving step {}...", k);
            self.N = k;
            match self.solve_at_current_time() {
                Ok(sol) => {
                        println!("✅ Step {} converged successfully.", k);
                        result = Some(sol);
                }
                Err(_) => {
                    println!("❌ Step {} failed, moving on.", k);
                    return None;
                }
            }
        }

        let unpacked_result = match result {
            Some(ref r) => r,
            None => return None,
        };
        let traj_result = self.extract_result(&unpacked_result);
        println!("Final position: {:?}", traj_result.positions.last().unwrap());
        println!("Final velocity: {:?}", traj_result.velocities.last().unwrap());
        println!("Final mass: {:.3} kg", traj_result.masses.last().unwrap());
        println!("Final thrust: {:?}", traj_result.thrusts.last().unwrap());

        Some(traj_result)
    }
}
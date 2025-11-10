use clarabel::algebra::*;
use clarabel::solver::*;

pub struct lossless_solver {
    landing_point: [f64; 3],
    initial_position: [f64; 3],
    initial_velocity: [f64; 3],
    glide_slope: f64,
    use_glide_slope: bool,
    max_velocity: f64,
    dry_mass: f64,
    fuel_mass: f64,
    alpha: f64,
    lower_thrust_bound: f64,
    upper_thrust_bound: f64,
    tvc_range_rad: f64,
    delta_t: f64,
    pointing_direction: [f64; 3],
    N: i64,
}

impl Default for lossless_solver {
    fn default() -> Self {
        lossless_solver {
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

impl lossless_solver {
    const gravity: [f64; 3] = [0.0, 0.0, -9.81];

    pub fn new() -> Self {
        lossless_solver::default()
    }

    pub fn solve_at_time(&self, N: i64) {
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

        let mut rows = Vec::new();
        let mut cols = Vec::new();
        let mut vals = Vec::new();
        let mut b = Vec::new();
        let mut row_counter = 0;

        
        // Setting initial conditions

        // x0 = initial_position
        for i in 0..3 {
            rows.push(row_counter + i);
            cols.push(idx_x + i);
            vals.push(1.0);
            b.push(self.initial_position[i]);
        }
        row_counter += 3;

        // v0 = initial_velocity
        for i in 0..3 {
            rows.push(row_counter + i);
            cols.push(idx_v + i); // v0 indices
            vals.push(1.0);
            b.push(self.initial_velocity[i]);
        }
        row_counter += 3;

        // w0 = ln(m0)
        rows.push(row_counter);
        cols.push(idx_w);
        vals.push(1.0);
        b.push(m0.ln());
        row_counter += 1;

        
        // vehicle kinematics constraints
        for k in 0..N {
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
                rows.push(row_counter + i);
                cols.push(next_x_offset + i);
                vals.push(1.0);

                rows.push(row_counter + i);
                cols.push(curr_x_offset + i);
                vals.push(-1.0);

                rows.push(row_counter + i);
                cols.push(curr_v_offset + i);
                vals.push(-self.delta_t);

                b.push(0.0);
            }
            row_counter += 3;

            // v_{k+1} - v_k - dt * u_k = dt * g
            for i in 0..3 {
                rows.push(row_counter + i);
                cols.push(next_v_offset + i);
                vals.push(1.0);

                rows.push(row_counter + i);
                cols.push(curr_v_offset + i);
                vals.push(-1.0);

                rows.push(row_counter + i);
                cols.push(curr_u_ofset + i);
                vals.push(-self.delta_t);

                // b = dt * g
                b.push(self.delta_t * gravity[i]);
            }
            row_counter += 3;

            // w_{k+1} - w_k + dt * alpha * sigma_k = 0
            rows.push(row_counter);
            cols.push(next_w_offset);
            vals.push(1.0);

            rows.push(row_counter);
            cols.push(curr_w_offset);
            vals.push(-1.0);

            rows.push(row_counter);
            cols.push(curr_sigma_offset);
            vals.push(self.delta_t * self.alpha);

            b.push(0.0);
            row_counter += 1;
        }

        // End constraints

        
        // x_N = landing_point
        for i in 0..3 {
            rows.push(row_counter);
            cols.push(idx_x + 3 * N + i);
            vals.push(1.0);
            b.push(self.landing_point[i]);
            row_counter += 1;
        }

        // v_N = 0
        for i in 0..3 {
            rows.push(row_counter);
            cols.push(idx_v + 3 * N + i);
            vals.push(1.0);
            b.push(0.0);
            row_counter += 1;
        }

        let A = CscMatrix::from_triplets((row_counter, n_vars), &rows, &cols, &vals);

        /*
        SOCP constraints
        */
        let mut cones = Vec::new();
        let mut socp_rows = Vec::new();
        let mut socp_cols = Vec::new();
        let mut socp_vals = Vec::new();
        let mut h = Vec::new();
        let mut socp_row_counter = 0;

        let sin_theta = tvc_range.sin();

        for k in 0..N {
            let w_offset = idx_w + k;
            let u_offset = idx_u + 3 * k;
            let sigma_offset = idx_sigma + k;

            // One change I did make is to make the lower bound only a linear Taylor approximation instead of a quadratic one.
            let z_0 = (m0 + self.alpha * self.upper_thrust_bound * self.delta_t * k).ln();
            let z_0_exp = z_0.exp();
            let sigma_min_coefficient = self.lower_thrust_bound * z_0_exp;
            let sigma_min_h_val = sigma_min_coefficient * (1 + z_0);
            let sigma_max_coefficient = self.upper_thrust_bound * z_0_exp;
            let sigma_max_h_val = sigma_max_coefficient * (1 + z_0);

            // thrust bouunds
            cones.push(SupportedCone::SecondOrderCone { dim: 1 });
            socp_rows.push(socp_row_counter); socp_cols.push(sigma_offset); socp_vals.push(1.0); // sigma_k - sigma_min
            socp_rows.push(socp_row_counter); socp_cols.push(w_offset); vals.push(-sigma_min_coefficient);
            h.push(sigma_min_h_val);
            socp_row_counter += 1;
            
            cones.push(SupportedCone::SecondOrderCone { dim: 1 });
            socp_rows.push(socp_row_counter); socp_cols.push(sigma_offset); socp_vals.push(-1.0); // -sigma_k + sigma_max
            socp_rows.push(socp_row_counter); socp_cols.push(w_offset); vals.push(sigma_max_coefficient);
            h.push(-sigma_max_h_val);
            socp_row_counter += 1;

            // ---------- Thrust magnitude SOC: ||u_k|| <= sigma_k ----------
            socp_rows.push(socp_row_counter + 0); socp_cols.push(sigma_offset); socp_vals.push(-1.0); // -sigma_k
            socp_rows.push(socp_row_counter + 1); socp_cols.push(u_offset);     socp_vals.push(-1.0); // -u0
            socp_rows.push(socp_row_counter + 2); socp_cols.push(u_offset+1);   socp_vals.push(-1.0); // -u1
            socp_rows.push(socp_row_counter + 3); socp_cols.push(u_offset+2);   socp_vals.push(-1.0); // -u2
            cones.push(SupportedCone::SecondOrderCone { dim: 4 });
            h.extend_from_slice(&[0.0; 4]);
            socp_row_counter += 4;

            // ---------- Thrust pointing SOC: ||u_k - sigma_k*d|| <= sigma_k*sin(theta) ----------
            socp_rows.push(socp_row_counter + 0); socp_cols.push(sigma_offset); socp_vals.push(0.0); // top of SOC
            socp_rows.push(socp_row_counter + 1); socp_cols.push(u_offset);     socp_vals.push(1.0);
            socp_rows.push(socp_row_counter + 2); socp_cols.push(u_offset+1);   socp_vals.push(1.0);
            socp_rows.push(socp_row_counter + 3); socp_cols.push(u_offset+2);   socp_vals.push(1.0);
            cones.push(SupportedCone::SecondOrderCone { dim: 4 });
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
            //     socp_rows.push(socp_row_counter + 0); socp_cols.push(x_offset + 2); socp_vals.push(1.0); // x_k[2]
            //     socp_rows.push(socp_row_counter + 1); socp_cols.push(x_offset + 0); socp_vals.push(-1.0); // -x_k[0]
            //     socp_rows.push(socp_row_counter + 2); socp_cols.push(x_offset + 1); socp_vals.push(-1.0); // -x_k[1]
            //     cones.push(SupportedCone::SecondOrderCone { dim: 3 });
            //     h.push(0.0);
            //     h.push(0.0);
            //     h.push(0.0);
            //     socp_row_counter += 3;
            // }
        }

        // ---------- End mass SOC: w_N >= log(m_dry) ----------
        socp_rows.push(socp_row_counter);
        socp_cols.push(idx_w + N);
        socp_vals.push(1.0);
        cones.push(SupportedCone::SecondOrderCone { dim: 1 });
        h.push(self.dry_mass.ln());
        socp_row_counter += 1;

        let G = CscMatrix::from_triplets((socp_row_counter, n_vars), &socp_rows, &socp_cols, &socp_vals);

        /*
        Objective: minimize sum of sigma_k * delta_t
        */

        let mut c = vec![0.0; n_vars];
        // sigma variables start at idx_sigma
        for k in 0..N {
            c[idx_sigma + k] = delta_t; // weight = delta_t for discretization
        }

        // Create the problem struct
        let mut problem = Problem {
            c: &c,
            a_eq: Some(&A),
            b_eq: Some(&b),
            g: Some(&G),
            h: Some(&h),
            cones: Some(&cones),
        };

        let options = SolverOptions::default(); // can tweak tolerances here

        match solver::solve(&problem, &options) {
            Ok(sol) => Ok(sol),
            Err(err) => {
                eprintln!("Clarabel failed on timestep {}: {:?}", k, err);
                Err(err)
            }
        }
    }

    pub fn solve(&self) {
        let vel_norm = self.initial_velocity.iter()
            .map(|v| v * v)
            .sum::<f64>()
            .sqrt();
        
        let t_min = self.dry_mass * vel_norm / self.upper_thrust_bound;
        let t_max = self.fuel_mass / (self.alpha * self.lower_thrust_bound);

        let n_min = (t_min / self.delta_t).ceil() as i64;
        let n_max = (t_max / self.delta_t).floor() as i64;

        let mut results = Vec::new();

        for k in n_min..n_max {
            println!("Solving step {}...", k);

            match self.solve_timestep(k) {
                Ok(sol) => {
                    // Check if result is “satisfactory”
                    if self.is_solution_satisfactory(&sol) {
                        println!("✅ Step {} converged successfully.", k);
                        results.push(sol);
                    } else {
                        println!("⚠️ Step {} converged but unsatisfactory, skipping.", k);
                    }
                }
                Err(_) => {
                    println!("❌ Step {} failed, moving on.", k);
                    continue;
                }
            }
        }

        results
    }
}
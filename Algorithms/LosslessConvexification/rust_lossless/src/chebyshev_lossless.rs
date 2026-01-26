use clarabel::algebra::*;
use clarabel::solver::*;

use crate::lossless::{TrajectoryResult};

const GRAVITY: [f64; 3] = [0.0, 0.0, -9.81];

pub struct ChebyshevLosslessSolver {
    pub landing_point: [f64; 3],
    pub initial_position: [f64; 3],
    pub initial_velocity: [f64; 3],
    pub glide_slope: f64,
    pub use_glide_slope: bool,
    pub max_velocity: f64,
    pub dry_mass: f64,
    pub fuel_mass: f64,
    pub alpha: f64, // fuel consumption rate
    pub lower_thrust_bound: f64,
    pub upper_thrust_bound: f64,
    pub tvc_range_rad: f64,
    pub coarse_delta_t: f64,
    pub fine_delta_t: f64,
    pub delta_t: f64,
    pub pointing_direction: [f64; 3],
    // TODO: We are moving away from discretization into Chebyshev approximation:
    // We should track the order of our polynomial for our solver instead.
    pub N: i64, // Number of discretization intervals
}

impl Default for ChebyshevLosslessSolver {
    fn default() -> Self {
        ChebyshevLosslessSolver {
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
            coarse_delta_t: 1.0,
            fine_delta_t: 1.0,
            delta_t: 1.0,
            pointing_direction: [0.0, 0.0, 1.0],
            N: 1
        }
    }
}

impl ChebyshevLosslessSolver {
    pub fn new() -> Self {
        ChebyshevLosslessSolver::default()
    }

    /*
    TODO (Kinda): Feel free to add helper functions and structs to facilitate our implementation of the Chebyshev polynomials
        - For good programming practice, you should come back here to create any necessary helper functions.
        - Candidates for a helper function would be evaluating a polynomial, computing chebyshev nodes (optimal collocation points [-1,1])
          and creating different matricies we may need in our calculations. Whatever the implementation ends up calling for.
    */

    /* 
    Chebyshev–Gauss–Lobatto (CGL) nodes on [-1, 1]
    tau_i = cos(pi * i / N),  i = 0..=N
    
    Returns N+1 nodes in reverse of natural cosine order:
       i=0   ->  -1
       i=N   ->  1
    This is so that 0 maps to start (t=0) and N maps to end (t=T) in time domain.
    */
    pub fn cgl_nodes(n: usize) -> Vec<f64> {
        // By convention, N must be >= 1 to have endpoints and interior points.
        assert!(n >= 1, "CGL nodes require N >= 1");

        let nf = n as f64;
        let mut tau = Vec::with_capacity(n + 1);
        for i in 0..=n {
            // We flip i to n-i to ensuring that tau[0] = -1 and tau[n] = 1
            let theta = std::f64::consts::PI * ((n - i) as f64) / nf;
            tau.push(theta.cos());
        }
        tau
    }

    /*
     */
    pub fn cgl_diff_matrix(n: usize) -> Vec<Vec<f64>> {
        assert!(n >= 1, "CGL differentiation matrix requires N >= 1");
        let tau = Self::cgl_nodes(n);

        // Build c endpoint weights (double the weight of other nodes)
        let mut c = vec![1.0_f64; n + 1];
        c[0] = 2.0;
        c[n] = 2.0;

        // alpha matrix with alternating nodes
        let mut alpha = vec![0.0_f64; n + 1];
        for i in 0..=n {
            let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
            alpha[i] = c[i] * sign;
        }

        // Start with zeroed differentiation matrix, init for all i,j
        let mut d = vec![vec![0.0_f64; n + 1]; n + 1];

        // We fill off-diagonal entries with weights
        // essentially "how much does node j contribute to derivative at node i"
        for i in 0..=n {
            for j in 0..=n {
                if i != j { // don't divide by zero
                    d[i][j] = (alpha[i] / alpha[j]) / (tau[i] - tau[j]);
                }
            }
        }

        // Now fill diagonal entries with special cases
        // start and end are hardcoded, rest use formula
        let nf = n as f64;
        d[0][0] = -(2.0 * nf * nf + 1.0) / 6.0;
        d[n][n] =  (2.0 * nf * nf + 1.0) / 6.0;

        for i in 1..n {
            let ti = tau[i];
            d[i][i] = -ti / (2.0 * (1.0 - ti * ti));
        }

        // QOL / robustness (in case of weird floating point behavior)
        // Essentially a double check to ensure rows sum to zero
        // for i in 0..=n {
        //     let mut s = 0.0;
        //     for j in 0..=n {
        //         if i != j { 
        //             s += d[i][j]; 
        //         }
        //     }
        //     d[i][i] = -s;
        // }

        d
    }


    // pub fn solve_at_current_time(&mut self) -> Result<clarabel::solver::DefaultSolution<f64>, String> {
    pub fn solve_at_current_time(&mut self) -> Option<TrajectoryResult> {
        let m0 = self.dry_mass + self.fuel_mass;
        
        /* 
        TODO: We are switching from discrete timesteps to Chebyshev approximation.
            - We don't use the variables at the end of each timestep, but instead use the Coefficients of the polynomial
        */
        // Number of optimization variables
        let n_vars = 3 * (self.N + 1) // x
            + 3 * (self.N + 1) // v
            + (self.N + 1) // w
            + 3 * self.N // u
            + self.N; // sigma (thrust slack variable)

        // Index of each variable in the resulting vector
        let idx_x = 0;
        let idx_v = idx_x + 3 * (self.N + 1);
        let idx_w = idx_v + 3 * (self.N + 1);
        let idx_u = idx_w + (self.N + 1);
        let idx_sigma = idx_u + 3 * self.N;

        /*
        Equality constraints (Ax = b)
        Will be saved into (row, col, value) format
        */

        let mut cones = Vec::new();
        let mut rows: Vec<usize> = Vec::new();
        let mut cols: Vec<usize> = Vec::new();
        let mut vals = Vec::new();
        let mut b = Vec::new();
        let mut row_counter = 0;

        /*
        TODO: Initial conditions are based on our discretization approach. We ideally want to transfer this into a polynomial evaluation.
            - instead of setting x0, v0, w0 directly, can we just make sure wwhen we evaluate our polynomial at t = -1 (start time), we get accurate initial conditions.
        */
        // Setting initial conditions

        // x0 = initial_position
        for i in 0..3 {
            rows.push((row_counter + i) as usize);
            cols.push((idx_x + i) as usize);
            vals.push(1.0);
            b.push(self.initial_position[i as usize]);
        }
        row_counter += 3;
        cones.push(SupportedConeT::ZeroConeT(3));

        // v0 = initial_velocity
        for i in 0..3 {
            rows.push((row_counter + i) as usize);
            cols.push((idx_v + i) as usize); // v0 indices
            vals.push(1.0);
            b.push(self.initial_velocity[i as usize]);
        }
        row_counter += 3;
        cones.push(SupportedConeT::ZeroConeT(3));

        // w0 = ln(m0)
        rows.push(row_counter as usize);
        cols.push(idx_w as usize);
        vals.push(1.0);
        b.push(m0.ln());
        row_counter += 1;
        cones.push(SupportedConeT::ZeroConeT(1));

        
        /*
        TODO: We are switching from discretization to a numerical analysis method known as Pseudospectral collocation
            - TLDR we will use differentiation to enforce dynamics
            - for example we have x(t) represented by Chebyshev coefficients c_x, so dx/dt becomes c_x * (differentiation matrix)   
        */
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

            // Position Dynamics: x_{k+1} - x_k - dt * v_k = 0
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
            cones.push(SupportedConeT::ZeroConeT(3));

            // Velocity Dynamics: v_{k+1} - v_k - dt * u_k = dt * g
            for i in 0..3 {
                rows.push((row_counter + i) as usize);
                cols.push((next_v_offset + i) as usize);
                vals.push(1.0);

                rows.push((row_counter + i) as usize);
                cols.push((curr_v_offset + i) as usize);
                vals.push(-1.0);

                rows.push((row_counter + i) as usize);
                cols.push((curr_u_offset + i) as usize);
                vals.push(-self.delta_t);

                // b = dt * g
                b.push(self.delta_t * GRAVITY[i as usize]);
            }
            row_counter += 3;
            cones.push(SupportedConeT::ZeroConeT(3));

            // Mass dynamics: w_{k+1} - w_k + dt * alpha * sigma_k = 0
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
            cones.push(SupportedConeT::ZeroConeT(1));
        }

        /*
        TODO: We need similar changes to the initial conditions, but instead evaluated for the end of the trajectory.
        */
        // End constraints
        // x_N = landing_point
        for i in 0..3 {
            rows.push(row_counter as usize);
            cols.push((idx_x + 3 * self.N + i) as usize);
            vals.push(1.0);
            b.push(self.landing_point[i as usize]);
            row_counter += 1;
        }
        cones.push(SupportedConeT::ZeroConeT(3));

        // v_N = 0 (landing velocity)
        for i in 0..3 {
            rows.push(row_counter as usize);
            cols.push((idx_v + 3 * self.N + i) as usize);
            vals.push(1.0);
            b.push(0.0);
            row_counter += 1;
        }
        cones.push(SupportedConeT::ZeroConeT(3));

        /*
        TODO: We still use the Second Order Cone Problem (SOCP) constraints and apply them at discrete points
            - We need to evaluate the polynomials to get values at those points.
            - instead of using variable u_k we should evaluate the polynomial at k  
        */
        /*
        SOCP constraints
        */
        let sin_theta = self.tvc_range_rad.sin();

        for k in 0..self.N {
            let v_offset = idx_v + 3 * k;
            let w_offset = idx_w + k;
            let u_offset = idx_u + 3 * k;
            let sigma_offset = idx_sigma + k;

            // One change I did make is to make the lower bound only a linear Taylor approximation instead of a quadratic one.
            let z_0 = (m0 + self.alpha * self.upper_thrust_bound * self.delta_t * (k as f64)).ln();
            let exp_neg_z_0 = (-z_0).exp();
            let sigma_min_coefficient = self.lower_thrust_bound * exp_neg_z_0;
            let sigma_min_h_val = sigma_min_coefficient * (1.0 + z_0);
            let sigma_max_coefficient = self.upper_thrust_bound * exp_neg_z_0;
            let sigma_max_h_val = sigma_max_coefficient * (1.0 + z_0);

            // thrust bounds
            // min bound
            cones.push(SupportedConeT::NonnegativeConeT(1));
            rows.push(row_counter as usize); cols.push(sigma_offset as usize); vals.push(-1.0); // sigma_k - sigma_min
            rows.push(row_counter as usize); cols.push(w_offset as usize); vals.push(-sigma_min_coefficient);
            b.push(-sigma_min_h_val);
            row_counter += 1;
            
            // max bound
            cones.push(SupportedConeT::NonnegativeConeT(1));
            rows.push(row_counter as usize); cols.push(sigma_offset as usize); vals.push(1.0); // -sigma_k + sigma_max
            rows.push(row_counter as usize); cols.push(w_offset as usize); vals.push(sigma_max_coefficient);
            b.push(sigma_max_h_val);
            row_counter += 1;

            // ---------- Thrust magnitude SOC: ||u_k|| <= sigma_k ----------
            rows.push((row_counter as usize) + 0); cols.push(sigma_offset as usize); vals.push(-1.0); // -sigma_k
            rows.push((row_counter as usize) + 1); cols.push(u_offset as usize);     vals.push(-1.0); // -u0
            rows.push((row_counter as usize) + 2); cols.push((u_offset as usize) + 1);   vals.push(-1.0); // -u1
            rows.push((row_counter as usize) + 3); cols.push((u_offset as usize) + 2);   vals.push(-1.0); // -u2
            cones.push(SupportedConeT::SecondOrderConeT(4));
            b.push(0.0);
            b.push(0.0);
            b.push(0.0);
            b.push(0.0);
            row_counter += 4;

            // ---------- Thrust pointing SOC: ||u_k - sigma_k*d|| <= sigma_k*sin(theta) ----------
            rows.push((row_counter as usize) + 0); cols.push(sigma_offset as usize); vals.push(-sin_theta); // top of SOC
            rows.push((row_counter as usize) + 1); cols.push(u_offset as usize);     vals.push(1.0);
            rows.push((row_counter as usize) + 1); cols.push(sigma_offset as usize);     vals.push(-self.pointing_direction[0]);
            rows.push((row_counter as usize) + 2); cols.push((u_offset as usize) + 1);   vals.push(1.0);
            rows.push((row_counter as usize) + 2); cols.push(sigma_offset as usize);     vals.push(-self.pointing_direction[1]);
            rows.push((row_counter as usize) + 3); cols.push((u_offset as usize) + 2);   vals.push(1.0);
            rows.push((row_counter as usize) + 3); cols.push(sigma_offset as usize);     vals.push(-self.pointing_direction[2]);
            cones.push(SupportedConeT::SecondOrderConeT(4));
            b.push(0.0);
            b.push(0.0);
            b.push(0.0);
            b.push(0.0);
            row_counter += 4;

            if self.use_glide_slope {
                // ---------- Glide slope SOC: ||x_k[:2]|| <= x_k[2] / tan(glide_slope) ----------
                let x_offset = idx_x + 3 * k;
                rows.push((row_counter as usize) + 0); cols.push((x_offset as usize) + 2); vals.push(-1.0 / self.glide_slope.tan()); // x_k[2]
                rows.push((row_counter as usize) + 1); cols.push((x_offset as usize) + 0); vals.push(1.0); // -x_k[0]
                rows.push((row_counter as usize) + 2); cols.push((x_offset as usize) + 1); vals.push(1.0); // -x_k[1]
                cones.push(SupportedConeT::SecondOrderConeT(3));
                b.push(0.0);
                b.push(self.landing_point[0]);
                b.push(self.landing_point[1]);
                row_counter += 3;
            }            
            // ---------- Max velocity SOC: ||v_k|| <= max_velocity ----------
            rows.push((row_counter as usize) + 0); cols.push(v_offset as usize);     vals.push(0.0); // -v0
            rows.push((row_counter as usize) + 1); cols.push(v_offset as usize);     vals.push(-1.0); // -v0
            rows.push((row_counter as usize) + 2); cols.push((v_offset as usize) + 1);   vals.push(-1.0); // -v1
            rows.push((row_counter as usize) + 3); cols.push((v_offset as usize) + 2);   vals.push(-1.0); // -v2
            cones.push(SupportedConeT::SecondOrderConeT(4));
            b.push(self.max_velocity);
            b.push(0.0);
            b.push(0.0);
            b.push(0.0);
            row_counter += 4;

        }

        /*
        TODO: Similar to terminal position and velocity constraints, evaluate w polynomial at the final time.
        */
        // ---------- End mass SOC: w_N >= log(m_dry) ----------
        rows.push(row_counter as usize);
        cols.push((idx_w + self.N) as usize);
        vals.push(-1.0);
        cones.push(SupportedConeT::NonnegativeConeT(1));
        b.push(-self.dry_mass.ln());
        row_counter += 1;
        
        let A = Self::csc_from_triplets(row_counter as usize, n_vars as usize, &rows, &cols, &vals);

        /*
        TODO: For Chebyshev polynomials, the objective function changes to an integration over the normalized time domain [-1, 1]
        */
        /*
        Objective: minimize sum of sigma_k * delta_t (fuel consumption)
        */
        let mut c = vec![0.0; n_vars as usize];
        // sigma variables start at idx_sigma
        for k in 0..self.N {
            c[(idx_sigma + k) as usize] = self.delta_t; // weight = delta_t for discretization  
        }

        let prow = Vec::new();
        let pcol = Vec::new();
        let pval = Vec::new();
        let P = Self::csc_from_triplets(n_vars as usize, n_vars as usize, &prow, &pcol, &pval);


        let mut settings = DefaultSettings::default();
        settings.verbose = true;
        // settings.verbose = false;

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
            let traj_result = self.extract_result(&solver.solution);
            Some(traj_result)
            
        } else {
            println!("{}", format!("Solver failed with status {:?}", solver.solution.status));
            return None
        }
    }

    /*
    Helper function that converts the constraint triplets (row, col, value) to a Compressed Sparse Column (CSC) format that is utilized by our solver.
    Should remain unchanged for Chebyshev implementation
    */
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

    /*
    Helper function that extracts the trajectory result from the solver solution.

    TODO: With our current implementation of Chebyshev, the solution contains coefficients of our polynomials not values at nodes.
        -   To get a trajectory we need to:
            1) extract the coefficients of our polynomials for (x ,v, w, u)
            2) Define a set of evaluation points (Maybe like 100)
            3) Evaluate the polynomials at the points
        - Alternatively We could store the coefficients themselves to later re-evaluate the trajectory for graphing
            - This would mean changing the TrajectoryResult struct as well as how main.rs parses our results
    */
    fn extract_result(&self, result: &DefaultSolution<f64>) -> TrajectoryResult {

        // (Step 1) extract coefficents (idk yet gang shall return after diving into papers)

        let idx_x = 0;
        let idx_v = idx_x + 3 * (self.N + 1);
        let idx_w = idx_v + 3 * (self.N + 1);
        let idx_u = idx_w + (self.N + 1);
        let idx_sigma = idx_u + 3 * self.N;

        // Create dummy lambdas (closures) to evaluate each quantity at a normalized
        // time tau in [0,1]. These are simple piecewise-linear & built from result.x. 
        // They are dummies for now and should be replaced with Chebyshev polynomial evaluations.

        let xvec = &result.x;
        let n_nodes = (self.N + 1) as usize;
        let n_intervals = (self.N) as usize; // for u and sigma

        let position_fn = move |tau: f64| -> [f64; 3] {
            let tau = if tau.is_nan() { 0.0 } else if tau < 0.0 { 0.0 } else if tau > 1.0 { 1.0 } else { tau };
            if n_nodes == 0 { return [0.0; 3]; }
            if n_nodes == 1 {
                let s = idx_x as usize;
                return [xvec[s], xvec[s + 1], xvec[s + 2]];
            }
            let t = tau * ((n_nodes - 1) as f64);
            let k = t.floor() as usize;
            let alpha = if k >= n_nodes - 1 { 1.0 } else { t - (k as f64) };
            let s0 = (idx_x + 3 * (k as i64)) as usize;
            let s1 = (idx_x + 3 * (if k >= n_nodes - 1 { n_nodes - 1 } else { k + 1 } as i64)) as usize;
            [xvec[s0] * (1.0 - alpha) + xvec[s1] * alpha,
             xvec[s0 + 1] * (1.0 - alpha) + xvec[s1 + 1] * alpha,
             xvec[s0 + 2] * (1.0 - alpha) + xvec[s1 + 2] * alpha]
        };

        let velocity_fn = move |tau: f64| -> [f64; 3] {
            let tau = if tau.is_nan() { 0.0 } else if tau < 0.0 { 0.0 } else if tau > 1.0 { 1.0 } else { tau };
            if n_nodes == 0 { return [0.0; 3]; }
            if n_nodes == 1 {
                let s = idx_v as usize;
                return [xvec[s], xvec[s + 1], xvec[s + 2]];
            }
            let t = tau * ((n_nodes - 1) as f64);
            let k = t.floor() as usize;
            let alpha = if k >= n_nodes - 1 { 1.0 } else { t - (k as f64) };
            let s0 = (idx_v + 3 * (k as i64)) as usize;
            let s1 = (idx_v + 3 * (if k >= n_nodes - 1 { n_nodes - 1 } else { k + 1 } as i64)) as usize;
            [xvec[s0] * (1.0 - alpha) + xvec[s1] * alpha,
             xvec[s0 + 1] * (1.0 - alpha) + xvec[s1 + 1] * alpha,
             xvec[s0 + 2] * (1.0 - alpha) + xvec[s1 + 2] * alpha]
        };

        let mass_fn = move |tau: f64| -> f64 {
            let tau = if tau.is_nan() { 0.0 } else if tau < 0.0 { 0.0 } else if tau > 1.0 { 1.0 } else { tau };
            if n_nodes == 0 { return 0.0; }
            if n_nodes == 1 {
                return xvec[idx_w as usize].exp();
            }
            let t = tau * ((n_nodes - 1) as f64);
            let k = t.floor() as usize;
            let alpha = if k >= n_nodes - 1 { 1.0 } else { t - (k as f64) };
            let w0 = xvec[(idx_w + (k as i64)) as usize].exp();
            let w1 = xvec[(idx_w + (if k >= n_nodes - 1 { n_nodes - 1 } else { k + 1 } as i64)) as usize].exp();
            w0 * (1.0 - alpha) + w1 * alpha
        };

        let thrust_fn = move |tau: f64| -> [f64; 3] {
            // piecewise-constant per interval
            let tau = if tau.is_nan() { 0.0 } else if tau < 0.0 { 0.0 } else if tau > 1.0 { 1.0 } else { tau };
            if n_intervals == 0 { return [0.0; 3]; }
            let s = tau * (n_intervals as f64);
            let k = if s >= (n_intervals as f64) { n_intervals - 1 } else { s.floor() as usize };
            let st = (idx_u + 3 * (k as i64)) as usize;
            [xvec[st], xvec[st + 1], xvec[st + 2]]
        };

        let sigma_fn = move |tau: f64| -> f64 {
            let tau = if tau.is_nan() { 0.0 } else if tau < 0.0 { 0.0 } else if tau > 1.0 { 1.0 } else { tau };
            if n_intervals == 0 { return 0.0; }
            let s = tau * (n_intervals as f64);
            let k = if s >= (n_intervals as f64) { n_intervals - 1 } else { s.floor() as usize };
            xvec[(idx_sigma + (k as i64)) as usize]
        };

        // (Step 2) Predefined evaluation points (example: 101 points evenly spaced on [0,1])
        let eval_points = 101usize;

        let mut positions: Vec<[f64; 3]> = Vec::with_capacity(eval_points);
        let mut velocities: Vec<[f64; 3]> = Vec::with_capacity(eval_points);
        let mut masses: Vec<f64> = Vec::with_capacity(eval_points);
        let mut thrusts: Vec<[f64; 3]> = Vec::with_capacity(eval_points);
        let mut sigmas: Vec<f64> = Vec::with_capacity(eval_points);

        // (Step 3) Evaluate polynomials at the evaluation points
        for i in 0..eval_points {
            let tau = if eval_points == 1 { 0.0 } else { (i as f64) / ((eval_points - 1) as f64) };
            positions.push(position_fn(tau));
            velocities.push(velocity_fn(tau));
            masses.push(mass_fn(tau));
            thrusts.push(thrust_fn(tau));
            sigmas.push(sigma_fn(tau));
        }

        TrajectoryResult {
            positions,
            velocities,
            masses,
            thrusts,
            sigmas,
        }
    }

    /*
    Main Solve Routine:
        1) Solves with course time to find a feasible solution
        2) Refines the solution with fine time
    TODO: This is solved using discretization. We need a new solving strategy for Chebyshev polynomials
        - Option 1: Fix polynomial order, search for flight time
            - Set the polynomial order and search over different total flight times. 
        - Option 2: Progressive p-refinement
            - Start with a low polynomial order and solve to find a feasible flight time, increasing the polynomial order 
    */
    pub fn solve(&mut self) -> Option<TrajectoryResult> {
        self.delta_t = self.coarse_delta_t;

        let vel_norm = self.initial_velocity.iter()
            .map(|v| v * v)
            .sum::<f64>()
            .sqrt();
        
        let t_min = self.dry_mass * vel_norm / self.upper_thrust_bound;
        let t_max = self.fuel_mass / (self.alpha * self.lower_thrust_bound);

        let n_min = (t_min / self.delta_t).ceil() as i64;
        let n_max = (t_max / self.delta_t).floor() as i64;

        let mut traj_result: Option<TrajectoryResult> = None;

        for k in n_min..n_max {
            println!("Solving step {}...", k);
            self.N = k;
            match self.solve_at_current_time() {
                Some(sol) => {
                    println!("✅ Step {} converged successfully.", k);
                    traj_result = Some(sol);
                    break;
                },
                None => println!(""),
            }
        }

        if traj_result.is_none() {
            println!("No successful solve found in the given time bounds.");
            return None;
        }

        self.N = ((self.N as f64) * self.coarse_delta_t / self.fine_delta_t).ceil() as i64;
        self.delta_t = self.fine_delta_t;
        
        match self.solve_at_current_time() {
            Some(sol) => {
                println!("✅ Fine solve converged successfully.");
                traj_result = Some(sol);
            },
            None => {
                println!("No successful solve found in the given time bounds.");
                return None;
            },
        }


        return traj_result
    }
}

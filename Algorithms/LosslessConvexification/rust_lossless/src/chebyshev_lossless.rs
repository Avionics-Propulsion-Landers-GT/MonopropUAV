use clarabel::algebra::*;
use clarabel::solver::*;
use std::f64::consts::PI;

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
    pub line_search_delta_t: f64,
    pub coarse_nodes: usize,
    pub fine_nodes: usize,
    N: usize,
    pub pointing_direction: [f64; 3],
    // TODO: We are moving away from discretization into Chebyshev approximation:
    // We should track the order of our polynomial for our solver instead.
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
            line_search_delta_t: 1.0,
            coarse_nodes: 1,
            fine_nodes: 1,
            N: 1,
            pointing_direction: [0.0, 0.0, 1.0],
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
    pub fn cgl_nodes(&self) -> Vec<f64> {
        // By convention, N must be >= 1 to have endpoints and interior points.
        assert!(self.N >= 1, "CGL nodes require N >= 1");

        let nf = self.N as f64;
        let mut tau = Vec::with_capacity(self.N + 1);
        for i in 0..=self.N {
            // We flip i to n-i to ensuring that tau[0] = -1 and tau[n] = 1
            let theta = PI * ((self.N - i) as f64) / nf;
            tau.push(theta.cos());
        }
        tau
    }

    /*
    Barycentric weights for CGL nodes on [-1, 1].
    (Helper function for evaluating Chebyshev polynomials)
    w_i = (-1)^i, halved at endpoints (i=0 and i=N)
    */
    pub fn cgl_barycentric_weights(&self) -> Vec<f64> {
        assert!(self.N >= 1, "CGL weights require N >= 1");
        let mut w = vec![0.0_f64; self.N + 1];

        for i in 0..=self.N {
            w[i] = if i % 2 == 0 { 1.0 } else { -1.0 }; // (-1)^i
            if i == 0 || i == self.N {
                w[i] *= 0.5;
            }
        }

        w
    }

    /*
    Barycentric interpolation on N nodes, forming an order N chebyshev polynomial.
    Evaluates the polynomial at any point x in [-1, 1] given:
        - nodes: CGL nodes
        - weights: the barycentric weights for those nodes
        - values: the function values at those nodes (e.g. from our optimization variables)
    */
    pub fn barycentric_eval(x: f64, nodes: &[f64], weights: &[f64], values: &[f64]) -> f64 {
        // Basic sanity checks to avoid silent mismatch bugs.
        assert!(!nodes.is_empty(), "barycentric_eval: empty nodes");
        assert!(nodes.len() == weights.len() && nodes.len() == values.len(), "barycentric_eval: length mismatch");

        // If x matches a node (within tolerance), return the exact value to avoid divide-by-zero.
        let eps = 1e-12;
        for i in 0..nodes.len() {
            if (x - nodes[i]).abs() <= eps {
                return values[i];
            }
        }

        // Barycentric formula: f(x) = (sum w_i * f_i / (x - x_i)) / (sum w_i / (x - x_i)).
        let mut num = 0.0_f64;
        let mut den = 0.0_f64;
        for i in 0..nodes.len() {
            let diff = x - nodes[i];
            let w_over_diff = weights[i] / diff;
            num += w_over_diff * values[i];
            den += w_over_diff;
        }

        // Final interpolated value.
        num / den
    }

    /*
    Vector barycentric interpolation on given nodes.
    */
    pub fn barycentric_eval_vec3(x: f64, nodes: &[f64], weights: &[f64], values: &[[f64; 3]]) -> [f64; 3] {
        // Basic sanity checks to avoid silent mismatch bugs.
        assert!(!nodes.is_empty(), "barycentric_eval_vec3: empty nodes");
        assert!(nodes.len() == weights.len() && nodes.len() == values.len(), "barycentric_eval_vec3: length mismatch");

        // If x matches a node (within tolerance), return the exact value to avoid divide-by-zero.
        let eps = 1e-12;
        for i in 0..nodes.len() {
            if (x - nodes[i]).abs() <= eps {
                return values[i];
            }
        }

        // Barycentric formula applied component-wise.
        let mut num = [0.0_f64; 3];
        let mut den = 0.0_f64;
        for i in 0..nodes.len() {
            let diff = x - nodes[i];
            let w_over_diff = weights[i] / diff;
            num[0] += w_over_diff * values[i][0];
            num[1] += w_over_diff * values[i][1];
            num[2] += w_over_diff * values[i][2];
            den += w_over_diff;
        }

        [num[0] / den, num[1] / den, num[2] / den]
    }

    /*
    Map time t in [0, final_time] to Chebyshev domain tau in [-1, 1].
    */
    pub fn time_to_tau(t: f64, final_time: f64) -> f64 {
        assert!(final_time > 0.0, "time_to_tau requires positive final_time");
        let mut tau = 2.0 * (t / final_time) - 1.0;
        if tau < -1.0 {
            tau = -1.0;
        } else if tau > 1.0 {
            tau = 1.0;
        }
        tau
    }

    /*
     */
    pub fn cgl_diff_matrix(&self) -> Vec<Vec<f64>> {
        assert!(self.N >= 1, "CGL differentiation matrix requires N >= 1");
        let tau = self.cgl_nodes();

        // Build c endpoint weights (double the weight of other nodes)
        let mut c = vec![1.0_f64; self.N + 1];
        c[0] = 2.0;
        c[self.N] = 2.0;

        // alpha matrix with alternating nodes
        let mut alpha = vec![0.0_f64; self.N + 1];
        for i in 0..=self.N {
            let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
            alpha[i] = c[i] * sign;
        }

        // Start with zeroed differentiation matrix, init for all i,j
        let mut d = vec![vec![0.0_f64; self.N + 1]; self.N + 1];

        // We fill off-diagonal entries with weights
        // essentially "how much does node j contribute to derivative at node i"
        for i in 0..=self.N {
            for j in 0..=self.N {
                if i != j { // don't divide by zero
                    d[i][j] = (alpha[i] / alpha[j]) / (tau[i] - tau[j]);
                }
            }
        }

        // Now fill diagonal entries with special cases
        // start and end are hardcoded, rest use formula
        let nf = self.N as f64;
        d[0][0] = -(2.0 * nf * nf + 1.0) / 6.0;
        d[self.N][self.N] =  (2.0 * nf * nf + 1.0) / 6.0;

        for i in 1..self.N {
            let ti = tau[i];
            d[i][i] = -ti / (2.0 * (1.0 - ti * ti));
        }

        // QOL / robustness (in case of weird floating point behavior)
        // Essentially a double check to ensure rows sum to zero
        // for i in 0..=self.N {
        //     let mut s = 0.0;
        //     for j in 0..=self.N {
        //         if i != j { 
        //             s += d[i][j]; 
        //         }
        //     }
        //     d[i][i] = -s;
        // }

        d
    }

    pub fn clenshaw_curtis_weights(&self) -> Vec<f64> {
        let mut w = vec![0.0; self.N + 1];
        let sum_coeff = 4.0 / (self.N as f64);

        let initial_value;
        let s_value;
        let j_value;

        if self.N % 2 == 0 {
            initial_value = 1.0 / ((self.N * self.N - 1) as f64);
            s_value = self.N / 2;
            j_value = self.N / 2;
        } else {
            initial_value = 1.0 / ((self.N * self.N) as f64);
            s_value = (self.N - 1) / 2;
            j_value = (self.N - 1) / 2;
        }

        w[0] = initial_value;
        w[self.N] = w[0];

        for s in 1..(s_value + 1) {
            let mut value = 0.0;

            for j in 0..(j_value + 1) {
                let value_increment = (1.0 / (1.0 - 4.0 * (j as f64).powi(2))) * ((2.0 * PI * (j as f64) * (s as f64)) / (self.N as f64)).cos();
                if j == 0 || j == j_value {
                    value += value_increment / 2.0;
                } else {
                    value += value_increment;
                }
            }

            value *= sum_coeff;

            w[s] = value;
            w[self.N - s] = value;
        }

        w
    }


    // pub fn solve_at_current_time(&mut self) -> Result<clarabel::solver::DefaultSolution<f64>, String> {
    pub fn solve_at_current_time(&mut self, current_time: f64) -> Option<TrajectoryResult> {
        let m0 = self.dry_mass + self.fuel_mass;
        
        const num_vars_per_node: usize = 11; // x (3), v (3), w (1), sigma (1), u (3)
        let num_nodes = (self.N + 1) as usize;
        let n_vars = num_vars_per_node * num_nodes;
        let X_INDEX = 0;
        let V_INDEX = X_INDEX + 3 * num_nodes;
        let W_INDEX = V_INDEX + 3 * num_nodes;
        let SIGMA_INDEX = W_INDEX + num_nodes;
        let U_INDEX = SIGMA_INDEX + num_nodes;

        let D = self.cgl_diff_matrix();
        
        /*
        TODO: We still use the Second Order Cone Problem (SOCP) constraints and apply them at discrete points
            - We need to evaluate the polynomials to get values at those points.
            - instead of using variable u_k we should evaluate the polynomial at k  
        */
        /*
        TODO: Similar to terminal position and velocity constraints, evaluate w polynomial at the final time.
        */

        // 1. Quadratic Objective Matrix (P)
        // ---------------------------
        let p = CscMatrix::zeros((n_vars, n_vars)); // Zero P (Linear Objective)

        // 2. Linear Objective Vector (q)
        // ---------------------------
        let mut q = vec![0.0; n_vars];
        
        // Calculate Clenshaw-Curtis quadrature weights for integration
        let weights = self.clenshaw_curtis_weights();
        for k in 0..=self.N {
            let sigma_idx = k * num_vars_per_node + 7;
            q[sigma_idx] = weights[k] * (current_time / 2.0); // Scale by time-mapping
        }

        // 3. Build Constraints (A matrix)
        // ---------------------------
        let mut rows: Vec<usize> = Vec::new();
        let mut cols: Vec<usize> = Vec::new();
        let mut vals: Vec<f64> = Vec::new();
        let mut b: Vec<f64> = Vec::new();
        let mut cones: Vec<SupportedConeT<f64>> = Vec::new();
        let mut row_idx = 0;

        // --- A. DYNAMICS (Equality Cone) ---
        // Position dynamics
        // Standard form: x_{k+1} - x_k - dt * v_k = 0
        // Chebyshev: D * x - (tf/2) * x_dot = 0
        // We loop through the diff matrix (D) manually.

        for dim in 0..3 {
            for k in 0..=self.N {
                // The D-Matrix part (sum over j)
                for j in 0..=self.N {
                    let d_val = D[k][j];
                    if d_val.abs() > 1e-9 {
                        let x_idx = X_INDEX + j * 3 + dim;
                        rows.push(row_idx);
                        cols.push(x_idx);
                        vals.push(d_val);
                    }
                }
                // The Velocity part (-tf/2 * v_k)
                let v_idx = V_INDEX + k * 3 + dim;
                rows.push(row_idx);
                cols.push(v_idx);
                vals.push(-0.5 * current_time);

                b.push(0.0);
                row_idx += 1;
            }
        }

        // Velocity Dynamics
        // Standard form: v_{k+1} - v_k - dt * u_k = dt * g
        // Chebyshev: D*v - (tf/2)*u = (tf/2)*g
        for dim in 0..3 {
            for k in 0..=self.N {
                // D-Matrix part
                for j in 0..=self.N {
                    let d_val = D[k][j];
                    if d_val.abs() > 1e-9 {
                        let v_idx = V_INDEX + j * 3 + dim;
                        rows.push(row_idx);
                        cols.push(v_idx);
                        vals.push(d_val);
                    }
                }
                // Control part (-tf/2 * u_k)
                let u_idx = U_INDEX + k * 3 + dim;
                rows.push(row_idx);
                cols.push(u_idx);
                vals.push(-0.5 * current_time);

                // RHS: (tf/2) * g
                b.push(0.5 * current_time * GRAVITY[dim]);
                row_idx += 1;
            }
        }

        // Mass Dynamics
        // Standard form: w_{k+1} - w_k + dt * alpha * sigma_k = 0
        // Chebyshev: D*w + (tf/2)*alpha*sigma = 0
        for k in 0..=self.N {
            for j in 0..=self.N {
                let d_val = D[k][j];
                if d_val.abs() > 1e-9 {
                    let w_idx = W_INDEX + j;
                    rows.push(row_idx);
                    cols.push(w_idx);
                    vals.push(d_val);
                }
            }
            let sigma_idx = SIGMA_INDEX + k;
            rows.push(row_idx);
            cols.push(sigma_idx);
            vals.push(0.5 * current_time * self.alpha);
            
            b.push(0.0);
            row_idx += 1;
        }


        // --- B. BOUNDARY CONDITIONS (Equality Cone) ---
        
        // Start State (r0, v0, z0)
        for dim in 0..3 {
            // r0
            rows.push(row_idx);
            cols.push(X_INDEX + dim);
            vals.push(1.0);
            b.push(self.initial_position[dim]);
            row_idx += 1;
            
            // v0
            rows.push(row_idx);
            cols.push(V_INDEX + dim);
            vals.push(1.0);
            b.push(self.initial_velocity[dim]);
            row_idx += 1;
        }

        // w0
        rows.push(row_idx);
        cols.push(W_INDEX);
        vals.push(1.0);
        b.push((self.fuel_mass + self.dry_mass).ln());
        row_idx += 1;

        // End State (rN = 0, vN = 0)
        for dim in 0..3 {
            // rN
            rows.push(row_idx);
            cols.push(X_INDEX + 3 * (num_nodes - 1) + dim);
            vals.push(1.0);
            b.push(0.0); // Target Pos
            row_idx += 1;
            
            // vN
            rows.push(row_idx);
            cols.push(V_INDEX + 3 * (num_nodes - 1) + dim);
            vals.push(1.0);
            b.push(0.0); // Target Vel
            row_idx += 1;
        }
        
        cones.push(SupportedConeT::ZeroConeT(row_idx)); // All equality constraints so far
        
        // --- C. DRY MASS CONSTRAINT (Inequality) ---
        // We must ensure the final mass is greater than or equal to the dry mass.
        // w_N >= ln(m_dry)  -->  w_N - ln(m_dry) >= 0

        // Clarabel expects: s = b - Ax, where s >= 0
        // We want: w_N - ln(m_dry) >= 0
        // Let s = w_N - ln(m_dry)
        // s = (-ln(m_dry)) - (-1.0 * w_N)
        // So: b = -ln(m_dry), A_val = -1.0

        rows.push(row_idx);
        cols.push(W_INDEX + (num_nodes - 1)); // w_N
        vals.push(-1.0); 
        b.push(-self.dry_mass.ln());

        row_idx += 1;
        cones.push(SupportedConeT::NonnegativeConeT(1));


        // --- C. SOC CONSTRAINTS (Thrust, Pointing, Glide, Velocity) ---
        // ||u|| <= sigma  -->  (sigma, ux, uy, uz) in SOC
        let sin_theta = self.tvc_range_rad.sin();
        for k in 0..=self.N {
            // Thrust magnitude constraint: ||u_k|| <= sigma_k
            let x_idx = X_INDEX + 3 * k;
            let v_idx = V_INDEX + 3 * k;
            let sigma_idx = SIGMA_INDEX + k;
            let w_idx = W_INDEX + k;
            let u_idx = U_INDEX + 3 * k;

            // Row 1: -sigma + s0 = 0
            rows.push(row_idx);
            cols.push(sigma_idx);
            vals.push(-1.0);
            b.push(0.0);
            row_idx += 1;

            // Rows 2-4: -u + s = 0
            for dim in 0..3 {
                rows.push(row_idx);
                cols.push(u_idx + dim);
                vals.push(-1.0);
                b.push(0.0);
                row_idx += 1;
            }
            cones.push(SupportedConeT::SecondOrderConeT(4));


            // Calculate Time at Node k (Crucial for Linearization)
            // Chebyshev Time Map: t = (tf / 2) * (tau + 1)
            // tau_k = -cos(pi * k / N)  -> Goes from -1 to 1
            let tau_k = -(PI * k as f64 / self.N as f64).cos();
            let t_current = (current_time / 2.0) * (tau_k + 1.0);

            let z_0 = (m0 + self.alpha * self.upper_thrust_bound * t_current).ln();
            let exp_neg_z_0 = (-z_0).exp();
            let sigma_min_coeff = self.lower_thrust_bound * exp_neg_z_0;
            let sigma_min_h_val = sigma_min_coeff * (1.0 + z_0);
            let sigma_max_coeff = self.upper_thrust_bound * exp_neg_z_0;
            let sigma_max_h_val = sigma_max_coeff * (1.0 + z_0);

            // thrust bounds
            // min bound
            cones.push(SupportedConeT::NonnegativeConeT(1));
            rows.push(row_idx as usize); cols.push(sigma_idx as usize); vals.push(-1.0); // sigma_k - sigma_min
            rows.push(row_idx as usize); cols.push(w_idx as usize); vals.push(-sigma_min_coeff);
            b.push(-sigma_min_h_val);
            row_idx += 1;
            
            // max bound
            cones.push(SupportedConeT::NonnegativeConeT(1));
            rows.push(row_idx as usize); cols.push(sigma_idx as usize); vals.push(1.0); // -sigma_k + sigma_max
            rows.push(row_idx as usize); cols.push(w_idx as usize); vals.push(sigma_max_coeff);
            b.push(sigma_max_h_val);
            row_idx += 1;

            
            // ---------- Thrust magnitude SOC: ||u_k|| <= sigma_k ----------
            rows.push((row_idx as usize) + 0); cols.push(sigma_idx as usize); vals.push(-1.0); // -sigma_k
            rows.push((row_idx as usize) + 1); cols.push(u_idx as usize);     vals.push(-1.0); // -u0
            rows.push((row_idx as usize) + 2); cols.push((u_idx as usize) + 1);   vals.push(-1.0); // -u1
            rows.push((row_idx as usize) + 3); cols.push((u_idx as usize) + 2);   vals.push(-1.0); // -u2
            cones.push(SupportedConeT::SecondOrderConeT(4));
            b.push(0.0);
            b.push(0.0);
            b.push(0.0);
            b.push(0.0);
            row_idx += 4;

            
            // ---------- Thrust pointing SOC: ||u_k - sigma_k*d|| <= sigma_k*sin(theta) ----------
            rows.push((row_idx as usize) + 0); cols.push(sigma_idx as usize); vals.push(-sin_theta); // top of SOC
            rows.push((row_idx as usize) + 1); cols.push(u_idx as usize);     vals.push(1.0);
            rows.push((row_idx as usize) + 1); cols.push(sigma_idx as usize);     vals.push(-self.pointing_direction[0]);
            rows.push((row_idx as usize) + 2); cols.push((u_idx as usize) + 1);   vals.push(1.0);
            rows.push((row_idx as usize) + 2); cols.push(sigma_idx as usize);     vals.push(-self.pointing_direction[1]);
            rows.push((row_idx as usize) + 3); cols.push((u_idx as usize) + 2);   vals.push(1.0);
            rows.push((row_idx as usize) + 3); cols.push(sigma_idx as usize);     vals.push(-self.pointing_direction[2]);
            cones.push(SupportedConeT::SecondOrderConeT(4));
            b.push(0.0);
            b.push(0.0);
            b.push(0.0);
            b.push(0.0);
            row_idx += 4;


            // ---------- Glide slope SOC: ||x_k[:2]|| <= x_k[2] / tan(glide_slope) ----------
            if self.use_glide_slope {
                rows.push((row_idx as usize) + 0); cols.push((x_idx as usize) + 2); vals.push(-1.0 / self.glide_slope.tan()); // x_k[2]
                rows.push((row_idx as usize) + 1); cols.push((x_idx as usize) + 0); vals.push(1.0); // -x_k[0]
                rows.push((row_idx as usize) + 2); cols.push((x_idx as usize) + 1); vals.push(1.0); // -x_k[1]
                cones.push(SupportedConeT::SecondOrderConeT(3));
                b.push(0.0);
                b.push(self.landing_point[0]);
                b.push(self.landing_point[1]);
                row_idx += 3;
            }

            
            // ---------- Max velocity SOC: ||v_k|| <= max_velocity ----------
            // rows.push((row_idx as usize) + 0); cols.push(v_idx as usize);     vals.push(0.0); // -v0
            // No need to push a zero value
            rows.push((row_idx as usize) + 1); cols.push(v_idx as usize);     vals.push(-1.0); // -v0
            rows.push((row_idx as usize) + 2); cols.push((v_idx as usize) + 1);   vals.push(-1.0); // -v1
            rows.push((row_idx as usize) + 3); cols.push((v_idx as usize) + 2);   vals.push(-1.0); // -v2
            cones.push(SupportedConeT::SecondOrderConeT(4));
            b.push(self.max_velocity);
            b.push(0.0);
            b.push(0.0);
            b.push(0.0);
            row_idx += 4;
        }


        // 4. Solve
        // ---------------------------
        let a_csc = Self::csc_from_triplets(row_idx as usize, n_vars as usize, &rows, &cols, &vals);

        let mut settings = DefaultSettings::default(); // Adjust tolerances here if needed
        settings.verbose = true;
        
        let mut solver = DefaultSolver::new(&p, &q, &a_csc, &b, &cones, settings);

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
    Helper function that extracts the trajectory result from the solver solution.

    TODO: With our current implementation of Chebyshev, the solution contains coefficients of our polynomials not values at nodes.
        -   To get a trajectory we need to:
            1) extract the coefficients of our polynomials for (x ,v, w, u)
            2) Define a set of evaluation points (Maybe like 100)
            3) Evaluate the polynomials at the points
        - Alternatively We could store the coefficients themselves to later re-evaluate the trajectory for graphing
            - This would mean changing the TrajectoryResult struct as well as how main.rs parses our results

    UPDATE:
        We are continuing with barycentric interpolation, allowing us to evaluate the trajectory at any point in time
        It iterpolates along the entire N-order polynomial with numerical stability
    
    */
    fn extract_result(&self, result: &DefaultSolution<f64>) -> TrajectoryResult {

        // (Step 1) extract coefficents (idk yet gang shall return after diving into papers)

        let num_nodes = (self.N + 1) as usize;
        let X_INDEX = 0;
        let V_INDEX = X_INDEX + 3 * num_nodes;
        let W_INDEX = V_INDEX + 3 * num_nodes;
        let SIGMA_INDEX = W_INDEX + num_nodes;
        let U_INDEX = SIGMA_INDEX + num_nodes;

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
                let s = X_INDEX as usize;
                return [xvec[s], xvec[s + 1], xvec[s + 2]];
            }
            let t = tau * ((n_nodes - 1) as f64);
            let k = t.floor() as usize;
            let alpha = if k >= n_nodes - 1 { 1.0 } else { t - (k as f64) };
            let s0 = (X_INDEX + 3 * (k as usize)) as usize;
            let s1 = (X_INDEX + 3 * (if k >= n_nodes - 1 { n_nodes - 1 } else { k + 1 } as usize));
            [xvec[s0] * (1.0 - alpha) + xvec[s1] * alpha,
             xvec[s0 + 1] * (1.0 - alpha) + xvec[s1 + 1] * alpha,
             xvec[s0 + 2] * (1.0 - alpha) + xvec[s1 + 2] * alpha]
        };

        let velocity_fn = move |tau: f64| -> [f64; 3] {
            let tau = if tau.is_nan() { 0.0 } else if tau < 0.0 { 0.0 } else if tau > 1.0 { 1.0 } else { tau };
            if n_nodes == 0 { return [0.0; 3]; }
            if n_nodes == 1 {
                let s = V_INDEX as usize;
                return [xvec[s], xvec[s + 1], xvec[s + 2]];
            }
            let t = tau * ((n_nodes - 1) as f64);
            let k = t.floor() as usize;
            let alpha = if k >= n_nodes - 1 { 1.0 } else { t - (k as f64) };
            let s0 = (V_INDEX + 3 * (k as usize)) as usize;
            let s1 = (V_INDEX + 3 * (if k >= n_nodes - 1 { n_nodes - 1 } else { k + 1 }));
            [xvec[s0] * (1.0 - alpha) + xvec[s1] * alpha,
             xvec[s0 + 1] * (1.0 - alpha) + xvec[s1 + 1] * alpha,
             xvec[s0 + 2] * (1.0 - alpha) + xvec[s1 + 2] * alpha]
        };

        let mass_fn = move |tau: f64| -> f64 {
            let tau = if tau.is_nan() { 0.0 } else if tau < 0.0 { 0.0 } else if tau > 1.0 { 1.0 } else { tau };
            if n_nodes == 0 { return 0.0; }
            if n_nodes == 1 {
                return xvec[W_INDEX as usize].exp();
            }
            let t = tau * ((n_nodes - 1) as f64);
            let k = t.floor() as usize;
            let alpha = if k >= n_nodes - 1 { 1.0 } else { t - (k as f64) };
            let w0 = xvec[(W_INDEX + (k as usize)) as usize].exp();
            let w1 = xvec[(W_INDEX + (if k >= n_nodes - 1 { n_nodes - 1 } else { k + 1 } as usize))].exp();
            w0 * (1.0 - alpha) + w1 * alpha
        };

        let sigma_fn = move |tau: f64| -> f64 {
            let tau = if tau.is_nan() { 0.0 } else if tau < 0.0 { 0.0 } else if tau > 1.0 { 1.0 } else { tau };
            if n_intervals == 0 { return 0.0; }
            let s = tau * (n_intervals as f64);
            let k = if s >= (n_intervals as f64) { n_intervals - 1 } else { s.floor() as usize };
            xvec[(SIGMA_INDEX + (k as usize)) as usize]
        };

        let thrust_fn = move |tau: f64| -> [f64; 3] {
            // piecewise-constant per interval
            let tau = if tau.is_nan() { 0.0 } else if tau < 0.0 { 0.0 } else if tau > 1.0 { 1.0 } else { tau };
            if n_intervals == 0 { return [0.0; 3]; }
            let s = tau * (n_intervals as f64);
            let k = if s >= (n_intervals as f64) { n_intervals - 1 } else { s.floor() as usize };
            let st = (U_INDEX + 3 * (k as usize));
            [xvec[st], xvec[st + 1], xvec[st + 2]]
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
        // self.N = self.coarse_nodes;

        // let vel_norm = self.initial_velocity.iter()
        //     .map(|v| v * v)
        //     .sum::<f64>()
        //     .sqrt();
        
        // let t_min = self.dry_mass * vel_norm / self.upper_thrust_bound;
        // let t_max = self.fuel_mass / (self.alpha * self.lower_thrust_bound);
        // let mut current_time = t_min;

        // let mut traj_result: Option<TrajectoryResult> = None;

        // while current_time <= t_max {
        //     println!("Solving step {}...", current_time);
        //     match self.solve_at_current_time(current_time) {
        //         Some(sol) => {
        //             println!("✅ Step {} converged successfully.", k);
        //             traj_result = Some(sol);
        //             break;
        //         },
        //         None => println!(""),
        //     }
        //     current_time += self.line_search_delta_t;
        // }

        // if traj_result.is_none() {
        //     println!("No successful solve found in the given time bounds.");
        //     return None;
        // }

        // self.N = ((self.N as f64) * self.coarse_delta_t / self.fine_delta_t).ceil() as i64;
        // self.delta_t = self.fine_delta_t;
        
        // match self.solve_at_current_time() {
        //     Some(sol) => {
        //         println!("✅ Fine solve converged successfully.");
        //         traj_result = Some(sol);
        //     },
        //     None => {
        //         println!("No successful solve found in the given time bounds.");
        //         return None;
        //     },
        // }


        // return traj_result
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
}



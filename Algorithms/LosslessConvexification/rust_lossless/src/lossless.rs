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
    pub fn new() -> Self {
        lossless_solver::default()
    }

    pub fn solve_at_time(&self, n: i64) {
        let m0 = self.dry_mass + self.fuel_mass;
        
        let n_vars = 3 * (self.N + 1) // x
            + 3 * (self.N + 1) // v
            + (self.N + 1) // w
            + 3 * self.N // u
            + self.N; // sigma

        let idx_x = 0;
        let idx_v = idx_x + 3 * (self.N + 1);
        let idx_w = idx_v + 3 * (self.N + 1);
        let idx_u = idx_w + (self.N + 1);
        let idx_sigma = idx_u + 3 * self.N;
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



    }
}
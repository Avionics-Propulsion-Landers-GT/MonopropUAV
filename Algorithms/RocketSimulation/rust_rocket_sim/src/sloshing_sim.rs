use nalgebra::{Vector3, UnitQuaternion};
use std::ops::{Add, Mul};

#[derive(Debug, Clone, Copy)]
pub struct TankConfig {
    pub radius_m: f64,
    pub length_m: f64,
    pub density_liquid: f64,
    pub gravity: f64,
}

impl TankConfig {
    pub fn new(radius_m: f64, length_m: f64, density_liquid: f64, gravity: f64) -> Self {
        Self {
            radius_m,
            length_m,
            density_liquid,
            gravity,
        }
    }
}

impl TankConfig {
    pub fn volume_tank(&self) -> f64 {
        std::f64::consts::PI * self.radius_m.powi(2) * self.length_m
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SloshParams {
    pub m_frac: f64,
    pub c_lin: f64,
    pub alpha: f64,
    pub omega_scale: f64,
}

impl SloshParams {
    pub fn new(m_frac: f64, c_lin: f64, alpha: f64, omega_scale: f64) -> Self {
        Self {
            m_frac,
            c_lin,
            alpha,
            omega_scale,
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct SloshState {
    pub x: f64,
    pub y: f64,
    pub x_dot: f64,
    pub y_dot: f64,
}

// Idiomatic Rust: Overloading operators makes the RK4 math beautiful
impl Add for SloshState {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            x_dot: self.x_dot + other.x_dot,
            y_dot: self.y_dot + other.y_dot,
        }
    }
}

impl Mul<f64> for SloshState {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
            x_dot: self.x_dot * scalar,
            y_dot: self.y_dot * scalar,
        }
    }
}

#[derive(Clone, Debug)]
pub struct SloshModel {
    pub tank_cfg: TankConfig,
    pub params: SloshParams,
    pub state: SloshState,
    pub m_liquid: f64,
    pub fill_frac: f64,
    
    // Internal dynamic parameters
    m_eq: f64,
    k_lin: f64,
    omega_n: f64,
}

impl SloshModel {
    pub fn new(
        tank_cfg: TankConfig,
        params: SloshParams,
        initial_fill_frac: f64,
        initial_state: SloshState,
    ) -> Self {
        let fill_frac = initial_fill_frac.clamp(0.0, 1.0);
        let m_liquid = fill_frac * tank_cfg.volume_tank() * tank_cfg.density_liquid;

        let mut model = Self {
            tank_cfg,
            params,
            state: initial_state,
            m_liquid,
            fill_frac,
            m_eq: 0.0,
            k_lin: 0.0,
            omega_n: 0.0,
        };

        model.update_equivalent_parameters();
        model
    }

    fn update_equivalent_parameters(&mut self) {
        let k_wave = 1.841 / self.tank_cfg.radius_m;
        let h = self.fill_frac * self.tank_cfg.length_m;

        // Dynamic natural frequency based on fluid height
        self.omega_n = self.params.omega_scale * (self.tank_cfg.gravity * k_wave * (k_wave * h).tanh()).sqrt();

        // THE FIX: Isotropic mass! (No 0.5 multiplier)
        self.m_eq = self.params.m_frac * self.m_liquid;

        // Linear stiffness
        self.k_lin = self.m_eq * self.omega_n.powi(2);
    }

    fn rhs(&self, state: SloshState, a_lat_x: f64, a_lat_y: f64) -> SloshState {
        // Degenerate case guard: if tank is basically empty, freeze slosh
        if self.m_eq <= 1e-6 {
            return SloshState { x_dot: 0.0, y_dot: 0.0, x: 0.0, y: 0.0 };
        }

        let c = self.params.c_lin;
        let k = self.k_lin;
        let alpha = self.params.alpha;

        // Duffing Equation: mx'' + cx' + kx + αx³ = -m * a_lat
        let x_dot_dot = (-c * state.x_dot - k * state.x - alpha * state.x.powi(3) - self.m_eq * a_lat_x) / self.m_eq;
        let y_dot_dot = (-c * state.y_dot - k * state.y - alpha * state.y.powi(3) - self.m_eq * a_lat_y) / self.m_eq;

        SloshState {
            x: state.x_dot,
            y: state.y_dot,
            x_dot: x_dot_dot,
            y_dot: y_dot_dot,
        }
    }

    /// Advances the slosh model and returns the reaction force in the Body Frame
    pub fn step(
        &mut self,
        dt: f64,
        lin_acc_body: Vector3<f64>,
        attitude: UnitQuaternion<f64>,
        mdot: f64,
    ) -> Vector3<f64> {
        // 1. Update fluid mass and dynamics
        self.m_liquid = (self.m_liquid - mdot * dt).max(0.0);
        self.fill_frac = self.m_liquid / (self.tank_cfg.density_liquid * self.tank_cfg.volume_tank());
        self.update_equivalent_parameters();

        // 2. THE FIX: Correct Gravity Transformation (World -> Body)
        // Using the inverse quaternion perfectly maps the world gravity vector into the tilted body frame.
        let g_world = Vector3::new(0.0, 0.0, self.tank_cfg.gravity);
        let g_body = attitude.inverse() * g_world;

        // Apparent acceleration felt by the fluid
        let a_total_body = lin_acc_body + g_body;
        let a_lat_x = a_total_body.x;
        let a_lat_y = a_total_body.y;

        // 3. RK4 Integration (Cleaned up via operator overloading)
        let k1 = self.rhs(self.state, a_lat_x, a_lat_y);
        let k2 = self.rhs(self.state + k1 * (dt / 2.0), a_lat_x, a_lat_y);
        let k3 = self.rhs(self.state + k2 * (dt / 2.0), a_lat_x, a_lat_y);
        let k4 = self.rhs(self.state + k3 * dt, a_lat_x, a_lat_y);

        self.state = self.state + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);

        // 4. Calculate Restoring Force
        let f_x = self.params.c_lin * self.state.x_dot 
                + self.k_lin * self.state.x 
                + self.params.alpha * self.state.x.powi(3);
                
        let f_y = self.params.c_lin * self.state.y_dot 
                + self.k_lin * self.state.y 
                + self.params.alpha * self.state.y.powi(3);

        Vector3::new(f_x, f_y, 0.0)
    }
}
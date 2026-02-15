#![no_std] // So the Teensy can use it
use serde::{Serialize, Deserialize};

// TELEMETRY: Drone -> Ground
// The state vector your MPC needs
#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub struct DroneTelemetry {
    pub timestamp_us: u64,  // Drone clock
    pub orientation: [f32; 4], // Quaternion (qx, qy, qz, qw)
    pub position: [f32; 3],    // x, y, z (if you have local position est)
    pub velocity: [f32; 3],    // vx, vy, vz
    pub angular_vel: [f32; 3], // wx, wy, wz
    pub battery: f32,
}

// COMMAND: Ground -> Drone
// The setpoints your MPC generates
#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub struct GroundCommand {
    pub seq_id: u32,       // To detect packet loss
    pub thrust_seq: [f32; 10],       // Normalized Thrust (0.0 - 1.0) or Force in Newtons (n.b. thrust hasn't been normalized yet in the solver)
    pub gimbal_theta_seq: [f32; 10], // Gimbal angle theta sequence (radians)
    pub gimbal_phi_seq: [f32; 10],   // Gimbal angle phi sequence (radians)
    pub body_rates: [f32; 3], // Feedforward rates (optional but recommended)
}
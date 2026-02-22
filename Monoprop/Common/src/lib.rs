#![no_std] // So the Teensy can use it
use serde::{Serialize, Deserialize};

// TELEMETRY: Drone -> Ground
// The state vector your MPC needs
#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub struct DroneTelemetry {
    pub timestamp_us: u64,  // Drone clock
    pub orientation: [f32; 4], // Quaternion (w, x, y, z)
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
    pub thrust: f32,       // Normalized Thrust (0.0 - 1.0) or Force in Newtons
    pub target_q: [f32; 4], // Desired Attitude (Quaternion)
    pub body_rates: [f32; 3], // Feedforward rates (optional but recommended)
}
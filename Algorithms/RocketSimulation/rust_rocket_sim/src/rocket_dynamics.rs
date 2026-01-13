use nalgebra::{Vector3, UnitQuaternion, Quaternion};

use crate::device_sim::*;

/// Constants for the simulation
const dt: f64 = 0.01; // Time step in seconds

#[derive(Debug, Clone)]
pub struct Rocket {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub accel: Vector3<f64>,
    
    pub attitude: UnitQuaternion<f64>,
    pub ang_vel: Vector3<f64>,
    pub ang_accel: Vector3<f64>,

    pub dry_mass: f64,
    pub propellant_mass: f64,

    pub inertia_tensor: Vector3<f64>, // Simplified diagonal inertia matrix

    pub tvc_range: f64,
    pub tvc: TVC,

    thrust_vector: Vector3<f64>,

    pub imu: IMU,
    pub gps: GPS,
    pub uwb: UWB,
}

impl Rocket {
    pub fn new(position: Vector3<f64>, velocity: Vector3<f64>, accel: Vector3<f64>, attitude: UnitQuaternion<f64>, ang_vel: Vector3<f64>, ang_accel: Vector3<f64>, dry_mass: f64, propellant_mass: f64, inertia_tensor: Vector3<f64>, tvc_range: f64, tvc: TVC) -> Self {
        Self {
            position,
            velocity,
            accel,
            attitude,
            ang_vel,
            ang_accel,
            dry_mass,
            propellant_mass,
            inertia_tensor,
            tvc_range,
            tvc,    
        }
    }

    /// Update state based on applied forces and torques
    /// forces: Force vector in World Frame
    /// torques: Torque vector in Body Frame
    pub fn step(&mut self, control_input: Vector3<f64>) {
        // 1. Translational Dynamics (F = ma)
        // Gravity (assuming -Z is down)
        let gravity = Vector3::new(0.0, 0.0, -9.81);
        let total_force = forces_world + (gravity * self.mass);
        
        self.acceleration = total_force / self.mass;
        self.velocity += self.acceleration * dt;
        self.position += self.velocity * dt;

        // 2. Rotational Dynamics (Euler's rotation equations)
        // Torque = I * alpha + omega x (I * omega)
        // alpha = I_inv * (Torque - omega x (I * omega))
        
        // We calculate I * omega manually since inertia is a diagonal Vector3 here
        let i_omega = Vector3::new(
            self.inertia_tensor.x * self.angular_velocity.x,
            self.inertia_tensor.y * self.angular_velocity.y,
            self.inertia_tensor.z * self.angular_velocity.z,
        );

        let gyroscopic_term = self.angular_velocity.cross(&i_omega);
        let net_torque = torques_body - gyroscopic_term;

        // Angular acceleration (alpha)
        let alpha = Vector3::new(
            net_torque.x / self.inertia_tensor.x,
            net_torque.y / self.inertia_tensor.y,
            net_torque.z / self.inertia_tensor.z,
        );

        // Update Angular Velocity
        self.angular_velocity += alpha * dt;

        // 3. Update Attitude (Quaternion Integration)
        // q_dot = 0.5 * quaternion(0, omega) * q
        let omega_quat = Quaternion::new(
            0.0, 
            self.angular_velocity.x, 
            self.angular_velocity.y, 
            self.angular_velocity.z
        );
        
        // Standard approach: q_new = q_old + (0.5 * omega * q_old) * dt
        // nalgebra handles the multiplication logic for us
        let delta_q = UnitQuaternion::from_quaternion(omega_quat); 
        
        // Note: For small timesteps, we approximate the integration.
        // A common robust method is integrating the angle axis directly:
        let angle = self.angular_velocity.norm() * dt;
        if angle > 1e-6 {
            let axis = UnitQuaternion::from_axis_angle(
                &nalgebra::Unit::new_normalize(self.angular_velocity), 
                angle
            );
            // Apply rotation: new_attitude = old_attitude * delta_rotation
            self.attitude = self.attitude * axis;
        }
    }
}

fn main() {
    let mut rocket = Rocket::new();

    println!("Starting Simulation...");
    println!("Initial State: Z = {:.2} m", rocket.position.z);

    // Simulation Loop
    for i in 0..100 {
        // Simulate a thruster firing upwards (World Frame Z)
        // Force = 20,000 N (enough to overcome gravity: 1000kg * 9.81 = 9810 N)
        let thrust_force = Vector3::new(0.0, 0.0, 20000.0);
        
        // Simulate a small torque causing a spin (Body Frame)
        let control_torque = Vector3::new(0.0, 0.0, 10.0);

        rocket.step(thrust_force, control_torque);

        if i % 10 == 0 {
            println!(
                "T={:.2}s | Pos Z: {:.2} | Vel Z: {:.2} | Roll Rate: {:.4}", 
                (i as f64) * dt,
                rocket.position.z, 
                rocket.velocity.z,
                rocket.angular_velocity.z
            );
        }
    }
}
use nalgebra::{Vector3, Vector4, UnitQuaternion, Quaternion};

use crate::device_sim::*;

#[derive(Debug, Clone)]
pub struct Rocket {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub accel: Vector3<f64>,
    
    pub attitude: UnitQuaternion<f64>,
    pub ang_vel: Vector3<f64>,
    pub ang_accel: Vector3<f64>,

    pub dry_mass: f64,
    pub nitrogen_tank_empty_mass: f64, // the mass of the empty tank
    pub nitrogen_mass: f64,
    starting_nitrogen_mass: f64,
    pub nitrogen_tank_empty_offset: Vector3<f64>, // Vector offset from frame CoM to the empty nitrogen tank CoM
    pub nitrogen_tank_full_offset: Vector3<f64>, // Vector offset from the empty nitrogen tank CoM to the full nitrogen tank CoM
    pub nitrous_tank_empty_mass: f64, // the mass of the empty tank
    pub pressurizing_nitrogen_mass: f64,
    starting_pressurizing_nitrogen_mass: f64,
    pub nitrous_mass: f64,
    starting_nitrous_mass: f64,
    pub nitrous_tank_empty_offset: Vector3<f64>, // Vector offset from frame CoM to the empty nitrous tank CoM (no nitrous or nitrogen in it)
    pub nitrous_tank_full_offset: Vector3<f64>, // Vector offset from the empty nitrous tank CoM to the nitrous tank CoM when it's full of nitrous
    pub nitrous_tank_depleted_offset: Vector3<f64>, // Vector offset from the empty nitrous tank CoM to the nitrous tank CoM when all of the nitrous has been depleted
    pub tvc_module_empty_mass: f64, // the mass of the empty tvc module
    pub fuel_grain_mass: f64,
    starting_fuel_grain_mass: f64,
    pub frame_com_to_gimbal: Vector3<f64>, // Vector offset from frame CoM to the gimbal point (thrust plate)
    pub gimbal_to_tvc_com: Vector3<f64>, // Vector offset from gimbal point to the TVC's center of mass

    pub inertia_tensor: Vector3<f64>, // Simplified diagonal inertia matrix

    pub tvc_range: f64,
    pub tvc: TVC,

    pub rcs: RCS,

    thrust_vector: Vector3<f64>,

    pub imu: IMU,
    pub gps: GPS,
    pub uwb: UWB,

    pub com_to_ground: Vector3<f64>, // Distance from center of mass to ground (for ground interaction)

    system_time: f64,
}

impl Rocket {
    pub fn new(position: Vector3<f64>, velocity: Vector3<f64>, accel: Vector3<f64>, attitude: UnitQuaternion<f64>, ang_vel: Vector3<f64>, ang_accel: Vector3<f64>, dry_mass: f64, nitrogen_tank_empty_mass: f64, starting_nitrogen_mass: f64, nitrogen_tank_empty_offset: Vector3<f64>, nitrogen_tank_full_offset: Vector3<f64>, nitrous_tank_empty_mass: f64, starting_pressurizing_nitrogen_mass: f64, starting_nitrous_mass: f64, nitrous_tank_empty_offset: Vector3<f64>, nitrous_tank_full_offset: Vector3<f64>, nitrous_tank_depleted_offset: Vector3<f64>, tvc_module_empty_mass: f64, starting_fuel_grain_mass: f64, frame_com_to_gimbal: Vector3<f64>, gimbal_to_tvc_com: Vector3<f64>, inertia_tensor: Vector3<f64>, tvc_range: f64, tvc: TVC, rcs: RCS, imu: IMU, gps: GPS, uwb: UWB, com_to_ground: Vector3<f64>) -> Self {
        let mut rocket = Self {
            position,
            velocity,
            accel,
            attitude,
            ang_vel,
            ang_accel,
            dry_mass,
            nitrogen_tank_empty_mass,
            nitrogen_mass: starting_nitrogen_mass,
            starting_nitrogen_mass,
            nitrogen_tank_empty_offset,
            nitrogen_tank_full_offset,
            nitrous_tank_empty_mass,
            pressurizing_nitrogen_mass: starting_pressurizing_nitrogen_mass,
            starting_pressurizing_nitrogen_mass,
            nitrous_mass: starting_nitrous_mass,
            starting_nitrous_mass,
            nitrous_tank_empty_offset,
            nitrous_tank_full_offset,
            nitrous_tank_depleted_offset,
            tvc_module_empty_mass,
            fuel_grain_mass: starting_fuel_grain_mass,
            starting_fuel_grain_mass,
            frame_com_to_gimbal,
            gimbal_to_tvc_com,
            inertia_tensor,
            tvc_range,
            tvc,
            rcs,
            thrust_vector: Vector3::zeros(),
            imu,
            gps,
            uwb,
            com_to_ground,
            system_time: 0.0,
        };

        rocket.imu.update(rocket.accel, rocket.ang_vel, rocket.attitude, rocket.system_time);
        rocket.gps.update(rocket.position, rocket.system_time);
        rocket.uwb.update(rocket.position, rocket.system_time);

        rocket
    }

    /// Update state based on applied forces and torques
    /// forces: Force vector in World Frame
    /// torques: Torque vector in Body Frame
    /// Returns true if the step is successful and false if the simulation has ended (hit the ground)
    pub fn step(&mut self, control_input: Vector4<f64>, outside_forces: Vector3<f64>, outside_torques: Vector3<f64>, dt: f64) -> bool {
        let rotated_offset = self.attitude.transform_vector(&self.com_to_ground);
        let rocket_bottom = self.position + rotated_offset;
        if rocket_bottom.z <= 0.0 {
            return false; // Indicate that we've hit the ground
        }
        

        // Update Sensors
        self.imu.update(self.accel, self.ang_vel, self.attitude, self.system_time);
        self.gps.update(self.position, self.system_time);
        self.uwb.update(self.position, self.system_time);

        // Update actuated devices
        let tvc_effect: TVCEffect = self.tvc.update(Vector3::new(control_input.x, control_input.y, control_input.z), self.nitrogen_mass, self.pressurizing_nitrogen_mass, self.nitrous_mass, self.fuel_grain_mass, dt, self.system_time);
        self.nitrogen_mass = tvc_effect.nitrogen_mass;
        self.pressurizing_nitrogen_mass = tvc_effect.pressurizing_nitrogen_mass;
        self.nitrous_mass = tvc_effect.nitrous_mass;
        self.fuel_grain_mass = tvc_effect.fuel_grain_mass;

        // TODO: implement throttle controller
        // TODO: talk to team and change the control vector to have 4 dimensions (add in rcs control command)
        let rcs_command = control_input.w; // Assuming the 4th element of control_input is for RCS
        let rcs_effect = self.rcs.update(rcs_command, self.nitrogen_mass, dt, self.system_time);
        self.nitrogen_mass = rcs_effect.nitrogen_mass;


        self.system_time += dt;
        let mass = self.get_mass();

        // Translational Dynamics
        let gravity = Vector3::new(0.0, 0.0, -9.81);
        let total_force = outside_forces + (gravity * mass) + self.attitude.transform_vector(&tvc_effect.thrust);

        self.accel = total_force / mass;        
        self.velocity += self.accel * dt;
        self.position += self.velocity * dt;

        // 2. Rotational Dynamics (Euler's rotation equations)
        // Torque = I * alpha + omega x (I * omega)
        // alpha = I_inv * (Torque - omega x (I * omega))
        
        // We calculate I * omega manually since inertia is a diagonal Vector3 here
        let i_omega = Vector3::new(
            self.inertia_tensor.x * self.ang_vel.x,
            self.inertia_tensor.y * self.ang_vel.y,
            self.inertia_tensor.z * self.ang_vel.z,
        );

        let gyro_torque = self.ang_vel.cross(&i_omega);
        let net_torque = outside_torques - gyro_torque + tvc_effect.torque + rcs_effect.torque;

        // Angular acceleration (alpha)
        let alpha = Vector3::new(
            net_torque.x / self.inertia_tensor.x,
            net_torque.y / self.inertia_tensor.y,
            net_torque.z / self.inertia_tensor.z,
        );

        // Update Angular Velocity
        self.ang_vel += alpha * dt;

        // 3. Update Attitude (Quaternion Integration)
        // q_dot = 0.5 * quaternion(0, omega) * q
        let omega_quat = Quaternion::new(
            0.0, 
            self.ang_vel.x, 
            self.ang_vel.y, 
            self.ang_vel.z
        );
        
        // Standard approach: q_new = q_old + (0.5 * omega * q_old) * dt
        // nalgebra handles the multiplication logic for us
        let delta_q = UnitQuaternion::from_quaternion(omega_quat); 
        
        // Note: For small timesteps, we approximate the integration.
        // A common robust method is integrating the angle axis directly:
        let angle = self.ang_vel.norm() * dt;
        if angle > 1e-6 {
            let axis = UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(self.ang_vel), angle);
            // Apply rotation: new_attitude = old_attitude * delta_rotation
            self.attitude = self.attitude * axis;
        }

        return true; // Indicate successful step
    }

    pub fn get_mass(&self) -> f64 {
        self.dry_mass + self.nitrogen_tank_empty_mass + self.nitrogen_mass + self.nitrous_tank_empty_mass + self.pressurizing_nitrogen_mass + self.nitrous_mass + self.tvc_module_empty_mass + self.fuel_grain_mass
    }

    // State vector: [x, y, z, qx, qy, qz, qw, x_dot, y_dot, z_dot, wx, wy, wz]
    // This is formatted for use in the MPC controller
    pub fn get_state(&self) -> Array1<f64> {
        let mut state = Array1::zeros(13);
        state[0] = self.position.x;
        state[1] = self.position.y;
        state[2] = self.position.z;
        state[3] = self.attitude.i;
        state[4] = self.attitude.j;
        state[5] = self.attitude.k;
        state[6] = self.attitude.w;
        state[7] = self.velocity.x;
        state[8] = self.velocity.y;
        state[9] = self.velocity.z;
        state[10] = self.ang_vel.x;
        state[11] = self.ang_vel.y;
        state[12] = self.ang_vel.z;

        state
    }
}

fn main() {
    // let mut rocket = Rocket::new();

    // println!("Starting Simulation...");
    // println!("Initial State: Z = {:.2} m", rocket.position.z);

    // // Simulation Loop
    // for i in 0..100 {
    //     // Simulate a thruster firing upwards (World Frame Z)
    //     // Force = 20,000 N (enough to overcome gravity: 1000kg * 9.81 = 9810 N)
    //     let thrust_force = Vector3::new(0.0, 0.0, 20000.0);
        
    //     // Simulate a small torque causing a spin (Body Frame)
    //     let control_torque = Vector3::new(0.0, 0.0, 10.0);

    //     rocket.step(thrust_force, control_torque);

    //     if i % 10 == 0 {
    //         println!(
    //             "T={:.2}s | Pos Z: {:.2} | Vel Z: {:.2} | Roll Rate: {:.4}", 
    //             (i as f64) * dt,
    //             rocket.position.z, 
    //             rocket.velocity.z,
    //             rocket.angular_velocity.z
    //         );
    //     }
    // }
}
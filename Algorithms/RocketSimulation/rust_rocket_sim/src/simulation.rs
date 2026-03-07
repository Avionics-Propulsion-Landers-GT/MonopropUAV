use crate::rocket_dynamics::*;
use crate::device_sim::*;
use crate::algorithms::*;
use crate::sloshing_sim::*;
use crate::fluid_dynamics::*;
use nalgebra::{Matrix3, Vector3, Vector4, UnitQuaternion};
use ndarray::{Array1, Array2};

#[derive(Debug)]
pub struct Simulation {
    pub rocket: Rocket,
    pub mpc: MPC,
    pub lossless: Lossless,
    pub debug: bool,
    pub dt: f64,
    pub current_time: f64,
    pub has_exceeded_angle: bool,
    pub min_time: f64,
    pub traj_stage: i32,
    pub traj_timer: f64,
    pub start_state: String,
    pub end_state: String,
    pub end_stage: i32,
    pub rec: Option<rerun::RecordingStream>,
}

impl Default for Simulation {
    fn default() -> Self {
        Self {
            rocket: Self::get_rocket(),
            mpc: Self::get_mpc(),
            lossless: Self::get_lossless(),
            dt: 0.02,
            current_time: 0.0,
            debug: false,
            has_exceeded_angle: false,
            min_time: 10.0,
            traj_stage: 0,
            traj_timer: 0.0,
            start_state: "ascent".to_string(),
            end_state: "none".to_string(),
            end_stage: 0,
            rec: None,
        }
    }
}

impl Simulation {
    pub fn new(rocket: Rocket, mpc: MPC, lossless: Lossless, dt: f64, current_time: f64, debug: bool, has_exceeded_angle: bool, min_time: f64, traj_stage: i32, traj_timer: f64, start_state: String, end_state: String) -> Self {
        let mut simulation = Self {
            rocket,
            mpc,
            lossless,
            dt,
            current_time,
            debug,
            has_exceeded_angle,
            min_time,
            traj_stage,
            traj_timer,
            start_state,
            end_state,
            end_stage: 0,
            rec: None,
        };

        simulation
    }

    pub fn init(&mut self) {
        if self.debug {
            println!("Connecting to Rerun Viewer...");

            // FIX: Use .connect_grpc() as the compiler suggested.
            // This connects to the 'rerun --web-viewer' running in your other terminal.
            self.rec = Some(
                rerun::RecordingStreamBuilder::new("rocket_sim")
                    .connect_grpc()
                    .expect("🚨 FATAL: Failed to connect to the Rerun viewer!")
            );

            println!("Connected! Sending data...");

            // --- The Rest of Your Simulation Code ---
            
            // 1. Ground
            let _ = self.rec.as_ref().unwrap().log(
                "world/ground",
                &rerun::Boxes3D::from_centers_and_half_sizes(
                    [(0.0, 0.0, -0.05)], // Center: Shift down slightly so y=0 is the top surface
                    [(100.0, 100.0, 0.05)], // Half-sizes: 200x200 wide, 0.1 thick
                )
                .with_colors([rerun::Color::from_rgb(40, 40, 40)]) // Dark Grey
                .with_fill_mode(rerun::FillMode::Solid), // Make it solid, not wireframe
            );
        }

        self.has_exceeded_angle = false;
        
        match &self.start_state {
            val if *val == "ascent".to_string() => self.traj_stage = 0,
            val if *val == "hover".to_string() => self.traj_stage = 1,
            val if *val == "descent".to_string() => self.traj_stage = 2,
            _ => self.traj_stage = 0,
        }
        match &self.end_state {
            val if *val == "ascent".to_string() => self.end_stage = 0,
            val if *val == "hover".to_string() => self.end_stage = 1,
            val if *val == "descent".to_string() => self.end_stage = 2,
            _ => self.end_stage = -1,
        }
    }

    pub fn step(&mut self) -> bool {
        if self.traj_stage == self.end_stage {
            return false;
        }

        let mass = self.rocket.get_mass();

        let world_z_axis = self.rocket.attitude * Vector3::z();
        let cos_theta = world_z_axis.z;
        let _ = cos_theta.clamp(-1.0, 1.0).acos();
        if cos_theta < 0.965925826289{
            self.has_exceeded_angle = true;
        }

        // TODO: get xref_traj and uref_traj in a way that allows for hovering (i.e. automatically toggling lossless usage on/off, will likely need to set up trajectory state machine)
        let xref_traj;
        let uref_traj;
        (xref_traj, uref_traj) = self.get_ref_traj();

        let mut control_input = Vector4::new(0.0, 0.0, 0.0, 0.0);
        let mut outside_forces = Vector3::new(0.0, 0.0, 0.0);
        let mut outside_torques = Vector3::new(0.0, 0.0, 0.0);

        let control_sequence = self.mpc.update(&self.rocket.get_state(), &xref_traj, &uref_traj, mass, &self.rocket.get_moi_mpc(), self.current_time); // Placeholder reference trajectory and warm start
        
        let last_solve_time = self.mpc.last_solve_time;// 1. Calculate the fractional progress between MPC steps (0.0 to 1.0)
        let time_since_mpc_solve = self.current_time - last_solve_time;
        let raw_index = (time_since_mpc_solve / self.mpc.dt) as usize;
        let max_index = control_sequence.len().saturating_sub(1);
        let current_step_index = raw_index.min(max_index);
        let progress = 0.0;
        // 2. Grab the current command (u0) and the next predicted command (u1)
        let u_0 = &control_sequence[current_step_index];
        // Safety check: ensure the sequence actually has a future step to blend into
        let u_1 = if current_step_index + 1 <= max_index {
            &control_sequence[current_step_index + 1]
        } else {
            u_0
        };
        // 3. Interpolate! 
        // Because u_0 and u_1 are ndarray::Array1<f64>, Rust lets you do the vector math directly.
        let current_control = u_0 + progress * (u_1 - u_0);

        let control_input = Vector4::new(current_control[0], current_control[1], current_control[2], 0.0); // Convert first control input to Vector4 (assuming the 4th component is not used for now)

        if !self.rocket.step(control_input, outside_forces, outside_torques, self.dt) && self.current_time > self.min_time {
            return false;
        }
        
        if self.debug {
            println!("XREF_TRAJ{:?}", xref_traj);
            println!("Rocket State: {:?}", self.rocket.get_state());
            println!("Rocket Mass: {}", mass);
            println!("Control Input: {:?}", control_input);
            println!("Total Force: {:?}", self.rocket.debug_info.total_force);
            println!("Thrust Vector: {:?}", self.rocket.thrust_vector);

            // Log Vehicle
            let normalized_thrust_vector = self.rocket.thrust_vector / 1000.0; // Scale for visualization
            let rocket_color;
            if self.has_exceeded_angle {
                rocket_color = rerun::Color::from_rgb(255, 0, 0);
            } else {
                rocket_color = rerun::Color::from_rgb(0, 255, 0);
            }
            let rotated_offset = self.rocket.attitude.transform_vector(&self.rocket.com_to_ground);
            let _ = self.rec.as_ref().unwrap().log(
                "world/rocket",
                &rerun::Arrows3D::from_vectors([((-2.0 * rotated_offset.x) as f32, (-2.0 * rotated_offset.y) as f32, (-2.0 * rotated_offset.z) as f32)])
                    .with_origins([[(self.rocket.position.x+rotated_offset.x) as f32, (self.rocket.position.y+rotated_offset.y) as f32, (self.rocket.position.z+rotated_offset.z) as f32]])
                    .with_colors([rocket_color]) 
            );

            // Log Thrust Vector
            let _ = self.rec.as_ref().unwrap().log(
                "world/thrust_vector",
                &rerun::Arrows3D::from_vectors([((normalized_thrust_vector.x) as f32, (normalized_thrust_vector.y) as f32, (normalized_thrust_vector.z) as f32)])
                    .with_origins([[(self.rocket.position.x+rotated_offset.x) as f32, (self.rocket.position.y+rotated_offset.y) as f32, (self.rocket.position.z+rotated_offset.z) as f32]])
                    .with_colors([rerun::Color::from_rgb(0, 0, 255)]) 
            );

            // Log the rocket's position for visualization
            let _ = self.rec.as_ref().unwrap().log(
                "world/rocket",
                &rerun::Points3D::new([(self.rocket.position.x as f32, self.rocket.position.y as f32, self.rocket.position.z as f32)])
                    .with_colors([rocket_color]) // Red color for the rocket
                    .with_radii([0.1]) // Size of the point representing the rocket
            );

            // Log the rocket's position for visualization
            let _ = self.rec.as_ref().unwrap().log(
                "world/timer",
                &rerun::Points3D::new([(self.current_time as f32, 0.0, 0.0)])
                    .with_colors([rerun::Color::from_rgb(0, 255, 0)]) // Green color for the timer
                    .with_radii([0.1]) // Size of the point representing the timer
            );
        }

        self.current_time += self.dt;

        true
    }

    pub fn finish_sim(&mut self) -> (bool, Array1<f64>) {
        if self.debug {
            let _ = self.rocket.save_debug_to_csv("simulation.csv");

            println!("Nitrogen mass: {}", self.rocket.nitrogen_mass);
            println!("Pressurizing nitrogen mass: {}", self.rocket.pressurizing_nitrogen_mass);
            println!("Nitrous mass: {}", self.rocket.nitrous_mass);
            println!("Fuel grain mass: {}", self.rocket.fuel_grain_mass);
        }

        return (self.has_exceeded_angle, self.rocket.get_state())
    }

    pub fn get_ref_traj(&mut self) -> (Vec<Array1<f64>>, Vec<Array1<f64>>) {
        let xref_traj;
        let uref_traj;

        if self.traj_stage == 0 {
            let trajectory = self.lossless.update([self.rocket.position.x, self.rocket.position.y, self.rocket.position.z], [self.rocket.velocity.x, self.rocket.velocity.y, self.rocket.velocity.z], [0.0, 0.0, 50.0], self.rocket.get_mass() - self.rocket.get_dry_mass(), self.current_time);
            (xref_traj, uref_traj) = self.get_mpc_reference(&trajectory, self.current_time - self.lossless.last_solve_time, self.rocket.attitude, self.mpc.min_thrust, self.mpc.dt, self.lossless.fine_delta_t, self.mpc.n_steps + 1);
            if self.rocket.position.z >= 25.0 {
                self.traj_stage = 1;
                self.traj_timer = self.current_time;
            }
            // uref_traj = vec![Array1::from(vec![0.0, 0.0, mass * 9.81]); mpc.n_steps]; // Warm start with zero control inputs
        } else if self.traj_stage == 1 {
            self.mpc.q = Array2::<f64>::from_diag(&Array1::from(vec![
                80.0, 80.0, 100.0,   // position x, y, z
                600000.0, 600000.0, 600000.0, 0.0, // quaternion qx, qy, qz, qw
                30.0, 30.0, 10.0,        // linear velocities x_dot, y_dot, z_dot
                100.0, 100.0, 100.0          // angular velocities wx, wy, wz
            ]));
            self.mpc.r = Array2::<f64>::from_diag(&Array1::from(vec![50.0, 50.0, 1.0]));
            self.mpc.qn = Array2::<f64>::from_diag(&Array1::from(vec![
                85.0, 85.0, 100.0,   // position x, y, z
                1000000.0, 1000000.0, 1000000.0, 0.0, // quaternion qx, qy, qz, qw
                40.0, 40.0, 60.0,        // linear velocities x_dot, y_dot, z_dot
                50.0, 50.0, 50.0          // angular velocities wx, wy, wz
            ]));
            xref_traj = vec![Array1::from(vec![0.0, 0.0, 50.0, // x, y, z
                                                0.0, 0.0, 0.0, 1.0, // qx, qy, qz, qw (upright)
                                                0.0, 0.0, 0.0, // x_dot, y_dot, z_dot
                                                0.0, 0.0, 0.0]); // wx, wy, wz
                                                self.mpc.n_steps + 1];
            uref_traj = vec![Array1::from(vec![0.0, 0.0, self.rocket.get_mass() * 9.81]); self.mpc.n_steps]; // Warm start with zero control inputs
            if self.current_time - self.traj_timer >= 10.0 {
                self.traj_stage = 2;
                self.traj_timer = self.current_time;
            }
        } else if self.traj_stage == 2 {
            if self.rocket.position.z > 10.0 {
                self.mpc.q = Array2::<f64>::from_diag(&Array1::from(vec![
                    50.0, 50.0, 70.0,   // position x, y, z
                    150000.0, 150000.0, 150000.0, 0.0, // quaternion qx, qy, qz, qw
                    30.0, 30.0, 100.0,        // linear velocities x_dot, y_dot, z_dot
                    500.0, 500.0, 500.0          // angular velocities wx, wy, wz
                ]));
                self.mpc.r = Array2::<f64>::from_diag(&Array1::from(vec![50.0, 50.0, 1.0]));
                self.mpc.qn = Array2::<f64>::from_diag(&Array1::from(vec![
                    20.0, 20.0, 600.0,   // position x, y, z
                    175000.0, 175000.0, 175000.0, 0.0, // quaternion qx, qy, qz, qw
                    30.0, 30.0, 25000.0,        // linear velocities x_dot, y_dot, z_dot
                    1000.0, 1000.0, 1000.0          // angular velocities wx, wy, wz
                ]));
            } else if self.rocket.position.z > 3.0 {
                self.mpc.q = Array2::<f64>::from_diag(&Array1::from(vec![
                    50.0, 50.0, 70.0,   // position x, y, z
                    150000.0, 150000.0, 150000.0, 0.0, // quaternion qx, qy, qz, qw
                    30.0, 30.0, 100.0,        // linear velocities x_dot, y_dot, z_dot
                    500.0, 500.0, 500.0          // angular velocities wx, wy, wz
                ]));
                self.mpc.r = Array2::<f64>::from_diag(&Array1::from(vec![50.0, 50.0, 1.0]));
                self.mpc.qn = Array2::<f64>::from_diag(&Array1::from(vec![
                    10000.0, 10000.0, 1000.0,   // position x, y, z
                    175000.0, 175000.0, 175000.0, 0.0, // quaternion qx, qy, qz, qw
                    30.0, 30.0, 200000.0,        // linear velocities x_dot, y_dot, z_dot
                    1000.0, 1000.0, 1000.0          // angular velocities wx, wy, wz
                ]));
            } else {
                self.mpc.q = Array2::<f64>::from_diag(&Array1::from(vec![
                    50.0, 50.0, 70.0,   // position x, y, z
                    150000.0, 150000.0, 150000.0, 0.0, // quaternion qx, qy, qz, qw
                    30.0, 30.0, 100.0,        // linear velocities x_dot, y_dot, z_dot
                    500.0, 500.0, 500.0          // angular velocities wx, wy, wz
                ]));
                self.mpc.r = Array2::<f64>::from_diag(&Array1::from(vec![50.0, 50.0, 1.0]));
                self.mpc.qn = Array2::<f64>::from_diag(&Array1::from(vec![
                    4000.0, 4000.0, 500.0,   // position x, y, z
                    175000.0, 175000.0, 175000.0, 0.0, // quaternion qx, qy, qz, qw
                    30.0, 30.0, 450000.0,        // linear velocities x_dot, y_dot, z_dot
                    1000.0, 1000.0, 1000.0          // angular velocities wx, wy, wz
                ]));
            }
            self.lossless.lower_thrust_bound = 500.0;
            self.lossless.flip_glide_slope = false;
            self.lossless.use_glide_slope = true;
            self.lossless.max_velocity = 5.0;
            // if self.rocket.velocity.norm() >= 5.0 {
            //     self.lossless.max_velocity = self.rocket.velocity.norm() * 1.25;
            // }
            let trajectory = self.lossless.update([self.rocket.position.x, self.rocket.position.y, self.rocket.position.z], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], self.rocket.get_mass() - self.rocket.get_dry_mass(), self.current_time);
            (xref_traj, uref_traj) = self.get_mpc_reference(&trajectory, self.current_time - self.lossless.last_solve_time, self.rocket.attitude, self.mpc.max_thrust, self.mpc.dt, self.lossless.fine_delta_t, self.mpc.n_steps + 1);
            if self.traj_timer > 20.0 {
                self.traj_stage = -1;
                self.end_stage = -1;
            }
        } else {
            self.traj_stage = 1;
            self.traj_timer = self.current_time;
            self.end_stage = 2;
            return self.get_ref_traj();
        }

        (xref_traj, uref_traj)
    }

    pub fn get_rocket() -> Rocket {
        let position = Vector3::new(0.0, 0.0, 0.0);
        let velocity = Vector3::new(0.0, 0.0, 0.0);
        let acceleration = Vector3::new(0.0, 0.0, 0.0);
        let attitude = UnitQuaternion::identity();
        let angular_velocity = Vector3::new(0.0, 0.0, 0.0);
        let angular_acceleration = Vector3::new(0.0, 0.0, 0.0);

        let frame_mass = 15.66;
        let nitrogen_tank_empty_mass = 15.6;
        let starting_nitrogen_mass = 5.0;
        let nitrogen_tank_offset = Vector3::new(-0.02, -0.01, 0.76);
        let nitrous_tank_empty_mass = 15.86;
        let starting_pressurizing_nitrogen_mass = 0.0;
        let starting_nitrous_mass = 16.0;
        let nitrous_tank_offset = Vector3::new(-0.02, -0.01, 0.592);
        let tvc_module_empty_mass = 8.76;
        let starting_fuel_grain_mass = 3.0;
        let frame_com_to_gimbal = Vector3::new(-0.02, -0.01, -0.24);
        let gimbal_to_tvc_com = Vector3::new(0.0, 0.0, -0.25);
        
        let frame_moi = Matrix3::new(6.65, 0.35, -0.02,
                                    0.35, 6.65, -0.02,
                                    -0.02, -0.02, 2.3);
        let dry_nitrogen_moi = Matrix3::new(0.9275, 0.0, 0.0,
                                            0.0, 0.9275, 0.0,
                                            0.0, 0.0, 1.055);
        let wet_nitrogen_moi = Matrix3::new(1.0075, 0.0, 0.0,
                                            0.0, 1.0075, 0.0,
                                            0.0, 0.0, 1.055);
        let nitrous_tank_radius = 0.0;
        let nitrous_tank_length = 0.0;
        let nitrous_level = 0.85 * 0.75;
        let dry_nitrous_moi = Matrix3::new(1.31, 0.0, 0.0,
                                            0.0, 1.31, 0.0,
                                            0.0, 0.0, 0.21);
        let dry_tvc_moi = Matrix3::new(0.64, 0.0, 0.0,
                                        0.0, 0.64, 0.0,
                                        0.0, 0.0, 0.06);
        let wet_tvc_moi = Matrix3::new(0.84, 0.0, 0.0,
                                        0.0, 0.84, 0.0,
                                        0.0, 0.0, 0.07);
        let tvc_range = 15_f64.to_radians();

        let mtv_angle = 60.0;
        let mtv_ang_vel = 0.0;
        let mtv_ang_accel = 0.0;
        let mtv_unloaded_speed = 60.0;
        let mtv_stall_torque = 70.0;
        let mtv_p_gain = 1000.0;
        let mtv_valve_torque = 30.0;
        let mtv_update_rate = 200.0;
        let mtv = MTV::new(mtv_angle, mtv_ang_vel, mtv_ang_accel, mtv_unloaded_speed, mtv_stall_torque, mtv_p_gain, mtv_valve_torque, starting_fuel_grain_mass, mtv_update_rate);

        let actuator_start_position = 0.0;
        let actuator_extension_limit = 0.08;
        let acuator_unloaded_speed = 0.14986;
        let actuator_stall_force = 102.06;
        let actuator_p_gain = 10.0;
        let actuator_pos_noise_sigma = 0.0;
        let actuator_update_rate = 200.0;
        let x_actuator = TVCActuator::new(actuator_start_position, actuator_extension_limit, acuator_unloaded_speed, actuator_stall_force, actuator_p_gain, actuator_pos_noise_sigma, actuator_update_rate);
        let y_actuator = TVCActuator::new(actuator_start_position, actuator_extension_limit, acuator_unloaded_speed, actuator_stall_force, actuator_p_gain, actuator_pos_noise_sigma, actuator_update_rate);
        
        let tvc_actuator_lever_arm = 0.1;
        // TODO: Whenever properly implementing TVC, fix these values
        let tvc_max_fuel_inertia = 3.0;
        let tvc_min_fuel_inertia = 8.0;
        let tvc = TVC::new(mtv, x_actuator, y_actuator, tvc_actuator_lever_arm, frame_com_to_gimbal, tvc_max_fuel_inertia, tvc_min_fuel_inertia, starting_fuel_grain_mass);

        let rcs_thrust = 10.0;
        let rcs_lever_arm = 0.15;
        let rcs_nitrogen_consumption_rate = 0.1;
        let rcs_update_rate = 10.0;
        let rcs = RCS::new(rcs_thrust, rcs_lever_arm, rcs_nitrogen_consumption_rate, rcs_update_rate);

        let imu_accel_noise_sigma = Vector3::new(0.00137, 0.00137, 0.00137);
        let imu_accel_offset = Vector3::new(0.0, 0.0, 0.0);
        let imu_gyro_noise_sigma = Vector3::new(0.000061, 0.000061, 0.000061    );
        let imu_gyro_drift = Vector3::new(0.0, 0.0, 0.0);
        let imu_mag_noise_sigma = Vector3::new(14.0e-9, 14.0e-9, 14.0e-9);
        let imu_mag_offset = Vector3::new(0.0, 0.0, 0.0);
        let imu_earth_magnetic_field = Vector3::new(
                                                    -0.0000020,  // -2.0 microTesla East
                                                    0.0000220,  // +22.0 microTesla North
                                                    -0.0000443); // -44.3 microTesla Up (Pointing Down!)
        let imu_update_rate = 300.0;
        let imu = IMU::new(imu_accel_noise_sigma, imu_accel_offset, imu_gyro_noise_sigma, imu_gyro_drift, imu_mag_noise_sigma, imu_mag_offset, imu_earth_magnetic_field, imu_update_rate);
        
        let gps_position_noise_sigma = Vector3::zeros();
        let gps_position_offset = Vector3::zeros();
        let gps_update_rate = 5.0;
        let gps = GPS::new(gps_position_noise_sigma, gps_position_offset, gps_update_rate);

        let uwb_position_noise_sigma = Vector3::zeros();
        let uwb_position_offset = Vector3::zeros();
        let uwb_origin = Vector3::zeros();
        let uwb_range = 8.0;
        let uwb_update_rate = 5.0;
        let uwb = UWB::new(uwb_position_noise_sigma, uwb_position_offset, uwb_origin, uwb_range, uwb_update_rate);

        let slosh_model = Self::get_slosh_model();
        let nist_data = NistData::new();
        let nitrogen_iso_data = IsoData::new("isobaric_nitrogen.csv");
        let nitrous_iso_data = IsoData::new("isobaric_liquid_nitrous_oxide.csv");
        let port_d = 0.05;
        let nitrous_m_dot = 0.0;

        let com_to_ground = Vector3::new(0.0, 0.0, -1.5);


        Rocket::new(position, velocity, acceleration, attitude, angular_velocity, angular_acceleration, frame_mass, nitrogen_tank_empty_mass, starting_nitrogen_mass, nitrogen_tank_offset, nitrous_tank_empty_mass, starting_pressurizing_nitrogen_mass, starting_nitrous_mass, nitrous_tank_offset, tvc_module_empty_mass, starting_fuel_grain_mass, frame_com_to_gimbal, gimbal_to_tvc_com, frame_moi, dry_nitrogen_moi, wet_nitrogen_moi, nitrous_tank_radius, nitrous_tank_length, nitrous_level, dry_nitrous_moi, dry_tvc_moi, wet_tvc_moi, tvc_range, tvc, rcs, imu, gps, uwb, slosh_model, nist_data, nitrogen_iso_data, nitrous_iso_data, port_d, nitrous_m_dot, com_to_ground)
    }

    fn get_mpc() -> MPC {
        let n = 13; // [x, y, z, qx, qy, qz, qw, x_dot, y_dot, z_dot, wx, wy, wz]
        let m = 3;  // [gimbal_theta, gimbal_phi, thrust]
        let n_steps = 10;
        let dt = 0.8;
        // let integral_gains = (0.001, 0.001, 0.002);
        let integral_gains = (0.0, 0.0, 0.0);
        let q = Array2::<f64>::from_diag(&Array1::from(vec![
            20.0, 20.0, 70.0,   // position x, y, z
            6000.0, 6000.0, 6000.0, 0.0, // quaternion qx, qy, qz, qw
            30.0, 30.0, 12000.0,        // linear velocities x_dot, y_dot, z_dot
            100.0, 100.0, 100.0          // angular velocities wx, wy, wz
        ]));
        let r = Array2::<f64>::from_diag(&Array1::from(vec![50.0, 50.0, 1.0]));
        let qn = Array2::<f64>::from_diag(&Array1::from(vec![
            40.0, 40.0, 80.0,   // position x, y, z
            10000.0, 10000.0, 10000.0, 0.0, // quaternion qx, qy, qz, qw
            20.0, 20.0, 50000.0,        // linear velocities x_dot, y_dot, z_dot
            50.0, 50.0, 50.0          // angular velocities wx, wy, wz
        ]));
        // let smoothing_weight = Array1::from(vec![150.0, 150.0, -2.0]);
        let smoothing_weight = Array1::from(vec![0.0, 0.0, 0.0]);
        let panoc_cache_tolerance = 1e-4;
        let panoc_cache_lbfgs_memory = 20;
        let min_thrust = 400.0;
        let max_thrust = 1000.0;
        let gimbal_limit = 15_f64.to_radians();
        let system_time = -1.0;
        let update_rate = 50.0;

        MPC::new(n, m, n_steps, dt, integral_gains, q, r, qn, smoothing_weight, panoc_cache_tolerance, panoc_cache_lbfgs_memory, min_thrust, max_thrust, gimbal_limit, system_time, update_rate)
    }

    fn get_lossless() -> Lossless {
        let max_velocity = 10.0;
        let dry_mass = 61.0;
        let alpha = 1.0 / (9.81 * 180.0);
        let lower_thrust_bound = 400.0;
        let upper_thrust_bound = 900.0;
        let tvc_range_rad = 15_f64.to_radians();
        let coarse_delta_t = 0.25;
        let fine_delta_t = 0.1;
        let glide_slope = 0.05_f64.to_radians();
        let use_glide_slope = true;
        let flip_glide_slope = true;
        let system_time = -1.0;
        let update_rate = 3.0;

        Lossless::new(max_velocity, dry_mass, alpha, lower_thrust_bound, upper_thrust_bound, tvc_range_rad, coarse_delta_t, fine_delta_t, glide_slope, use_glide_slope, flip_glide_slope, [0.0; 3], system_time, update_rate)
    }

    fn get_slosh_model() -> SloshModel {
        let radius_m = 0.12;
        let length_m = 0.75;
        let density_liquid = 731.18;
        let gravity = 9.81;
        let tank_config = TankConfig::new(radius_m, length_m, density_liquid, gravity);

        let m_frac = 0.5;
        let c_lin = 200.0;
        let alpha = 0.0;
        let omega_scale = 1.0;
        let slosh_params = SloshParams::new(m_frac, c_lin, alpha, omega_scale);

        let initial_fill_frac = 0.85;

        let slosh_state = SloshState::default();

        SloshModel::new(tank_config, slosh_params, initial_fill_frac, slosh_state)
    }

    // TODO: make all the print satements locked behind an if self.debug {} statement
    pub fn get_mpc_reference(
        &mut self,
        traj: &lossless::TrajectoryResult,
        current_time: f64,
        initial_attitude: UnitQuaternion<f64>,
        default_thrust: f64,
        mpc_dt: f64,
        lossless_dt: f64,
        n_steps: usize,
    ) -> (Vec<Array1<f64>>, Vec<Array1<f64>>) {
        let mut xref_traj = Vec::with_capacity(n_steps);
        let num_points = traj.positions.len();
        if num_points == 0 {
            panic!("TrajectoryResult is empty!");
        }

        let mut uref_traj = Vec::with_capacity(n_steps - 1);
        let real_thrusts: Vec<[f64; 1]> = traj.thrusts.iter()
            .zip(&traj.masses)
            .map(|(u, &m)| {
                let world_thrust = Vector3::new(u[0] * m, u[1] * m, u[2] * m);
                [world_thrust.norm()] 
            })
            .collect();
        let num_thrusts = traj.thrusts.len();

        // Generate the state for the current time, plus each future step in the horizon
        for i in 0..=n_steps {
            // 1. What exact time is the MPC predicting for this step?
            let t_target = current_time + (i as f64) * mpc_dt;

            let mut interp_p = [0.0; 3];
            let mut interp_v = [0.0; 3];
            let mut interp_u = [0.0; 3];

            // 2. Are we past the end of the flight profile?
            if t_target >= traj.time_of_flight_s || num_points == 1 {
                // Park the rocket at the final coordinate
                interp_p = traj.positions[num_points - 1];
                interp_v = [0.0, 0.0, 0.0]; // Force target velocity to 0 to hold the hover
                interp_u = [0.0, 0.0, default_thrust];
                
            } else if t_target <= 0.0 {
                // We haven't launched yet (or t_target is 0)
                interp_p = traj.positions[0];
                interp_v = traj.velocities[0];
                if num_thrusts > 0 {
                    interp_u = [0.0, 0.0, real_thrusts[0][0]];
                }
                
            } else {
                // 3. Interpolate perfectly between the two closest trajectory points
                let exact_idx = t_target / lossless_dt;
                let base_idx = exact_idx.floor() as usize;
                
                // Prevent out-of-bounds on the array lookup
                let safe_idx = base_idx.min(num_points.saturating_sub(2));
                
                // Calculate the fraction (0.0 to 1.0) between the two points
                // let frac = (t_target - (safe_idx as f64) * lossless_dt) / lossless_dt;
                // let clamped_frac = frac.clamp(0.0, 1.0);
                // 3. Subtract to get the float portion, then clamp to lock out-of-bounds to 1.0
                let clamped_frac = (exact_idx - safe_idx as f64).clamp(0.0, 1.0);

                let p0 = traj.positions[safe_idx];
                let p1 = traj.positions[safe_idx + 1];
                let v0 = traj.velocities[safe_idx];
                let v1 = traj.velocities[safe_idx + 1];

                let max_u_idx = num_thrusts.saturating_sub(2);
                let idx_u = base_idx.min(max_u_idx);
                let frac_u = (exact_idx - idx_u as f64).clamp(0.0, 1.0);

                let u0 = if num_thrusts > 0 { [0.0, 0.0, real_thrusts[idx_u][0]] } else { [0.0, 0.0, default_thrust] };
                let u1 = if num_thrusts > 1 { [0.0, 0.0, real_thrusts[idx_u + 1][0]] } else { u0 };

                for j in 0..3 {
                    interp_p[j] = p0[j] + clamped_frac * (p1[j] - p0[j]);
                    interp_v[j] = v0[j] + clamped_frac * (v1[j] - v0[j]);
                    interp_u[j] = u0[j] + clamped_frac * (u1[j] - u0[j]);
                }
            }

            // 4. Map the interpolated data into the 13-element MPC state array
            // Order: [x, y, z, qx, qy, qz, qw, x_dot, y_dot, z_dot, wx, wy, wz]
            let state = Array1::from(vec![
                interp_p[0], interp_p[1], interp_p[2], // Target Position
                0.0, 0.0, 0.0, 1.0,                    // Target Attitude (Upright)
                interp_v[0], interp_v[1], interp_v[2], // Target Velocity
                0.0, 0.0, 0.0                          // Target Angular Velocity (Zero spin)
            ]);

            xref_traj.push(state);
            // Push controls ONLY for the current step and future steps, excluding the terminal boundary
            if i < n_steps - 1 {
                uref_traj.push(Array1::from(vec![interp_u[0], interp_u[1], interp_u[2]]));
            }
        }

        println!("UREF_TRAJ: {:?}", uref_traj);
        
        (xref_traj, uref_traj)
    }
}
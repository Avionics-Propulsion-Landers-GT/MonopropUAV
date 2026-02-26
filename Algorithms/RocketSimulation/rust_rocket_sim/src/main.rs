mod rocket_dynamics;
mod device_sim;
mod algorithms;
mod sloshing_sim;
mod fluid_dynamics;
use crate::rocket_dynamics::*;
use crate::device_sim::*;
use crate::algorithms::*;
use crate::sloshing_sim::*;
use crate::fluid_dynamics::*;
use nalgebra::{Matrix3, Vector3, Vector4, UnitQuaternion};
use ndarray::{Array1, Array2};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Connecting to Rerun Viewer...");

    // FIX: Use .connect_grpc() as the compiler suggested.
    // This connects to the 'rerun --web-viewer' running in your other terminal.
    let rec = rerun::RecordingStreamBuilder::new("rocket_sim")
        .connect_grpc()?; 

    println!("Connected! Sending data...");

    // --- The Rest of Your Simulation Code ---
    
    // 1. Ground
    rec.log(
        "world/ground",
        &rerun::Boxes3D::from_centers_and_half_sizes(
            [(0.0, 0.0, -0.05)], // Center: Shift down slightly so y=0 is the top surface
            [(100.0, 100.0, 0.05)], // Half-sizes: 200x200 wide, 0.1 thick
        )
        .with_colors([rerun::Color::from_rgb(40, 40, 40)]) // Dark Grey
        .with_fill_mode(rerun::FillMode::Solid), // Make it solid, not wireframe
    )?;

    let mut rocket = get_rocket();

    let mut mpc = get_mpc();
    let mut hover_u_warm = vec![Array1::from(vec![0.0, 0.0, rocket.get_mass() * 9.81]); mpc.n_steps]; // Warm start with zero control inputs

    let mut lossless = get_lossless();
    let mut at_hover = -1;
    let mut hover_start_time = 0.0;

    let mut control_input = Vector4::new(0.0, 0.0, 0.0, 0.0);
    let mut outside_forces = Vector3::new(0.0, 0.0, 0.0);
    let mut outside_torques = Vector3::new(0.0, 0.0, 0.0);
    let mut current_time = 0.0;
    let dt = 0.02; // 20 ms time step
    let min_time = 10.0;
    loop {
        // For this example, we'll just keep the control input zero and not apply any outside forces or torques.
        // In a real simulation, you'd update these based on your control algorithms and environmental effects.
        let mass = rocket.get_mass();
        // hover_u_warm = vec![Array1::from(vec![0.0, 0.0, mass * 9.81]); mpc.n_steps]; // Warm start with zero control inputs
        
        let mut xref_traj;
        if at_hover == -1 {
            let mut trajectory  = lossless.update([rocket.position.x, rocket.position.y, rocket.position.z], [rocket.velocity.x, rocket.velocity.y, rocket.velocity.z], [0.0, 0.0, 50.0], mass - rocket.get_dry_mass(), current_time);
            xref_traj = get_mpc_reference(&trajectory, current_time - lossless.last_solve_time, mpc.dt, lossless.fine_delta_t, mpc.n_steps + 1);
            if rocket.position.z >= 45.0 {
                at_hover = 0;
                hover_start_time = current_time;
            }
        } else if at_hover == 0 {
            xref_traj = vec![Array1::from(vec![0.0, 0.0, 50.0, // x, y, z
                                                0.0, 0.0, 0.0, 1.0, // qx, qy, qz, qw (upright)
                                                0.0, 0.0, 0.0, // x_dot, y_dot, z_dot
                                                0.0, 0.0, 0.0]); // wx, wy, wz
                                                mpc.n_steps + 1];
            if current_time - hover_start_time >= 10.0 {
                at_hover = 1;
            }
        } else {
            mpc.q = Array2::<f64>::from_diag(&Array1::from(vec![
                20.0, 20.0, 200.0,   // position x, y, z
                300.0, 300.0, 300.0, 0.0, // quaternion qx, qy, qz, qw
                30.0, 30.0, 300.0,        // linear velocities x_dot, y_dot, z_dot
                10.0, 10.0, 10.0          // angular velocities wx, wy, wz
            ]));
            mpc.r = Array2::<f64>::from_diag(&Array1::from(vec![50.0, 50.0, 0.0]));
            mpc.qn = Array2::<f64>::from_diag(&Array1::from(vec![
                20.0, 20.0, 300.0,   // position x, y, z
                350.0, 350.0, 350.0, 0.0, // quaternion qx, qy, qz, qw
                40.0, 40.0, 400.0,        // linear velocities x_dot, y_dot, z_dot
                5.0, 5.0, 5.0          // angular velocities wx, wy, wz
            ]));
            lossless.use_glide_slope = true;
            lossless.max_velocity = 10.0;
            // if rocket.velocity.norm() >= 5.0 {
            //     lossless.max_velocity = rocket.velocity.norm() * 1.25;
            // }
            let mut trajectory = lossless.update([rocket.position.x, rocket.position.y, rocket.position.z], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], mass - rocket.get_dry_mass(), current_time);
            xref_traj = get_mpc_reference(&trajectory, current_time - lossless.last_solve_time, mpc.dt, lossless.fine_delta_t, mpc.n_steps + 1);
        }

        println!("XREF_TRAJ{:?}", xref_traj);

        let control_sequence = mpc.update(&rocket.get_state(), &xref_traj, &hover_u_warm, mass, current_time); // Placeholder reference trajectory and warm start
        let last_solve_time = mpc.last_solve_time;// 1. Calculate the fractional progress between MPC steps (0.0 to 1.0)
        let time_since_solve = current_time - last_solve_time;
        let raw_index = (time_since_solve / mpc.dt) as usize;
        let max_index = control_sequence.len().saturating_sub(1);
        let current_step_index = raw_index.min(max_index);
        // let progress = (time_since_solve % mpc.dt) / mpc.dt;
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
        // let control_input = Vector4::new(0.0, 0.0, mass * 9.81, 0.0); // Override with hover control input for testing

        println!("Rocket State: {:?}", rocket.get_state());
        println!("Rocket Mass: {}", mass);
        println!("Control Input: {:?}", control_input);
        println!("Total Force: {:?}", rocket.debug_info.total_force);
        println!("Thrust Vector: {:?}", rocket.thrust_vector);

        if !rocket.step(control_input, outside_forces, outside_torques, dt) && current_time > min_time {
            break;
        }

        // Log Vehicle
        let rotated_offset = rocket.attitude.transform_vector(&rocket.com_to_ground);
        rec.log(
            "world/rocket",
            &rerun::Arrows3D::from_vectors([((-2.0 * rotated_offset.x) as f32, (-2.0 * rotated_offset.y) as f32, (-2.0 * rotated_offset.z) as f32)])
                .with_origins([[(rocket.position.x+rotated_offset.x) as f32, (rocket.position.y+rotated_offset.y) as f32, (rocket.position.z+rotated_offset.z) as f32]])
                .with_colors([rerun::Color::from_rgb(255, 0, 0)]) 
        )?;

        // Log Thrust Vector
        let normalized_thrust_vector = rocket.thrust_vector / 1000.0; // Scale for visualization
        rec.log(
            "world/thrust_vector",
            &rerun::Arrows3D::from_vectors([((normalized_thrust_vector.x) as f32, (normalized_thrust_vector.y) as f32, (normalized_thrust_vector.z) as f32)])
                .with_origins([[(rocket.position.x+rotated_offset.x) as f32, (rocket.position.y+rotated_offset.y) as f32, (rocket.position.z+rotated_offset.z) as f32]])
                .with_colors([rerun::Color::from_rgb(0, 0, 255)]) 
        )?;

        // Log the rocket's position for visualization
        rec.log(
            "world/rocket",
            &rerun::Points3D::new([(rocket.position.x as f32, rocket.position.y as f32, rocket.position.z as f32)])
                .with_colors([rerun::Color::from_rgb(255, 0, 0)]) // Red color for the rocket
                .with_radii([0.1]) // Size of the point representing the rocket
        )?;

        // Log the rocket's position for visualization
        rec.log(
            "world/timer",
            &rerun::Points3D::new([(current_time as f32, 0.0, 0.0)])
                .with_colors([rerun::Color::from_rgb(0, 255, 0)]) // Green color for the timer
                .with_radii([0.1]) // Size of the point representing the timer
        )?;

        current_time += dt;
        // std::thread::sleep(std::time::Duration::from_millis(20)); // Sleep to simulate real-time progression
    }

    println!("Nitrogen mass: {}", rocket.nitrogen_mass);
    println!("Pressurizing nitrogen mass: {}", rocket.pressurizing_nitrogen_mass);
    println!("Nitrous mass: {}", rocket.nitrous_mass);
    println!("Fuel grain mass: {}", rocket.fuel_grain_mass);

    Ok(())
}

fn get_rocket() -> Rocket {
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
    let actuator_extension_limit = 0.3;
    let acuator_unloaded_speed = 2.0;
    let actuator_stall_torque = 300.0;
    let actuator_p_gain = 10.0;
    let actuator_pos_noise_sigma = 0.0;
    let actuator_update_rate = 200.0;
    let x_actuator = TVCActuator::new(actuator_start_position, actuator_extension_limit, acuator_unloaded_speed, actuator_stall_torque, actuator_p_gain, actuator_pos_noise_sigma, actuator_update_rate);
    let y_actuator = TVCActuator::new(actuator_start_position, actuator_extension_limit, acuator_unloaded_speed, actuator_stall_torque, actuator_p_gain, actuator_pos_noise_sigma, actuator_update_rate);
    
    let tvc_actuator_lever_arm = 0.15;
    let tvc_max_fuel_inertia = 10.0;
    let tvc_max_fuel_inertia = 3.0;
    let tvc_min_fuel_inertia = 8.0;
    let tvc = TVC::new(mtv, x_actuator, y_actuator, tvc_actuator_lever_arm, frame_com_to_gimbal, tvc_max_fuel_inertia, tvc_min_fuel_inertia, starting_fuel_grain_mass);

    let rcs_thrust = 10.0;
    let rcs_lever_arm = 0.15;
    let rcs_nitrogen_consumption_rate = 0.1;
    let rcs_update_rate = 10.0;
    let rcs = RCS::new(rcs_thrust, rcs_lever_arm, rcs_nitrogen_consumption_rate, rcs_update_rate);

    let imu_accel_noise_sigma = Vector3::zeros();
    let imu_accel_offset = Vector3::zeros();
    let imu_gyro_noise_sigma = Vector3::zeros();
    let imu_gyro_drift = Vector3::zeros();
    let imu_mag_noise_sigma = Vector3::zeros();
    let imu_mag_offset = Vector3::zeros();
    let imu_earth_magnetic_field = Vector3::new(0.000005, 0.0, 0.0);
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

    let slosh_model = get_slosh_model();
    let nist_data = NistData::new();
    let nitrogen_iso_data = IsoData::new("isobaric_nitrogen.csv");
    let nitrous_iso_data = IsoData::new("isobaric_liquid_nitrous_oxide.csv");
    let port_d = 0.05;

    let com_to_ground = Vector3::new(0.0, 0.0, -1.5);


    Rocket::new(position, velocity, acceleration, attitude, angular_velocity, angular_acceleration, frame_mass, nitrogen_tank_empty_mass, starting_nitrogen_mass, nitrogen_tank_offset, nitrous_tank_empty_mass, starting_pressurizing_nitrogen_mass, starting_nitrous_mass, nitrous_tank_offset, tvc_module_empty_mass, starting_fuel_grain_mass, frame_com_to_gimbal, gimbal_to_tvc_com, frame_moi, dry_nitrogen_moi, wet_nitrogen_moi, nitrous_tank_radius, nitrous_tank_length, nitrous_level, dry_nitrous_moi, dry_tvc_moi, wet_tvc_moi, tvc_range, tvc, rcs, imu, gps, uwb, slosh_model, nist_data, nitrogen_iso_data, nitrous_iso_data, port_d, com_to_ground)
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
        120.0, 120.0, 120.0, 0.0, // quaternion qx, qy, qz, qw
        30.0, 30.0, 10.0,        // linear velocities x_dot, y_dot, z_dot
        10.0, 10.0, 10.0          // angular velocities wx, wy, wz
    ]));
    let r = Array2::<f64>::from_diag(&Array1::from(vec![50.0, 50.0, 0.0]));
    let qn = Array2::<f64>::from_diag(&Array1::from(vec![
        20.0, 20.0, 800.0,   // position x, y, z
        150.0, 150.0, 150.0, 0.0, // quaternion qx, qy, qz, qw
        40.0, 40.0, 50.0,        // linear velocities x_dot, y_dot, z_dot
        5.0, 5.0, 5.0          // angular velocities wx, wy, wz
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
    let max_velocity = 50.0;
    let dry_mass = 61.0;
    let alpha = 1.0 / (9.81 * 180.0);
    let lower_thrust_bound = 400.0;
    let upper_thrust_bound = 1000.0;
    let tvc_range_rad = 15_f64.to_radians();
    let coarse_delta_t = 0.25;
    let fine_delta_t = 0.1;
    let glide_slope = 5.0_f64.to_radians();
    let use_glide_slope = false;
    let system_time = -1.0;
    let update_rate = 3.0;

    Lossless::new(max_velocity, dry_mass, alpha, lower_thrust_bound, upper_thrust_bound, tvc_range_rad, coarse_delta_t, fine_delta_t, glide_slope, use_glide_slope, [0.0; 3], system_time, update_rate)
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

pub fn get_mpc_reference(
    traj: &algorithms::lossless::TrajectoryResult,
    current_time: f64,
    mpc_dt: f64,
    lossless_dt: f64,
    n_steps: usize,
) -> Vec<Array1<f64>> {
    let mut mpc_reference = Vec::with_capacity(n_steps + 1);
    let num_points = traj.positions.len();

    if num_points == 0 {
        panic!("TrajectoryResult is empty!");
    }

    // Generate the state for the current time, plus each future step in the horizon
    for i in 0..=n_steps {
        // 1. What exact time is the MPC predicting for this step?
        let t_target = current_time + (i as f64) * mpc_dt;

        let mut interp_p = [0.0; 3];
        let mut interp_v = [0.0; 3];

        // 2. Are we past the end of the flight profile?
        if t_target >= traj.time_of_flight_s || num_points == 1 {
            // Park the rocket at the final coordinate
            interp_p = traj.positions[num_points - 1];
            interp_v = [0.0, 0.0, 0.0]; // Force target velocity to 0 to hold the hover
            
        } else if t_target <= 0.0 {
            // We haven't launched yet (or t_target is 0)
            interp_p = traj.positions[0];
            interp_v = traj.velocities[0];
            
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

            for j in 0..3 {
                interp_p[j] = p0[j] + clamped_frac * (p1[j] - p0[j]);
                interp_v[j] = v0[j] + clamped_frac * (v1[j] - v0[j]);
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

        mpc_reference.push(state);
    }

    mpc_reference
}
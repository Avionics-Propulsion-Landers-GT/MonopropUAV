mod rocket_dynamics;
mod device_sim;
mod algorithms;
use crate::rocket_dynamics::*;
use crate::device_sim::*;
use crate::algorithms::*;
use nalgebra::{Vector3, Vector4, UnitQuaternion};
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
    let rocket_init_state = rocket.get_state();
    let hover_xref_traj = vec![rocket_init_state; mpc.n_steps + 1]; // Reference trajectory to hover at the initial position
    let mut hover_u_warm = vec![Array1::from(vec![0.0, 0.0, rocket.get_mass() * 9.81]); mpc.n_steps]; // Warm start with zero control inputs

    let mut control_input = Vector4::new(0.0, 0.0, 0.0, 0.0);
    let mut outside_forces = Vector3::new(0.0, 0.0, 0.0);
    let mut outside_torques = Vector3::new(0.0, 0.0, 0.0);
    let dt = 0.02; // 20 ms time step
    loop {
        // For this example, we'll just keep the control input zero and not apply any outside forces or torques.
        // In a real simulation, you'd update these based on your control algorithms and environmental effects.
        let mass = rocket.get_mass();
        // hover_u_warm = vec![Array1::from(vec![0.0, 0.0, mass * 9.81]); mpc.n_steps]; // Warm start with zero control inputs

        let control_sequence = mpc.solve(&rocket.get_state(), &hover_xref_traj, &hover_u_warm, mass); // Placeholder reference trajectory and warm start

        let control_input = Vector4::new(control_sequence[1][0], control_sequence[1][1], control_sequence[1][2], 0.0); // Convert first control input to Vector4 (assuming the 4th component is not used for now)
        // let control_input = Vector4::new(0.0, 0.0, mass * 9.81, 0.0); // Override with hover control input for testing

        println!("Rocket State: {:?}", rocket.get_state());
        println!("Rocket Mass: {}", mass);
        println!("Control Input: {:?}", control_input);
        println!("Total Force: {:?}", rocket.debug_info.total_force);
        println!("Thrust Vector: {:?}", rocket.debug_info.thrust_vector);

        if !rocket.step(control_input, outside_forces, outside_torques, dt) {
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
        let normalized_thrust_vector = rocket.debug_info.thrust_vector / 1000.0; // Scale for visualization
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

        // std::thread::sleep(std::time::Duration::from_millis(20)); // Sleep to simulate real-time progression
    }

    println!("Nitrogen mass: {}", rocket.nitrogen_mass);
    println!("Pressurizing nitrogen mass: {}", rocket.pressurizing_nitrogen_mass);
    println!("Nitrous mass: {}", rocket.nitrous_mass);
    println!("Fuel grain mass: {}", rocket.fuel_grain_mass);

    Ok(())
}

fn get_rocket() -> Rocket {
    let position = Vector3::new(0.0, 0.0, 50.0);
    let velocity = Vector3::new(0.0, 0.0, 0.0);
    let acceleration = Vector3::new(0.0, 0.0, 0.0);
    let attitude = UnitQuaternion::identity();
    let angular_velocity = Vector3::new(0.0, 0.0, 0.0);
    let angular_acceleration = Vector3::new(0.0, 0.0, 0.0);

    let dry_mass = 50.0;
    let nitrogen_tank_empty_mass = 0.0;
    let starting_nitrogen_mass = 5.0;
    let nitrogen_tank_empty_offset = Vector3::new(0.0, 0.0, 0.0);
    let nitrogen_tank_full_offset = Vector3::new(0.0, 0.0, 0.0);
    let nitrous_tank_empty_mass = 0.0;
    let starting_pressurizing_nitrogen_mass = 5.0;
    let starting_nitrous_mass = 20.0;
    let nitrous_tank_empty_offset = Vector3::new(0.0, 0.0, 0.0);
    let nitrous_tank_full_offset = Vector3::new(0.0, 0.0, 0.0);
    let nitrous_tank_depleted_offset = Vector3::new(0.0, 0.0, 0.0);
    let tvc_module_empty_mass = 0.0;
    let starting_fuel_grain_mass = 4.0;
    let frame_com_to_gimbal = Vector3::new(0.0, 0.0, -1.0);
    let gimbal_to_tvc_com = Vector3::new(0.0, 0.0, 0.0);
    let inertia_tensor = Vector3::new(15.0, 15.0, 8.0);
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

    let com_to_ground = Vector3::new(0.0, 0.0, -1.5);


    Rocket::new(position, velocity, acceleration, attitude, angular_velocity, angular_acceleration, dry_mass, nitrogen_tank_empty_mass, starting_nitrogen_mass, nitrogen_tank_empty_offset, nitrogen_tank_full_offset, nitrous_tank_empty_mass, starting_pressurizing_nitrogen_mass, starting_nitrous_mass, nitrous_tank_empty_offset, nitrous_tank_full_offset, nitrous_tank_depleted_offset, tvc_module_empty_mass, starting_fuel_grain_mass, frame_com_to_gimbal, gimbal_to_tvc_com, inertia_tensor, tvc_range, tvc, rcs, imu, gps, uwb, com_to_ground)
}

fn get_mpc() -> MPC {
    let n = 13; // [x, y, z, qx, qy, qz, qw, x_dot, y_dot, z_dot, wx, wy, wz]
    let m = 3;  // [gimbal_theta, gimbal_phi, thrust]
    let n_steps = 10;
    let dt = 0.1;
    let integral_gains = (0.01, 0.01, 0.2);
    // let integral_gains = (0.0, 0.0, 0.0);
    let q = Array2::<f64>::from_diag(&Array1::from(vec![
        2000.0, 2000.0, 20000.0,   // position x, y, z
        5000.0, 5000.0, 10.0, 0.0, // quaternion qx, qy, qz, qw
        30.0, 30.0, 10.0,        // linear velocities x_dot, y_dot, z_dot
        10.0, 10.0, 10.0          // angular velocities wx, wy, wz
    ]));
    let r = Array2::<f64>::from_diag(&Array1::from(vec![5.0, 5.0, 0.0]));
    let qn = Array2::<f64>::from_diag(&Array1::from(vec![
        2000.0, 2000.0, 20000.0,   // position x, y, z
        5000.0, 5000.0, 10.0, 10.0, // quaternion qx, qy, qz, qw
        10.0, 10.0, 10.0,        // linear velocities x_dot, y_dot, z_dot
        5.0, 5.0, 5.0          // angular velocities wx, wy, wz
    ]));
    let smoothing_weight = Array1::from(vec![15000.0, 15000.0, 0.0002]);
    let smoothing_weight = Array1::from(vec![0.0, 0.0, 0.0]);
    let panoc_cache_tolerance = 1e-4;
    let panoc_cache_lbfgs_memory = 20;
    let min_thrust = 400.0;
    let max_thrust = 3000.0;
    let gimbal_limit = 15_f64.to_radians();

    MPC::new(n, m, n_steps, dt, integral_gains, q, r, qn, smoothing_weight, panoc_cache_tolerance, panoc_cache_lbfgs_memory, min_thrust, max_thrust, gimbal_limit)
}
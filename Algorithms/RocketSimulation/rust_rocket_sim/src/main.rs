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
    let hover_xref_traj = vec![rocket.get_state(); mpc.n_steps + 1]; // Reference trajectory to hover at the initial position
    let hover_u_warm = vec![Array1::from(vec![0.0, 0.0, rocket.get_mass() * 9.81]); mpc.n_steps + 1]; // Warm start with zero control inputs

    let mut control_input = Vector4::new(0.0, 0.0, 0.0, 0.0);
    let mut outside_forces = Vector3::new(0.0, 0.0, 0.0);
    let mut outside_torques = Vector3::new(0.0, 0.0, 0.0);
    let dt = 0.02; // 20 ms time step
    loop {
        // For this example, we'll just keep the control input zero and not apply any outside forces or torques.
        // In a real simulation, you'd update these based on your control algorithms and environmental effects.
        let control_sequence = mpc.solve(&rocket.get_state(), &hover_xref_traj, &hover_u_warm); // Placeholder reference trajectory and warm start
        
        let control_input = Vector4::new(control_sequence[0][0], control_sequence[0][1], control_sequence[0][2], 0.0); // Convert first control input to Vector4 (assuming the 4th component is not used for now)

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
        
        // Log the rocket's position for visualization
        rec.log(
            "world/rocket",
            &rerun::Points3D::new([(rocket.position.x as f32, rocket.position.y as f32, rocket.position.z as f32)])
                .with_colors([rerun::Color::from_rgb(255, 0, 0)]) // Red color for the rocket
                .with_radii([0.1]) // Size of the point representing the rocket
        )?;

        std::thread::sleep(std::time::Duration::from_millis(20)); // Sleep to simulate real-time progression
    }

    Ok(())
}

fn get_rocket() -> Rocket {
    let position = Vector3::new(0.0, 0.0, 10.0);
    let velocity = Vector3::new(0.0, 0.0, 0.0);
    let acceleration = Vector3::new(0.0, 0.0, 0.0);
    let attitude = UnitQuaternion::identity();
    let angular_velocity = Vector3::new(0.0, 0.0, 0.0);
    let angular_acceleration = Vector3::new(0.0, 0.0, 0.0);
    let u = [0.0, 0.0, 600.0, 0.0];

    let dry_mass = 50.0;
    let nitrogen_tank_empty_mass = 8.0;
    let starting_nitrogen_mass = 10.0;
    let nitrogen_tank_empty_offset = Vector3::new(0.0, 0.0, 0.0);
    let nitrogen_tank_full_offset = Vector3::new(0.0, 0.0, 0.0);
    let nitrous_tank_empty_mass = 8.0;
    let starting_pressurizing_nitrogen_mass = 5.0;
    let starting_nitrous_mass = 20.0;
    let nitrous_tank_empty_offset = Vector3::new(0.0, 0.0, 0.0);
    let nitrous_tank_full_offset = Vector3::new(0.0, 0.0, 0.0);
    let nitrous_tank_depleted_offset = Vector3::new(0.0, 0.0, 0.0);
    let tvc_module_empty_mass = 5.0;
    let starting_fuel_grain_mass = 4.0;
    let frame_com_to_gimbal = Vector3::new(0.0, 0.0, 0.0);
    let gimbal_to_tvc_com = Vector3::new(0.0, 0.0, 0.0);
    let inertia_tensor = Vector3::new(15.0, 15.0, 8.0);
    let tvc_range = 15_f64.to_radians();

    let mtv = MTV::new(0.0, 0.0, 0.0, 60.0, 70.0, 10.0, 30.0, 8.0, 200.0);
    let x_actuator = TVCActuator::new(0.0, 0.3, 2.0, 300.0, 10.0, 0.0, 200.0);
    let y_actuator = TVCActuator::new(0.0, 0.3, 2.0, 300.0, 10.0, 0.0, 200.0);
    let tvc = TVC::new(mtv, x_actuator, y_actuator, 0.15, Vector3::new(0.0, 0.0, -1.0), 10.0, 3.0, 8.0);
    let rcs = RCS::new(10.0, 0.15, 0.1, 10.0);
    let imu = IMU::new(Vector3::zeros(), Vector3::zeros(), Vector3::zeros(), Vector3::zeros(), Vector3::zeros(), Vector3::zeros(), Vector3::new(0.000005, 0.0, 0.0), 300.0);
    let gps = GPS::new(Vector3::zeros(), Vector3::zeros(), 5.0);
    let uwb = UWB::new(Vector3::zeros(), Vector3::zeros(), Vector3::zeros(), 8.0, 5.0);

    let com_to_ground = Vector3::new(0.0, 0.0, -1.5);


    Rocket::new(position, velocity, acceleration, attitude, angular_velocity, angular_acceleration, dry_mass, nitrogen_tank_empty_mass, starting_nitrogen_mass, nitrogen_tank_empty_offset, nitrogen_tank_full_offset, nitrous_tank_empty_mass, starting_pressurizing_nitrogen_mass, starting_nitrous_mass, nitrous_tank_empty_offset, nitrous_tank_full_offset, nitrous_tank_depleted_offset, tvc_module_empty_mass, starting_fuel_grain_mass, frame_com_to_gimbal, gimbal_to_tvc_com, inertia_tensor, tvc_range, tvc, rcs, imu, gps, uwb, com_to_ground)
}

fn get_mpc() -> MPC {
    let n = 13; // [x, y, z, qx, qy, qz, qw, x_dot, y_dot, z_dot, wx, wy, wz]
    let m = 3;  // [gimbal_theta, gimbal_phi, thrust]
    let n_steps = 10;
    let dt = 0.1;
    let integral_gains = (0.01, 0.01, 0.2);
    let q = Array2::<f64>::from_diag(&Array1::from(vec![
        20.0, 20.0, 200.0,   // position x, y, z
        0.0, 0.0, 0.0, 0.0, // quaternion qx, qy, qz, qw
        3.0, 3.0, 1.0,        // linear velocities x_dot, y_dot, z_dot
        1.0, 1.0, 1.0          // angular velocities wx, wy, wz
    ]));
    let r = Array2::<f64>::from_diag(&Array1::from(vec![0.0, 0.0, 0.0]));
    let qn = Array2::<f64>::from_diag(&Array1::from(vec![
        200.0, 200.0, 5000.0,   // position x, y, z
        10.0, 10.0, 10.0, 10.0, // quaternion qx, qy, qz, qw
        10.0, 10.0, 10.0,        // linear velocities x_dot, y_dot, z_dot
        5.0, 5.0, 5.0          // angular velocities wx, wy, wz
    ]));
    let smoothing_weight = Array1::from(vec![1500.0, 1500.0, 0.02]);
    let panoc_cache_tolerance = 1e-4;
    let panoc_cache_lbfgs_memory = 20;
    let min_thrust = 400.0;
    let max_thrust = 1000.0;
    let gimbal_limit = 15_f64.to_radians();

    MPC::new(n, m, n_steps, dt, integral_gains, q, r, qn, smoothing_weight, panoc_cache_tolerance, panoc_cache_lbfgs_memory, min_thrust, max_thrust, gimbal_limit)
}
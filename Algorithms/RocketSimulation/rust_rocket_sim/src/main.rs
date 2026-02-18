mod rocket_dynamics;
mod device_sim;
mod algorithms;
use crate::rocket_dynamics::Rocket;
use crate::device_sim::*;
use nalgebra::{Vector3, Vector4, UnitQuaternion};

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


    let mut rocket = Rocket::new(position, velocity, acceleration, attitude, angular_velocity, angular_acceleration, dry_mass, nitrogen_tank_empty_mass, starting_nitrogen_mass, nitrogen_tank_empty_offset, nitrogen_tank_full_offset, nitrous_tank_empty_mass, starting_pressurizing_nitrogen_mass, starting_nitrous_mass, nitrous_tank_empty_offset, nitrous_tank_full_offset, nitrous_tank_depleted_offset, tvc_module_empty_mass, starting_fuel_grain_mass, frame_com_to_gimbal, gimbal_to_tvc_com, inertia_tensor, tvc_range, tvc, rcs, imu, gps, uwb, com_to_ground);

    let mut control_input = Vector4::new(0.0, 0.0, 0.0, 0.0);
    let mut outside_forces = Vector3::new(0.0, 0.0, 0.0);
    let mut outside_torques = Vector3::new(0.0, 0.0, 0.0);
    let dt = 0.02; // 20 ms time step
    while rocket.step(control_input, outside_forces, outside_torques, dt) {
        // For this example, we'll just keep the control input zero and not apply any outside forces or torques.
        // In a real simulation, you'd update these based on your control algorithms and environmental effects.
        let rotated_offset = rocket.attitude.transform_vector(&rocket.com_to_ground);

        // Log Vehicle
        rec.log(
            "world/rocket",
            &rerun::Arrows3D::from_vectors([(-rotated_offset.x as f32, -rotated_offset.y as f32, -rotated_offset.z as f32)]) 
                .with_origins([(rocket.position.x as f32, rocket.position.y as f32, rocket.position.z as f32)])                    
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

fn mpc() {
   // Problem sizes
    let n = 13; // [x, y, z, qx, qy, qz, qw, x_dot, y_dot, z_dot, wx, wy, wz]
    let m = 3;  // [gimbal_theta, gimbal_phi, thrust]
    let n_steps = 10;
    let dt = 0.1;

    // Initial state: at origin, level, stationary, quaternion [0,0,0,1]
    let mut x = Array1::<f64>::zeros(n);
    x[2] = 5.0;
    x[6] = 1.0; // qw = 1 (unit quaternion)

    // Hover at set point
    let mut xref = Array1::<f64>::zeros(n);
    
    xref[6] = 1.0; // reference orientation: level (unit quaternion)

    let mut z_integral = 0.0;
    let mut y_integral = 0.0;
    let mut x_integral = 0.0;
    let ki_z = 0.2;
    let ki_y = 0.01;
    let ki_x = 0.01;

    // Reference trajectory
    let mut xref_traj = Array2::from_shape_fn((n_steps + 1, n), |(_, j)| xref[j]);
    let mut xref_traj_vec: Vec<Array1<f64>> = xref_traj.axis_iter(ndarray::Axis(0)).map(|row| row.to_owned()).collect();

    // Warm start: hover thrust (thrust = mass * gravity, gimbal angles = 0)
    let mut m_rocket = 80.0;
    let g = 9.81;
    let hover_thrust = m_rocket * g;
    let mut u_warm = Array2::<f64>::zeros((n_steps, m));
    for i in 0..n_steps {
        u_warm[[i, 2]] = hover_thrust; // thrust
    }

    // Costs: penalize position, orientation, velocities, angular rates
    let q_vec = vec![
        20.0, 20.0, 200.0,   // position x, y, z
        0.0, 0.0, 0.0, 0.0, // quaternion qx, qy, qz, qw
        3.0, 3.0, 1.0,        // linear velocities x_dot, y_dot, z_dot
        1.0, 1.0, 1.0          // angular velocities wx, wy, wz
    ];
    let q = Array2::<f64>::from_diag(&Array1::from(q_vec.clone()));
    let r = Array2::<f64>::from_diag(&Array1::from(vec![0000.0, 0000.0, 0.00]));
    // let qn = q.clone();
    let qn = Array2::<f64>::from_diag(&Array1::from(vec![
        200.0, 200.0, 5000.0,   // position x, y, z
        10.0, 10.0, 10.0, 10.0, // quaternion qx, qy, qz, qw
        10.0, 10.0, 10.0,        // linear velocities x_dot, y_dot, z_dot
        5.0, 5.0, 5.0          // angular velocities wx, wy, wz
    ]));

    // Store state and control history for plotting
    let mut u_prev = Array1::<f64>::zeros(m);

    let tolerance = 1e-4;
    let lbfgs_memory = 20;
    let n_dim_u = m * n_steps;
    let mut panoc_cache = optimization_engine::panoc::PANOCCache::new(n_dim_u, tolerance, lbfgs_memory);

    // smoothing weight vector (for gimbal_theta, gimbal_phi, thrust)
    let smoothing_weight = Array1::from(vec![1500.0, 1500.0, 0.02]);

    for k in 0..iters {
        if k as f64 * dt <= 10.0 {
            // change reference point after 5 seconds
            let f_iters = iters as f64;
            let f_k = k as f64;
            xref[0] = -0.0 * (f_k*dt)/10.0;
            xref[1] = 0.0 * (f_k*dt)/10.0;
            xref[2] = 25.0 * (f_k*dt)/10.0 + 3.0;

            xref_traj = Array2::from_shape_fn((n_steps + 1, n), |(_, j)| xref[j]);
            xref_traj_vec = xref_traj.axis_iter(ndarray::Axis(0)).map(|row| row.to_owned()).collect();
             
        } else if k as f64 * dt > 15.0 && k as f64 * dt <= 25.0 {
            // change reference point after 10 seconds
            let f_iters = iters as f64;
            let f_k = k as f64;
            xref[0] = 0.0 * ((f_k*dt)-15.0)/10.0;
            xref[1] = 0.0 * ((f_k*dt)-15.0)/10.0;
            xref[2] = 25.0 - 25.0 * ((f_k*dt)-15.0)/10.0;

            xref_traj = Array2::from_shape_fn((n_steps + 1, n), |(_, j)| xref[j]);
            xref_traj_vec = xref_traj.axis_iter(ndarray::Axis(0)).map(|row| row.to_owned()).collect();
        } 

        
        xref[0] = 0.0;
        xref[1] = 0.0;
        xref[2] = 5.0;

        let z_error = xref[2] - x[2];
        z_integral += z_error * dt;
        let mut x_ref_mod = xref.clone();
        x_ref_mod[2] += ki_z * z_integral; // modify z reference with integral term
        for i in 0..xref_traj_vec.len() {
            xref_traj_vec[i][2] = x_ref_mod[2];
        }
        let x_error = xref[0] - x[0];
        x_integral += x_error * dt;
        x_ref_mod[0] += ki_x * x_integral; // modify x reference with integral term
        for i in 0..xref_traj_vec.len() {
            xref_traj_vec[i][0] = x_ref_mod[0];
        }
        let y_error = xref[1] - x[1];
        y_integral += y_error * dt;
        x_ref_mod[1] += ki_y * y_integral; // modify y reference with integral term
        for i in 0..xref_traj_vec.len() {
            xref_traj_vec[i][1] = x_ref_mod[1];
        }

        // Solve MPC to get optimal control sequence
        // solve using OpEn
        let (mut u_apply, u_warm) = mpc_crate::OpEnSolve(&x, &u_warm.axis_iter(ndarray::Axis(0)).map(|row| row.to_owned()).collect(), &xref_traj_vec, &q, &r, &qn, &smoothing_weight, &mut panoc_cache, min_thrust, max_thrust, gimbal_limit);

        // exponential filter
        if k >= 1 {
            let alpha = 0.4; // smoothing factor
            u_apply = alpha * &u_apply + (1.0 - alpha) * &u_prev;
        }
        u_prev = u_apply.clone();
    }
}
mod rocket_dynamics;
mod device_sim;
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

    let inertia_tensor = Vector3::new(15.0, 15.0, 8.0);

    let mtv = MTV::new(0.0, 0.0, 0.0, 60.0, 70.0, 10.0, 30.0, 8.0, 200.0);
    let x_actuator = TVCActuator::new(0.0, 0.3, 2.0, 300.0, 10.0, 0.0, 200.0);
    let y_actuator = TVCActuator::new(0.0, 0.3, 2.0, 300.0, 10.0, 0.0, 200.0);
    let tvc = TVC::new(mtv, x_actuator, y_actuator, 0.15, Vector3::new(0.0, 0.0, -1.0), 10.0, 3.0, 8.0);
    let rcs = RCS::new(10.0, 0.15, 0.1, 10.0);
    let imu = IMU::new(Vector3::zeros(), Vector3::zeros(), Vector3::zeros(), Vector3::zeros(), Vector3::zeros(), Vector3::zeros(), Vector3::new(0.000005, 0.0, 0.0), 300.0);
    let gps = GPS::new(Vector3::zeros(), Vector3::zeros(), 5.0);
    let uwb = UWB::new(Vector3::zeros(), Vector3::zeros(), Vector3::zeros(), 8.0, 5.0);

    let mut rocket = Rocket::new(position, velocity, acceleration, attitude, angular_velocity, angular_acceleration, 50.0, 10.0, 0.0, 20.0, 8.0, inertia_tensor, 15_f64.to_radians(), tvc, rcs, imu, gps, uwb, Vector3::new(0.0, 0.0, -1.5));

    let mut control_input = Vector4::new(0.0, 0.0, 0.0, 0.0);
    let mut outside_forces = Vector3::new(0.0, 0.0, 0.0);
    let mut outside_torques = Vector3::new(5.0, 0.0, 0.0);
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
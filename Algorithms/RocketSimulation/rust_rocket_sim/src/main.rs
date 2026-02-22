mod rocket_dynamics;
mod device_sim;

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


    let position = [0.0, 0.0, 0.0];
    let velocity = [0.0, 0.0, 0.0];
    let attitude = [0.0, 0.0, 0.0, 0.0];
    let angular_velocity = [0.0, 0.0, 0.0];
    let u = [0.0, 0.0, 0.0, 0.0];


    // 2. Loop
    let mut t_step = 0;
    loop {
        let t = t_step as f32 * 0.02;

        let x = (t * 2.0).cos() * 5.0;
        let y = (t * 2.0).sin() * 5.0;
        let z = 1.0 + (t * 0.1); 

        // Log Vehicle
        rec.log(
            "world/rocket",
            &rerun::Arrows3D::from_vectors([(0.0, 0.0, 1.5)]) 
                .with_origins([(x, y, z)])                    
                .with_colors([rerun::Color::from_rgb(255, 0, 0)]) 
        )?;
        
        // Log Trail
        rec.log(
            "world/path",
            &rerun::Points3D::new([(x, y, z)])
                .with_colors([rerun::Color::from_rgb(200, 200, 200)])
                .with_radii([0.05])
        )?;

        t_step += 1;
        std::thread::sleep(std::time::Duration::from_millis(16));
    }
}

pub fn normalize_vector(vector: (f32, f32, f32)) -> (f32, f32, f32) {
    let length = (vector.0.powi(2) + vector.1.powi(2) + vector.2.powi(2)).sqrt();
    if length == 0.0 {
        (0.0, 0.0, 0.0)
    } else {
        (vector.0 / length, vector.1 / length, vector.2 / length)
    }
}
mod lossless;
use crate::lossless::LosslessSolver;

fn main() {
    let mut solver = LosslessSolver {
        landing_point: [0.0, 0.0, 0.0], // This is the point where you want to end up.
        initial_position: [0.0, 0.0, 50.0], // This is the point where you start from.
        initial_velocity: [0.0, 0.0, 0.0], // This is the velocity you start with.
        max_velocity: 500.0, // This is the maximum velocity the vehicle can/should achieve in flight.
        dry_mass: 50.0, // This is the mass of the vehicle, without any fuel/propellant on board.
        fuel_mass: 30.0, // This is the mass of the fuel/propellant.
        alpha: 1.0/(9.81 * 180.0), // This is the conversion ratio from thrust to delta mass.
        lower_thrust_bound: 1000.0 * 0.4, // This is the minimum thrust the vehicle must keep.
        upper_thrust_bound: 1000.0, // This is the maximmum thrust the vehicle can attain.
        tvc_range_rad: 15_f64.to_radians(), // This is the range from the vertical axis that the thrust vector control can deviate.
        coarse_delta_t: 0.5, // This is the dt used to solve for the time frame of the trajectory.
        fine_delta_t: 0.05, // This is the dt used to solve for the higher resolution trajectory.
        use_glide_slope: false, // This determines if the glide slope constraint is used. The glide slope constraint ensures that the vehicle stays above an upward spreading cone centered on the landing point.
        glide_slope: 5_f64.to_radians(), // This is the angle of the glide slope constraint.
        N: 20, // This is the number of time steps the solver uses. It is set here, but is recalculated internally solve() is called. This is simply exposed so that the number of time steps can be accessed externally, if necessary.
        ..Default::default()
    };
    
    let mut traj_result = solver.solve();
    

    match traj_result {
        Some(result) => {
            println!("Final position: {:?}", result.positions.last().unwrap());
            println!("Final velocity: {:?}", result.velocities.last().unwrap());
            println!("Final mass: {:?}", result.masses.last().unwrap());
            println!("Final thrust: {:?}", result.thrusts.last().unwrap());
            
            write_trajectory_to_csv("trajectory.csv", &result)
                .expect("Failed to write CSV");
        }
        None => {
            eprintln!("Solve failed!");
        }
    }
}

use std::fs::File;
use std::io::{Write, BufWriter};
use crate::lossless::{TrajectoryResult};

pub fn write_trajectory_to_csv(filename: &str, traj: &TrajectoryResult) -> std::io::Result<()> {
    let file = File::create(filename)?;
    let mut writer = BufWriter::new(file);

    // Write header
    writeln!(writer, "t,x,y,z,vx,vy,vz,mass,ux,uy,uz,sigma")?;

    let n_steps = traj.positions.len();

    for k in 0..n_steps {
        let x = &traj.positions[k];
        let v = &traj.velocities[k];
        let m = traj.masses[k];
        let sigma = if k < traj.sigmas.len() { traj.sigmas[k] } else { 0.0 };
        let u = if k < traj.thrusts.len() { &traj.thrusts[k] } else { &[0.0, 0.0, 0.0] };

        writeln!(
            writer,
            "{},{},{},{},{},{},{},{},{},{},{},{}",
            k as f64,
            x[0], x[1], x[2],
            v[0], v[1], v[2],
            m,
            u[0], u[1], u[2],
            sigma
        )?;
    }

    writer.flush()?;
    Ok(())
}

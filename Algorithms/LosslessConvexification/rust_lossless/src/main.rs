use clarabel::algebra::*;
use clarabel::solver::*;
mod lossless;
use crate::lossless::LosslessSolver;

fn main() {
    let mut solver = LosslessSolver {
        landing_point: [0.0, 0.0, 0.0],
        initial_position: [0.0, 0.0, 50.0],
        initial_velocity: [0.0, 0.0, 1.0],
        dry_mass: 50.0,
        fuel_mass: 30.0,
        alpha: 1.0/(9.81 * 180.0),
        lower_thrust_bound: 1000.0 * 0.4,
        upper_thrust_bound: 1000.0,
        tvc_range_rad: 15_f64.to_radians(),
        delta_t: 0.5,
        N: 20,
        ..Default::default()
    };
    
    let traj_result = solver.solve();

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

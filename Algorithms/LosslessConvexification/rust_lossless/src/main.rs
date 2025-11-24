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
        delta_t: 2.0,
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
        }
        None => {
            eprintln!("Solve failed!");
        }
    }
}
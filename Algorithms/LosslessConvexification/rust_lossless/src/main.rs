mod lossless;
mod chebyshev_lossless;
use crate::chebyshev_lossless::*;
use crate::lossless::{LosslessSolver, SolveMetrics, SolveRunResult, TrajectoryResult};

fn main() {
    let mut _solver = LosslessSolver {
        landing_point: [0.0, 0.0, 0.0], // This is the point where you want to end up.
        // initial_position: [10.0, 20.0, 50.0], // This is the point where you start from.
        initial_position: [0.0, 0.0, 50.0], // This is the point where you start from.
        initial_velocity: [0.0, 0.0, 0.0], // This is the velocity you start with.
        max_velocity: 500.0, // This is the maximum velocity the vehicle can/should achieve in flight.
        dry_mass: 50.0, // This is the mass of the vehicle, without any fuel/propellant on board.
        fuel_mass: 30.0, // This is the mass of the fuel/propellant.
        alpha: 1.0/(9.81 * 180.0), // This is the conversion ratio from thrust to delta mass.
        lower_thrust_bound: 1000.0 * 0.4, // This is the minimum thrust the vehicle must keep.
        upper_thrust_bound: 1000.0, // This is the maximum thrust the vehicle can attain.
        tvc_range_rad: 15_f64.to_radians(), // This is the range from the vertical axis that the thrust vector control can deviate.
        // coarse_line_search_delta_t: 0.5, // TODO
        // fine_line_search_delta_t: 0.5,
        coarse_delta_t: 0.5, // This is the dt used to solve for the time frame of the trajectory.
        fine_delta_t: 0.0125, // This is the dt used to solve for the higher resolution trajectory.
        use_glide_slope: true, // This determines if the glide slope constraint is used. The glide slope constraint ensures that the vehicle stays above an upward spreading cone centered on the landing point.
        glide_slope: 5_f64.to_radians(), // This is the angle of the glide slope constraint.
        N: 20, // This is the number of time steps the solver uses. It is set here, but is recalculated internally solve() is called. This is simply exposed so that the number of time steps can be accessed externally, if necessary.
        ..Default::default()
    };

    // TODO: Make chebyshev not stupid :D shouldn't have the same number of nodes for each pass on the coarse search.
    // Also want to try a binary search - style implementation
    let mut chebyshev_solver = ChebyshevLosslessSolver {
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
        coarse_line_search_delta_t: 0.5,
        fine_line_search_delta_t: 0.05,
        coarse_nodes: 30, // This is the dt used to solve for the time frame of the trajectory.
        fine_nodes: 50, // This is the dt used to solve for the higher resolution trajectory.
        use_glide_slope: false, // This determines if the glide slope constraint is used. The glide slope constraint ensures that the vehicle stays above an upward spreading cone centered on the landing point.
        glide_slope: 5_f64.to_radians(), // This is the angle of the glide slope constraint.
        ..Default::default()
    };
    
    // let run_name = "zoh_3";
    // let solve_run = _solver.solve();
    let run_name = "cgl_3";
    let solve_run = chebyshev_solver.solve();
    
    append_solve_metrics_to_csv("solve_metrics.csv", run_name, &solve_run)
        .expect("Failed to write solve metrics CSV");

    match &solve_run.trajectory {
        Some(result) => {
            println!("Final position: {:?}", result.positions.last().unwrap());
            println!("Final velocity: {:?}", result.velocities.last().unwrap());
            println!("Final mass: {:?}", result.masses.last().unwrap());
            println!("Final thrust: {:?}", result.thrusts.last().unwrap());
            
            write_trajectory_to_csv(&format!("trajectory_{}.csv", run_name), result)
                .expect("Failed to write CSV");
        }
        None => {
            eprintln!("Solve failed!");
        }
    }
}

use std::fs::{File, OpenOptions};
use std::io::{Write, BufWriter};
use std::path::Path;

pub fn write_trajectory_to_csv(filename: &str, traj: &TrajectoryResult) -> std::io::Result<()> {
    let file = File::create(filename)?;
    let mut writer = BufWriter::new(file);

    // ADDED: "thrust_force" to the header
    writeln!(writer, "t,x,y,z,vx,vy,vz,mass,ux,uy,uz,sigma,thrust_force")?;

    let n_steps = traj.positions.len();

    for k in 0..n_steps {
        let x = &traj.positions[k];
        let v = &traj.velocities[k];
        let m = traj.masses[k];
        let sigma = if k < traj.sigmas.len() { traj.sigmas[k] } else { 0.0 };
        
        // Safely get u (control vector)
        let u = if k < traj.thrusts.len() { &traj.thrusts[k] } else { &[0.0, 0.0, 0.0] };

        // --- NEW CALCULATION ---
        // 1. Calculate Magnitude ||u|| (Euclidean norm)
        let u_mag = (u[0].powi(2) + u[1].powi(2) + u[2].powi(2)).sqrt();
        
        // 2. Calculate True Thrust Force = mass * ||u||
        let thrust_force = m * u_mag;

        writeln!(
            writer,
            "{},{},{},{},{},{},{},{},{},{},{},{},{}",
            k as f64,
            x[0], x[1], x[2],
            v[0], v[1], v[2],
            m,
            u[0], u[1], u[2],
            sigma,
            thrust_force // Write the new value to the row
        )?;
    }

    writer.flush()?;
    Ok(())
}

pub fn append_solve_metrics_to_csv(
    filename: &str,
    run_name: &str,
    solve_run: &SolveRunResult,
) -> std::io::Result<()> {
    const SOLVE_METRICS_HEADER_V1: &str =
        "run_name,pass,attempts,successes,iterations,clarabel_solve_time_s,wall_time_s,last_status";
    const SOLVE_METRICS_HEADER_V2: &str =
        "run_name,pass,attempts,successes,iterations,clarabel_solve_time_s,wall_time_s,last_status,time_of_flight_s";

    ensure_solve_metrics_header(filename, SOLVE_METRICS_HEADER_V1, SOLVE_METRICS_HEADER_V2)?;

    let file_exists = Path::new(filename).exists();
    let file = OpenOptions::new()
        .append(true)
        .create(true)
        .open(filename)?;
    let write_header = !file_exists || file.metadata()?.len() == 0;

    let mut writer = BufWriter::new(file);
    let time_of_flight_s = solve_run
        .trajectory
        .as_ref()
        .map(|trajectory| trajectory.time_of_flight_s);

    if write_header {
        writeln!(writer, "{}", SOLVE_METRICS_HEADER_V2)?;
    }

    write_solve_metrics_row(
        &mut writer,
        run_name,
        "coarse",
        &solve_run.coarse_metrics,
        time_of_flight_s,
    )?;
    write_solve_metrics_row(
        &mut writer,
        run_name,
        "fine",
        &solve_run.fine_metrics,
        time_of_flight_s,
    )?;
    write_solve_metrics_row(
        &mut writer,
        run_name,
        "total",
        &solve_run.total_metrics,
        time_of_flight_s,
    )?;

    writer.flush()?;
    Ok(())
}

fn ensure_solve_metrics_header(
    filename: &str,
    old_header: &str,
    new_header: &str,
) -> std::io::Result<()> {
    let path = Path::new(filename);
    if !path.exists() || path.metadata()?.len() == 0 {
        return Ok(());
    }

    let file_contents = std::fs::read_to_string(path)?;
    let mut lines = file_contents.lines();
    let existing_header = lines.next().unwrap_or_default().trim();

    if existing_header == new_header {
        return Ok(());
    }

    if existing_header == old_header {
        let mut migrated = String::new();
        migrated.push_str(new_header);
        migrated.push('\n');

        for line in lines {
            if line.trim().is_empty() {
                continue;
            }
            migrated.push_str(line);
            migrated.push(',');
            migrated.push('\n');
        }

        std::fs::write(path, migrated)?;
        return Ok(());
    }

    Err(std::io::Error::new(
        std::io::ErrorKind::InvalidData,
        format!(
            "Unsupported solve metrics CSV header in {}: {}",
            filename, existing_header
        ),
    ))
}

fn write_solve_metrics_row<W: Write>(
    writer: &mut W,
    run_name: &str,
    pass_name: &str,
    metrics: &SolveMetrics,
    time_of_flight_s: Option<f64>,
) -> std::io::Result<()> {
    let time_of_flight_str = time_of_flight_s
        .map(|time| format!("{:.6}", time))
        .unwrap_or_default();

    writeln!(
        writer,
        "{},{},{},{},{},{:.6},{:.6},{:?},{}",
        run_name,
        pass_name,
        metrics.attempts,
        metrics.successes,
        metrics.iterations,
        metrics.clarabel_solve_time_s,
        metrics.wall_time_s,
        metrics.last_status,
        time_of_flight_str
    )
}

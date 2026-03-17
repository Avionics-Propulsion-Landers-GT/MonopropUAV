use std::fs::{self, File};
use std::io::{self, BufWriter, Write};
use std::path::PathBuf;

use ndarray::{Array1, array};
use rust_ekf::{AttitudeEKF, AttitudeModel};

fn parse_measurement_row(line: &str) -> io::Result<[f64; 10]> {
    let columns: Vec<&str> = line.split(',').collect();
    if columns.len() < 10 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("expected at least 10 columns, found {}", columns.len()),
        ));
    }

    let parse = |index: usize| -> io::Result<f64> {
        columns[index].trim().parse::<f64>().map_err(|error| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                format!("failed to parse column {index}: {error}"),
            )
        })
    };

    Ok([
        parse(0)?, // time
        parse(4)?, // gyro_x
        parse(5)?, // gyro_y
        parse(6)?, // gyro_z
        parse(1)?, // accel_x
        parse(2)?, // accel_y
        parse(3)?, // accel_z
        parse(7)?, // mag_x
        parse(8)?, // mag_y
        parse(9)?, // mag_z
    ])
}

fn export_attitude_states_from_flight_data() -> io::Result<()> {
    let base_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("src/testing");
    let input_path = base_path.join("flight_data.csv");
    let output_path = base_path.join("attitude_output.csv");

    let csv_contents = fs::read_to_string(&input_path)?;

    let model = AttitudeModel::new(0.01);
    let initial_state = array![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let initial_measurement = Array1::zeros(9);

    let mut ekf = AttitudeEKF::new(
        initial_state,
        initial_measurement,
        0.01,
        0.1,
        0.01,
        1.0,
        model,
    );

    let mut outputs = Vec::new();
    let mut printed_error_covariance = true;
    for line in csv_contents.lines().skip(1).filter(|line| !line.trim().is_empty()) {
        let measurement = parse_measurement_row(line)?;
        ekf.predict();
        ekf.update(&measurement);

        let state = ekf.get_state();
        if  measurement[0] > 10.0 && !printed_error_covariance {
            printed_error_covariance = true;
            println!("Error Covariance: {}", ekf.get_covariance());
        }
        outputs.push([
            measurement[0],
            state[0],
            state[1],
            state[2],
            state[3],
            state[4],
            state[5],
        ]);
    }

    let file = File::create(&output_path)?;
    let mut writer = BufWriter::new(file);
    writeln!(writer, "time,roll,pitch,yaw,omega_x,omega_y,omega_z")?;

    for row in outputs {
        writeln!(
            writer,
            "{:.6},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12}",
            row[0], row[1], row[2], row[3], row[4], row[5], row[6]
        )?;
    }

    writer.flush()?;
    Ok(())
}

fn main() {
    if let Err(e) = export_attitude_states_from_flight_data() {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}
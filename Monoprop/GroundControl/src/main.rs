mod mpc_crate;

use mpc_crate::{mpc_main, dynamics};
use ndarray::{Array1, Array2, s};
use std::f64::consts::PI;
use optimization_engine::panoc::PANOCCache;

use std::process::Command;
use std::thread;
use std::net::UdpSocket;
use std::time::{Duration, Instant};
use crossbeam_channel::{bounded, Sender, Receiver};
use Common::{DroneTelemetry, GroundCommand}; // Your shared structs

use std::fs::File;
use std::io::{self, Write, BufRead};

// MPC Configuration Struct
struct MPCConfig {
    u_warm: Array2<f64>,
    xref_traj_vec: Vec<Array1<f64>>,
    q: Array2<f64>,
    r: Array2<f64>,
    qn: Array2<f64>,
    smoothing_weight: Array1<f64>,
    panoc_cache: PANOCCache,
}

enum ControlCommand {
    Go,
    Land,
    Kill,
}

// Configuration
const DRONE_IP: &str = "192.168.1.50:8888"; // IP of your ESP32
const GROUND_PORT: &str = "0.0.0.0:9999";   // Port listening on PC

fn main() -> anyhow::Result<()> {
    // 1. Setup UDP Socket
    let ground_port_num = GROUND_PORT
    .split(':')
    .nth(1)
    .expect("Invalid address format")
    .parse::<u16>()
    .expect("Invalid port number");
    force_clear_port(ground_port_num);
    let socket = UdpSocket::bind(GROUND_PORT)?;
    let socket_send = socket.try_clone()?;
    
    // Set a read timeout so the listener thread doesn't hang forever
    socket.set_read_timeout(Some(Duration::from_millis(100)))?;

    // 2. Channels for Thread Communication
    // Telemetry: Network Thread -> MPC Thread  
    let (telem_tx, telem_rx) = bounded::<DroneTelemetry>(5);

    // Control Commands: Input Thread -> Main Thread
    let (cmd_tx, cmd_rx) = bounded::<ControlCommand>(5);
    
    // 3. Spawn User Input Thread
    thread::spawn(move || {
        let stdin = io::stdin();
        println!("Type 'go' to activate control, 'land' to command landing, or 'kill' to exit.");
        for line in stdin.lock().lines() {
            if let Ok(cmd) = line {
                let cmd = cmd.trim().to_lowercase();
                if cmd == "go" {
                    let _ = cmd_tx.send(ControlCommand::Go);
                    println!("Sending control commands!");
                } else if cmd == "land" {
                    let _ = cmd_tx.send(ControlCommand::Land);
                    println!("Landing command issued.");
                } else if cmd == "kill" {
                    println!("Exiting program.");
                    let _ = cmd_tx.send(ControlCommand::Kill);
                    break;
                }
            }
        }
    });
    
    // 4. Spawn Network Listener Thread
    thread::spawn(move || {
        let mut buf = [0u8; 1024];
        loop {
            match socket.recv_from(&mut buf) {
                Ok((size, _src)) => {
                    // Deserialize bytes back into Struct
                    if let Ok(telemetry) = postcard::from_bytes::<DroneTelemetry>(&buf[..size]) {
                        // Send to main thread (overwrite old data if full)
                        let _ = telem_tx.try_send(telemetry);
                    }
                }
                Err(_) => { /* Handle timeout or error */ }
            }
        }
    });

    println!("Ground Station Started. Waiting for drone...");

    // 5. Main MPC Loop (Runs at e.g., 50Hz)
    let mut sequence_id = 0;

    // Define problem MPC parameters here
    // Problem sizes
    let n = 13; // [x, y, z, qx, qy, qz, qw, x_dot, y_dot, z_dot, wx, wy, wz]
    let m = 3;  // [gimbal_theta, gimbal_phi, thrust]
    let n_steps = 10; // MPC horizon 
    let t_total = 20.0; // how many seconds for flight
    let dt = 0.1; // prediction time step

    // Initial state: at origin, level, stationary, quaternion [0,0,0,1]
    let mut x = Array1::<f64>::zeros(n);
    x[6] = 1.0; // qw = 1 (unit quaternion)

    // Hover at set point
    let hover_alt = 1.0;
    let land_alt = 0.07;
    let mut xref = Array1::<f64>::zeros(n);
    xref[2] = hover_alt; // reference position: hover at 1m altitude
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

    // Bounds on control inputs
    // let gimbal_limit = 15.0 * PI / 180.0; // +/- 15 degrees
    // let thrust_min = 300.0;
    // let thrust_max = 1000.0;
    // let u_min = Array1::from(vec![-gimbal_limit, -gimbal_limit, thrust_min]);
    // let u_max = Array1::from(vec![gimbal_limit, gimbal_limit, thrust_max]);

    // Store state and control history for plotting
    let mut x_history = Vec::new();
    let mut u_history: Vec<Array2<f64>> = Vec::new();

    // PANOC parameters
    let tolerance = 1e-4;
    let lbfgs_memory = 20;
    let max_iter = 200;
    let n_dim_u = m * n_steps;
    let mut panoc_cache = optimization_engine::panoc::PANOCCache::new(n_dim_u, tolerance, lbfgs_memory);

    // smoothing weight vector (for gimbal_theta, gimbal_phi, thrust)
    let smoothing_weight = Array1::from(vec![1500.0, 1500.0, 0.02]);

    let mut mpc_config = MPCConfig {
        u_warm,
        xref_traj_vec,
        q,
        r,
        qn,
        smoothing_weight,
        panoc_cache,
    };

    let mut last_timestamp: Option<Instant> = None;

    let mut activated = false; // flag to indicate if we are sending control commands
    let mut first_telem_received = false; // flag for first telemetry packet
    let mut landing_initiated = false; // flag for landing sequence

    loop {
        let loop_start = Instant::now();

        // A. Get Latest State
        // Drain the channel to get the absolute newest packet
        let mut latest_telem: Option<DroneTelemetry> = None;
        while let Ok(t) = telem_rx.try_recv() {
            latest_telem = Some(t);
        }

        // Check for control commands (non-blocking)
        if let Ok(cmd) = cmd_rx.try_recv() {
            match cmd {
                ControlCommand::Go => {
                    activated = true;
                    landing_initiated = false;
                    xref[2] = hover_alt;
                    z_integral = 0.0;
                    y_integral = 0.0;
                    x_integral = 0.0;
                    println!("✓ Control commands activated.");
                }
                ControlCommand::Land => {
                    activated = true;
                    landing_initiated = true;
                    xref[2] = land_alt;
                    z_integral = 0.0;
                    y_integral = 0.0;
                    x_integral = 0.0;
                    println!("✓ Landing mode activated.");
                }
                ControlCommand::Kill => {
                    activated = false;
                    landing_initiated = false;
                    println!("Exiting program.");
                    break;
                }
            }
        }

        if let Some(telem) = latest_telem {
            if !first_telem_received {
                first_telem_received = true;
                println!("✓ Telemetry received from drone!");
                println!("Position: [{:.2}, {:.2}, {:.2}] m", telem.position[0], telem.position[1], telem.position[2]);
                println!("\nType 'go' to start sending control commands, 'land' to command landing, or 'kill' to exit.");
            }

            if !activated {
                continue; // Skip MPC until activated
            }

            // ============================================
            // B. RUN YOUR MPC SOLVER HERE
            // ============================================
            // Input: telem and mpc_config
            // Output: control sequence
            
            // store state data for post flight analysis
            x[0] = telem.position[0] as f64;
            x[1] = telem.position[1] as f64;
            x[2] = telem.position[2] as f64;
            x[3] = telem.orientation[0] as f64;  // qx
            x[4] = telem.orientation[1] as f64;  // qy
            x[5] = telem.orientation[2] as f64;  // qz
            x[6] = telem.orientation[3] as f64;  // qw
            x[7] = telem.velocity[0] as f64;
            x[8] = telem.velocity[1] as f64;
            x[9] = telem.velocity[2] as f64;
            x[10] = telem.angular_vel[0] as f64;
            x[11] = telem.angular_vel[1] as f64;
            x[12] = telem.angular_vel[2] as f64;
            x_history.push(x.clone());
            
            let now = Instant::now();
            let dt = if let Some(prev_time) = last_timestamp {
                now.duration_since(prev_time).as_secs_f64()
            } else {
                0.1 // default dt for first iteration
            };
            last_timestamp = Some(now);

            let z_error = xref[2] - x[2];
            z_integral += z_error * dt;
            let mut x_ref_mod = xref.clone();
            x_ref_mod[2] += ki_z * z_integral; // modify z reference with integral term
            for i in 0..mpc_config.xref_traj_vec.len() {
                mpc_config.xref_traj_vec[i][2] = x_ref_mod[2];
            }
            let x_error = xref[0] - x[0];
            x_integral += x_error * dt;
            x_ref_mod[0] += ki_x * x_integral; // modify x reference with integral term
            for i in 0..mpc_config.xref_traj_vec.len() {
                mpc_config.xref_traj_vec[i][0] = x_ref_mod[0];
            }
            let y_error = xref[1] - x[1];
            y_integral += y_error * dt;
            x_ref_mod[1] += ki_y * y_integral; // modify y reference with integral term
            for i in 0..mpc_config.xref_traj_vec.len() {
                mpc_config.xref_traj_vec[i][1] = x_ref_mod[1];
            }

            // Calculate the full control sequence
            let mut u_control_seq = run_mpc_solver(&telem, &mut mpc_config);

            // Exponential filter on control inputs
            if let Some(u_prev) = u_history.last() {
                let alpha = 0.4; // smoothing factor
                u_control_seq = alpha * &u_control_seq + (1.0 - alpha) * u_prev;
            }

            // Push u_control_seq to history for filtering
            u_history.push(u_control_seq.clone());

            // Convert to fixed array sizes for sending
            let mut thrust_seq: [f32; 10] = u_control_seq.slice(s![.., 2]).mapv(|v| v as f32).to_owned().as_slice().unwrap().try_into().unwrap();
            let gimbal_theta_seq: [f32; 10] = u_control_seq.slice(s![.., 0]).mapv(|v| v as f32).to_owned().as_slice().unwrap().try_into().unwrap();
            let gimbal_phi_seq: [f32; 10] = u_control_seq.slice(s![.., 1]).mapv(|v| v as f32).to_owned().as_slice().unwrap().try_into().unwrap();

            if landing_initiated {
                if telem.position[2] <= land_alt as f32 + 0.02f32 {
                    println!("Scaling down thrust at altitude {:.2} m", telem.position[2]);
                    
                    // Scale down thrust from 100% to 0% of commanded value over 2 seconds
                    // Track landing start time
                    static LANDING_START: std::sync::Once = std::sync::Once::new();
                    static mut LANDING_TIME: Option<Instant> = None;

                    LANDING_START.call_once(|| {
                        unsafe { LANDING_TIME = Some(Instant::now()); }
                    });

                    if let Some(start_time) = unsafe { LANDING_TIME } {
                        let elapsed = start_time.elapsed().as_secs_f64();

                        let mut scale_factor = 1.0;
                        if elapsed < 2.0 {
                            scale_factor = 1.0 - (elapsed / 2.0); // Linear scale from 1.0 to 0.0 over 2 seconds   
                        } else {
                            scale_factor = 0.0; // After 2 seconds, set to 0
                            activated = false; // Deactivate control commands
                            landing_initiated = false;
                            println!("✓ Landing complete. Control commands deactivated.");
                            break;
                        };
                        
                        // Scale down all thrust values in the sequence
                        for i in 0..thrust_seq.len() {
                            thrust_seq[i] *= scale_factor as f32;
                        }
                    }
                        
                }
            }

            // C. Prepare Command
            let cmd = GroundCommand {
                seq_id: sequence_id,
                thrust_seq: thrust_seq,
                gimbal_theta_seq: gimbal_theta_seq,
                gimbal_phi_seq: gimbal_phi_seq,
            };

            // D. Serialize and Send
            let data = postcard::to_stdvec(&cmd)?;
            socket_send.send_to(&data, DRONE_IP)?;
            
            sequence_id += 1;
        }

        // Maintain Loop Rate (e.g., 50Hz = 20ms)
        let elapsed = loop_start.elapsed();
        if elapsed < Duration::from_millis(20) {
            thread::sleep(Duration::from_millis(20) - elapsed);
        }


    }

    // save x_history and u_history data to csv files
        
    // Save x_history as CSV
    let mut x_file = File::create("x_history.csv")?;
    writeln!(x_file, "x,y,z,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz")?;
    for state in &x_history {
        writeln!(x_file, "{},{},{},{},{},{},{},{},{},{},{},{},{}", 
            state[0], state[1], state[2], state[3], state[4], 
            state[5], state[6], state[7], state[8], state[9],
            state[10], state[11], state[12])?;
    }

    // Save u_history as CSV
    let mut u_file = File::create("u_history.csv")?;
    writeln!(u_file, "gimbal_theta,gimbal_phi,thrust")?;
    for control_seq in &u_history {
        for row in control_seq.axis_iter(ndarray::Axis(0)) {
            writeln!(u_file, "{},{},{}", row[0], row[1], row[2])?;
        }
    }
        
        println!("Saved trajectory data to x_history.csv and u_history.csv");
        
        Ok(())
}

fn run_mpc_solver(telem: &DroneTelemetry, mpc_config: &mut MPCConfig) -> Array2<f64> {

    // unpack telemetry into state vector x
    let mut x = Array1::<f64>::zeros(13);

    x[0] = telem.position[0] as f64;
    x[1] = telem.position[1] as f64;
    x[2] = telem.position[2] as f64;
    x[3] = telem.orientation[0] as f64;  // qx
    x[4] = telem.orientation[1] as f64;  // qy
    x[5] = telem.orientation[2] as f64;  // qz
    x[6] = telem.orientation[3] as f64;  // qw
    x[7] = telem.velocity[0] as f64;
    x[8] = telem.velocity[1] as f64;
    x[9] = telem.velocity[2] as f64;
    x[10] = telem.angular_vel[0] as f64;
    x[11] = telem.angular_vel[1] as f64;
    x[12] = telem.angular_vel[2] as f64;

    // Call solver and return control actions
    let (mut u_apply, u_warm) = mpc_crate::OpEnSolve(&x, &mpc_config.u_warm.axis_iter(ndarray::Axis(0)).map(|row| row.to_owned()).collect(), &mpc_config.xref_traj_vec, &mpc_config.q, &mpc_config.r, &mpc_config.qn, &mpc_config.smoothing_weight, &mut mpc_config.panoc_cache);

    // Update warm start for next iteration
    mpc_config.u_warm = u_warm.clone();

    // Return the full control sequence (n_steps x m array)
    u_warm
}

fn force_clear_port(port: u16) {
    #[cfg(unix)] // Linux or MacOS
    {
        // 'fuser' command finds and kills processes on a port
        // -k: kill, -n udp: namespace UDP
        let _ = Command::new("fuser")
            .arg("-k")
            .arg("-n")
            .arg("udp")
            .arg(format!("{}", port))
            .output(); // We ignore the result; if it fails, maybe port was already clear
    }

    #[cfg(windows)]
    {
        // 1. Find the PID using netstat
        let output = Command::new("cmd")
            .args(&["/C", &format!("netstat -ano | findstr :{}", port)])
            .output()
            .expect("Failed to execute netstat");

        let stdout = String::from_utf8_lossy(&output.stdout);
        
        // 2. Parse the PID (Last token in the line)
        // Output looks like: "  UDP    0.0.0.0:9999       *:* 12345"
        if let Some(line) = stdout.lines().next() {
            if let Some(pid) = line.split_whitespace().last() {
                // 3. Kill the PID
                let _ = Command::new("taskkill")
                    .args(&["/F", "/PID", pid])
                    .output();
                println!("Killed zombie process (PID: {}) on port {}", pid, port);
            }
        }
    }
    
    // Give the OS a moment to release the resource
    thread::sleep(Duration::from_millis(500));
}
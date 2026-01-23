use std::process::Command;
use std::thread;
use std::net::UdpSocket;
use std::time::{Duration, Instant};
use crossbeam_channel::{bounded, Sender, Receiver};
use Common::{DroneTelemetry, GroundCommand}; // Your shared structs

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
    
    // 3. Spawn Network Listener Thread
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

    // 4. Main MPC Loop (Runs at e.g., 50Hz)
    let mut sequence_id = 0;
    
    loop {
        let loop_start = Instant::now();

        // A. Get Latest State
        // Drain the channel to get the absolute newest packet
        let mut latest_telem: Option<DroneTelemetry> = None;
        while let Ok(t) = telem_rx.try_recv() {
            latest_telem = Some(t);
        }

        if let Some(telem) = latest_telem {
            // ============================================
            // B. RUN YOUR MPC SOLVER HERE
            // ============================================
            // Input: telem (pos, vel, quat)
            // Output: next_thrust, next_quat
            
            println!("Telem: {:?}", telem.orientation);
            
            // Placeholder MPC logic
            let (cmd_thrust, cmd_q) = run_mpc_solver(&telem);

            // C. Prepare Command
            let cmd = GroundCommand {
                seq_id: sequence_id,
                thrust: cmd_thrust,
                target_q: cmd_q,
                body_rates: [0.0, 0.0, 0.0],
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
}

fn run_mpc_solver(telem: &DroneTelemetry) -> (f32, [f32; 4]) {
    // Call your solver (Acados, OSQP, etc.) here
    // Return (Thrust, Target_Quaternion)
    (0.5, [1.0, 0.0, 0.0, 0.0])
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
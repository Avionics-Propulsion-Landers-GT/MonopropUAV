#![no_std]
#![no_main]

mod sensor_fusion_crate;

use teensy4_bsp as bsp;
use bsp::board;
use cortex_m_rt::entry;
use postcard;
use Common::{DroneTelemetry, GroundCommand};

// State machine for monocopter
#[derive(Copy, Clone, PartialEq)]
enum DroneState {
    Idle,
    Flying,
    Landing,
}

#[entry]
fn main() -> ! {
    let board::Resources {
        mut gpio2, 
        pins,
        lpuart6,
        mut gpt1,
        ..
    } = board::t41(board::instances());

    // Create LPUART peripheral for communication with ESP32
    // where pins.p1 = LPUART6_TX, pins.p0 = LPUART6_RX
    let mut lpuart6 = board::lpuart(lpuart6, pins.p1, pins.p0, 115200);

    // Create the LED peripheral. There is only one LED on the Teensy 4.1, connected to pin 13.
    let led = board::led(&mut gpio2, pins.p13);
    
    gpt1.set_clock_source(bsp::hal::gpt::ClockSource::PeripheralClock);
    let mut delay = bsp::hal::timer::Blocking::<_, 1_000_000>::from_gpt(gpt1);

    let mut buf = [0u8; 256]; 
    let mut idx = 0;
    let mut state = DroneState::Idle;
    let mut latest_cmd: Option<GroundCommand> = None; // Store latest control command
    const GROUND_COMMAND_SIZE: usize = core::mem::size_of::<GroundCommand>();

    // initialize sensors (if that's smth we need to do idk
    sensor_fusion_crate::uwb_init();
    sensor_fusion_crate::imu_init();

    loop {
        // Read commands from ESP32
        match lpuart6.try_read() {
            Ok(Some(byte)) => { 
                if idx < buf.len() {
                    buf[idx] = byte;
                    idx += 1;
                }

                // Check for binary GroundCommand packet
                if idx >= GROUND_COMMAND_SIZE {
                    if let Ok(cmd) = postcard::from_bytes::<GroundCommand>(&buf[..GROUND_COMMAND_SIZE]) {
                        state = handle_ground_command(cmd, state, &mut lpuart6, &mut latest_cmd);
                    }
                    idx = 0; // Reset buffer after processing
                }

                // Check for text commands (e.g., "go\n" for state changes)
                if byte == b'\n' || byte == b'\r' {
                    if let Some(cmd) = recognize_command(&buf[..idx.saturating_sub(1)]) {
                        state = handle_command(cmd, state, &mut lpuart6);
                    }
                    idx = 0; // Reset buffer
                }
            }
            Ok(None) => {}
            Err(_) => {}
        }
        
        // Send telemetry when flying or landing
        if (state == DroneState::Flying || state == DroneState::Landing) {
            send_telemetry(&mut lpuart6);
        }
        
        // Control loop - only when flying or landing
        if state == DroneState::Flying || state == DroneState::Landing {
            if let Some(cmd) = &latest_cmd {
                // Apply current control: thrust, gimbal angles, body rates
                let thrust = cmd.thrust_seq[0];
                let theta = cmd.gimbal_theta_seq[0];
                let phi = cmd.gimbal_phi_seq[0];

                // TODO: Send to motors/servos
                // e.g., set_motor_thrust(thrust);
                // set_gimbal_angles(theta, phi);
                // apply_body_rates(rates);
            }
        }
        
        led.toggle();
        delay.block_ms(1);  // 1ms loop
    }
}

enum Command {
    Go,
    Land,
    Kill,
}

fn recognize_command(data: &[u8]) -> Option<Command> {
    match data {
        b"go" => Some(Command::Go),
        b"land" => Some(Command::Land),
        b"kill" => Some(Command::Kill),
        _ => None,
    }
}

fn handle_command(cmd: Command, current_state: DroneState, uart: &mut board::Lpuart6) -> DroneState {
    match cmd {
        Command::Go => {
            if current_state == DroneState::Idle {
                // send_ack(uart, b"ACK:GO\n");
                DroneState::Flying
            } else {
                // send_ack(uart, b"ERR:ALREADY_ACTIVE\n");
                current_state
            }
        }
        Command::Land => {
            if current_state == DroneState::Flying {
                // send_ack(uart, b"ACK:LAND\n");
                DroneState::Landing
            } else {
                // send_ack(uart, b"ERR:NOT_FLYING\n");
                current_state
            }
        }
        Command::Kill => {
            // send_ack(uart, b"ACK:KILL\n");
            DroneState::Idle
        }
    }
}

fn send_ack(uart: &mut board::Lpuart6, msg: &[u8]) {
    for &byte in msg {
        while !uart.try_write(byte) {}
    }
}

fn send_telemetry(uart: &mut board::Lpuart6) {
    // Get current timestamp 
    let timestamp_us = 0u64; // TODO: Implement proper timestamp

    // Collect sensor data (replace with real calls)
    let position = sensor_fusion_crate::uwb_get_position(); // [f32; 3]
    let orientation = sensor_fusion_crate::imu_get_orientation(); // [f32; 4] quaternion
    let velocity = sensor_fusion_crate::uwb_get_velocity(); // [f32; 3]
    let angular_vel = sensor_fusion_crate::imu_get_angular_velocity(); // [f32; 3]

    // Create telemetry struct
    let telemetry = DroneTelemetry {
        timestamp_us,
        orientation,
        position,
        velocity,
        angular_vel,
    };

    // Serialize to bytes
    let serialized = postcard::to_vec::<_, 64>(&telemetry).unwrap(); // 64-byte buffer

    // Send over UART
    for &byte in &serialized {
        while !uart.try_write(byte) {
            // Wait for TX buffer space
        }
    }
}

fn handle_ground_command(cmd: GroundCommand, current_state: DroneState, uart: &mut board::Lpuart6, latest_cmd: &mut Option<GroundCommand>) -> DroneState {
    // Store the command for continuous control
    *latest_cmd = Some(cmd);

    // Acknowledge receipt
    // send_ack(uart, b"ACK:CONTROL\n");

    current_state
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
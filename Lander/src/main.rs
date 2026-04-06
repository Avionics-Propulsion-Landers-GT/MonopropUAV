use nalgebra::{Vector3, Vector4, UnitQuaternion, Quaternion};
use ndarray::{Array1, Array2};
use std::time::{Duration, Instant};
use rust_ekf::{ExtendedKalmanFilter, AttitudeModel};
use rust_lossless::lossless::{LosslessSolver, TrajectoryResult};
use MPC::mpc_crate::{mpc_main};

const COS_MAX_TILT: f64 = 0.965925826289; //cos(19.2 degrees) -> calculated from flight termination (NEED TO BE RECALCULATED ONCE WE HAVE MORE NUMBERS FOR OUR ROCKET'S STRUCTURE) 

#[derive(Debug, Clone, Copy, PartialEq)]
enum FlightPhase {
    Ascent,
    Hover,
    Descent,
}

// Tracks Vehicle State, Definitely going to need a mutex on this 
#[derive(Debug, Clone)]
pub struct VehicleState {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub attitude: UnitQuaternion<f64>,
    pub angular_velocity: Vector3<f64>,
    pub mass: f64,
    pub dry_mass: f64,
}

impl Default for VehicleState {
    fn default() -> Self {
        Self {
            position: Vector3::new(0.0, 0.0, 0.0),
            velocity: Vector3::new(0.0, 0.0, 0.0),
            attitude: UnitQuaternion::from_quaternion(Quaternion::new(1.0, 0.0, 0.0, 0.0)),
            angular_velocity: Vector3::new(0.0, 0.0, 0.0),
            mass: 50.0,
            dry_mass: 20.0,
        }
    }
}

// We might need to move this around or change it depending on what firmware ends up looking like
#[derive(Debug, Clone)]
pub struct SensorData {
    pub timestamp: f64,
    pub imu_data: Option<[f64; 7]>, // [t, gx, gy, gz, ax, ay, az]
    pub mag_data: Option<[f64; 4]>, // [t, mx, my, mz]
    pub gps_data: Option<[f64; 4]>, // [t, x, y, z]
    pub uwb_data: Option<[f64; 4]>, // [t, x, y, z]
    pub chamber_pressure: Option<f64>,
    pub tank_pressure: Option<f64>,
}

#[derive(Debug)]
pub struct ControlLoopState {
    pub start_time: Instant,
    pub vehicle_state: VehicleState,
    pub sensor_fusion_state: Option<Array1<f64>>,
    pub trajectory_state: Option<rust_lossless::TrajectoryResult>,
    pub last_sensor_update: Instant,
    pub last_navigation_update: Instant,
    pub last_mpc_update: Instant,
    pub flight_terminated: bool,
    pub flight_phase: FlightPhase,
    pub last_state_time: Instant, // Time when we entered the current flight phase
}

impl Default for ControlLoopState {
    fn default() -> Self {
        let now = Instant::now();
        Self {
            start_time: now,
            vehicle_state: VehicleState::default(),
            sensor_fusion_state: None,
            trajectory_state: None,
            last_sensor_update: now,
            last_navigation_update: now,
            last_mpc_update: now,
            flight_terminated: false,
            flight_phase: FlightPhase::Ascent,
            last_state_time: now,
        }
    }
}

// TODO: We need to add the rest of our EKFs, maybe a different struct to contain each of the EKFs
pub struct ControlLoop {
    ekf_attitude: ExtendedKalmanFilter<AttitudeModel>,
    lossless: LosslessSolver,
    mpc: (),
    state: ControlLoopState,
    sensor_fusion_rate: f64,    // 500 Hz
    navigation_rate: f64,        // 1 Hz
    mpc_rate: f64,              // 50 Hz
}

impl ControlLoop {
    pub fn new() -> Self {
        let ekf_attitude = ExtendedKalmanFilter::new(
            Array1::from(vec![0.0; 6]),
            Array1::from(vec![0.0; 9]),
            0.002,
            Array2::eye(6) * 0.01,  // Process noise
            Array2::eye(9) * 0.1,   // Measurement noise
            Array2::eye(6) * 1.0,   // Initial covariance
            AttitudeModel::new(0.002)
        );
        let lossless = LosslessSolver::default();
        let mpc = ();
        
        Self {
            ekf_attitude,
            lossless,
            mpc,
            state: ControlLoopState::default(),
            sensor_fusion_rate: 500.0,
            navigation_rate: 1.0,
            mpc_rate: 50.0,
        }
    }

    pub fn initialize(&mut self, initial_state: VehicleState) {
        let now = Instant::now();
        self.state.vehicle_state = initial_state;
        self.state.start_time = now;
        self.state.last_sensor_update = now;
        self.state.last_navigation_update = now;
        self.state.last_mpc_update = now;
                
        println!("Control loop initialized");
    }

    fn update_sensor_fusion(&mut self, sensor_data: &SensorData) -> bool {
        let now = Instant::now();
        if now.duration_since(self.state.last_sensor_update) >= Duration::from_secs_f64(1.0 / self.sensor_fusion_rate) {
            // Update attitude EKF based on available sensors
            if let Some(imu) = sensor_data.imu_data {
                if let Some(mag) = sensor_data.mag_data {
                    // Use 9-axis data (IMU + magnetometer)
                    let measurements = [
                        imu[1], imu[2], imu[3], // gyro
                        imu[4], imu[5], imu[6], // accel
                        mag[1], mag[2], mag[3]  // mag
                    ];
                    self.ekf_attitude.update(&measurements);
                } else {
                    // Use 6-axis data (IMU only)
                    let measurements = [
                        imu[1], imu[2], imu[3], // gyro
                        imu[4], imu[5], imu[6]  // accel
                    ];
                    self.ekf_attitude.update(&measurements);
                }
                
                // TODO: We need to update the vehicle state with the EKF state. The Flight Termination Slides has information on What sensors to priortize for determining our location and pose.
            }
            
            self.state.last_sensor_update = now;
            return true;
        }
        false
    }

    fn update_navigation(&mut self, goal_position: [f64; 3]) -> bool {
        let now = Instant::now();
        if now.duration_since(self.state.last_navigation_update) >= Duration::from_secs_f64(1.0 / self.navigation_rate) {
            // Determine flight phase based on current state
            let flight_phase = self.determine_flight_phase(goal_position);
            
            // Update tracked phase if it changed
            if flight_phase != self.state.flight_phase {
                println!("Flight phase transition: {:?} -> {:?}", self.state.flight_phase, flight_phase);
                self.state.flight_phase = flight_phase;
                self.state.last_state_time = Instant::now(); // Reset state timer
            }
            
            match flight_phase {
                FlightPhase::Ascent => {
                    // Generate ascent trajectory to goal position
                    self.generate_ascent_trajectory(goal_position);
                },
                FlightPhase::Hover => {
                    // Generate hover trajectory (maintain position)
                    self.generate_hover_trajectory(goal_position);
                },
                FlightPhase::Descent => {
                    // Generate descent trajectory back to ground
                    self.generate_descent_trajectory();
                }
            }
            
            self.state.last_navigation_update = now;
            return true;
        }
        false
    }

    fn update_mpc(&mut self) -> Option<Vector4<f64>> {
        let now = Instant::now();
        if now.duration_since(self.state.last_mpc_update) >= Duration::from_secs_f64(1.0 / self.mpc_rate) {
            if let (Some(sensor_state), Some(trajectory)) = (&self.state.sensor_fusion_state, &self.state.trajectory_state) {
                // Convert sensor state to MPC format
                let mpc_state = self.vehicle_state_to_mpc_state(sensor_state);
                
                // Generate reference trajectory
                let (xref_traj, uref_traj) = self.generate_mpc_reference(trajectory);
                
                // Simple MPC implementation - just return hover thrust for now
                let control = Vector4::new(0.0, 0.0, self.state.vehicle_state.mass * 9.81, 0.0);
                
                self.state.last_mpc_update = now;
                return Some(control);
            }
        }
        None
    }

    fn vehicle_state_to_mpc_state(&self, sensor_state: &Array1<f64>) -> Array1<f64> {
        // Convert EKF state to MPC state format
        // EKF: [roll, pitch, yaw, gyro_x, gyro_y, gyro_z]
        // MPC: [x, y, z, qx, qy, qz, qw, x_dot, y_dot, z_dot, wx, wy, wz]
        let euler_attitude = self.state.vehicle_state.attitude.euler_angles();
        let quat = self.state.vehicle_state.attitude.quaternion();
        
        Array1::from(vec![
            self.state.vehicle_state.position.x,
            self.state.vehicle_state.position.y,
            self.state.vehicle_state.position.z,
            quat.i, quat.j, quat.k, quat.w,
            self.state.vehicle_state.velocity.x,
            self.state.vehicle_state.velocity.y,
            self.state.vehicle_state.velocity.z,
            sensor_state[3], sensor_state[4], sensor_state[5], // angular rates
        ])
    }

    fn generate_mpc_reference(&self, trajectory: &rust_lossless::TrajectoryResult) -> (Vec<Array1<f64>>, Vec<Array1<f64>>) {
        let mut xref_traj = Vec::with_capacity(20);
        let mut uref_traj = Vec::with_capacity(20);
        
        for i in 0..=20 { // Fixed horizon of 20 steps
            let t_target = self.state.start_time.elapsed().as_secs_f64() + (i as f64) * 0.02; // 50 Hz = 0.02s
            
            // Simple interpolation/extrapolation logic
            let (interp_p, interp_v, interp_u) = if t_target >= trajectory.time_of_flight_s || trajectory.positions.is_empty() {
                // Use last known values or hover
                if let Some(&last_pos) = trajectory.positions.last() {
                    (last_pos, [0.0, 0.0, 0.0], [0.0, 0.0, self.state.vehicle_state.mass * 9.81])
                } else {
                    ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, self.state.vehicle_state.mass * 9.81])
                }
            } else {
                // Find trajectory point at time t_target
                let index = (t_target / trajectory.time_of_flight_s * (trajectory.positions.len() - 1) as f64) as usize;
                let index = index.min(trajectory.positions.len() - 1);
                
                let pos = trajectory.positions[index];
                let vel = if index < trajectory.velocities.len() {
                    trajectory.velocities[index]
                } else {
                    [0.0, 0.0, 0.0]
                };
                let thrust = if index < trajectory.thrusts.len() {
                    trajectory.thrusts[index]
                } else {
                    [0.0, 0.0, self.state.vehicle_state.mass * 9.81]
                };
                
                (pos, vel, thrust)
            };
            
            // Create reference state for MPC
            let xref = Array1::from(vec![
                interp_p[0], interp_p[1], interp_p[2], // position
                0.0, 0.0, 0.0, 1.0,                  // quaternion (identity for now)
                interp_v[0], interp_v[1], interp_v[2], // velocity
                0.0, 0.0, 0.0                        // angular velocity
            ]);
            
            let uref = Array1::from(vec![interp_u[0], interp_u[1], interp_u[2]]);
            
            xref_traj.push(xref);
            uref_traj.push(uref);
        }
        
        (xref_traj, uref_traj)
    }

    fn check_flight_termination(&self, sensor_data: &SensorData) -> bool {
        // 1. IMU failure - terminate immediately
        if sensor_data.imu_data.is_none() {
            return true;
        }

        // 2. GPS or UWB failure - attempt landing
        if sensor_data.gps_data.is_none() && sensor_data.uwb_data.is_none() {
            // Should trigger landing mode
        }

        // 3. Both GPS and UWB failure - use accelerometer integration
        if sensor_data.gps_data.is_none() && sensor_data.uwb_data.is_none() {
            // Should trigger emergency landing
        }

        // 4. Pressure sensor failure
        if sensor_data.chamber_pressure.is_none() || sensor_data.tank_pressure.is_none() {
            // Should trigger landing with lookup table
        }

        // 5. Angle constraint check (19.2 degrees)
        let world_z_axis = self.state.vehicle_state.attitude * Vector3::z();
        let cos_theta = world_z_axis.z;
        if cos_theta < COS_MAX_TILT {
            return true;
        }

        // 6. Position constraint check
        if let Some(_trajectory) = &self.state.trajectory_state {
            let current_pos = self.state.vehicle_state.position;
            let desired_pos = Vector3::new(0.0, 0.0, 0.0); // Simplified desired position
            
            let distance_from_path = (current_pos - desired_pos).norm();
            if distance_from_path > 2.0 {
                // Should trigger immediate landing
            }
            
            if current_pos.z < 0.0 {
                return true; // Below ground level
            }
        }

        false
    }

    pub fn step(&mut self, sensor_data: &SensorData, goal_position: [f64; 3]) -> Option<Vector4<f64>> {
        // Check flight termination conditions
        if self.check_flight_termination(sensor_data) {
            self.state.flight_terminated = true;
            println!("Flight terminated!");
            return None;
        }
        
        // Update sensor fusion (500 Hz)
        self.update_sensor_fusion(sensor_data);
        
        // Update navigation (1 Hz)
        self.update_navigation(goal_position);
        
        // Update MPC (50 Hz)
        self.update_mpc()
    }

    pub fn get_state(&self) -> &ControlLoopState {
        &self.state
    }

    fn determine_flight_phase(&self, goal_position: [f64; 3]) -> FlightPhase {
        let current_altitude = self.state.vehicle_state.position.z;
        let goal_altitude = goal_position[2];
        
        // Use tracked phase with transitions based on altitude and time
        match self.state.flight_phase {
            FlightPhase::Ascent => {
                // Transition to hover when reaching target altitude
                if current_altitude >= goal_altitude * 0.95 {
                    FlightPhase::Hover
                } else {
                    FlightPhase::Ascent
                }
            },
            FlightPhase::Hover => {
                // Hover for 20 seconds in this phase, then transition to descent
                if self.state.last_state_time.elapsed().as_secs_f64() >= 20.0 {
                    FlightPhase::Descent
                } else {
                    FlightPhase::Hover
                }
            },
            FlightPhase::Descent => {
                // Stay in descent until landing
                FlightPhase::Descent
            }
        }
    }

    fn generate_ascent_trajectory(&mut self, goal_position: [f64; 3]) {
        // Configure lossless for ascent trajectory
        self.lossless.initial_position = [
            self.state.vehicle_state.position.x,
            self.state.vehicle_state.position.y,
            self.state.vehicle_state.position.z
        ];
        self.lossless.initial_velocity = [
            self.state.vehicle_state.velocity.x,
            self.state.vehicle_state.velocity.y,
            self.state.vehicle_state.velocity.z
        ];
        self.lossless.landing_point = goal_position;
        self.lossless.dry_mass = self.state.vehicle_state.dry_mass;
        self.lossless.fuel_mass = self.state.vehicle_state.mass - self.state.vehicle_state.dry_mass;
        
        let solve_result = self.lossless.solve();
        if let Some(trajectory) = solve_result.trajectory {
            self.state.trajectory_state = Some(trajectory);
            println!("Ascent trajectory generated: {:.2}s flight time", solve_result.fine_time_of_flight_s.unwrap_or(0.0));
        } else {
            println!("Failed to generate ascent trajectory");
        }
    }

    fn generate_hover_trajectory(&mut self, goal_position: [f64; 3]) {
        // Generate simple hover trajectory (maintain position)
        let hover_duration = 20.0; // 20 seconds hover
        let positions = vec![goal_position; 21]; // 1 Hz for 20 seconds
        let velocities = vec![[0.0, 0.0, 0.0]; 21];
        let thrusts = vec![[0.0, 0.0, self.state.vehicle_state.mass * 9.81]; 20];
        let masses = vec![self.state.vehicle_state.mass; 21];
        let sigmas = vec![1.0; 20];
        
        let trajectory = TrajectoryResult {
            positions,
            velocities,
            masses,
            thrusts,
            sigmas,
            time_of_flight_s: hover_duration,
        };
        
        self.state.trajectory_state = Some(trajectory);
        println!("Hover trajectory generated: {:.2}s duration", hover_duration);
    }

    fn generate_descent_trajectory(&mut self) {
        // Configure lossless for descent trajectory (reverse of ascent)
        let current_pos = [
            self.state.vehicle_state.position.x,
            self.state.vehicle_state.position.y,
            self.state.vehicle_state.position.z
        ];
        let current_vel = [
            self.state.vehicle_state.velocity.x,
            self.state.vehicle_state.velocity.y,
            self.state.vehicle_state.velocity.z
        ];
        
        // Set landing point to ground level
        self.lossless.initial_position = current_pos;
        self.lossless.initial_velocity = current_vel;
        self.lossless.landing_point = [0.0, 0.0, 0.0]; // Ground level
        self.lossless.dry_mass = self.state.vehicle_state.dry_mass;
        self.lossless.fuel_mass = self.state.vehicle_state.mass - self.state.vehicle_state.dry_mass;
        
        let solve_result = self.lossless.solve();
        if let Some(trajectory) = solve_result.trajectory {
            self.state.trajectory_state = Some(trajectory);
            println!("Descent trajectory generated: {:.2}s flight time", solve_result.fine_time_of_flight_s.unwrap_or(0.0));
        } else {
            println!("Failed to generate descent trajectory");
        }
    }
}

#[tokio::main]
async fn main() {
    println!("VTVL Rocket Control Loop Starting...");
    
    let mut control_loop = ControlLoop::new();
    let initial_state = VehicleState::default();
    control_loop.initialize(initial_state);
    
    let goal_position = [0.0, 0.0, 50.0]; 
    
    let mut last_time = Instant::now();
    let target_dt = Duration::from_micros(2000); // 500 Hz = 2ms
    
    println!("Control loop running at 500 Hz...");
    
    loop {
        let loop_start = Instant::now();
        
        // Init sensor readings
        let sensor_data = SensorData {
            timestamp: control_loop.get_state().start_time.elapsed().as_secs_f64(),
            imu_data: None,
            mag_data: None,
            gps_data: None,
            uwb_data: None,
            chamber_pressure: None,
            tank_pressure: None,
        };
        
        // Step the control loop
        if let Some(control_output) = control_loop.step(&sensor_data, goal_position) {
            if control_loop.get_state().flight_terminated {
                println!("Flight terminated - zeroing controls");
                break;
            }
            
            // Here you would send control_output to actuators
            // println!("Control: {:?}", control_output);
        }
        
        // Maintain timing
        let elapsed = loop_start.elapsed();
        if elapsed < target_dt {
            tokio::time::sleep(target_dt - elapsed).await;
        }
        
        // Print status every second
        let elapsed_time = control_loop.get_state().start_time.elapsed().as_secs_f64();
        if (elapsed_time * 100.0) as i64 % 100 == 0 {
            println!("Time: {:.2}s, Alt: {:.2}m, Phase: {:?}", 
                elapsed_time,
                control_loop.get_state().vehicle_state.position.z,
                control_loop.get_state().flight_phase
            );
        }
    }
    
    println!("Control loop ended");
}

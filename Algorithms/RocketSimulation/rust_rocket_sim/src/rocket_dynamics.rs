use nalgebra::{Matrix3, Vector3, Vector4, UnitQuaternion, Quaternion};

use crate::device_sim::*;
use crate::sloshing_sim::*;
use crate::fluid_dynamics::*;
use ndarray::Array1;
use std::error::Error;
use std::fs::File;

#[derive(Debug, Clone)]
pub struct Rocket {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub accel: Vector3<f64>,
    
    pub attitude: UnitQuaternion<f64>,
    pub ang_vel: Vector3<f64>,
    pub ang_accel: Vector3<f64>,

    pub frame_mass: f64,
    pub nitrogen_tank_empty_mass: f64, // the mass of the empty tank
    pub nitrogen_mass: f64,
    starting_nitrogen_mass: f64,
    pub nitrogen_tank_offset: Vector3<f64>, // Vector offset from frame CoM to the nitrogen tank CoM
    pub nitrous_tank_empty_mass: f64, // the mass of the empty tank
    pub pressurizing_nitrogen_mass: f64,
    starting_pressurizing_nitrogen_mass: f64,
    pub nitrous_mass: f64,
    starting_nitrous_mass: f64,
    pub nitrous_tank_offset: Vector3<f64>, // Vector offset from frame CoM to the nitrous tank CoM
    pub tvc_module_empty_mass: f64, // the mass of the empty tvc module
    pub fuel_grain_mass: f64,
    starting_fuel_grain_mass: f64,
    pub frame_com_to_gimbal: Vector3<f64>, // Vector offset from frame CoM to the gimbal point (thrust plate)
    pub gimbal_to_tvc_com: Vector3<f64>, // Vector offset from gimbal point to the TVC's center of mass

    pub frame_moi: Matrix3<f64>,
    pub dry_nitrogen_moi: Matrix3<f64>,
    pub wet_nitrogen_moi: Matrix3<f64>,
    pub nitrous_tank_radius: f64,
    pub nitrous_tank_length: f64,
    pub nitrous_level: f64,
    pub dry_nitrous_moi: Matrix3<f64>,
    pub dry_tvc_moi: Matrix3<f64>,
    pub wet_tvc_moi: Matrix3<f64>,

    pub tvc_range: f64,
    pub tvc: TVC,

    pub rcs: RCS,

    pub thrust_vector: Vector3<f64>,

    pub imu: IMU,
    pub gps: GPS,
    pub uwb: UWB,

    pub sloshing_model: SloshModel,

    pub nist_data: NistData,
    pub nitrogen_iso_data: IsoData,
    pub nitrous_iso_data: IsoData,
    pub port_d: f64,
    pub nitrous_m_dot: f64,

    pub com_to_ground: Vector3<f64>, // Distance from center of mass to ground (for ground interaction)

    system_time: f64,

    pub debug_info: RocketDebugInfo,
}

#[derive(Debug, Clone)]
pub struct RocketDebugInfo{
    pub thrust_torque: Vector3<f64>,
    pub total_force: Vector3<f64>,
    pub total_torque: Vector3<f64>,
    pub times: Vec<f64>,
    pub com_offsets: Vec<Vector3<f64>>,
    pub mois: Vec<Matrix3<f64>>,
    pub thrusts: Vec<Vector3<f64>>,
    pub slosh_forces: Vec<Vector3<f64>>,
    pub nitrous_m_dots: Vec<f64>,
    pub valve_angles: Vec<f64>,
    pub chamber_pressures: Vec<f64>,
    pub of_ratios: Vec<f64>,
    pub isps: Vec<f64>,
    pub cstars: Vec<f64>,
    pub port_ds: Vec<f64>,
    pub fuel_masses: Vec<f64>,
    pub nitrous_masses: Vec<f64>,
    pub nitrogen_n2_tank_masses: Vec<f64>,
    pub nitrogen_n2o_tank_masses: Vec<f64>,
}

impl Rocket {
    pub fn new(position: Vector3<f64>, velocity: Vector3<f64>, accel: Vector3<f64>, attitude: UnitQuaternion<f64>, ang_vel: Vector3<f64>, ang_accel: Vector3<f64>, frame_mass: f64, nitrogen_tank_empty_mass: f64, starting_nitrogen_mass: f64, nitrogen_tank_offset: Vector3<f64>, nitrous_tank_empty_mass: f64, starting_pressurizing_nitrogen_mass: f64, starting_nitrous_mass: f64, nitrous_tank_offset: Vector3<f64>, tvc_module_empty_mass: f64, starting_fuel_grain_mass: f64, frame_com_to_gimbal: Vector3<f64>, gimbal_to_tvc_com: Vector3<f64>, frame_moi: Matrix3<f64>, dry_nitrogen_moi: Matrix3<f64>, wet_nitrogen_moi: Matrix3<f64>, nitrous_tank_radius: f64, nitrous_tank_length: f64, nitrous_level: f64, dry_nitrous_moi: Matrix3<f64>, dry_tvc_moi: Matrix3<f64>, wet_tvc_moi: Matrix3<f64>, tvc_range: f64, tvc: TVC, rcs: RCS, imu: IMU, gps: GPS, uwb: UWB, sloshing_model: SloshModel, nist_data: NistData, nitrogen_iso_data: IsoData, nitrous_iso_data: IsoData, port_d: f64, nitrous_m_dot: f64, com_to_ground: Vector3<f64>) -> Self {
        let mut rocket = Self {
            position,
            velocity,
            accel,
            attitude,
            ang_vel,
            ang_accel,
            frame_mass,
            nitrogen_tank_empty_mass,
            nitrogen_mass: starting_nitrogen_mass,
            starting_nitrogen_mass,
            nitrogen_tank_offset,
            nitrous_tank_empty_mass,
            pressurizing_nitrogen_mass: starting_pressurizing_nitrogen_mass,
            starting_pressurizing_nitrogen_mass,
            nitrous_mass: starting_nitrous_mass,
            starting_nitrous_mass,
            nitrous_tank_offset,
            tvc_module_empty_mass,
            fuel_grain_mass: starting_fuel_grain_mass,
            starting_fuel_grain_mass,
            frame_com_to_gimbal,
            gimbal_to_tvc_com,
            frame_moi,
            dry_nitrogen_moi,
            wet_nitrogen_moi,
            nitrous_tank_radius,
            nitrous_tank_length,
            nitrous_level,
            dry_nitrous_moi,
            dry_tvc_moi,
            wet_tvc_moi,
            tvc_range,
            tvc,
            rcs,
            thrust_vector: Vector3::zeros(),
            imu,
            gps,
            uwb,
            sloshing_model,
            nist_data,
            nitrogen_iso_data,
            nitrous_iso_data,
            port_d,
            nitrous_m_dot,
            com_to_ground,
            system_time: 0.0,
            debug_info: RocketDebugInfo {
                thrust_torque: Vector3::zeros(),
                total_force: Vector3::zeros(),
                total_torque: Vector3::zeros(),
                times: Vec::new(),
                com_offsets: Vec::new(),
                mois: Vec::new(),
                thrusts: Vec::new(),
                slosh_forces: Vec::new(),
                nitrous_m_dots: Vec::new(),
                valve_angles: Vec::new(),
                chamber_pressures: Vec::new(),
                of_ratios: Vec::new(),
                isps: Vec::new(),
                cstars: Vec::new(),
                port_ds: Vec::new(),
                fuel_masses: Vec::new(),
                nitrous_masses: Vec::new(),
                nitrogen_n2_tank_masses: Vec::new(),
                nitrogen_n2o_tank_masses: Vec::new(),
            },
        };

        rocket.imu.update(rocket.accel, rocket.ang_vel, rocket.attitude, rocket.system_time);
        rocket.gps.update(rocket.position, rocket.system_time);
        rocket.uwb.update(rocket.position, rocket.system_time);

        rocket
    }

    /// Update state based on applied forces and torques
    /// forces: Force vector in World Frame
    /// torques: Torque vector in Body Frame
    /// Returns true if the step is successful and false if the simulation has ended (hit the ground)
    pub fn step(&mut self, control_input: Vector4<f64>, outside_forces: Vector3<f64>, outside_torques: Vector3<f64>, dt: f64) -> bool {
        let rotated_offset = self.attitude.transform_vector(&self.com_to_ground);
        let rocket_bottom = self.position + rotated_offset;
        if rocket_bottom.z < 0.0 {
            // 1. Shift the rocket UP by the exact amount it went underground.
            // (Because rocket_bottom.z is negative, subtracting it adds positive height)
            self.position.z -= rocket_bottom.z; // Add a small buffer of 1 cm to prevent immediate re-collision on the next frame

            // 2. CRITICAL: Kill the downward velocity!
            // If you don't do this, the math still thinks it's falling at -5m/s, 
            // and it will just instantly clip back underground on the next frame.
            self.velocity.x = 0.0;
            self.velocity.y = 0.0;
            if self.velocity.z < 0.0 { // Assuming your velocity variable is named this
                self.velocity.z = 0.0;
            }
            return false; // Indicate that we've hit the ground
        }
        
        self.debug_info.times.push(self.system_time);

        // Update Sensors
        self.imu.update(self.accel, self.ang_vel, self.attitude, self.system_time);
        self.gps.update(self.position, self.system_time);
        self.uwb.update(self.position, self.system_time);

        let com_offset = self.get_com_offset(self.thrust_vector);
        let moi = self.get_moi(self.thrust_vector, com_offset);

        // Update actuated devices
        let tvc_effect: TVCEffect = self.tvc.update(Vector3::new(control_input.x, control_input.y, control_input.z), self.frame_com_to_gimbal - com_offset, self.nitrogen_mass, self.pressurizing_nitrogen_mass, self.nitrous_mass, self.fuel_grain_mass, dt, self.system_time);
        self.nitrogen_mass = tvc_effect.nitrogen_mass;
        self.pressurizing_nitrogen_mass = tvc_effect.pressurizing_nitrogen_mass;
        self.nitrous_mass = tvc_effect.nitrous_mass;
        self.fuel_grain_mass = tvc_effect.fuel_grain_mass;

        // TODO: implement throttle controller
        // TODO: talk to team and change the control vector to have 4 dimensions (add in rcs control command)
        let rcs_command = control_input.w; // Assuming the 4th element of control_input is for RCS
        let rcs_effect = self.rcs.update(rcs_command, self.nitrogen_mass, dt, self.system_time);
        self.nitrogen_mass = rcs_effect.nitrogen_mass;

        
        // Fluid dynamics update
        let thrust_command = control_input[2];
        let is_rcs_on = control_input[3] != 0.0;
        let n2o_mass = self.nitrous_mass;
        let port_d = self.port_d;
        let a = 0.0000722;
        let n = 0.67;
        let fluid_dynamics_dt = dt;
        let sat_n2o = &self.nist_data;
        let isobaric_n2o = &self.nitrous_iso_data;
        let runtank_vol = 0.032;
        let tank_d = 0.254;
        // Input some fluid properties
        let temp = 301.15; // Temp in Kelvin --> 82.4F
        // Interpolate N2O Density
        let rho_n2o = interp1(&self.nitrous_iso_data.t_iso, &self.nitrous_iso_data.rho_iso, temp);
        // Interpolate N2 Density
        let rho_n2 = interp1(&self.nitrogen_iso_data.t_iso, &self.nitrogen_iso_data.rho_iso, temp);
        let n2_mass_total = self.nitrogen_mass + self.pressurizing_nitrogen_mass;
        let n2_mass_flowrate_rcs = 0.0085 * 2.0;
        let fluid_dynamics_output = fluid_dynamics_update(thrust_command, is_rcs_on, n2o_mass, port_d, a, n, fluid_dynamics_dt, &sat_n2o, &isobaric_n2o, runtank_vol, tank_d, rho_n2o, rho_n2, n2_mass_total, n2_mass_flowrate_rcs);

        self.fuel_grain_mass = fluid_dynamics_output.new_fuel_mass;
        self.nitrous_mass = fluid_dynamics_output.new_n2o_mass;
        self.nitrous_level = fluid_dynamics_output.new_n2o_level;
        self.nitrogen_mass = fluid_dynamics_output.new_n2_mass_storagetanks;
        self.pressurizing_nitrogen_mass = fluid_dynamics_output.new_n2_mass_runtank;
        self.port_d = fluid_dynamics_output.new_port_d;
        self.nitrous_m_dot = fluid_dynamics_output.mdot_ox;

        self.debug_info.com_offsets.push(com_offset);
        self.debug_info.mois.push(moi);
        self.debug_info.nitrous_m_dots.push(self.nitrous_m_dot);
        self.debug_info.valve_angles.push(fluid_dynamics_output.valve_angle); // This is in degrees!
        self.debug_info.chamber_pressures.push(fluid_dynamics_output.pc_bar);
        self.debug_info.of_ratios.push(fluid_dynamics_output.of_ratio_realized);
        self.debug_info.isps.push(fluid_dynamics_output.isp_realized);
        self.debug_info.cstars.push(fluid_dynamics_output.cstar_realized);
        self.debug_info.port_ds.push(self.port_d);
        self.debug_info.fuel_masses.push(self.fuel_grain_mass);
        self.debug_info.nitrous_masses.push(self.nitrous_mass);
        self.debug_info.nitrogen_n2_tank_masses.push(self.nitrogen_mass);
        self.debug_info.nitrogen_n2o_tank_masses.push(self.pressurizing_nitrogen_mass);


        self.system_time += dt;
        let mass = self.get_mass();

        let slosh_force = self.sloshing_model.step(dt, self.accel, self.attitude, self.nitrous_m_dot);

        // Translational Dynamics
        let gravity = Vector3::new(0.0, 0.0, -9.81);
        self.thrust_vector = self.attitude.transform_vector(&tvc_effect.thrust);
        let slosh_force_world = self.attitude.transform_vector(&slosh_force);
        let body_vel = self.attitude.transform_vector(&self.velocity);
        let drag = 0.5 * 1.225 * Vector3::new(body_vel.x.powi(2) * 2.0 * 1.2, body_vel.y.powi(2) * 2.0 * 1.2, body_vel.z.powi(2) * 0.85 * 0.25);
        let total_force = outside_forces + (gravity * mass) + self.thrust_vector + slosh_force_world + drag;
        self.debug_info.total_force = total_force;
        self.debug_info.thrusts.push(tvc_effect.thrust);
        self.debug_info.slosh_forces.push(slosh_force);

        self.accel = total_force / mass;        
        self.velocity += self.accel * dt;
        self.position += self.velocity * dt;

        // 2. Rotational Dynamics (Euler's rotation equations)
        // Torque = I * alpha + omega x (I * omega)
        // alpha = I_inv * (Torque - omega x (I * omega))
        
        // We calculate I * omega manually since inertia is a diagonal Vector3 here
        let i_omega = moi * self.ang_vel;

        let gyro_torque = self.ang_vel.cross(&i_omega);
        let slosh_lever_arm = self.nitrous_tank_offset - com_offset;
        let slosh_torque = slosh_lever_arm.cross(&slosh_force);
        let net_torque = outside_torques - gyro_torque + tvc_effect.torque + rcs_effect.torque + slosh_torque;
        self.debug_info.thrust_torque = tvc_effect.torque;
        self.debug_info.total_torque = net_torque;

        // Angular acceleration (alpha)
        let moi_inv = moi.try_inverse().expect("MOI matrix must be invertible");
        let alpha = moi_inv * net_torque;

        // Update Angular Velocity
        self.ang_vel += alpha * dt;

        // 3. Update Attitude (Quaternion Integration)
        // q_dot = 0.5 * quaternion(0, omega) * q
        let omega_quat = Quaternion::new(
            0.0, 
            self.ang_vel.x, 
            self.ang_vel.y, 
            self.ang_vel.z
        );
        
        // Standard approach: q_new = q_old + (0.5 * omega * q_old) * dt
        // nalgebra handles the multiplication logic for us
        let delta_q = UnitQuaternion::from_quaternion(omega_quat); 
        
        // Note: For small timesteps, we approximate the integration.
        // A common robust method is integrating the angle axis directly:
        let angle = self.ang_vel.norm() * dt;
        if angle > 1e-6 {
            let axis = UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(self.ang_vel), angle);
            // Apply rotation: new_attitude = old_attitude * delta_rotation
            self.attitude = self.attitude * axis;
        }

        return true; // Indicate successful step
    }

    pub fn get_mass(&self) -> f64 {
        self.get_dry_mass() + self.nitrogen_mass + self.pressurizing_nitrogen_mass + self.nitrous_mass + self.fuel_grain_mass
    }

    pub fn get_dry_mass(&self) -> f64 {
        self.frame_mass + self.nitrogen_tank_empty_mass + self.nitrous_tank_empty_mass + self.tvc_module_empty_mass
    }

    // This is expressed as a vector offset from the frame CoM
    pub fn get_com_offset(&self, thrust_vector: Vector3<f64>) -> Vector3<f64> {
        let total_mass = self.get_mass();

        let nitrogen_com = self.nitrogen_tank_offset;

        let liquid_nitrous_tank_com_offset = Vector3::new(0.0, 0.0, (self.nitrous_level - self.nitrous_tank_length) / 2.0);
        let pressurizing_nitrogen_tank_com_offset = Vector3::new(0.0, 0.0, (self.nitrous_tank_length - self.nitrous_level) / 2.0);
        let nitrous_tank_mass = self.nitrous_tank_empty_mass + self.pressurizing_nitrogen_mass + self.nitrous_mass;
        let nitrous_com = self.nitrous_tank_offset + (self.nitrous_mass / (nitrous_tank_mass)) * liquid_nitrous_tank_com_offset + (self.pressurizing_nitrogen_mass / (nitrous_tank_mass)) * pressurizing_nitrogen_tank_com_offset;
        
        let thrust_direction = thrust_vector.normalize();
        let vertical = Vector3::new(0.0, 0.0, 1.0);
        let rotation = UnitQuaternion::rotation_between(&vertical, &thrust_direction).unwrap_or_else(|| UnitQuaternion::identity());
        let tvc_com = self.frame_com_to_gimbal + rotation * self.gimbal_to_tvc_com;


        let combined_com = (nitrogen_com * (self.nitrogen_tank_empty_mass + self.nitrogen_mass) / total_mass
                            + nitrous_com * (nitrous_tank_mass) / total_mass
                            + tvc_com * (self.tvc_module_empty_mass + self.fuel_grain_mass) / total_mass);
        
        combined_com
    }

    pub fn get_moi(&self, thrust_vector: Vector3<f64>, com_offset: Vector3<f64>) -> Matrix3<f64> {
        let nitrogen_moi = self.transform_moi(self.weight_matrices(self.nitrogen_mass / self.starting_nitrogen_mass, self.wet_nitrogen_moi, self.dry_nitrogen_moi), self.nitrogen_tank_offset + com_offset, self.nitrogen_tank_empty_mass + self.nitrogen_mass);
        
        let liquid_nitrous_tank_com_offset = Vector3::new(0.0, 0.0, (self.nitrous_level - self.nitrous_tank_length) / 2.0);
        let pressurizing_nitrogen_tank_com_offset = Vector3::new(0.0, 0.0, (self.nitrous_tank_length - self.nitrous_level) / 2.0);
        let liquid_nitrous_moi = self.transform_moi(self.get_cylinder_moi(self.nitrous_tank_radius, self.nitrous_tank_length, self.nitrous_mass), -liquid_nitrous_tank_com_offset, self.nitrous_mass);
        let pressurizing_nitrogen_moi = self.transform_moi(self.get_cylinder_moi(self.nitrous_tank_radius, self.nitrous_tank_length, self.pressurizing_nitrogen_mass), -pressurizing_nitrogen_tank_com_offset, self.pressurizing_nitrogen_mass);
        let nitrous_moi = self.transform_moi(self.dry_nitrous_moi, self.nitrous_tank_offset + com_offset, self.nitrous_tank_empty_mass + self.pressurizing_nitrogen_mass + self.nitrous_mass);
        
        let thrust_direction = thrust_vector.normalize();
        let vertical = Vector3::new(0.0, 0.0, 1.0);
        let rotation = UnitQuaternion::rotation_between(&vertical, &thrust_direction).unwrap_or_else(|| UnitQuaternion::identity());
        let rotated_dry_tvc_moi = rotation * self.gimbal_to_tvc_com;
        let tvc_moi = self.transform_moi(self.weight_matrices(self.fuel_grain_mass / self.starting_fuel_grain_mass, self.wet_tvc_moi, self.dry_tvc_moi), com_offset + self.frame_com_to_gimbal + rotated_dry_tvc_moi, self.tvc_module_empty_mass + self.fuel_grain_mass);
        
        
        let moi = self.frame_moi
            + nitrogen_moi
            + nitrous_moi
            + tvc_moi;
        
        moi
    }

    pub fn get_cylinder_moi(&self, radius: f64, height: f64, mass: f64) -> Matrix3<f64> {
        Matrix3::new((mass * (3.0 * radius.powi(2) + height.powi(2))) / 12.0, 0.0, 0.0,
                    0.0, (mass * (3.0 * radius.powi(2) + height.powi(2))) / 12.0, 0.0,
                    0.0, 0.0, 0.5 * mass * radius.powi(2))
    }

    // d is a vector from old center of rotation to new center of rotation
    pub fn transform_moi(&self, moi: Matrix3<f64>, d: Vector3<f64>, mass: f64) -> Matrix3<f64> {
        let y2_z2 = mass * (d.y * d.y + d.z * d.z);
        let x2_z2 = mass * (d.x * d.x + d.z * d.z);
        let x2_y2 = mass * (d.x * d.x + d.y * d.y);
        let neg_xy = -mass * d.x * d.y;
        let neg_xz = -mass * d.x * d.z;
        let neg_yz = -mass * d.y * d.z;

        Matrix3::new(
            moi[(0, 0)] + y2_z2,
            moi[(0, 1)] + neg_xy,
            moi[(0, 2)] + neg_xz,
            moi[(1, 0)] + neg_xy,
            moi[(1, 1)] + x2_z2,
            moi[(1, 2)] + neg_yz,
            moi[(2, 0)] + neg_xz,
            moi[(2, 1)] + neg_yz,
            moi[(2, 2)] + x2_y2,
        )
    }

    pub fn weight_matrices(&self, weight: f64, matrix1: Matrix3<f64>, matrix2: Matrix3<f64>) -> Matrix3<f64> {
        weight * matrix1 + (1.0 - weight) * matrix2
    }

    pub fn weight_vectors(&self, weight: f64, vector1: Vector3<f64>, vector2: Vector3<f64>) -> Vector3<f64> {
        weight * vector1 + (1.0 - weight) * vector2
    }

    // State vector: [x, y, z, qx, qy, qz, qw, x_dot, y_dot, z_dot, wx, wy, wz]
    // This is formatted for use in the MPC controller
    pub fn get_state(&self) -> Array1<f64> {
        let mut state = Array1::zeros(13);
        state[0] = self.position.x;
        state[1] = self.position.y;
        state[2] = self.position.z;
        state[3] = self.attitude.i;
        state[4] = self.attitude.j;
        state[5] = self.attitude.k;
        state[6] = self.attitude.w;
        state[7] = self.velocity.x;
        state[8] = self.velocity.y;
        state[9] = self.velocity.z;
        state[10] = self.ang_vel.x;
        state[11] = self.ang_vel.y;
        state[12] = self.ang_vel.z;

        state
    }

    pub fn save_debug_to_csv(&self, file_path: &str) -> Result<(), Box<dyn Error>> {
        let file = File::create(file_path)?;
        let mut wtr = csv::Writer::from_writer(file);

        // 1. Write the flattened CSV Header
        wtr.write_record(&[
            "time",
            // COM Offset (Vector3)
            "com_x", "com_y", "com_z",
            // MOI Tensor (Matrix3 - Symmetric 6 values)
            "i_xx", "i_yy", "i_zz", "i_xy", "i_xz", "i_yz",
            // Thrust (Vector3)
            "thrust_x", "thrust_y", "thrust_z",
            // Slosh Force (Vector3)
            "slosh_x", "slosh_y", "slosh_z",
            // Thermodynamic & Mass Scalars
            "nitrous_m_dot", "valve_angle", "chamber_pressure",
            "of_ratio", "isp", "cstar", "port_d",
            "fuel_mass", "nitrous_mass", "n2_tank_mass", "n2o_tank_mass"
        ])?;

        let num_records = self.debug_info.times.len();

        // 2. Loop through every time step and write the row
        for i in 0..num_records {
            // Safety check: Prevents a crash if one array forgot to push a value during a tick!
            if i >= self.debug_info.com_offsets.len() || i >= self.debug_info.mois.len() || i >= self.debug_info.thrusts.len() {
                eprintln!("Warning: Data arrays out of sync at row {}. Stopping export early.", i);
                break;
            }

            let com = self.debug_info.com_offsets[i];
            let moi = self.debug_info.mois[i];
            let thrust = self.debug_info.thrusts[i];
            let slosh = self.debug_info.slosh_forces[i];

            // 3. Write the row data
            wtr.write_record(&[
                self.debug_info.times[i].to_string(),
                
                com.x.to_string(), com.y.to_string(), com.z.to_string(),
                
                moi[(0,0)].to_string(), moi[(1,1)].to_string(), moi[(2,2)].to_string(),
                moi[(0,1)].to_string(), moi[(0,2)].to_string(), moi[(1,2)].to_string(),
                
                thrust.x.to_string(), thrust.y.to_string(), thrust.z.to_string(),
                
                slosh.x.to_string(), slosh.y.to_string(), slosh.z.to_string(),
                
                self.debug_info.nitrous_m_dots[i].to_string(),
                self.debug_info.valve_angles[i].to_string(),
                self.debug_info.chamber_pressures[i].to_string(),
                self.debug_info.of_ratios[i].to_string(),
                self.debug_info.isps[i].to_string(),
                self.debug_info.cstars[i].to_string(),
                self.debug_info.port_ds[i].to_string(),
                self.debug_info.fuel_masses[i].to_string(),
                self.debug_info.nitrous_masses[i].to_string(),
                self.debug_info.nitrogen_n2_tank_masses[i].to_string(),
                self.debug_info.nitrogen_n2o_tank_masses[i].to_string(),
            ])?;
        }

        wtr.flush()?;
        println!("Successfully exported {} rows of telemetry to '{}'", num_records, file_path);
        
        Ok(())
    }
}

fn main() {
    // let mut rocket = Rocket::new();

    // println!("Starting Simulation...");
    // println!("Initial State: Z = {:.2} m", rocket.position.z);

    // // Simulation Loop
    // for i in 0..100 {
    //     // Simulate a thruster firing upwards (World Frame Z)
    //     // Force = 20,000 N (enough to overcome gravity: 1000kg * 9.81 = 9810 N)
    //     let thrust_force = Vector3::new(0.0, 0.0, 20000.0);
        
    //     // Simulate a small torque causing a spin (Body Frame)
    //     let control_torque = Vector3::new(0.0, 0.0, 10.0);

    //     rocket.step(thrust_force, control_torque);

    //     if i % 10 == 0 {
    //         println!(
    //             "T={:.2}s | Pos Z: {:.2} | Vel Z: {:.2} | Roll Rate: {:.4}", 
    //             (i as f64) * dt,
    //             rocket.position.z, 
    //             rocket.velocity.z,
    //             rocket.angular_velocity.z
    //         );
    //     }
    // }
}
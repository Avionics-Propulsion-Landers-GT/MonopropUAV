use nalgebra::{Vector2, Vector3, UnitQuaternion};
use rand::prelude::*;
use rand_distr::{Normal, Distribution};


/*
Acceleration units are in m/s^2
Gyro units are in rad/s
Mag units are in teslas
*/
#[derive(Debug, Clone)]
pub struct IMU {
    pub accel_noise_sigma: Vector3<f64>,
    pub accel_offset: Vector3<f64>,

    pub gyro_noise_sigma: Vector3<f64>,
    pub gyro_drift: Vector3<f64>,

    pub mag_noise_sigma: Vector3<f64>,
    pub mag_offset: Vector3<f64>,
    pub earth_mag: Vector3<f64>,

    pub last_reading: IMUReading,

    pub update_rate: f64, // This is in updates per second
    system_time: f64,
}

// These readings are in the body frame
#[derive(Debug, Clone)]
pub struct IMUReading {
    pub accel: Vector3<f64>,
    pub gyro: Vector3<f64>,
    pub mag: Vector3<f64>,
}

impl IMU {
    pub fn new(accel_noise_sigma: Vector3<f64>, accel_offset: Vector3<f64>, gyro_noise_sigma: Vector3<f64>, gyro_drift: Vector3<f64>, mag_noise_sigma: Vector3<f64>, mag_offset: Vector3<f64>, earth_mag: Vector3<f64>, update_rate: f64) -> Self {
        Self {
            accel_noise_sigma,
            accel_offset,
            gyro_noise_sigma,
            gyro_drift,
            mag_noise_sigma,
            mag_offset,
            earth_mag,
            last_reading: IMUReading {
                accel: Vector3::zeros(),
                gyro: Vector3::zeros(),
                mag: Vector3::zeros(),
            },
            update_rate,
            system_time: 0.0,
        }
    }

    pub fn update(&mut self, accel: Vector3<f64>, ang_vel: Vector3<f64>, attitude: UnitQuaternion<f64>, system_time: f64) -> IMUReading {
        let elapsed_time = self.system_time - system_time;
        if elapsed_time < 1.0 / self.update_rate {
            return self.last_reading;
        }

        self.system_time = system_time;

        let mut rng = rand::rng();

        // 1. ACCELEROMETER
        // Accelerometers measure "Proper Acceleration" (Acceleration - Gravity).
        // If the rocket is sitting on the pad, accel is 0, but the sensor feels 1g UP.
        // We assume Gravity is -9.81 in Z.
        let gravity = Vector3::new(0.0, 0.0, -9.81);
        let proper_accel = accel - gravity;
        
        // Rotate into Body Frame: q_inverse * vector
        let proper_accel_body = attitude.inverse_transform_vector(&proper_accel);
        
        // Add Bias + Noise
        let measured_accel = proper_accel_body 
            + self.accel_offset 
            + noise(&self.accel_noise_sigma, &mut rng);


        // 2. GYROSCOPE
        // Gyros measure angular velocity directly in the Body Frame.
        // Usually no rotation needed if input is already body rates.
        let ang_vel_body = attitude.inverse_transform_vector(&ang_vel);

        let measured_gyro = ang_vel_body 
            + self.gyro_drift 
            + noise(&self.gyro_noise_sigma, &mut rng);


        // 3. MAGNETOMETER
        // Rotates the known Earth magnetic vector (World) into the Body frame.
        let mag_body = attitude.inverse_transform_vector(&self.earth_mag);
        
        let measured_mag = mag_body 
            + self.mag_offset 
            + noise(&self.mag_noise_sigma, &mut rng);

        self.last_reading = IMUReading {
            accel: measured_accel,
            gyro: measured_gyro,
            mag: measured_mag,
        };

        self.last_reading
    }
}


#[derive(Debug, Clone)]
pub struct GPS {
    pub pos_noise_sigma: Vector3<f64>,
    pub pos_offset: Vector3<f64>,
    pub last_reading: GPSReading,

    pub update_rate: f64, // This is in updates per second
    system_time: f64,
}

#[derive(Debug, Clone)]
pub struct GPSReading {
    pub position: Vector3<f64>,
}

impl GPS {
    pub fn new(pos_noise_sigma: Vector3<f64>, pos_offset: Vector3<f64>, update_rate: f64) -> Self {
        Self {
            pos_noise_sigma,
            pos_offset,
            last_reading: GPSReading {
                position: Vector3::zeros(),
            },
            update_rate,
            system_time: 0.0,
        }
    }

    pub fn update(&mut self, position: Vector3<f64>, system_time: f64) -> GPSReading {
        let elapsed_time = self.system_time - system_time;
        if elapsed_time < 1.0 / self.update_rate {
            return self.last_reading;
        }
        
        self.system_time = system_time;

        let mut rng = rand::rng();

        let noisy_position = position + self.pos_offset + noise(&self.pos_noise_sigma, &mut rng);

        self.last_reading = GPSReading {
            position: noisy_position,
        };

        self.last_reading
    }
}


#[derive(Debug, Clone)]
pub struct UWB {
    pub pos_noise_sigma: Vector3<f64>,
    pub pos_offset: Vector3<f64>,
    pub origin: Vector3<f64>,
    pub range: f64,
    pub last_reading: UWBReading,

    pub update_rate: f64, // This is in updates per second
    system_time: f64,
}

#[derive(Debug, Clone)]
pub struct UWBReading {
    pub position: Vector3<f64>,
}

impl UWB {
    pub fn new(pos_noise_sigma: Vector3<f64>, pos_offset: Vector3<f64>, origin: Vector3<f64>, range: f64, update_rate: f64) -> Self {
        Self {
            pos_noise_sigma,
            pos_offset,
            origin,
            range,
            last_reading: UWBReading {
                position: origin.clone(),
            },
            update_rate,
            system_time: 0.0,
        }
    }

    pub fn update(&mut self, position: Vector3<f64>, system_time: f64) -> UWBReading {
        let elapsed_time = self.system_time - system_time;
        if elapsed_time < 1.0 / self.update_rate {
            return self.last_reading;
        }
        
        self.system_time = system_time;

        if (position - self.origin).norm() > self.range {
            // Out of range behavior simply returns the last valid read. Check with Avionics for actual behavior
            return self.last_reading;
        } else {
            let mut rng = rand::rng();

            let noisy_position = position + self.pos_offset + noise(&self.pos_noise_sigma, &mut rng);

            self.last_reading = UWBReading {
                position: noisy_position,
            };

            return self.last_reading;
        }
    }
}


#[derive(Debug, Clone)]
pub struct TVCActuator {
    position: f64,
    velocity: f64,
    accel: f64,

    pub extension_limit: f64,
    pub unloaded_speed: f64,
    pub stall_force: f64,
    pub p_gain: f64,

    pub update_rate: f64, // This is in updates per second
    system_time: f64,
}

impl TVCActuator {
    pub fn new (start_position: f64, extension_limit: f64, unloaded_speed: f64, stall_force: f64, p_gain: f64, update_rate: f64) -> Self {
        Self {
            position: start_position,
            velocity: 0.0,
            extension_limit,
            unloaded_speed,
            stall_force,
            p_gain,
            update_rate,
            system_time: 0.0,
        }
    }

    pub fn update(&mut self, target_position: f64, load_force: f64, dt: f64, system_time: f64) {
        let elapsed_time = self.system_time - system_time;
        if elapsed_time < 1.0 / self.update_rate {
            // velocity stays constant, therefore acceleration is zero, but position does update
            self.accel = 0.0;
            self.position += self.velocity * dt;
            self.position = self.position.clamp(0.0, self.extension_limit);
            return;
        }
        
        self.system_time = system_time;

        let error = target_position - self.position;
        let command = error * self.p_gain;

        let load_factor = (load_force.abs() / self.stall_force).min(1.0);
        let speed_limit = self.unloaded_speed * (1.0 - load_factor);

        let prev_velocity = self.velocity;
        self.velocity = command.clamp(-speed_limit, speed_limit);
        self.accel = (self.velocity - prev_velocity) / dt;
        self.position += self.velocity * dt;
        self.position = self.position.clamp(0.0, self.extension_limit);
    }

    pub fn get_accel(&self) -> f64 {
        self.accel
    }
}

#[derive(Debug, Clone)]
pub struct MTV {
    pub angle: f64,
    pub ang_vel: f64,
    pub ang_accel: f64,

    pub unloaded_speed: f64,
    pub stall_torque: f64,
    pub p_gain: f64,

    pub valve_torque: f64,

    starting_fuel_grain_mass: f64,

    pub update_rate: f64,
    system_time: f64,
}

#[derive(Debug, Clone)]
pub struct MTVEffect {
    pub nitrous_mass: f64,
    pub fuel_grain_mass: f64,
    pub thrust: f64,
}

impl MTV {
    pub fn new(angle: f64, ang_vel: f64, ang_accel: f64, unloaded_speed: f64, stall_torque: f64, p_gain: f64, valve_torque: f64, starting_fuel_grain_mass: f64, update_rate: f64) -> Self {
        Self {
            angle,
            ang_vel,
            ang_accel,
            unloaded_speed,
            stall_torque,
            p_gain,
            valve_torque,
            starting_fuel_grain_mass,
            update_rate,
            system_time: 0.0,
        }
    }

    pub fn update(&mut self, target_thrust: f64, nitrous_mass: f64, fuel_grain_mass: f64, dt: f64, system_time: f64) {
        let elapsed_time = self.system_time - system_time;
        if elapsed_time < 1.0 / self.update_rate {
            // angular velocity stays constant, therefore angular acceleration is zero, but angle does update
            self.ang_accel = 0.0;
            self.angle += self.ang_vel * dt;
            self.angle = clamp(self.angle, 0.0, 90.0);
            return;
        }

        self.system_time = system_time;

        // TODO: convert the target thrust into a target angle
        let target_angle = 0.0;

        let error = target_angle - self.angle;
        let command = error * self.p_gain;

        let load_factor = (self.valve_torque / self.stall_torque).min(1.0);
        let speed_limit = self.unloaded_speed * (1.0 - load_factor);

        let prev_ang_vel = self.ang_vel;
        self.ang_vel = command.clamp(-speed_limit, speed_limit);
        self.ang_accel = (self.ang_vel - prev_ang_vel) / dt;
        self.angle += self.ang_vel * dt;
        self.angle = clamp(self.angle, 0.0, 90.0);

        let thrust = get_thrust(nitrous_mass, fuel_grain_mass);
        // TODO: update the nitrous and fuel grain masses

        return MTVEffect {
            nitrous_mass,
            fuel_grain_mass,
            thrust,
        }
    }

    pub fn get_thrust(&mut self, nitrous_mass: f64, fuel_grain_mass: f64) -> f64 {
        // TODO: use the current throttle angle to find the flow rate, then determine thrust
        // Remember to take into account either mass running out
        // TODO: also model the thrust decay somehow?
        0.0
    }
}

#[derive(Debug, Clone)]
pub struct TVC {
    pub mtv: MTV,
    pub x_actuator: TVCActuator,
    pub y_actuator: TVCActuator,
    ang_vel: Vector3<f64>,
    ang_accel: Vector3<f64>,
    pub tvc_torque: Vector3<f64>,
    pub actuator_lever_arm: f64,
    pub tvc_lever_arm: Vector3<f64>,
    pub max_fuel_inertia: f64,
    pub min_fuel_inertia: f64,
    starting_fuel_grain_mass: f64,
}

#[derive(Debug, Clone)]
pub struct TVCEffect {
    pub torque: Vector3<f64>,
    pub nitrous_mass: f64,
    pub fuel_grain_mass: f64,
}

impl TVC {
    // Our actuators appear to have a stall force of 400 lbf (need to convert to metric) and an unloaded speed of 4.777173913 in/s (also convert)
    pub fn new(mtv: MTV, x_actuator: TVCActuator, y_actuator: TVCActuator, actuator_lever_arm: f64, tvc_lever_arm: Vector3<f64>, max_fuel_inertia: f64, min_fuel_inertia: f64, starting_fuel_grain_mass: f64) -> Self {
        Self {
            mtv,
            x_actuator,
            y_actuator,
            ang_vel: Vector3::zeros(),
            ang_accel: Vector3::zeros(),
            tvc_torque: Vector3::zeros(),
            actuator_lever_arm,
            tvc_lever_arm: Vector3<f64>,
            max_fuel_inertia,
            min_fuel_inertia,
            starting_fuel_grain_mass,
        }
    }

    // engine is 11 kg with fuel, awaiting empty mass data
    // Returns the reaction torque applied to the rocket body
    // only the first two elements of command are used, the third is thrust
    pub fn update(&mut self, command: Vector3<f64>, nitrous_mass: f64, fuel_grain_mass: f64, dt: f64, system_time: f64) -> Vector3<f64> {
        let mtv_effect = self.mtv.update(command[2], nitrous_mass, fuel_grain_mass, dt, system_time);

        let engine_inertia = self.min_fuel_inertia + (self.max_fuel_inertia - self.min_fuel_inertia) * (fuel_grain_mass / self.starting_fuel_grain_mass);
        let x_load = (engine_inertia * self.x_actuator.get_accel()) / self.actuator_lever_arm.powi(2);
        let y_load = (engine_inertia * self.y_actuator.get_accel()) / self.actuator_lever_arm.powi(2);

        // TODO: translate gimbal angle commands to actuator target positions and update ang vels and ang accel
        let x_target = 0.0;
        let y_target = 0.0;

        self.x_actuator.update(x_target, x_load, dt, system_time);
        self.y_actuator.update(y_target, y_load, dt, system_time);

        let reaction_torque = self.ang_accel * engine_inertia * -1.0;

        let pitch_rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), pitch_angle);
        let yaw_rot = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), yaw_angle);
        let gimbal_rotation = yaw_rot * pitch_rot;

        let thrust_vector = gimbal_rotation.transform_vector(Vector3::new(0.0, 0.0, mtv_effect.thrust));
        let thrust_torque = self.tvc_lever_arm.cross(&thrust_vector);

        self.tvc_torque = reaction_torque + thrust_torque;

        return self.tvc_torque.clone();

        return TVCEffect {
            torque,
            nitrous_mass: mtv_effect.nitrous_mass,
            fuel_grain_mass: mtv_effect.fuel_grain_mass,
        }
    }
}


#[derive(Debug, Clone)]
pub struct RCS {
    pub thrust: f64, // this is per thruster
    pub lever_arm: f64,
    last_command: f64,

    pub nitrogen_consumption_rate: f64, // this is in kg/s

    pub update_rate: f64, // This is in updates per second
    system_time: f64,
}

#[derive(Debug, Clone)]
pub struct RCSEffect {
    pub torque: Vector3<f64>,
    pub nitrogen_mass: f64,
}

impl RCS {
    pub fn new(thrust: f64, lever_arm: f64, nitrogen_consumption_rate: f64, update_rate: f64) -> Self{
        Self {
            thrust,
            lever_arm,
            last_command = 0.0,
            nitrogen_consumption_rate
            update_rate,
            system_time: 0.0,
        }
    }

    // The command is either positive, 0, or negative. positive is roll right, negative is roll left
    // returns the torque on the rocket body
    pub fn update(&mut self, command: f64, nitrogen_mass: f64, dt: f64, system_time: f64) -> RCSEffect {
        let mut remaining_nitrogen_mass = nitrogen_mass;
        let mut actual_command = command;

        let elapsed_time = self.system_time - system_time;
        if elapsed_time < 1.0 / self.update_rate {
            actual_command = self.last_command;
        } else {
            self.system_time = system_time;
        }
        
        let torque_magnitude = 2.0 * self.thrust * self.lever_arm;
        let mut torque;
        if (actual_command == 0.0) {
            // No action needed
            torque = Vector3::zeros();
        } else if actual_command > 0.0 {
            // Positive Command -> Negative Z Torque
            // This will roll clockwise from a top view
            torque = Vector3::new(0.0, 0.0, -torque_magnitude);
            remaining_nitrogen_mass -= self.nitrogen_consumption_rate * dt;
        } else {
            // Negative Command -> Positive Z Torque
            // This will roll counterclockwise from a top view
            torque = Vector3::new(0.0, 0.0, torque_magnitude);
            remaining_nitrogen_mass -= self.nitrogen_consumption_rate * dt;
        }

        RCSEffect {
            torque,
            nitrogen_mass: remaining_nitrogen_mass,
        }
    }
}

// Helper to generate 3D noise
fn noise(sigma: &Vector3<f64>, rng: &mut ThreadRng) -> Vector3<f64> {
    let n_x = Normal::new(0.0, sigma.x).unwrap().sample(rng);
    let n_y = Normal::new(0.0, sigma.y).unwrap().sample(rng);
    let n_z = Normal::new(0.0, sigma.z).unwrap().sample(rng);
    Vector3::new(n_x, n_y, n_z)
}
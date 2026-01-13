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
}

// These readings are in the body frame
#[derive(Debug, Clone)]
pub struct IMUReading {
    pub accel: Vector3<f64>,
    pub gyro: Vector3<f64>,
    pub mag: Vector3<f64>,
}

impl IMU {
    pub fn new(accel_noise_sigma: Vector3<f64>, accel_offset: Vector3<f64>, gyro_noise_sigma: Vector3<f64>, gyro_drift: Vector3<f64>, mag_noise_sigma: Vector3<f64>, mag_offset: Vector3<f64>, earth_mag: Vector3<f64>) -> Self {
        Self {
            accel_noise_sigma,
            accel_offset,
            gyro_noise_sigma,
            gyro_drift,
            mag_noise_sigma,
            mag_offset,
            earth_mag,
        }
    }

    pub fn update(&mut self, accel: Vector3<f64>, ang_vel: Vector3<f64>, attitude: UnitQuaternion<f64>) -> IMUReading {
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

        IMUReading {
            accel: measured_accel,
            gyro: measured_gyro,
            mag: measured_mag,
        }
    }
}


#[derive(Debug, Clone)]
pub struct GPS {
    pub pos_noise_sigma: Vector3<f64>,
    pub pos_offset: Vector3<f64>,
}

#[derive(Debug, Clone)]
pub struct GPSReading {
    pub position: Vector3<f64>,
}

impl GPS {
    pub fn new(pos_noise_sigma: Vector3<f64>, pos_offset: Vector3<f64>) -> Self {
        Self {
            pos_noise_sigma,
            pos_offset,
        }
    }

    pub fn update(&mut self, position: Vector3<f64>) -> GPSReading {
        let mut rng = rand::rng();

        let noisy_position = position + self.pos_offset + noise(&self.pos_noise_sigma, &mut rng);

        GPSReading {
            position: noisy_position,
        }
    }
}


#[derive(Debug, Clone)]
pub struct UWB {
    pub pos_noise_sigma: Vector3<f64>,
    pub pos_offset: Vector3<f64>,
    pub origin: Vector3<f64>,
    pub range: f64,
    last_valid_position: Vector3<f64>,
}

#[derive(Debug, Clone)]
pub struct UWBReading {
    pub position: Vector3<f64>,
}

impl UWB {
    pub fn new(pos_noise_sigma: Vector3<f64>, pos_offset: Vector3<f64>, origin: Vector3<f64>, range: f64) -> Self {
        Self {
            pos_noise_sigma,
            pos_offset,
            origin,
            range,
            last_valid_position: origin.clone(),
        }
    }

    pub fn update(&mut self, position: Vector3<f64>) -> UWBReading {
        if (position - self.origin).norm() > self.range {
            // Out of range behavior simply returns the last valid read. Check with Avionics for actual behavior
            UWBReading {
                position: self.last_valid_position,
            }
        } else {
            let mut rng = rand::rng();

            let noisy_position = position + self.pos_offset + noise(&self.pos_noise_sigma, &mut rng);

            UWBReading {
                position: noisy_position,
            }
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
}

impl TVCActuator {
    pub fn new (start_position: f64, extension_limit: f64, unloaded_speed: f64, stall_force: f64, p_gain: f64) -> Self {
        Self {
            position: start_position,
            velocity: 0.0,
            extension_limit,
            unloaded_speed,
            stall_force,
            p_gain,
        }
    }

    pub fn update(&mut self, target_position: f64, load_force: f64, dt: f64) {
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
pub struct TVC {
    pub x_actuator: TVCActuator,
    pub y_actuator: TVCActuator,
    pub lever_arm: f64,
    pub max_fuel_inertia: f64,
    pub min_fuel_inertia: f64,
}

impl TVC {
    pub fn new(x_actuator: TVC_Actuator, y_actuator: TVC_Actuator, lever_arm: f64, max_fuel_inertia: f64, min_fuel_inertia: f64) -> Self {
        Self {
            x_actuator,
            y_actuator,
            max_fuel_inertia,
            min_fuel_inertia,
        }
    }

    // percent fuel usage should be a decimal between 0 and 1
    pub fn update(&mut self, command: Vector2<f64>, percent_fuel_usage: f64, dt: f64) {
        let engine_inertia = self.min_fuel_inertia + (self.max_fuel_inertia - self.min_fuel_inertia) * percent_fuel_usage;
        let x_load = (engine_inertia * self.x_actuator.get_accel()) / self.lever_arm.powi(2);
        let y_load = (engine_inertia * self.y_actuator.get_accel()) / self.lever_arm.powi(2);

        // TODO: translate gimbal angle commands to actuator target positions
        let x_target = 0.0;
        let y_target = 0.0;

        self.x_actuator.update(x_target, x_load, dt);
        self.y_actuator.update(y_target, y_load, dt);
    }
}

// Helper to generate 3D noise
fn noise(sigma: &Vector3<f64>, rng: &mut ThreadRng) -> Vector3<f64> {
    let n_x = Normal::new(0.0, sigma.x).unwrap().sample(rng);
    let n_y = Normal::new(0.0, sigma.y).unwrap().sample(rng);
    let n_z = Normal::new(0.0, sigma.z).unwrap().sample(rng);
    Vector3::new(n_x, n_y, n_z)
}
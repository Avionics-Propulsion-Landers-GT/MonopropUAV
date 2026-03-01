use nalgebra::Vector3;
use rand::rng;
use rand_distr::{Normal, Distribution};

// ---------------------------------------------------------------------------
// Wind Profile
// ---------------------------------------------------------------------------
// Maps altitude bands to mean wind vectors in the world frame.
// Between bands we just linearly interpolate, same as interp1 in fluid_dynamics.

#[derive(Debug, Clone)]
pub struct WindBand {
    pub altitude_m: f64,
    pub wind_mps: Vector3<f64>, // mean wind [m/s] in world frame (x=east, y=north, z=up)
}

#[derive(Debug, Clone)]
pub struct WindProfile {
    pub bands: Vec<WindBand>,
}

impl WindProfile {
    pub fn new(bands: Vec<WindBand>) -> Self {
        // Sort ascending by altitude so interpolation always works
        let mut bands = bands;
        bands.sort_by(|a, b| a.altitude_m.partial_cmp(&b.altitude_m).unwrap());
        Self { bands }
    }

    // Returns the mean wind at the given altitude by linearly interpolating between bands.
    // Below the lowest band we hold the bottom value, above the highest we hold the top.
    pub fn sample(&self, altitude_m: f64) -> Vector3<f64> {
        if self.bands.is_empty() {
            return Vector3::zeros();
        }
        let n = self.bands.len();

        // Clamp below
        if altitude_m <= self.bands[0].altitude_m {
            return self.bands[0].wind_mps;
        }
        // Clamp above
        if altitude_m >= self.bands[n - 1].altitude_m {
            return self.bands[n - 1].wind_mps;
        }

        // Find the bracket and interpolate
        for i in 0..n - 1 {
            let lo = &self.bands[i];
            let hi = &self.bands[i + 1];
            if altitude_m >= lo.altitude_m && altitude_m <= hi.altitude_m {
                let t = (altitude_m - lo.altitude_m) / (hi.altitude_m - lo.altitude_m);
                return lo.wind_mps + t * (hi.wind_mps - lo.wind_mps);
            }
        }

        // Shouldn't reach here, but be safe
        Vector3::zeros()
    }
}

// ---------------------------------------------------------------------------
// Gust Model
// ---------------------------------------------------------------------------
// Models stochastic wind turbulence using a first-order Gauss-Markov process
// on each axis independently.
//
// Physical intuition: real wind gusts are correlated in time — they don't
// change instantaneously.  tau_s controls how long a gust "remembers" its
// previous state.  sigma_mps controls how intense the turbulence is.
//
// The update equation is:
//   z[k+1] = exp(-dt/tau) * z[k] + sigma * sqrt(1 - exp(-2*dt/tau)) * N(0,1)
//
// This is exactly the discrete-time solution to the Ornstein-Uhlenbeck SDE,
// which has a well-understood white-noise power spectral density shaped by
// the 1st-order low-pass Lorentzian — a good match to measured gust spectra.
//
// Gust factor: In wind engineering, the "gust factor" G = V_peak / V_mean.
// We model peak gusts as a separate one-shot impulse that decays over
// gust_duration_s, triggered at a user-specified time.  This separately
// captures that deterministic "worst-case" scenario.

#[derive(Debug, Clone)]
pub struct GustModel {
    // Turbulence intensity per axis [m/s]
    pub sigma_mps: Vector3<f64>,
    // Correlation time [s] — larger = smoother gusts, smaller = choppier
    pub tau_s: f64,

    // One-shot peak gust: fires at gust_start_s, lasts gust_duration_s
    pub gust_magnitude_mps: Vector3<f64>,
    pub gust_start_s: f64,
    pub gust_duration_s: f64,

    // Internal Gauss-Markov state
    state: Vector3<f64>,
}

impl GustModel {
    pub fn new(
        sigma_mps: Vector3<f64>,
        tau_s: f64,
        gust_magnitude_mps: Vector3<f64>,
        gust_start_s: f64,
        gust_duration_s: f64,
    ) -> Self {
        Self {
            sigma_mps,
            tau_s,
            gust_magnitude_mps,
            gust_start_s,
            gust_duration_s,
            state: Vector3::zeros(),
        }
    }

    // Steps the gust model and returns the total gust velocity [m/s] in world frame.
    pub fn step(&mut self, dt: f64, system_time: f64) -> Vector3<f64> {
        let mut rng = rng();

        // Gauss-Markov update: exact discrete-time solution, no Euler approximation needed
        let decay = (-dt / self.tau_s).exp();
        let noise_scale = (1.0 - (-2.0 * dt / self.tau_s).exp()).max(0.0).sqrt();

        let noise = Vector3::new(
            Normal::new(0.0, self.sigma_mps.x).unwrap().sample(&mut rng),
            Normal::new(0.0, self.sigma_mps.y).unwrap().sample(&mut rng),
            Normal::new(0.0, self.sigma_mps.z).unwrap().sample(&mut rng),
        );

        self.state = decay * self.state + noise_scale * noise;

        // One-shot peak gust: ramp up instantly, decay linearly over the duration.
        // This represents the "gust factor" peak — a brief, deterministic worst case
        // layered on top of the statistical background turbulence.
        let peak_gust = if system_time >= self.gust_start_s
            && system_time < self.gust_start_s + self.gust_duration_s
        {
            let progress = (system_time - self.gust_start_s) / self.gust_duration_s;
            // Linear decay: full magnitude at start, zero at end
            self.gust_magnitude_mps * (1.0 - progress)
        } else {
            Vector3::zeros()
        };

        self.state + peak_gust
    }
}

// ---------------------------------------------------------------------------
// Wind Model (top-level)
// ---------------------------------------------------------------------------
// Combines the steady profile and the gust model into a single output.
// Call step() every simulation tick to get the total wind velocity [m/s]
// in the world frame.

#[derive(Debug, Clone)]
pub struct WindModel {
    pub profile: WindProfile,
    pub gusts: GustModel,
}

impl WindModel {
    pub fn new(profile: WindProfile, gusts: GustModel) -> Self {
        Self { profile, gusts }
    }

    // Returns the total wind velocity vector in world frame [m/s] at the
    // given altitude.  The rocket dynamics should subtract this from the
    // rocket velocity to get the velocity relative to the air mass.
    pub fn step(&mut self, altitude_m: f64, dt: f64, system_time: f64) -> Vector3<f64> {
        let mean = self.profile.sample(altitude_m);
        let gust = self.gusts.step(dt, system_time);
        mean + gust
    }
}

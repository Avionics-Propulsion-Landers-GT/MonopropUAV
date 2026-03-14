use serde::Deserialize;
use std::error::Error;

// One row from the CSV. All angles in degrees, distances in meters.
// We only track cp_z_from_nose because this is an axisymmetric rocket —
// the lateral CP components are zero by geometry.
#[derive(Debug, Clone, Deserialize)]
pub struct AeroRecord {
    pub alpha_deg:      f64,  // angle of attack [deg]
    pub mach:           f64,  // freestream Mach number [-]
    pub cd:             f64,  // drag coefficient [-]
    pub area_ref:       f64,  // reference area [m^2]
    pub cp_z_from_nose: f64,  // CP offset from nose tip, along body +Z axis [m]
                               // (body Z runs nose-to-tail; positive = aft of nose)
}

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

// ─── AeroTable ───────────────────────────────────────────────────────────────

/// Holds the full aero dataset as a structured 2-D grid indexed by
/// [alpha_deg, mach]. Once built it's read-only, so Clone is cheap to derive.
#[derive(Debug, Clone)]
pub struct AeroTable {
    /// Sorted, unique AoA breakpoints [deg]
    pub alphas: Vec<f64>,
    /// Sorted, unique Mach breakpoints [-]
    pub machs: Vec<f64>,
    /// grid[i_alpha][i_mach] — the raw record at each (alpha, mach) corner
    grid: Vec<Vec<AeroRecord>>,
}

impl AeroTable {
    /// Load a CSV with columns: alpha_deg, mach, cd, area_ref, cp_z_from_nose.
    /// Rows can be in any order; we sort and grid them here.
    /// Returns Err if the file is missing or malformed — caller decides the fallback.
    pub fn from_csv(path: &str) -> Result<Self, Box<dyn Error>> {
        let mut rdr = csv::Reader::from_path(path)?;

        // Collect all rows first — we need to sort before building the grid.
        let mut records: Vec<AeroRecord> = rdr
            .deserialize()
            .collect::<Result<Vec<AeroRecord>, _>>()?;

        // Sort by alpha first, then Mach.  This means rows with the same alpha
        // end up contiguous, which is exactly the order we want to slice into columns.
        records.sort_by(|a, b| {
            a.alpha_deg
                .partial_cmp(&b.alpha_deg)
                .unwrap()
                .then(a.mach.partial_cmp(&b.mach).unwrap())
        });

        // Extract unique breakpoints (Vec is already sorted, just dedup).
        let mut alphas: Vec<f64> = records.iter().map(|r| r.alpha_deg).collect();
        alphas.dedup_by(|a, b| (*a - *b).abs() < 1e-9);

        let mut machs: Vec<f64> = records.iter().map(|r| r.mach).collect();
        machs.dedup_by(|a, b| (*a - *b).abs() < 1e-9);

        let n_alpha = alphas.len();
        let n_mach  = machs.len();

        if records.len() != n_alpha * n_mach {
            return Err(format!(
                "aero_table.csv: expected {} rows ({}α × {}M), got {}",
                n_alpha * n_mach, n_alpha, n_mach, records.len()
            ).into());
        }

        // Build the 2-D grid. Records sorted alpha-major means we can chunk them
        // directly: records[i_alpha*n_mach .. (i_alpha+1)*n_mach] = one alpha slice.
        let grid: Vec<Vec<AeroRecord>> = records
            .chunks(n_mach)
            .map(|chunk| chunk.to_vec())
            .collect();

        Ok(Self { alphas, machs, grid })
    }
}

// ─── Bilinear Interpolation ───────────────────────────────────────────────────

/// Interpolate between four corner AeroRecords using normalised coordinates
/// t_alpha ∈ [0,1] and t_mach ∈ [0,1], where 0 = low corner, 1 = high corner.
///
/// Standard bilinear weights:
///   (1-ta)(1-tm)·Q00 + ta(1-tm)·Q10 + (1-ta)tm·Q01 + ta·tm·Q11
///
/// Only the three physical scalars are mixed; alpha_deg and mach on the
/// returned record are set to the queried (clamped) values.
fn bilinear_interp(
    q00: &AeroRecord, q10: &AeroRecord,  // low-mach:  (low-alpha, high-alpha)
    q01: &AeroRecord, q11: &AeroRecord,  // high-mach: (low-alpha, high-alpha)
    t_alpha: f64,
    t_mach:  f64,
    alpha_query: f64,
    mach_query:  f64,
) -> AeroRecord {
    let w00 = (1.0 - t_alpha) * (1.0 - t_mach);
    let w10 =        t_alpha  * (1.0 - t_mach);
    let w01 = (1.0 - t_alpha) *        t_mach;
    let w11 =        t_alpha  *        t_mach;

    AeroRecord {
        alpha_deg:      alpha_query,
        mach:           mach_query,
        cd:             w00*q00.cd             + w10*q10.cd             + w01*q01.cd             + w11*q11.cd,
        area_ref:       w00*q00.area_ref       + w10*q10.area_ref       + w01*q01.area_ref       + w11*q11.area_ref,
        cp_z_from_nose: w00*q00.cp_z_from_nose + w10*q10.cp_z_from_nose + w01*q01.cp_z_from_nose + w11*q11.cp_z_from_nose,
    }
}

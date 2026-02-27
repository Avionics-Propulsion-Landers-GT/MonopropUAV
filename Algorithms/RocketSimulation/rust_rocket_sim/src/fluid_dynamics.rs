use std::f64::consts::PI;
use std::error::Error;
use std::fs::File;
use std::io::{BufRead, BufReader};

/// Struct to neatly package the 8 outputs from the solver
#[derive(Debug, Clone, Copy)]
pub struct FlowrateSolution {
    pub thrust_realized: f64,
    pub new_port_d: f64,
    pub fuel_mass: f64,
    pub mdot_ox: f64,
    pub pc_bar: f64,
    pub of_ratio_realized: f64,
    pub isp_realized: f64,
    pub cstar_realized: f64,
}

pub fn flowrate_solver(
    thrust_commanded: f64,
    a: f64,
    n: f64,
    port_d: f64,
    dt: f64,
) -> FlowrateSolution {
    // Fuel and engine parameters
    let fuel_length: f64 = 0.1559; // m
    let fuel_od: f64 = 0.177; // m
    let rho_f: f64 = 856.0; // kg/m^3
    let throat_diameter: f64 = 14.56 / 1000.0;
    let a_throat: f64 = PI * (throat_diameter / 2.0).powi(2); // Throat Area in m^2
    let cd_throat: f64 = 0.95; // Throat discharge coefficient

    let num_steps = 560; // 0.001 to 0.560 kg/s
    
    // Track the closest match to the commanded thrust
    let mut best_diff = f64::INFINITY;
    let mut best_solution = FlowrateSolution {
        thrust_realized: 0.0,
        new_port_d: port_d,
        fuel_mass: 0.0,
        mdot_ox: 0.0,
        pc_bar: 0.0,
        of_ratio_realized: 0.0,
        isp_realized: 0.0,
        cstar_realized: 0.0,
    };

    // Equivalent to mdot_ox_arr = [0.001:0.001:0.56]
    for i in 1..=num_steps {
        let mdot_ox = (i as f64) * 0.001;

        // 1. Solve for total port change (Dp_t)
        let power = 2.0 * n + 1.0;
        let term1 = port_d.powf(power);
        let term2 = power * 2.0_f64.powf(power) * a * mdot_ox.powf(n) * dt;
        let term3 = PI.powf(n);
        let dp_t = (term1 + (term2 / term3)).powf(1.0 / power);

        // 2. Solve for Oxidizer Mass Flux (Gox)
        let gox = (4.0 * mdot_ox) / (PI * dp_t.powi(2));

        // 3. Solve for Regression Rate
        let r_dot_t = a * gox.powf(n);

        // 4. Solve for Fuel Mass Flow Rate
        let mdot_f = rho_f * PI * dp_t * fuel_length * r_dot_t;

        // 5. O/F ratio and Total Mass Flow
        let of_rat = mdot_ox / mdot_f;
        let mdot_total = mdot_ox + mdot_f;

        // 6. Isp and C* Polynomials
        let isp = -0.009 * of_rat.powi(6)
            + 0.4167 * of_rat.powi(5)
            - 7.3421 * of_rat.powi(4)
            + 60.909 * of_rat.powi(3)
            - 250.95 * of_rat.powi(2)
            + 595.11 * of_rat
            + 1040.6; // m/s

        let cstar = -0.0059 * of_rat.powi(6)
            + 0.2715 * of_rat.powi(5)
            - 4.7368 * of_rat.powi(4)
            + 38.88 * of_rat.powi(3)
            - 159.09 * of_rat.powi(2)
            + 383.09 * of_rat
            + 693.7; // m/s

        // 7. Calculate thrust
        let thrust = isp * mdot_total;

        // 8. Find thrust closest to desired thrust
        let diff = (thrust - thrust_commanded).abs();
        
        if diff < best_diff {
            best_diff = diff;
            
            // Calculate final geometry constraints for this specific flow rate
            let fuelm = rho_f * (PI / 4.0) * (fuel_od.powi(2) - dp_t.powi(2)) * fuel_length;
            let pc = (mdot_total * cstar * 0.85 / (a_throat * cd_throat)) / 100_000.0;

            // Save this as the current winning solution
            best_solution = FlowrateSolution {
                thrust_realized: thrust,
                new_port_d: dp_t,
                fuel_mass: fuelm,
                mdot_ox,
                pc_bar: pc,
                of_ratio_realized: of_rat,
                isp_realized: isp,
                cstar_realized: cstar,
            };
        }
    }

    // Return the outputs for the closest matched thrust
    best_solution
}

/// Holds the Isobaric Fluid Data (State 1 Tank Properties)
#[derive(Clone, Debug, Default)]
pub struct IsoData {
    pub t_iso: Vec<f64>,       // [K]
    pub rho_iso: Vec<f64>,     // [kg/m^3]  <-- ADD THIS
    pub h_l_iso_kj: Vec<f64>,  // [kJ/kg]
}

impl IsoData {
    pub fn new(file_name : &str) -> Self {
        if file_name == "isobaric_liquid_nitrous_oxide.csv" {
            return Self::parse_csv(include_str!("isobaric_liquid_nitrous_oxide.csv"));
        } else {
            return Self::parse_csv(include_str!("isobaric_nitrogen.csv"));
        }
    }

    pub fn parse_csv(csv_content: &str) -> Self {
        // Bakes the CSV directly into the executable at compile time
        // (Adjust path to "../N2O_NIST_sat_table.csv" if the file is in your root folder)        
        let mut data = IsoData::default();
        let mut lines = csv_content.lines();

        // 1. Skip the header row
        lines.next();

        // 2. Parse the data rows
        for line in lines {
            let text: &str = line;
            if text.trim().is_empty() {
                continue;
            }

            let cols: Vec<&str> = text.split(',').collect();

            // 0: Temperature, 1: Pressure, 2: Density, ..., 5: Enthalpy
            if cols.len() >= 6 {
                data.t_iso.push(cols[0].trim().parse::<f64>().expect("Failed to parse t_iso"));
                data.rho_iso.push(cols[2].trim().parse::<f64>().expect("Failed to parse rho_iso"));
                data.h_l_iso_kj.push(cols[5].trim().parse::<f64>().expect("Failed to parse h_l_iso_kj"));
            }
        }
        
        data
    }
}

/// Holds the Saturated Fluid Data (Expansion/Flashing Properties)
#[derive(Clone, Debug, Default)]
pub struct NistData {
    pub t_sat: Vec<f64>,       // [K]
    pub p_sat_mpa: Vec<f64>,   // [MPa]
    pub rho_l: Vec<f64>,       // [kg/m^3]
    pub rho_v: Vec<f64>,       // [kg/m^3]
    pub h_l_kj: Vec<f64>,      // [kJ/kg]
    pub h_v_kj: Vec<f64>,      // [kJ/kg]
}

impl NistData {
    pub fn new() -> Self {
        Self::parse_csv("N2O_NIST_sat_table.csv")
    }

    pub fn parse_csv(csv_content: &str) -> Self {
        // Bakes the CSV directly into the executable at compile time
        // (Adjust path to "../N2O_NIST_sat_table.csv" if the file is in your root folder)        
        let mut data = NistData::default();
        let mut lines = csv_content.lines();

        // 1. Skip the header row
        lines.next();

        // 2. Parse the data rows
        for line in lines {
            let text: &str = line;
            if text.trim().is_empty() {
                continue;
            }

            let cols: Vec<&str> = text.split(',').collect();

            // 0: Temperature_K, 1: Pressure_MPa, 2: rho_L, 3: h_L, 4: s_L, 5: rho_V, 6: h_V
            if cols.len() >= 7 {
                data.t_sat.push(cols[0].trim().parse::<f64>().expect("Failed to parse t_sat"));
                data.p_sat_mpa.push(cols[1].trim().parse::<f64>().expect("Failed to parse p_sat_mpa"));
                data.rho_l.push(cols[2].trim().parse::<f64>().expect("Failed to parse rho_l"));
                data.h_l_kj.push(cols[3].trim().parse::<f64>().expect("Failed to parse h_l_kj"));
                data.rho_v.push(cols[5].trim().parse::<f64>().expect("Failed to parse rho_v"));
                data.h_v_kj.push(cols[6].trim().parse::<f64>().expect("Failed to parse h_v_kj"));
            }
        }
        
        data
    }
}

/// The output of the Angle Solver
#[derive(Debug, Clone, Copy)]
pub struct AngleSolution {
    pub theta_deg: f64,
    pub a_ev: f64,
}

/// Fast 1D Linear Interpolator (equivalent to MATLAB's `interp1` with 'extrap')
pub fn interp1(x: &[f64], y: &[f64], x_query: f64) -> f64 {
    let n = x.len();

    // --- THE FIX: Guard against empty or 1-element arrays ---
    if n == 0 {
        return 0.0; // Return a default value if the array is empty
    }
    if n == 1 {
        return y[0]; // If there's only one data point, return it
    }
    // --------------------------------------------------------

    if x_query <= x[0] {
        // Extrapolate left
        let slope = (y[1] - y[0]) / (x[1] - x[0]);
        return y[0] + slope * (x_query - x[0]);
    }
    if x_query >= x[n - 1] {
        // Extrapolate right
        let slope = (y[n - 1] - y[n - 2]) / (x[n - 1] - x[n - 2]);
        return y[n - 1] + slope * (x_query - x[n - 1]);
    }
    // Interpolate
    for i in 0..n - 1 {
        if x_query >= x[i] && x_query <= x[i + 1] {
            let slope = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
            return y[i] + slope * (x_query - x[i]);
        }
    }
    0.0
}

/// The Dyer Model (Computes mass flow rate through an orifice)
fn dyer_model(
    p_in: f64, p_out: f64, h_in: f64, p_sat_in: f64, 
    area: f64, cd: f64, nist: &NistData
) -> f64 {
    if p_in <= p_out {
        return 0.0;
    }

    // Convert NIST MPa array to Pa on the fly for lookups
    let p_sat_pa: Vec<f64> = nist.p_sat_mpa.iter().map(|&p| p * 1e6).collect();
    let h_l_j: Vec<f64> = nist.h_l_kj.iter().map(|&h| h * 1000.0).collect();
    let h_v_j: Vec<f64> = nist.h_v_kj.iter().map(|&h| h * 1000.0).collect();

    let hl_in = interp1(&p_sat_pa, &h_l_j, p_in);
    let hv_in = interp1(&p_sat_pa, &h_v_j, p_in);
    let hl_out = interp1(&p_sat_pa, &h_l_j, p_out);
    let hv_out = interp1(&p_sat_pa, &h_v_j, p_out);
    let rhol_in = interp1(&p_sat_pa, &nist.rho_l, p_in);
    let rhol_out = interp1(&p_sat_pa, &nist.rho_l, p_out);
    let rhov_out = interp1(&p_sat_pa, &nist.rho_v, p_out);

    let denom_in = hv_in - hl_in;
    let x_in = if denom_in.abs() < 1e-9 { 0.0 } else { ((h_in - hl_in) / denom_in).clamp(0.0, 1.0) };

    let m_dot_spi = cd * area * (2.0 * rhol_in * (p_in - p_out)).sqrt();

    let denom_out = hv_out - hl_out;
    let x_out = if denom_out.abs() < 1e-9 { 0.0 } else { ((h_in - hl_out) / denom_out).clamp(0.0, 1.0) };

    let rho_mix_out = 1.0 / ((x_out / rhov_out) + ((1.0 - x_out) / rhol_out));
    let h_mix_out = hl_out + x_out * (hv_out - hl_out);

    let m_dot_hem = if h_in <= h_mix_out {
        0.0
    } else {
        cd * area * rho_mix_out * (2.0 * (h_in - h_mix_out)).sqrt()
    };

    let k = if x_in > 0.0 {
        0.0
    } else {
        let delta_p_subcool = p_in - p_out;
        if delta_p_subcool <= 1e-5 {
            0.0
        } else {
            (delta_p_subcool / (p_sat_in - p_out).max(1e-6)).sqrt()
        }
    };

    (1.0 - 1.0 / (1.0 + k)) * m_dot_spi + (1.0 / (1.0 + k)) * m_dot_hem
}

/// The Main Angle Solver
pub fn angle_solver(
    m_dot_target: f64,
    pc_bar: f64,
    iso: &IsoData,
    nist: &NistData,
) -> AngleSolution {
    // 1. SYSTEM INPUTS
    let p1_mpa: f64 = 6.5;
    let t1_k: f64 = 294.26;
    let p_ch_mpa: f64 = pc_bar / 10.0;
    let a_inj: f64 = 3.9528e-5;
    let cd_inj: f64 = 0.8;
    let cd_valve: f64 = 0.8;

    let p1: f64 = p1_mpa * 1e6;
    let p0: f64 = p_ch_mpa * 1e6;

    // 2. NIST DATA & INTERPOLATION
    let h_l_iso_j: Vec<f64> = iso.h_l_iso_kj.iter().map(|&h| h * 1000.0).collect();
    let p_sat_pa: Vec<f64> = nist.p_sat_mpa.iter().map(|&p| p * 1e6).collect();

    let h1: f64 = interp1(&iso.t_iso, &h_l_iso_j, t1_k);
    let p_sat_at_t1: f64 = interp1(&nist.t_sat, &p_sat_pa, t1_k);

    let pmin_sat: f64 = p_sat_pa.first().cloned().unwrap_or(0.0);
    let pmax_sat: f64 = p_sat_pa.last().cloned().unwrap_or(f64::MAX);

    // Safety Bounds
    let p_lo_lim: f64 = (p0 * 1.001).max(pmin_sat * 1.001);
    let p_hi_lim: f64 = (p1 * 0.999).min(pmax_sat * 0.999);

    if p_lo_lim >= p_hi_lim {
        panic!("Valid pressure range collapsed. Check P0/P1 vs saturation table bounds.");
    }

    // 3. PART A: Bracket search for a sign change (MATLAB's 60-point grid)
    let num_points = 60;
    let step = (p_hi_lim - p_lo_lim) / (num_points - 1) as f64;
    
    let mut p_br_lo = 0.0;
    let mut p_br_hi = 0.0;
    let mut found = false;
    let mut prev_f = dyer_model(p_lo_lim, p0, h1, p_sat_at_t1, a_inj, cd_inj, nist) - m_dot_target;

    for i in 1..num_points {
        let p_guess = p_lo_lim + (i as f64) * step;
        let f_val = dyer_model(p_guess, p0, h1, p_sat_at_t1, a_inj, cd_inj, nist) - m_dot_target;

        if prev_f.signum() != f_val.signum() {
            p_br_lo = p_guess - step; // The previous guess
            p_br_hi = p_guess;        // The current guess
            found = true;
            break;
        }
        prev_f = f_val;
    }

    if !found {
        panic!("No sign change found for f(P)=mdot-mdot_target. Requested mdot is likely impossible.");
    }

    // 4. BISECTION ROOT FINDER (Replaces MATLAB's fzero)
    let mut p2_solution = 0.0;
    for _ in 0..50 { // 50 iterations guarantees extreme sub-pascal precision
        p2_solution = (p_br_lo + p_br_hi) / 2.0;
        let f_mid = dyer_model(p2_solution, p0, h1, p_sat_at_t1, a_inj, cd_inj, nist) - m_dot_target;

        if f_mid.abs() < 1e-9 { break; }

        let f_lo = dyer_model(p_br_lo, p0, h1, p_sat_at_t1, a_inj, cd_inj, nist) - m_dot_target;
        if f_mid.signum() == f_lo.signum() {
            p_br_lo = p2_solution;
        } else {
            p_br_hi = p2_solution;
        }
    }

    // 5. PART B: Size the valve area A_ev
    let unit_flow_rate = dyer_model(p1, p2_solution, h1, p_sat_at_t1, 1.0, 1.0, nist);

    if unit_flow_rate <= 0.0 {
        panic!("Unit_Flow_Rate <= 0. Valve sizing cannot proceed.");
    }

    let a_ev = m_dot_target / (cd_valve * unit_flow_rate);

    // 6. COUPLING: Convert A_ev to valve mechanical angle
    let theta = -1e14 * a_ev.powi(3) + 1e10 * a_ev.powi(2) + 959373.0 * a_ev - 1.4629;

    AngleSolution {
        theta_deg: theta,
        a_ev,
    }
}

#[derive(Debug, Clone, Copy)]
pub struct FluidDynamicsOutput {
    pub thrust_realized: f64,
    pub valve_angle: f64,
    pub mdot_ox: f64,
    pub new_fuel_mass: f64,
    pub new_n2o_mass: f64,
    pub new_n2o_level: f64,
    pub new_n2_mass_runtank: f64,
    pub new_n2_mass_storagetanks: f64,
    pub new_port_d: f64,
    pub of_ratio_realized: f64,
    pub isp_realized: f64,
    pub cstar_realized: f64,
    pub pc_bar: f64,
}

pub fn fluid_dynamics_update(
    thrust_commanded: f64,
    is_rcs_on: bool,
    n2o_mass: f64,
    port_d: f64,
    a: f64,
    n: f64,
    dt: f64,
    sat_n2o: &NistData,
    isobaric_n2o: &IsoData,
    runtank_vol: f64,
    tank_d: f64,
    rho_n2o: f64,
    rho_n2: f64,
    n2_mass_total: f64,
    n2_mass_flowrate_rcs: f64,
) -> FluidDynamicsOutput {
    
    // 1. Calculate the required mass flow rate and resulting change in fuel grain
    let fr_sol = flowrate_solver(thrust_commanded, a, n, port_d, dt);
    
    // 2. Calculate the MTV angle 
    let ang_sol = angle_solver(fr_sol.mdot_ox, fr_sol.pc_bar, isobaric_n2o, sat_n2o);

    // 3. Mass tracking
    let new_n2o_mass = n2o_mass - fr_sol.mdot_ox * dt;
    
    // 4. Calculate Nitrogen mass
    let new_n2_mass_runtank = (runtank_vol - new_n2o_mass * (1.0 / rho_n2o)) * rho_n2;
    let mut new_n2_mass_storagetanks = n2_mass_total - new_n2_mass_runtank;
    
    if is_rcs_on {
        new_n2_mass_storagetanks -= n2_mass_flowrate_rcs * dt;
    }

    // 5. Calculate Nitrous fluid level
    let tank_cross_sectional_area = std::f64::consts::PI * (tank_d / 2.0).powi(2);
    let new_n2o_level = (new_n2o_mass / rho_n2o) / tank_cross_sectional_area;

    // 6. Return outputs neatly packaged
    FluidDynamicsOutput {
        thrust_realized: fr_sol.thrust_realized,
        valve_angle: ang_sol.theta_deg,
        mdot_ox: fr_sol.mdot_ox,
        new_fuel_mass: fr_sol.fuel_mass,
        new_n2o_mass,
        new_n2o_level,
        new_n2_mass_runtank,
        new_n2_mass_storagetanks,
        new_port_d: fr_sol.new_port_d,
        of_ratio_realized: fr_sol.of_ratio_realized,
        isp_realized: fr_sol.isp_realized,
        cstar_realized: fr_sol.cstar_realized,
        pc_bar: fr_sol.pc_bar,
    }
}
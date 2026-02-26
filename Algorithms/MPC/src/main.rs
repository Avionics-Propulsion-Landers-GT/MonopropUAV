mod mpc_crate;
mod externaldynamics;

// use mpc_crate for MPC functions
use mpc_crate::{mpc_main, dynamics};
use externaldynamics::{real_dynamics};
use ndarray::{Array1, Array2};
use std::f64::consts::PI;

// plotting
use plotters::prelude::*;

fn main() {

   // Problem sizes
    let n = 13; // [x, y, z, qx, qy, qz, qw, x_dot, y_dot, z_dot, wx, wy, wz]
    let m = 3;  // [gimbal_theta, gimbal_phi, thrust]
    let n_steps = 10;
    // let t_total = 60.0; // 60 seconds for lander scenario
    let t_total = 20.0; // 20 seconds for tethered hover scenario
    let dt = 0.1;
    let iters = (t_total / dt) as usize;

    // Initial state: at origin, level, stationary, quaternion [0,0,0,1]
    let mut x = Array1::<f64>::zeros(n);
    x[2] = 0.0;
    x[6] = 1.0; // qw = 1 (unit quaternion)

    // Hover at set point
    let mut xref = Array1::<f64>::zeros(n);
    
    xref[6] = 1.0; // reference orientation: level (unit quaternion)

    let mut z_integral = 0.0;
    let mut y_integral = 0.0;
    let mut x_integral = 0.0;
    let ki_z = 0.0; //0.2, 0.01, 0.01
    let ki_y = 0.0;
    let ki_x = 0.0;

    // Reference trajectory
    let mut xref_traj = Array2::from_shape_fn((n_steps + 1, n), |(_, j)| xref[j]);
    let mut xref_traj_vec: Vec<Array1<f64>> = xref_traj.axis_iter(ndarray::Axis(0)).map(|row| row.to_owned()).collect();

    // Warm start: hover thrust (thrust = mass * gravity, gimbal angles = 0)
    let mut m_rocket = 80.0;
    let g = 9.81;
    let hover_thrust = m_rocket * g;
    let mut u_warm = Array2::<f64>::zeros((n_steps, m));
    for i in 0..n_steps {
        u_warm[[i, 2]] = hover_thrust; // thrust
    }

    // Costs: penalize position, orientation, velocities, angular rates
    let q_vec = vec![
        10.0, 10.0, 80.0,   // position x, y, z
        220.0, 220.0, 220.0, 220.0, // quaternion qx, qy, qz, qw
        30.0, 30.0, 10.0,        // linear velocities x_dot, y_dot, z_dot
        10.0, 10.0, 10.0          // angular velocities wx, wy, wz
    ];
    let q = Array2::<f64>::from_diag(&Array1::from(q_vec.clone()));
    let r = Array2::<f64>::from_diag(&Array1::from(vec![5.0, 5.0, 0.00]));
    // let qn = q.clone();
    let qn = Array2::<f64>::from_diag(&Array1::from(vec![
        10.0, 10.0, 80.0,   // position x, y, z
        300.0, 300.0, 300.0, 300.0, // quaternion qx, qy, qz, qw
        40.0, 40.0, 20.0,        // linear velocities x_dot, y_dot, z_dot
        5.0, 5.0, 5.0          // angular velocities wx, wy, wz
    ]));

    // Bounds on control inputs
    let gimbal_limit = 15.0 * PI / 180.0; // +/- 15 degrees
    let thrust_min = 300.0;
    let thrust_max = 1000.0;
    let u_min = Array1::from(vec![-gimbal_limit, -gimbal_limit, thrust_min]);
    let u_max = Array1::from(vec![gimbal_limit, gimbal_limit, thrust_max]);

    // Store state and control history for plotting
    let mut x_history = Vec::with_capacity(iters);
    let mut u_history: Vec<Array1<f64>> = Vec::with_capacity(iters);

    let tolerance = 1e-4;
    let lbfgs_memory = 20;
    let max_iter = 200;
    let n_dim_u = m * n_steps;
    let mut panoc_cache = optimization_engine::panoc::PANOCCache::new(n_dim_u, tolerance, lbfgs_memory);

    // smoothing weight vector (for gimbal_theta, gimbal_phi, thrust)
    let smoothing_weight = Array1::from(vec![1500.0, 1500.0, 0.02]);

    for k in 0..iters {

        if k as f64 * dt <= 10.0 {
            
            // change reference point after 5 seconds
            let f_iters = iters as f64;
            let f_k = k as f64;
            xref[0] = -0.0 * (f_k*dt)/10.0;
            xref[1] = 0.0 * (f_k*dt)/10.0;
            xref[2] = 20.0 * (f_k*dt)/10.0 + 3.0;

            xref_traj = Array2::from_shape_fn((n_steps + 1, n), |(_, j)| xref[j]);
            xref_traj_vec = xref_traj.axis_iter(ndarray::Axis(0)).map(|row| row.to_owned()).collect();
             
        } else if k as f64 * dt > 15.0 && k as f64 * dt <= 25.0 {
            
            // change reference point after 10 seconds
            let f_iters = iters as f64;
            let f_k = k as f64;
            xref[0] = 0.0 * ((f_k*dt)-15.0)/10.0;
            xref[1] = 0.0 * ((f_k*dt)-15.0)/10.0;
            xref[2] = 20.0 - 20.0 * ((f_k*dt)-15.0)/10.0;

            xref_traj = Array2::from_shape_fn((n_steps + 1, n), |(_, j)| xref[j]);
            xref_traj_vec = xref_traj.axis_iter(ndarray::Axis(0)).map(|row| row.to_owned()).collect();
        } 

        
        xref[0] = 0.0;
        xref[1] = 0.0;
        xref[2] = 25.0;

        let z_error = xref[2] - x[2];
        z_integral += z_error * dt;
        let mut x_ref_mod = xref.clone();
        x_ref_mod[2] += ki_z * z_integral; // modify z reference with integral term
        for i in 0..xref_traj_vec.len() {
            xref_traj_vec[i][2] = x_ref_mod[2];
        }
        let x_error = xref[0] - x[0];
        x_integral += x_error * dt;
        x_ref_mod[0] += ki_x * x_integral; // modify x reference with integral term
        for i in 0..xref_traj_vec.len() {
            xref_traj_vec[i][0] = x_ref_mod[0];
        }
        let y_error = xref[1] - x[1];
        y_integral += y_error * dt;
        x_ref_mod[1] += ki_y * y_integral; // modify y reference with integral term
        for i in 0..xref_traj_vec.len() {
            xref_traj_vec[i][1] = x_ref_mod[1];
        }

        // display progress
        println!("Time step: {}/{}", x_history.len() + 1, iters);
        // Solve MPC to get optimal control sequence

        // solve using mpc_main
        // let mut u_warm_vec: Vec<Array1<f64>> = u_warm.axis_iter(ndarray::Axis(0)).map(|row| row.to_owned()).collect();
        // let (u_warm_vec, x_new, mut u_apply) = mpc_main(&x, &mut u_warm_vec, &xref_traj_vec, &q, &r, &qn, &u_min, &u_max, 2, 0.05);
        
        // OR we solve using OpEn
        let (mut u_apply, u_warm) = mpc_crate::OpEnSolve(&x, &u_warm.axis_iter(ndarray::Axis(0)).map(|row| row.to_owned()).collect(), &xref_traj_vec, &q, &r, &qn, &smoothing_weight, &mut panoc_cache, m_rocket ,thrust_min, thrust_max, gimbal_limit);

        // exponential filter
        if k >= 1 {
            let alpha = 0.4; // smoothing factor
            let u_prev = u_history[k - 1].clone();
            u_apply = alpha * &u_apply + (1.0 - alpha) * &u_prev;
        } 

        u_history.push(u_apply.clone());

        m_rocket = m_rocket - 0.00 * u_apply[2] * dt; // decrease mass directly proportional to thrust
        println!("Time step: {}, Mass: {}", k as f64 * dt, m_rocket);

        // Simulate dynamics for one time step
        x = real_dynamics(&x, &u_apply, Some(m_rocket));

        // print x and u for debugging
        // println!("Time step: {}", x_history.len());
        // println!("u: {:?}", u_apply);
        // println!("x: {:?}", x);

        x_history.push(x.clone());
        
    }

    // Plot results
    let root_area = BitMapBackend::new("mpc_simulation.png", (1920, 1080)).into_drawing_area();
    root_area.fill(&WHITE).unwrap();
    {
        let mut chart = ChartBuilder::on(&root_area)
            .caption("Rocket Position", ("sans-serif", 40).into_font())
            .margin(10)
            .x_label_area_size(40)
            .y_label_area_size(80)
            .build_cartesian_2d(0f64..t_total, -30f64..30f64)
            .unwrap();

        chart.configure_mesh()
            .x_desc("Time (s)")
            .y_desc("Position (m)")
            .draw()
            .unwrap();

        let x_data: Vec<f64> = (0..iters).map(|i| i as f64 * dt).collect();
        let y_data: Vec<f64> = x_history.iter().map(|x| x[0]).collect();
        let z_data: Vec<f64> = x_history.iter().map(|x| x[1]).collect();
        let z_pos_data: Vec<f64> = x_history.iter().map(|x| x[2]).collect();

        chart.draw_series(LineSeries::new(
            x_data.iter().cloned().zip(y_data.iter().cloned()),
            RED.stroke_width(5),
        )).unwrap().label("X Position").legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], RED.stroke_width(3)));

        chart.draw_series(LineSeries::new(
            x_data.iter().cloned().zip(z_data.iter().cloned()),
            BLUE.stroke_width(5),
        )).unwrap().label("Y Position").legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLUE.stroke_width(3)));

        chart.draw_series(LineSeries::new(
            x_data.iter().cloned().zip(z_pos_data.iter().cloned()),
            GREEN.stroke_width(5),
        )).unwrap().label("Z Position").legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], GREEN.stroke_width(3)));

        chart.configure_series_labels()
            .background_style(&WHITE.mix(0.8))
            .border_style(&BLACK)
            .draw()
            .unwrap();
    }

    // Plot thrust on its own graph
    {
        let thrust_area = BitMapBackend::new("mpc_thrust.png", (960, 540)).into_drawing_area();
        thrust_area.fill(&WHITE).unwrap();
        let mut chart = ChartBuilder::on(&thrust_area)
            .caption("Thrust Input", ("sans-serif", 30).into_font())
            .margin(10)
            .x_label_area_size(40)
            .y_label_area_size(60)
            .build_cartesian_2d(0f64..t_total, thrust_min..thrust_max)
            .unwrap();
        chart.configure_mesh()
            .x_desc("Time (s)")
            .y_desc("Thrust (N)")
            .draw()
            .unwrap();
        let x_data: Vec<f64> = (0..iters).map(|i| i as f64 * dt).collect();
        let thrust_data: Vec<f64> = u_history.iter().map(|u| u[2]).collect();
        chart.draw_series(LineSeries::new(
            x_data.iter().cloned().zip(thrust_data.iter().cloned()),
            &BLACK,
        )).unwrap().label("Thrust").legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &BLACK));
        chart.configure_series_labels()
            .background_style(&WHITE.mix(0.8))
            .border_style(&BLACK)
            .draw()
            .unwrap();
    }

    // Plot gimbal angles (in degrees) on their own graph
    {
        let gimbal_area = BitMapBackend::new("mpc_gimbal.png", (960, 540)).into_drawing_area();
        gimbal_area.fill(&WHITE).unwrap();
        let mut chart = ChartBuilder::on(&gimbal_area)
            .caption("Gimbal Angles", ("sans-serif", 30).into_font())
            .margin(10)
            .x_label_area_size(40)
            .y_label_area_size(60)
            .build_cartesian_2d(0f64..t_total, -gimbal_limit.to_degrees()..gimbal_limit.to_degrees())
            .unwrap();
        chart.configure_mesh()
            .x_desc("Time (s)")
            .y_desc("Angle (deg)")
            .draw()
            .unwrap();
        let x_data: Vec<f64> = (0..iters).map(|i| i as f64 * dt).collect();
        let gimbal_theta_data: Vec<f64> = u_history.iter().map(|u| u[0].to_degrees()).collect();
        let gimbal_phi_data: Vec<f64> = u_history.iter().map(|u| u[1].to_degrees()).collect();
        chart.draw_series(LineSeries::new(
            x_data.iter().cloned().zip(gimbal_theta_data.iter().cloned()),
            &RED,
        )).unwrap().label("Gimbal Theta (deg)").legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));
        chart.draw_series(LineSeries::new(
            x_data.iter().cloned().zip(gimbal_phi_data.iter().cloned()),
            &BLUE,
        )).unwrap().label("Gimbal Phi (deg)").legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &BLUE));
        chart.configure_series_labels()
            .background_style(&WHITE.mix(0.8))
            .border_style(&BLACK)
            .draw()
            .unwrap();
    }
    println!("MPC simulation complete. Plot saved to mpc_simulation.png");
}

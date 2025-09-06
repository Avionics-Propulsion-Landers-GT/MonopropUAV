mod mpc_crate;

// use mpc_crate for MPC functions
use mpc_crate::{mpc_main, dynamics};
use ndarray::{Array1, Array2};
use std::f64::consts::PI;

// plotting
use plotters::prelude::*;

fn main() {

   // Problem sizes
    let n = 13; // [x, y, z, qx, qy, qz, qw, x_dot, y_dot, z_dot, wx, wy, wz]
    let m = 3;  // [gimbal_theta, gimbal_phi, thrust]
    let n_steps = 10;
    let t_total = 60.0;
    let dt = 0.1;
    let iters = (t_total / dt) as usize;

    // Initial state: at origin, level, stationary, quaternion [0,0,0,1]
    let mut x = Array1::<f64>::zeros(n);
    x[6] = 1.0; // qw = 1 (unit quaternion)

    // Hover at set point
    let mut xref = Array1::<f64>::zeros(n);
    xref[0] = -4.0;
    xref[1] = 4.0;
    xref[2] = 10.0;
    xref[6] = 1.0; // reference orientation: level (unit quaternion)

    // Reference trajectory
    let xref_traj = Array2::from_shape_fn((n_steps + 1, n), |(_, j)| xref[j]);
    let xref_traj_vec: Vec<Array1<f64>> = xref_traj.axis_iter(ndarray::Axis(0)).map(|row| row.to_owned()).collect();

    // Warm start: hover thrust (thrust = mass * gravity, gimbal angles = 0)
    let m_rocket = 1.0;
    let g = 9.81;
    let hover_thrust = m_rocket * g;
    let mut u_warm = Array2::<f64>::zeros((n_steps, m));
    for i in 0..n_steps {
        u_warm[[i, 2]] = hover_thrust; // thrust
    }

    // Costs: penalize position, orientation, velocities, angular rates
    let q_vec = vec![
        70.0, 70.0, 200.0,   // position x, y, z
        200.0, 200.0, 200.0, 200.0, // quaternion qx, qy, qz, qw
        50.0, 50.0, 100.0,        // linear velocities x_dot, y_dot, z_dot
        5.0, 5.0, 5.0          // angular velocities wx, wy, wz
    ];
    let q = Array2::<f64>::from_diag(&Array1::from(q_vec.clone()));
    let r = Array2::<f64>::from_diag(&Array1::from(vec![400.0, 400.0, 0.1]));
    let qn = q.clone();

    // Bounds on control inputs
    let gimbal_limit = 10.0 * PI / 180.0; // +/- 10 degrees
    let thrust_min = 0.0;
    let thrust_max = 20.0;
    let u_min = Array1::from(vec![-gimbal_limit, -gimbal_limit, thrust_min]);
    let u_max = Array1::from(vec![gimbal_limit, gimbal_limit, thrust_max]);

    // Store state and control history for plotting
    let mut x_history = Vec::with_capacity(iters);
    let mut u_history = Vec::with_capacity(iters);

    for _ in 0..iters {
        // display progress
        println!("Time step: {}/{}", x_history.len(), iters);
        // Solve MPC to get optimal control sequence
        // 3 sqp iter
        let mut u_warm_vec: Vec<Array1<f64>> = u_warm.axis_iter(ndarray::Axis(0)).map(|row| row.to_owned()).collect();
        //let (u_warm, x_new, u_apply) = mpc_main(&x, &xref_traj, &u_warm, &q, &r, &qn, &u_min, &u_max, 3, 0.1);
        let (u_warm_out, x_new, u_apply) = mpc_main(&x, &mut u_warm_vec, &xref_traj_vec, &q, &r, &qn, &u_min, &u_max, 2, 0.05);
        // Apply first control input
        u_history.push(u_apply.clone());

        // Simulate dynamics for one time step
        x = dynamics(&x, &u_apply);

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
            .build_cartesian_2d(0f64..t_total, -5f64..15f64)
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
            &RED,
        )).unwrap().label("X Position").legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));

        chart.draw_series(LineSeries::new(
            x_data.iter().cloned().zip(z_data.iter().cloned()),
            &BLUE,
        )).unwrap().label("Y Position").legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &BLUE));

        chart.draw_series(LineSeries::new(
            x_data.iter().cloned().zip(z_pos_data.iter().cloned()),
            &GREEN,
        )).unwrap().label("Z Position").legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &GREEN));

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
            &GREEN,
        )).unwrap().label("Thrust").legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &GREEN));
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
            &MAGENTA,
        )).unwrap().label("Gimbal Theta (deg)").legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &MAGENTA));
        chart.draw_series(LineSeries::new(
            x_data.iter().cloned().zip(gimbal_phi_data.iter().cloned()),
            &CYAN,
        )).unwrap().label("Gimbal Phi (deg)").legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &CYAN));
        chart.configure_series_labels()
            .background_style(&WHITE.mix(0.8))
            .border_style(&BLACK)
            .draw()
            .unwrap();
    }
    println!("MPC simulation complete. Plot saved to mpc_simulation.png");
}

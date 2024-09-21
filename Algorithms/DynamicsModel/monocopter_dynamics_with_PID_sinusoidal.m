function monocopter_dynamics_with_path_and_orientation_control()
    % Simulation parameters
    dt = 0.01;  % Time step (seconds)
    t_end = 10;  % Simulation time (seconds)
    num_steps = floor(t_end / dt);  % Number of time steps

    % Physical constants and parameters
    m = 100.0;  % Mass of the monocopter (kg)
    g = 9.81;  % Gravitational acceleration (m/s^2)
    k_D = 0.1;  % Drag coefficient
    k_wind = 0.05;  % Wind drag coefficient
    T_max = 10;  % Max thrust magnitude (N)

    % Moments of inertia (kg*m^2)
    I_x = 1.0;
    I_y = 2.0;
    I_z = 1.0;
    I = [I_x, I_y, I_z];  % Inertia tensor diagonal elements

    % Initial conditions
    r = [0; 0; 500];  % Initial position (m)
    v = [0; 0; 0];  % Initial velocity (m/s)
    q = [1; 0; 0; 0];  % Initial quaternion (neutral orientation)
    omega = [0; 0; 0];  % Initial angular velocity (rad/s)
    thrust_dir_body = [0; 0; 1];  % Thrust in the body frame (along z-axis)

    % Pre-allocate arrays to store results
    position_history = zeros(3, num_steps);
    velocity_history = zeros(3, num_steps);
    quaternion_history = zeros(4, num_steps);
    angular_velocity_history = zeros(3, num_steps);
    error_history = zeros(3, num_steps);  % Store error at each time step
    yaw_pitch_history = zeros(2, num_steps);  % Store yaw and pitch control signals

    % Path following variables
    r_circle = 5;  % Radius of circular path
    omega_path = 0.1;  % Angular velocity of circular path (rad/s)
    z_const = 2;  % Constant height along the z-axis

    % PID controller gains for position (example)
    Kp_pos = 1;  % Proportional gain
    Ki_pos = 0.1;  % Integral gain
    Kd_pos = 0.5;  % Derivative gain
    integral_error_pos = [0; 0; 0];
    prev_error_pos = [0; 0; 0];

    % PID controller gains for yaw and pitch control
    Kp_yaw = 1.5;
    Kp_pitch = 1.5;
    
    % Wind parameters (sinusoidal model)
    A_wind = 0;  % Amplitude of the wind (m/s)
    omega_wind = 0;  % Frequency of the wind oscillation (rad/s)
    phase_shift_x = 0;  % Phase shift for wind in x-direction
    phase_shift_y = 0;  % Phase shift for wind in y-direction
    phase_shift_z = 0;  % Phase shift for wind in z-direction

    % Pre-allocate arrays to store the defined path for the whole duration
    defined_path = zeros(3, num_steps);

    % Main simulation loop
    for step = 1:num_steps
        t = (step - 1) * dt;  % Current time

        % Define the desired path (circular path in the xy-plane)
        reference_x = r_circle * cos(omega_path * t);
        reference_y = r_circle * sin(omega_path * t);
        reference_z = z_const;
        reference_pos = [reference_x; reference_y; reference_z];
        
        % Store the desired path position
        defined_path(:, step) = reference_pos;

        % Calculate the position error (difference between current position and path)
        error_pos = reference_pos - r;
        error_history(:, step) = error_pos;  % Store error for analysis

        % PID control for following the path
        integral_error_pos = integral_error_pos + error_pos * dt;  % Accumulate integral error
        derivative_error_pos = (error_pos - prev_error_pos) / dt;  % Derivative of the error
        prev_error_pos = error_pos;

        % PID controller for thrust adjustment
        control_signal_pos = Kp_pos * error_pos + Ki_pos * integral_error_pos + Kd_pos * derivative_error_pos;

        % Thrust adjustment based on the control signal
        T = min(T_max, norm(control_signal_pos));  % Cap thrust at max value

        % Calculate desired yaw and pitch based on the direction towards the defined path
        desired_direction = error_pos / norm(error_pos);  % Normalize the direction

        % Calculate desired yaw and pitch (desired orientation)
        desired_yaw = atan2(desired_direction(2), desired_direction(1));  % Yaw around the z-axis
        desired_pitch = atan2(desired_direction(3), sqrt(desired_direction(1)^2 + desired_direction(2)^2));  % Pitch towards the path
        
        % Yaw and pitch control based on PID control (simplified)
        current_yaw = atan2(2 * (q(1) * q(4) + q(2) * q(3)), 1 - 2 * (q(3)^2 + q(4)^2));  % Extract current yaw from quaternion
        current_pitch = asin(2 * (q(1) * q(3) - q(4) * q(2)));  % Extract current pitch from quaternion
        
        % PID control for yaw and pitch
        yaw_control_signal = Kp_yaw * (desired_yaw - current_yaw);
        pitch_control_signal = Kp_pitch * (desired_pitch - current_pitch);

        yaw_pitch_history(:, step) = [yaw_control_signal; pitch_control_signal];  % Store yaw and pitch control signals

        % Apply yaw and pitch control to the angular velocities (for simplicity, assume direct control)
        omega(3) = yaw_control_signal;  % Yaw control affects rotation around z-axis
        omega(2) = pitch_control_signal;  % Pitch control affects rotation around y-axis

        % Sinusoidal wind velocity model
        v_wind = [
            A_wind * sin(omega_wind * t + phase_shift_x);
            A_wind * sin(omega_wind * t + phase_shift_y);
            A_wind * sin(omega_wind * t + phase_shift_z)
        ];

        % Store history
        position_history(:, step) = r;
        velocity_history(:, step) = v;
        quaternion_history(:, step) = q;
        angular_velocity_history(:, step) = omega;

        % 1. Compute rotational dynamics (Euler's equations)
        M = [0; 0; 0];  % No external torques in this case
        omega_dot = euler_equations(omega, M, I);

        % Update angular velocity
        omega = omega + omega_dot * dt;

        % 2. Compute quaternion kinematics
        q_dot = 0.5 * omega_matrix(omega) * q;

        % Update quaternion and normalize
        q = q + q_dot * dt;
        q = q / norm(q);  % Normalize to maintain unit quaternion

        % 3. Compute translational dynamics (Newton's second law)
        % Compute the rotation matrix from the quaternion
        R_b_to_w = quat_to_rotation_matrix(q);

        % Thrust in world frame
        thrust_world = R_b_to_w * thrust_dir_body * T;

        % Relative velocity between the monocopter and the wind
        v_rel = v - v_wind;

        % Wind force proportional to relative velocity squared
        F_wind = -k_wind * norm(v_rel) * v_rel;

        % Net force = thrust - gravity - drag + wind force
        gravity = [0; 0; -m * g];
        drag = -k_D * v;
        F_net = thrust_world + gravity + drag + F_wind;

        % Acceleration
        a = F_net / m;

        % Update velocity and position
        v = v + a * dt;
        r = r + v * dt;
    end

    % Plot results with arrows and error history
    plot_results_with_arrows_and_error(position_history, velocity_history, quaternion_history, angular_velocity_history, error_history, defined_path, yaw_pitch_history, dt);
end

%% Function to calculate the angular acceleration using Euler's equations
function omega_dot = euler_equations(omega, M, I)
    % Extract moments of inertia
    I_x = I(1);
    I_y = I(2);
    I_z = I(3);

    % Euler's equations for rotational motion
    omega_x = omega(1);
    omega_y = omega(2);
    omega_z = omega(3);
    
    % Angular acceleration components
    omega_dot_x = (M(1) + (I_y - I_z) * omega_y * omega_z) / I_x;
    omega_dot_y = (M(2) + (I_z - I_x) * omega_z * omega_x) / I_y;
    omega_dot_z = (M(3) + (I_x - I_y) * omega_x * omega_y) / I_z;

    % Return angular accelerations
    omega_dot = [omega_dot_x; omega_dot_y; omega_dot_z];
end

%% Function to compute the angular velocity matrix Omega for quaternion kinematics
function Omega = omega_matrix(omega)
    omega_x = omega(1);
    omega_y = omega(2);
    omega_z = omega(3);

    Omega = [0, -omega_x, -omega_y, -omega_z;
             omega_x, 0, omega_z, -omega_y;
             omega_y, -omega_z, 0, omega_x;
             omega_z, omega_y, -omega_x, 0];
end

%% Function to convert a quaternion to a rotation matrix
function R = quat_to_rotation_matrix(q)
    w = q(1);
    x = q(2);
    y = q(3);
    z = q(4);

    % Compute the rotation matrix
    R = [1 - 2*y^2 - 2*z^2, 2*x*y - 2*z*w, 2*x*z + 2*y*w;
         2*x*y + 2*z*w, 1 - 2*x^2 - 2*z^2, 2*y*z - 2*x*w;
         2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x^2 - 2*y^2];
end

%% Function to plot simulation results with arrows, error history, and yaw/pitch history
function plot_results_with_arrows_and_error(position_history, velocity_history, quaternion_history, angular_velocity_history, error_history, defined_path, yaw_pitch_history, dt)
    time = (0:size(position_history, 2) - 1) * dt;

    % 3D Trajectory plot with arrows for velocity and defined path
    figure;
    plot3(position_history(1, :), position_history(2, :), position_history(3, :), 'b-', 'LineWidth', 1.5);
    hold on;

    % Plot defined path (in green)
    plot3(defined_path(1, :), defined_path(2, :), defined_path(3, :), 'g--', 'LineWidth', 2);

    % Plot velocity vectors as arrows (downsample for better visualization)
    step_size = 50;  % Adjust for fewer arrows (change based on your needs)
    quiver3(position_history(1, 1:step_size:end), position_history(2, 1:step_size:end), ...
        position_history(3, 1:step_size:end), velocity_history(1, 1:step_size:end), ...
        velocity_history(2, 1:step_size:end), velocity_history(3, 1:step_size:end), 0.5, 'r');

    grid on;
    title('Monoprop 3D Trajectory and Defined Path');
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    zlabel('Z Position (m)');
    legend('Actual Trajectory', 'Defined Path', 'Velocity Arrows');
    view(3);  % Set 3D view for better visualization
    hold off;

    % Position, velocity, quaternion and angular velocity history
    figure;
    subplot(5, 1, 1);
    plot(time, position_history(1, :), 'r', time, position_history(2, :), 'g', time, position_history(3, :), 'b');
    title('Position (m)');
    xlabel('Time (s)');
    legend('x', 'y', 'z');

    subplot(5, 1, 2);
    plot(time, velocity_history(1, :), 'r', time, velocity_history(2, :), 'g', time, velocity_history(3, :), 'b');
    title('Velocity (m/s)');
    xlabel('Time (s)');
    legend('x', 'y', 'z');

    subplot(5, 1, 3);
    plot(time, quaternion_history(1, :), 'r', time, quaternion_history(2, :), 'g', time, quaternion_history(3, :), 'b', time, quaternion_history(4, :), 'k');
    title('Quaternion');
    xlabel('Time (s)');
    legend('w', 'x', 'y', 'z');

    subplot(5, 1, 4);
    plot(time, error_history(1, :), 'r', time, error_history(2, :), 'g', time, error_history(3, :), 'b');
    title('Position Error (m)');
    xlabel('Time (s)');
    legend('Error X', 'Error Y', 'Error Z');

    % Plot the yaw and pitch control signals over time
    subplot(5, 1, 5);
    plot(time, yaw_pitch_history(1, :), 'r', time, yaw_pitch_history(2, :), 'g');
    title('Yaw and Pitch Control Signals');
    xlabel('Time (s)');
    legend('Yaw Control', 'Pitch Control');
end

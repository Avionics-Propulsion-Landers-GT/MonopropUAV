function monocopter_dynamics_with_wind_PID()
    % Simulation parameters
    dt = 0.01;      % Time step (seconds)
    t_end = 20;     % Simulation time (seconds)
    t = 0:dt:t_end; % Time vector
    num_steps = length(t);  % Number of time steps

    % Physical constants and parameters
    m = 1.0;       % Mass of the monocopter (kg)
    g = 9.81;      % Gravitational acceleration (m/s^2)
    k_D = 0.1;     % Drag coefficient
    k_wind = 0.05; % Wind drag coefficient
    T_max = 20;    % Maximum thrust magnitude (N)
    T_min = 0;     % Minimum thrust magnitude (N)

    % Moments of inertia (kg*m^2)
    I_x = 1.0;
    I_y = 2.0;
    I_z = 1.0;
    I = [I_x, I_y, I_z];  % Inertia tensor diagonal elements

    % PID controller gains (to be tuned)
    Kp = 2.0;      % Proportional gain
    Ki = 0.0;      % Integral gain
    Kd = 1.0;      % Derivative gain

    % Desired trajectory (ascent and descent)
    z_desired = zeros(1, num_steps);
    T_half = t_end / 2;
    for i = 1:num_steps
        if t(i) <= T_half
            % Ascent phase
            z_desired(i) = (100 / T_half) * t(i);
        else
            % Descent phase
            z_desired(i) = 100 - (100 / T_half) * (t(i) - T_half);
        end
    end

    % Initial conditions
    r = [0; 0; 0];          % Initial position (m)
    v = [0; 0; 0];          % Initial velocity (m/s)
    q = [1; 0; 0; 0];       % Initial quaternion (neutral orientation)
    omega = [0; 0; 0];      % Initial angular velocity (rad/s)
    thrust_dir_body = [0; 0; 1];  % Thrust direction in the body frame

    % Initialize PID controller variables
    error_integral = 0;
    previous_error = 0;

    % Pre-allocate arrays to store results
    position_history = zeros(3, num_steps);
    velocity_history = zeros(3, num_steps);
    quaternion_history = zeros(4, num_steps);
    angular_velocity_history = zeros(3, num_steps);
    error_history = zeros(1, num_steps);

    % Initial wind velocity (random starting point)
    v_wind = [randn(); randn(); randn()] * 2;  % Random initial wind velocity in m/s

    % Main simulation loop
    for step = 1:num_steps
        % Current time
        t_current = t(step);

        % Desired position at current time
        z_target = z_desired(step);

        % Compute position error
        error = z_target - r(3);
        error_history(step) = error;

        % Update integral and derivative of error
        error_integral = error_integral + error * dt;
        error_derivative = (error - previous_error) / dt;

        % PID controller output (desired acceleration in Z)
        u = Kp * error + Ki * error_integral + Kd * error_derivative;

        % Update previous error
        previous_error = error;

        % Compute required thrust in Z-direction
        % Total thrust needed to achieve desired acceleration
        F_thrust_z = m * (u + g);  % Add gravity to account for weight

        % Limit thrust to physical capabilities
        F_thrust_z = min(max(F_thrust_z, T_min), T_max);

        % Update thrust vector in body frame (assuming vertical orientation)
        thrust_body = thrust_dir_body * F_thrust_z;

        % Compute the rotation matrix from the quaternion
        R_b_to_w = quat_to_rotation_matrix(q);

        % Thrust in world frame
        thrust_world = R_b_to_w * thrust_body;

        % Random wind velocity (updates over time with noise)
        v_wind = v_wind + 0.1 * randn(3,1);  % Smooth random walk for wind changes

        % Relative velocity between the monocopter and the wind
        v_rel = v - v_wind;

        % Wind force proportional to relative velocity squared
        F_wind = -k_wind * norm(v_rel) * v_rel;

        % Net force = thrust + wind force - gravity - drag
        gravity = [0; 0; -m * g];
        drag = -k_D * v;
        F_net = thrust_world + F_wind + drag + gravity;

        % Acceleration
        a = F_net / m;

        % Update velocity and position
        v = v + a * dt;
        r = r + v * dt;

        % Store history
        position_history(:, step) = r;
        velocity_history(:, step) = v;
        quaternion_history(:, step) = q;
        angular_velocity_history(:, step) = omega;

        % Rotational dynamics (can be extended as needed)
        % For simplicity, assuming no rotation change (omega_dot = 0)
        omega_dot = [0; 0; 0];  % Placeholder for rotational dynamics

        % Update angular velocity
        omega = omega + omega_dot * dt;

        % Compute quaternion kinematics (assuming small rotations)
        q_dot = 0.5 * omega_matrix(omega) * q;

        % Update quaternion and normalize
        q = q + q_dot * dt;
        q = q / norm(q);  % Normalize to maintain unit quaternion
    end

    % Plot results
    plot_results(position_history, velocity_history, quaternion_history, angular_velocity_history, error_history, dt, t, z_desired);
end

%% Function to calculate the angular acceleration using Euler's equations
function omega_dot = euler_equations(omega, M, I)
    % Extract moments of inertia
    I_x = I(1);
    I_y = I(2);
    I_z = I(3);

    % Euler's equations
    omega_x = omega(1);
    omega_y = omega(2);
    omega_z = omega(3);
    omega_dot_x = (M(1) + (I_y - I_z) * omega_y * omega_z) / I_x;
    omega_dot_y = (M(2) + (I_z - I_x) * omega_z * omega_x) / I_y;
    omega_dot_z = (M(3) + (I_x - I_y) * omega_x * omega_y) / I_z;

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
    R = [1 - 2*y^2 - 2*z^2,   2*x*y - 2*z*w,       2*x*z + 2*y*w;
         2*x*y + 2*z*w,       1 - 2*x^2 - 2*z^2,   2*y*z - 2*x*w;
         2*x*z - 2*y*w,       2*y*z + 2*x*w,       1 - 2*x^2 - 2*y^2];
end

%% Function to plot simulation results
function plot_results(position_history, velocity_history, quaternion_history, angular_velocity_history, error_history, dt, t, z_desired)
    time = t;

    % 3D Trajectory plot
    figure;
    plot3(position_history(1, :), position_history(2, :), position_history(3, :), 'b-', 'LineWidth', 1.5);
    hold on;
    plot3(zeros(size(z_desired)), zeros(size(z_desired)), z_desired, 'r--', 'LineWidth', 1.5);
    grid on;
    title('Monocopter 3D Trajectory');
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    zlabel('Z Position (m)');
    legend('Actual Path', 'Desired Path');
    view(3);  % Set 3D view for better visualization

    % Position over time
    figure;
    subplot(3, 1, 1);
    plot(time, position_history(1, :), 'r', time, position_history(2, :), 'g', time, position_history(3, :), 'b', time, z_desired, 'k--');
    title('Position (m)');
    xlabel('Time (s)');
    ylabel('Position (m)');
    legend('x', 'y', 'z', 'z_{desired}');

    % Velocity over time
    subplot(3, 1, 2);
    plot(time, velocity_history(1, :), 'r', time, velocity_history(2, :), 'g', time, velocity_history(3, :), 'b');
    title('Velocity (m/s)');
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    legend('v_x', 'v_y', 'v_z');

    % Position error over time
    subplot(3, 1, 3);
    plot(time, error_history, 'k');
    title('Position Error in Z (m)');
    xlabel('Time (s)');
    ylabel('Error (m)');
    legend('Error in Z');
end

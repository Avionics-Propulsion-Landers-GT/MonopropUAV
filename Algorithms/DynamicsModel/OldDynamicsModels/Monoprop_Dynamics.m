function monocopter_dynamics_with_wind()
    % Simulation parameters
    dt = 0.01;  % Time step (seconds)
    t_end = 10;  % Simulation time (seconds)
    num_steps = floor(t_end / dt);  % Number of time steps

    % Physical constants and parameters
    m = 1.0;  % Mass of the monocopter (kg)
    g = 9.81;  % Gravitational acceleration (m/s^2)
    k_D = 0.1;  % Drag coefficient
    k_wind = 0.05;  % Wind drag coefficient
    T = 10;  % Thrust magnitude (N)

    % Moments of inertia (kg*m^2)
    I_x = 1.0;
    I_y = 2.0;
    I_z = 1.0;
    I = [I_x, I_y, I_z];  % Inertia tensor diagonal elements

    % Initial conditions
    r = [0; 0; 0];  % Initial position (m)
    v = [0; 0; 0];  % Initial velocity (m/s)
    q = [1; 0; 0; 0];  % Initial quaternion (neutral orientation)
    omega = [0; 0; 0];  % Initial angular velocity (rad/s)
    thrust_dir_body = [0; 0; 1.5];  % Thrust in the body frame (along z-axis)

    % Pre-allocate arrays to store results
    position_history = zeros(3, num_steps);
    velocity_history = zeros(3, num_steps);
    quaternion_history = zeros(4, num_steps);
    angular_velocity_history = zeros(3, num_steps);

    % Initial wind velocity (random starting point)
    v_wind = [randn(); randn(); randn()] * 2;  % Random initial wind velocity in m/s

    % Main simulation loop
    for step = 1:num_steps
        t = (step - 1) * dt;  % Current time

        % Random wind velocity (updates over time with noise)
        % Apply Gaussian noise and a smoothing factor to simulate randomness
        v_wind = v_wind + 0.1 * randn(3, 1);  % Smooth random walk for wind changes

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

    % Plot results
    plot_results(position_history, velocity_history, quaternion_history, angular_velocity_history, dt);
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
    R = [1 - 2*y^2 - 2*z^2, 2*x*y - 2*z*w, 2*x*z + 2*y*w;
         2*x*y + 2*z*w, 1 - 2*x^2 - 2*z^2, 2*y*z - 2*x*w;
         2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x^2 - 2*y^2];
end

%% Function to plot simulation results
function plot_results(position_history, velocity_history, quaternion_history, angular_velocity_history, dt)
    time = (0:size(position_history, 2) - 1) * dt;

    % 3D Trajectory plot
    figure;
    plot3(position_history(1, :), position_history(2, :), position_history(3, :), 'b-', 'LineWidth', 1.5);
    grid on;
    title('Monoprop 3D Trajectory');
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    zlabel('Z Position (m)');
    view(3);  % Set 3D view for better visualization

    % Position, velocity, quaternion and angular velocity history
    figure;
    subplot(3, 1, 1);
    plot(time, position_history(1, :), 'r', time, position_history(2, :), 'g', time, position_history(3, :), 'b');
    title('Position (m)');
    xlabel('Time (s)');
    legend('x', 'y', 'z');

    subplot(3, 1, 2);
    plot(time, velocity_history(1, :), 'r', time, velocity_history(2, :), 'g', time, velocity_history(3, :), 'b');
    title('Velocity (m/s)');
    xlabel('Time (s)');
    legend('x', 'y', 'z');

    subplot(3, 1, 3);
    plot(time, quaternion_history(1, :), 'r', time, quaternion_history(2, :), 'g', time, quaternion_history(3, :), 'b', time, quaternion_history(4, :), 'k');
    title('Quaternion');
    xlabel('Time (s)');
    legend('w', 'x', 'y', 'z');
end

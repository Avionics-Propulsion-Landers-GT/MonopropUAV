function monocopter_dynamics_with_pid()
    % Simulation parameters
    dt = 0.01;  % Time step (seconds)
    t_end = 20;  % Total simulation time (seconds)
    num_steps = floor(t_end / dt);  % Number of time steps

    % Physical constants and parameters
    m = 1.0;  % Mass of the monocopter (kg)
    g = 9.81;  % Gravitational acceleration (m/s^2)
    k_D = 0.1;  % Drag coefficient

    % Moments of inertia (kg*m^2)
    I = diag([1.0, 2.0, 1.0]);  % Inertia tensor (3x3 diagonal matrix)

    % Initial conditions
    r = [0; 0; 0];  % Initial position (m)
    v = [0; 0; 0];  % Initial velocity (m/s)
    q = [1; 0; 0; 0];  % Initial quaternion (neutral orientation)
    omega = [0; 0; 0];  % Initial angular velocity (rad/s)

    % Target trajectory points
    target_positions = [[0; 0; 0], [0; 0; 100], [0; 0; 0]];  % from (0,0,0) to (0,0,100) and back
    target_index = 1;  % Start at the first target position

    % PID Controller parameters
    Kp = 1.0;  % Proportional gain
    Ki = 0.1;  % Integral gain
    Kd = 0.5;  % Derivative gain
    integral = [0; 0; 0];  % Integral error

    % Pre-allocate arrays to store results
    position_history = zeros(3, num_steps);
    velocity_history = zeros(3, num_steps);
    quaternion_history = zeros(4, num_steps);
    angular_velocity_history = zeros(3, num_steps);

    % Main simulation loop
    for step = 1:num_steps
        t = (step - 1) * dt;  % Current time

        % Store history
        position_history(:, step) = r;
        velocity_history(:, step) = v;
        quaternion_history(:, step) = q;
        angular_velocity_history(:, step) = omega;

        % Calculate position error
        target_pos = target_positions(:, target_index);
        position_error = target_pos - r;

        % PID Control
        integral = integral + position_error * dt;  % Integral error
        derivative = position_error / dt;  % Derivative error
        thrust = Kp * position_error + Ki * integral + Kd * derivative;

        % Clamping thrust to avoid extreme values
        thrust = min(max(thrust, 0), 20);  % Limit thrust to a maximum of 20 N

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

        % Ensure thrust_vector is a column vector
        thrust_vector = [0; 0; thrust];  % Thrust acts along the z-axis in body frame

        % Thrust in world frame
        thrust_world = R_b_to_w * thrust_vector;  % Thrust acts in the world frame

        % Net force = thrust - gravity - drag
        gravity = [0; 0; -m * g];  % Gravity acts downwards
        drag = -k_D * v;  % Drag force
        F_net = thrust_world + gravity + drag;  % Net force on the vehicle

        % Acceleration
        a = F_net / m;  % Compute acceleration using Newton's second law

        % Update velocity and position
        v = v + a * dt;  % Update velocity
        r = r + v * dt;  % Update position

        % Check for reaching the target
        if norm(position_error) < 0.1 && target_index < size(target_positions, 2)
            target_index = target_index + 1;  % Move to the next target position
            integral = [0; 0; 0];  % Reset integral error when reaching a target
        end
    end

    % Plot results
    plot_results(position_history, velocity_history, quaternion_history, angular_velocity_history, dt);
end

%% Function to calculate the angular acceleration using Euler's equations
function omega_dot = euler_equations(omega, M, I)
    % Extract moments of inertia
    I_x = I(1, 1);
    I_y = I(2, 2);
    I_z = I(3, 3);

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
    title('Monocopter 3D Trajectory');
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    zlabel('Z Position (m)');
    view(3);  % Set 3D view for better visualization

    % Position, velocity, quaternion, and angular velocity history
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
    title('Quaternions');
    xlabel('Time (s)');
    legend('q1', 'q2', 'q3', 'q4');

    % Angular velocity plot
    figure;
    plot(time, angular_velocity_history(1, :), 'r', time, angular_velocity_history(2, :), 'g', time, angular_velocity_history(3, :), 'b');
    title('Angular Velocity (rad/s)');
    xlabel('Time (s)');
    legend('wx', 'wy', 'wz');
end

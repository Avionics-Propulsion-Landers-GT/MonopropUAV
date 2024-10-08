function monocopter_dynamics_with_wind_PID_improved()
    % Simulation parameters
    dt = 0.01;      % Time step (seconds)
    t_end = 40;     % Simulation time (seconds)
    t = 0:dt:t_end; % Time vector
    num_steps = length(t);  % Number of time steps

    % Physical constants and parameters
    m = 1.0;       % Mass of the monocopter (kg)
    g = 9.81;      % Gravitational acceleration (m/s^2)
    k_D = 0.1;     % Drag coefficient
    k_wind = 0.0;  % Wind drag coefficient set to zero to remove wind disturbances
    T_max = 50;    % Maximum thrust magnitude (N)
    T_min = 0;     % Minimum thrust magnitude (N)

    % Moments of inertia (kg*m^2)
    I_x = 0.05;
    I_y = 0.05;
    I_z = 0.1;
    I = diag([I_x, I_y, I_z]);  % Inertia tensor

    % PID controller gains for position control
    Kp_pos = [1.5; 1.5; 2.0];    % Proportional gains for X, Y, Z
    Ki_pos = [0.0; 0.0; 0.0];    % Integral gains for X, Y, Z
    Kd_pos = [1.0; 1.0; 1.5];    % Derivative gains for X, Y, Z

    % PID gains for attitude control
    Kp_att = [8.0; 8.0; 0.0];  % Proportional gains for Roll, Pitch, Yaw
    Kd_att = [2.0; 2.0; 0.0];  % Derivative gains for Roll, Pitch, Yaw

    % Desired trajectory with hover phases
    position_desired = zeros(3, num_steps);
    T_ascent = 10;       % Time to ascend
    T_hover_top = 5;     % Hover time at the top
    T_descent = 10;      % Time to descend
    T_hover_bottom = t_end - (T_ascent + T_hover_top + T_descent); % Hover time at the bottom

    for i = 1:num_steps
        if t(i) <= T_ascent
            % Ascent phase
            z_desired = (100 / T_ascent) * t(i);
        elseif t(i) <= T_ascent + T_hover_top
            % Hover at the top
            z_desired = 100;
        elseif t(i) <= T_ascent + T_hover_top + T_descent
            % Descent phase
            z_desired = 100 - (100 / T_descent) * (t(i) - T_ascent - T_hover_top);
        else
            % Hover at the bottom
            z_desired = 0;
        end
        position_desired(:, i) = [0; 0; z_desired];
    end

    % Initial conditions
    r = [0; 0; 0];          % Initial position (m)
    v = [0; 0; 0];          % Initial velocity (m/s)
    q = [1; 0; 0; 0];       % Initial quaternion (neutral orientation)
    omega = [0; 0; 0];      % Initial angular velocity (rad/s)

    % Initialize PID controller variables
    previous_error = zeros(3, 1);

    % Pre-allocate arrays to store results
    position_history = zeros(3, num_steps);
    velocity_history = zeros(3, num_steps);
    acceleration_history = zeros(3, num_steps);
    quaternion_history = zeros(4, num_steps);
    angular_velocity_history = zeros(3, num_steps);
    error_history = zeros(3, num_steps);

    % Initialize wind velocity array (no wind)
    v_wind = zeros(3, num_steps);

    % Main simulation loop
    for step = 1:num_steps
        % Current time
        t_current = t(step);

        % Desired position at current time
        r_desired = position_desired(:, step);

        % Compute position error
        error = r_desired - r;
        error_history(:, step) = error;

        % Compute derivative of error
        if step == 1
            error_derivative = zeros(3, 1);
        else
            error_derivative = (error - previous_error) / dt;
        end

        % PID controller output (desired acceleration in X, Y, Z)
        u = Kp_pos .* error + Kd_pos .* error_derivative;

        % Update previous error
        previous_error = error;

        % Compute desired total thrust (assuming mass normalized)
        F_total = m * (u(3) + g);
        if F_total > T_max
            F_total = T_max;
        elseif F_total < T_min
            F_total = T_min;
        end

        % Compute desired roll and pitch angles
        theta_des = u(1) / g;  % Desired pitch angle (rad)
        phi_des = -u(2) / g;   % Desired roll angle (rad)

        % Limit desired angles to ±30 degrees
        max_angle = deg2rad(30);
        theta_des = max(min(theta_des, max_angle), -max_angle);
        phi_des = max(min(phi_des, max_angle), -max_angle);

        % Desired yaw angle (keep at zero)
        psi_des = 0;

        % Compute desired rotation matrix from desired Euler angles
        R_desired = euler_angles_to_rotation_matrix(phi_des, theta_des, psi_des);

        % Convert current quaternion to rotation matrix
        R_current = quat_to_rotation_matrix(q);

        % Compute rotation error (rotation matrix)
        R_error = R_desired * R_current';

        % Convert rotation error to Euler angles
        euler_error = rotation_matrix_to_euler_angles(R_error);  % euler_error is 3x1 vector

        % Desired angular velocities (proportional to rotation errors)
        omega_desired = Kp_att .* euler_error;  % Removed transpose here

        % Compute angular acceleration required
        omega_error = omega_desired - omega;
        omega_dot = Kd_att .* omega_error;

        % Update angular velocity
        omega = omega + omega_dot * dt;

        % Compute quaternion kinematics
        q_dot = 0.5 * omega_matrix(omega) * q;

        % Update quaternion and normalize
        q = q + q_dot * dt;
        q = q / norm(q);  % Normalize to maintain unit quaternion

        % Compute the rotation matrix from the updated quaternion
        R_b_to_w = quat_to_rotation_matrix(q);

        % Thrust in body frame (assumed to be along z-axis)
        thrust_body = [0; 0; F_total];

        % Thrust in world frame
        thrust_world = R_b_to_w * thrust_body;

        % Wind velocity at current time (no wind)
        v_wind_current = v_wind(:, step);

        % Relative velocity between the monocopter and the wind
        v_rel = v - v_wind_current;

        % Wind force (zero in this case)
        F_wind = -k_wind * norm(v_rel) * v_rel;

        % Net force = thrust + wind force - gravity - drag
        gravity = [0; 0; -m * g];
        drag = -k_D * v;
        F_net = thrust_world + F_wind + drag + gravity;

        % Acceleration
        a = F_net / m;
        acceleration_history(:, step) = a;  % Store acceleration history

        % Update velocity and position
        v = v + a * dt;
        r = r + v * dt;

        % Store history
        position_history(:, step) = r;
        velocity_history(:, step) = v;
        quaternion_history(:, step) = q;
        angular_velocity_history(:, step) = omega;
    end

    % Plot results
    plot_results(position_history, velocity_history, acceleration_history, quaternion_history, angular_velocity_history, error_history, dt, t, position_desired);
end  % End of main function

%% Function to compute rotation matrix from Euler angles
function R = euler_angles_to_rotation_matrix(phi, theta, psi)
    % Rotation matrix from body frame to world frame
    R = [
        cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta);
        sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), sin(phi)*cos(theta);
        cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), cos(phi)*cos(theta)
    ];
end

%% Function to convert rotation matrix to Euler angles
function euler = rotation_matrix_to_euler_angles(R)
    % Ensure numerical stability
    if abs(R(3,1)) >= 1
        theta = -sign(R(3,1)) * pi / 2;
        phi = 0;
        psi = atan2(-R(1,2), -R(1,3));
    else
        theta = -asin(R(3,1));
        phi = atan2(R(3,2)/cos(theta), R(3,3)/cos(theta));
        psi = atan2(R(2,1)/cos(theta), R(1,1)/cos(theta));
    end
    euler = [phi; theta; psi];
end

%% Function to compute the angular velocity matrix Omega for quaternion kinematics
function Omega = omega_matrix(omega)
    omega_x = omega(1);
    omega_y = omega(2);
    omega_z = omega(3);

    Omega = [
        0, -omega_x, -omega_y, -omega_z;
        omega_x, 0, omega_z, -omega_y;
        omega_y, -omega_z, 0, omega_x;
        omega_z, omega_y, -omega_x, 0
    ];
end

%% Function to convert a quaternion to a rotation matrix
function R = quat_to_rotation_matrix(q)
    w = q(1);
    x = q(2);
    y = q(3);
    z = q(4);

    % Compute the rotation matrix
    R = [
        1 - 2*y^2 - 2*z^2,   2*x*y - 2*z*w,       2*x*z + 2*y*w;
        2*x*y + 2*z*w,       1 - 2*x^2 - 2*z^2,   2*y*z - 2*x*w;
        2*x*z - 2*y*w,       2*y*z + 2*x*w,       1 - 2*x^2 - 2*y^2
    ];
end

%% Function to plot simulation results
function plot_results(position_history, velocity_history, acceleration_history, quaternion_history, angular_velocity_history, error_history, dt, t, position_desired)
    time = t;

    % 3D Trajectory plot
    figure;
    plot3(position_history(1, :), position_history(2, :), position_history(3, :), 'b-', 'LineWidth', 1.5);
    hold on;
    plot3(position_desired(1, :), position_desired(2, :), position_desired(3, :), 'r--', 'LineWidth', 1.5);
    grid on;
    title('Monocopter 3D Trajectory');
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    zlabel('Z Position (m)');
    legend('Actual Path', 'Desired Path');
    view(3);  % Set 3D view for better visualization

    % Position, Velocity, Acceleration, Position Error, Angular Velocity
    figure;
    subplot(6, 1, 1);
    plot(time, position_history(1, :), 'r', time, position_history(2, :), 'g', time, position_history(3, :), 'b');
    hold on;
    plot(time, position_desired(1, :), 'r--', time, position_desired(2, :), 'g--', time, position_desired(3, :), 'b--');
    title('Position (m)');
    xlabel('Time (s)');
    ylabel('Position (m)');
    legend('x', 'y', 'z', 'x_{desired}', 'y_{desired}', 'z_{desired}');

    subplot(6, 1, 2);
    plot(time, velocity_history(1, :), 'r', time, velocity_history(2, :), 'g', time, velocity_history(3, :), 'b');
    title('Velocity (m/s)');
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    legend('v_x', 'v_y', 'v_z');

    subplot(6, 1, 3);
    plot(time, acceleration_history(1, :), 'r', time, acceleration_history(2, :), 'g', time, acceleration_history(3, :), 'b');
    title('Acceleration (m/s^2)');
    xlabel('Time (s)');
    ylabel('Acceleration (m/s^2)');
    legend('a_x', 'a_y', 'a_z');

    subplot(6, 1, 4);
    plot(time, error_history(1, :), 'r', time, error_history(2, :), 'g', time, error_history(3, :), 'b');
    title('Position Error (m)');
    xlabel('Time (s)');
    ylabel('Error (m)');
    legend('Error in X', 'Error in Y', 'Error in Z');

    subplot(6, 1, 5);
    plot(time, angular_velocity_history(1, :), 'r', time, angular_velocity_history(2, :), 'g', time, angular_velocity_history(3, :), 'b');
    title('Angular Velocity (rad/s)');
    xlabel('Time (s)');
    ylabel('Angular Velocity (rad/s)');
    legend('\omega_x', '\omega_y', '\omega_z');

    % Quaternion components over time
    subplot(6, 1, 6);
    plot(time, quaternion_history(1, :), 'k', time, quaternion_history(2, :), 'r', time, quaternion_history(3, :), 'g', time, quaternion_history(4, :), 'b');
    title('Quaternion Components');
    xlabel('Time (s)');
    ylabel('Quaternion Value');
    legend('q_0', 'q_1', 'q_2', 'q_3');
end  % End of plot_results function

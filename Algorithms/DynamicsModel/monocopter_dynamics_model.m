% Monoprop physical parameters
global m_static;
m_static = 1; % TODO: set
global m_gimbal_top;
m_gimbal_top = 1; % TODO: set
global m_gimbal_bottom;
m_gimbal_bottom = 1; % TODO: set
global m;
m = m_static + m_gimbal_top + m_gimbal_bottom;
global gimbal_top_COM_offset;
gimbal_top_COM_offset = [0; 0; 0]; % TODO: set
global gimbal_bottom_COM_offset;
gimbal_bottom_COM_offset = [0; 0; 0]; % TODO: set
global g;
g = 9.81;
global COM_offset;
COM_offset = [0; 0; 0];
global COP;
COP = [0; 0; 0]; % TODO: set
global COP_offset;
COP_offset = [0; 0; 0];
global gimbal_offset;
gimbal_offset = [0; 0; 0]; % TODO: set
global gimbal_x_distance;
gimbal_x_distance = [0; 0; 0]; % TODO: set
global gimbal_y_distance;
gimbal_y_distance = [0; 0; 0]; % TODO: set
global Cd_x;
Cd_x = 0.1; % TODO: set
global Cd_y;
Cd_y = 0.1; % TODO: set
global Cd_z;
Cd_z = 0.1; % TODO: set
global Mp_x;
Mp_x = 0; % TODO: set
global Mp_y;
Mp_y = 0; % TODO: set
global Mp_z;
Mp_z = 0; % TODO: set
global A_x;
A_x = 0.7; % TODO: set
global A_y;
A_y = 0.7; % TODO: set
global A_z;
A_z = 0.3; % TODO: set
global air_density;
air_density = 1.225;
global I_xx;
I_xx = 1; % TODO: set
global I_yy;
I_yy = 1; % TODO: set
global I_zz;
I_zz = 1; % TODO: set
global I_xy;
I_xy = 0; % TODO: set
global I_xz;
I_xz = 0; % TODO: set
global I_yz;
I_yz = 0; % TODO: set
global I;
I = [I_xx, I_xy, I_xz;
    I_xy, I_yy, I_yz;
    I_xz, I_yz, I_zz];
global Inv_I;
Inv_I = inv(I);
global gimbal_top_I;
gimbal_top_I = 1; % TODO: set
global gimbal_bottom_I;
gimbal_bottom_I = 1; % TODO: set
global T_max;
T_max = 100; % TODO: set
global T_min;
T_min = 0;
global gimbal_speed;
gimbal_speed = 5; % TODO: set
global gimbal_acceleration;
gimbal_acceleration =2; % TODO: set
global dt; %discrete time step for the simulation
dt = 0.001;

simulate
%% simulation function
function simulate()
    global m;
    global g;
    global T_max;
    global T_min;
    global gimbal_speed;
    global dt;

    % Simulation parameters
    t_end = 40; % Simulation time (seconds): The total duration for which the simulation runs.
    t = 0:dt:t_end; % Time vector: Discretized time values for the simulation, spanning from 0 to t_end with increments of dt.
    num_steps = length(t);  % Number of time steps: Determines the total number of iterations based on the time vector.

    % initial conditions
    pos = [0; 0; 0];
    vel = [0; 0; 0];
    att = [1; 0; 0; 0];
    ang_vel = [0; 0; 0];
    thrust_gimbal_ang_vel = [0; 0; 0];
    thrust_gimbal_goal = [0; 0; 0];
    thrust_gimbal = [0; 0; 0]; %testing

    % Pre-allocate arrays to store results
    position_history = zeros(3, num_steps);       % Stores the rocket's position (X, Y, Z) over time.
    velocity_history = zeros(3, num_steps);       % Stores the rocket's velocity (X, Y, Z) over time.
    acceleration_history = zeros(3, num_steps);   % Stores the rocket's acceleration (X, Y, Z) over time.
    attitude_history = zeros(4, num_steps);     % Stores the rocket's quaternion attitude (orientation) over time.
    angular_velocity_history = zeros(3, num_steps); % Stores the rocket's angular velocity (roll, pitch, yaw) over time.
    angular_acceleration_history = zeros(3, num_steps); % Stores the rocket's angular acceleration (roll, pitch, yaw) over time.
    position_error_history = zeros(3, num_steps);          % Stores the position error (X, Y, Z) at each time step.
    attitude_error_history = zeros(4, num_steps);          % Stores the quaternion attitude error at each time step.
    attitude_error_history_euler = zeros(3, num_steps); %Stores the euler angle attitude error over time
    gimbal_cmd_history = zeros(3, num_steps); %Stores the gimbal cmd history over time
    gimbal_actuation_history = zeros(3, num_steps); %Stores the gimbal's actuated angles over time

    % Desired trajectory with hover phases
    position_desired = zeros(3, num_steps); % Desired position trajectory (X, Y, Z) over time, initialized to zero.
    T_ascent = 10;       % Time to ascend (seconds): Duration during which the rocket moves upward.
    T_hover_top = 5;     % Hover time at the top (seconds): Duration of stable hover at the highest point of the trajectory.
    T_descent = 10;      % Time to descend (seconds): Duration during which the rocket moves downward toward the starting point.
    T_hover_bottom = t_end - (T_ascent + T_hover_top + T_descent); % Hover time at the bottom (seconds): Remaining time to hover near the ground.

    for i = 1:num_steps
        % Loop over each time step in the simulation
        
        if t(i) <= T_ascent
            % Ascent phase: The rocket climbs vertically from the ground to the target altitude.
            z_desired = (100 / T_ascent) * t(i); % Linear ascent trajectory in the Z-direction.
                                                 % z_desired gradually increases from 0 to 100 meters
                                                 % over the ascent duration of T_ascent seconds.
        elseif t(i) <= T_ascent + T_hover_top
            % Hover at the top phase: The rocket maintains a stable altitude at the peak.
            z_desired = 100;  % Desired altitude stays constant at 100 meters during this phase.
        elseif t(i) <= T_ascent + T_hover_top + T_descent
            % Descent phase: The rocket descends back to the ground in a controlled manner.
            z_desired = 100 - (100 / T_descent) * (t(i) - T_ascent - T_hover_top);  
                                                 % Linear descent trajectory in the Z-direction,
                                                 % reducing z_desired from 100 meters to 0.
        else
            % Hover at the bottom phase: The rocket remains stable near the ground level.
            z_desired = 0;  % Desired altitude is 0 meters for the remainder of the simulation.
        end
        
        % Update the desired position vector at the current time step
        position_desired(:, i) = [0; 0; z_desired];  % Desired position is always (0, 0, z_desired),
                                                    % assuming no lateral movement (X and Y remain 0).
    end

    % Initialize wind velocity array (no wind)
    v_wind = zeros(3, num_steps);  % Wind velocity (X, Y, Z) at each time step is set to zero, as wind effects
                                   % are disabled in this simulation.

    % Control Scheme

    % PID controller gains for position control
    Kp_pos = [1.5; 1.5; 2.0];    % Proportional gains for X, Y, Z: Define how aggressively the controller corrects position errors in each axis.
    Ki_pos = [0.0; 0.0; 0.0];    % Integral gains for X, Y, Z: Used to address steady-state errors in position control, currently inactive (set to 0).
    Kd_pos = [1.0; 1.0; 1.5];    % Derivative gains for X, Y, Z: Define the damping behavior by responding to changes in position error rates.

    % PID gains for attitude control
    Kp_att = [8.0; 8.0; 0.0];  % Proportional gains for Roll, Pitch, Yaw: Control the rocket's angular orientation. 
                               % Higher values ensure quicker corrections but may cause instability if set too high.
    Kd_att = [2.0; 2.0; 0.0];  % Derivative gains for Roll, Pitch, Yaw: Provides damping to prevent oscillations in attitude control.

    % Time vector
    x = 0:dt:t_end;
    n = length(x);
    
    % Output vector (initialize)
    gimbal_cmd = zeros(3, n); 
    
    % Parameters
    max_angle = pi/12; % max magnitude in rad
    min_duration = 1;  % seconds
    max_duration = 5;  % seconds
    
    %Generate random x cmds
    i = 1;
    while i <= n
        % Random duration in seconds, converted to steps
        duration_sec = min_duration + (max_duration - min_duration) * rand();
        duration_steps = round(duration_sec / dt);
        
        % Ensure we don't overshoot the vector length
        end_i = min(i + duration_steps - 1, n);
        
        % Random command magnitude (positive or negative)
        mag = (2*rand() - 1) * max_angle; % range: [-pi/12, +pi/12]
        
        % Apply to segment
        gimbal_cmd(1, i:end_i) = mag;
        
        % Move to next segment
        i = end_i + 1;
    end

    %Generate random y cmds
    i = 1;
    while i <= n
        % Random duration in seconds, converted to steps
        duration_sec = min_duration + (max_duration - min_duration) * rand();
        duration_steps = round(duration_sec / dt);
        
        % Ensure we don't overshoot the vector length
        end_i = min(i + duration_steps - 1, n);
        
        % Random command magnitude (positive or negative)
        mag = (2*rand() - 1) * max_angle; % range: [-pi/12, +pi/12]
        
        % Apply to segment
        gimbal_cmd(2, i:end_i) = mag;
        
        % Move to next segment
        i = end_i + 1;
    end
 
    for step = 1:num_steps
        % Current time
        t_current = t(step);  % Fetch the current time step value.

    
        target_position = position_desired(:, step);
        position_error = pos - target_position;
        attitude_error = quaternion_multiply([1; 0; 0; 0;], quaternion_inverse(att));
        if norm(attitude_error) > 0
            attitude_error = attitude_error / norm(attitude_error);
        end
        euler_attitude_error = quaternion_to_euler(attitude_error);
        gimbal_goal = [clip(euler_attitude_error(1), -pi/12, pi/12); clip(euler_attitude_error(2), -pi/12, pi/12); 0];

        thrust_gimbal_goal = gimbal_cmd(:, step); %delete later when doing LQR

        gimbal_info = update_TVC(thrust_gimbal, thrust_gimbal_ang_vel, thrust_gimbal_goal, dt);
        thrust_gimbal = gimbal_info{1}; %thrust_gimbal needs to be a 3 x 1 vector
        thrust_gimbal_ang_vel = gimbal_info{2};
        F_thrust_mag = clip(50, T_min, T_max); % TODO: determine thrust via control scheme

        % TODO: look into what forces we need: gravity, thrust, aerodynamic drag, aerodynamic torques
        calculate_COM_and_COP_offset(thrust_gimbal)
        F_gravity = [0; 0; -m * g];
        thrust = get_thrust_body(F_thrust_mag, thrust_gimbal);
        %thrust = get_thrust_body(F_thrust_mag, thrust_gimbal_goal);
        F_thrust_body = thrust{1};
        drag = get_drag_body(att, vel, v_wind(:, step));
        F_drag_body = drag{1};
        
        F_net = F_gravity + to_world_frame_quaternion(att) * (F_thrust_body + F_drag_body);
    
        T_gimbal_body = gimbal_info{3};
        T_thrust_body = thrust{2};
        T_drag_body = drag{2};
        T_net = to_world_frame_quaternion(att) * (T_thrust_body + T_drag_body + T_gimbal_body);
    
        state = update_dynamics(pos, vel, att, ang_vel, F_net, T_net, thrust_gimbal_ang_vel, dt);
        pos = state{1};
        vel = state{2};
        accel = state{3};
        att = state{4};
        ang_vel = state{5};
        ang_accel = state{6};

        % Store history
        position_history(:, step) = pos;  % Record position history.
        velocity_history(:, step) = vel;  % Record velocity history.
        acceleration_history(:, step) = accel;  % Record acceleration history.
        attitude_history(:, step) = att;  % Record quaternion attitude history.
        angular_velocity_history(:, step) = ang_vel;  % Record angular velocity history.
        angular_acceleration_history(:, step) = ang_accel;  % Record angular acceleration history.
        position_error_history(:, step) = position_error;  % Record position error history.
        attitude_error_history(:, step) = attitude_error;  % Record quaternion attitude error history.
        attitude_error_history_euler(:, step) = euler_attitude_error; % Record euler angle attitude error history
        gimbal_cmd_history(:, step) = thrust_gimbal_goal; % Record gimbal command history
        gimbal_actuation_history(:, step) = thrust_gimbal; % Record actual gimbal actuation history (what its actual position is, not the gimbal cmd)
    end
    % Store to CSV file
    filename = 'histories.csv';
    writematrix(position_history', filename);

    %close any open figures
    close all

    %plot trajectory
    figure('Name', 'Trajectory Plot')
    plot3(position_history(1,:), position_history(2,:), position_history(3,:), 'lineWidth', 2)
    %plot trajectory coordinates separately
    figure('Name', 'Trajectory Coordinates over Time')
    subplot(3, 1, 1)
    plot(1:num_steps, position_history(1, :))
    title('X')
    subplot(3, 1, 2)
    plot(1:num_steps, position_history(2, :))
    title('Y')
    subplot(3, 1, 3)
    plot(1:num_steps, position_history(3, :))
    title('Z')
    %plot acceleration
    figure('Name', 'Translational Acceleration')
    subplot(3, 1, 1)
    plot(1:num_steps, acceleration_history(1, :))
    title('a_X')
    subplot(3, 1, 2)
    plot(1:num_steps, acceleration_history(2, :))
    title('a_Y')
    subplot(3, 1, 3)
    plot(1:num_steps, acceleration_history(3, :))
    title('a_Z')
    %plot velocity
    figure('Name', 'Translational Velocity')
    subplot(3, 1, 1)
    plot(1:num_steps, velocity_history(1, :))
    title('v_X')
    subplot(3, 1, 2)
    plot(1:num_steps, velocity_history(2, :))
    title('v_Y')
    subplot(3, 1, 3)
    plot(1:num_steps, velocity_history(3, :))
    title('v_Z')
    %plot angular velocity
    %plot angular acceleration
    %plot attitude
    figure('Name', 'Gimbal')
    subplot(3, 1, 1)
    plot(1:num_steps, gimbal_cmd_history(1, :) * (180/pi))
    hold on
    plot(1:num_steps, gimbal_actuation_history(1, :) * (180/pi))
    title('X gimbal')
    legend('CMD', 'Actual')
    subplot(3, 1, 2)
    plot(1:num_steps,  gimbal_cmd_history(2, :) * (180/pi))
    hold on
    plot(1:num_steps,  gimbal_actuation_history(2, :) * (180/pi))
    title('Y gimbal')
    legend('CMD', 'Actual')
    subplot(3, 1, 3)
    plot(1:num_steps,  gimbal_cmd_history(3, :) * (180/pi))
    hold on
    plot(1:num_steps,  gimbal_actuation_history(3, :) * (180/pi))
    legend('CMD', 'Actual')
    title('Z gimbal')
end

%% updates the state with a given net force, net torque, and time step
function state = update_dynamics(pos, vel, att, ang_vel, F_net, T_net, gimbal_ang_vel, delta_t)
    global m;
    global I;
    global Inv_I;

    % translational components
    accel = F_net / m;
    vel = vel + accel * delta_t;
    pos = pos + vel * delta_t;
    % rotational component
    ang_accel = Inv_I * (T_net);% - cross(ang_vel, I * ang_vel));% - cross([gimbal_ang_vel(1); 0; 0], TVC_top_I * [gimbal_ang_vel(1); 0; 0]) - cross(gimbal_ang_vel, TVC_bottom_I * gimbal_ang_vel));
    ang_vel = ang_vel + ang_accel * delta_t;
    delta_att = ang_vel * delta_t;
    att = att + 0.5 * quaternion_multiply([0; delta_att(1); delta_att(2); delta_att(3)], att) * delta_t;
    if (norm(att) ~= 0)
        att = att / norm(att);
    end
    
    state = {pos, vel, accel, att, ang_vel, ang_accel}; %cell array of state variables (variables are of different size so can't concatenate)
end

%% calculates the center of mass offset from the static center of mass
function calculate_COM_and_COP_offset(thrust_gimbal)
    global m;
    global m_gimbal_top;
    global m_gimbal_bottom;
    global gimbal_offset;
    global gimbal_top_COM_offset;
    global gimbal_bottom_COM_offset;
    global gimbal_x_distance;
    global COM_offset;
    global COP;
    global COP_offset;

    top_gimbal_COM = (m_gimbal_top / m) * (gimbal_offset + get_extrinsic_x_rotation(thrust_gimbal(1)) * gimbal_top_COM_offset); 
    bottom_gimbal_COM = (m_gimbal_bottom / m) * (gimbal_offset + get_extrinsic_x_rotation(thrust_gimbal(1)) * gimbal_x_distance + get_extrinsic_rotation_matrix(thrust_gimbal) * gimbal_bottom_COM_offset);
    COM_offset = top_gimbal_COM + bottom_gimbal_COM;
    COP_offset = COP - COM_offset;
end

%% gets thrust in the body frame
function thrust = get_thrust_body(F_thrust_mag, thrust_gimbal)
    global gimbal_offset;
    global gimbal_x_distance;
    global gimbal_y_distance;
    global COM_offset;
    
    thrust_force_vector = F_thrust_mag * (get_extrinsic_rotation_matrix(thrust_gimbal) * [0;0;1]); 
    thrust_torque_vector = cross(COM_offset + gimbal_offset + get_extrinsic_x_rotation(thrust_gimbal(1)) * gimbal_x_distance + get_extrinsic_rotation_matrix(thrust_gimbal) * gimbal_y_distance, thrust_force_vector);
    thrust = {thrust_force_vector, thrust_torque_vector}; %cell array of force and torque vectors
end

%% gets drag force in the body frame
function drag = get_drag_body(att, vel, v_wind)
    global air_density;
    global Cd_x;
    global Cd_y;
    global Cd_z;
    global A_x;
    global A_y;
    global A_z;
    global COP_offset;
    
    vel_relative = to_body_frame_quaternion(att) * (vel - v_wind);
    drag_force =  - 0.5 * air_density * diag([Cd_x*A_x, Cd_y*A_y, Cd_z*A_z]) * (norm(vel_relative) * vel_relative); %drag force points in opposite direction to relative velocity
    drag_torque = cross(COP_offset, drag_force);
    drag = {drag_force, drag_torque}; %cell array of force and torque vectors
end

%% updates the TVC gimbal movement and gets torque from the TVC gimbal movement
function info = update_TVC(gimbal, gimbal_ang_vel, gimbal_goal, delta_t)
    global gimbal_speed;
    global gimbal_acceleration;
    global gimbal_top_I;
    global gimbal_bottom_I



    prev_gimbal_ang_vel = gimbal_ang_vel;
    gimbal_error = gimbal_goal - gimbal;
    stop_distance_x = (gimbal_ang_vel(1))^2 / (2 * gimbal_acceleration);
    stop_distance_y = (gimbal_ang_vel(2))^2 / (2 * gimbal_acceleration);
    %projected_decel = gimbal_ang_vel.^2 + 2 * diag([gimbal_acceleration * -sign(gimbal_error(1)), gimbal_acceleration * -sign(gimbal_error(2)), 0]) * gimbal_error;
    if abs(gimbal_error(1)) <= abs(stop_distance_x) && gimbal_ang_vel(1) ~= 0
        % decel
        gimbal_ang_vel(1) = gimbal_ang_vel(1) + gimbal_acceleration * -sign(gimbal_ang_vel(1)) * delta_t;
    else
        % accel
        gimbal_ang_vel(1) = gimbal_ang_vel(1) + gimbal_acceleration * sign(gimbal_error(1)) * delta_t;
    end

    if abs(gimbal_error(2)) <= abs(stop_distance_y) && gimbal_ang_vel(2) ~= 0
        % decel
        gimbal_ang_vel(2) = gimbal_ang_vel(2) + gimbal_acceleration * -sign(gimbal_ang_vel(2)) * delta_t;
    else
        % acel
        gimbal_ang_vel(2) = gimbal_ang_vel(2) + gimbal_acceleration * sign(gimbal_error(2)) * delta_t;
    end
    
% <<<<<<< HEAD
%     projected_decel = gimbal_ang_vel.^2 - 2 * diag([gimbal_acceleration * -sign(gimbal_error(1)), gimbal_acceleration * -sign(gimbal_error(2)), 0]) * gimbal_error;
%     projected_decel = diag(sign(projected_decel)) * sqrt(abs(projected_decel));
% 
%     if projected_decel(1) >= 0
%         % decel!!
%         gimbal_ang_vel(1) = gimbal_ang_vel(1) + gimbal_acceleration(1) .* -sign(gimbal_error(1)) * delta_t;
% =======
%     %projected_decel = gimbal_ang_vel.^2 + 2 * diag([gimbal_acceleration * -sign(gimbal_error(1)), gimbal_acceleration * -sign(gimbal_error(2)), 0]) * gimbal_error; %predicting final velocity
%     %projected_decel = diag(sign(projected_decel)) * sqrt(abs(projected_decel));
% 
%     prediction_horizon = 0.01; %look 0.01 seconds into the future
%     predicted_angle = gimbal + gimbal_ang_vel * prediction_horizon;
%     predicted_error = gimbal_goal - predicted_angle;
% 
%     if predicted_error(1) > 0 
%         gimbal_ang_vel(1) = gimbal_ang_vel(1) + gimbal_acceleration * delta_t; 
% >>>>>>> 3f38792a6d954d89f4573c4e68cd85a6d058724e
%     else
%         gimbal_ang_vel(1) = gimbal_ang_vel(1) - gimbal_acceleration * delta_t; 
%     end
% 
%     if predicted_error(2) > 0
%         gimbal_ang_vel(2) = gimbal_ang_vel(2) + gimbal_acceleration * delta_t; 
%     else
%         gimbal_ang_vel(2) = gimbal_ang_vel(2) - gimbal_acceleration * delta_t; 
%     end

    %{
    if abs(projected_distance(1)) >= abs(gimbal_error(1))
        % decel!!
        %gimbal_ang_vel(1) = gimbal_ang_vel(1) + gimbal_acceleration * -sign(gimbal_error(1)) * delta_t;
        gimbal_ang_vel(1) = gimbal_ang_vel(1) + gimbal_acceleration * sign(gimbal_error(1)) * delta_t; %accelerating in direction of sign seems to give the graph we expect?
    else
        % accel!!
        
        gimbal_ang_vel(1) = gimbal_ang_vel(1) + gimbal_acceleration * -sign(gimbal_error(1)) * delta_t; 
        
    end 
    
    
    if abs(projected_distance(2)) >= abs(gimbal_error(2))
        % decel!!
        %gimbal_ang_vel(2) = gimbal_ang_vel(2) + gimbal_acceleration * -sign(gimbal_error(2)) * delta_t;
        gimbal_ang_vel(2) = gimbal_ang_vel(2) + gimbal_acceleration * sign(gimbal_error(2)) * delta_t;
    else
        % accel!!
       
        gimbal_ang_vel(2) = gimbal_ang_vel(2) + gimbal_acceleration * -sign(gimbal_error(2)) * delta_t; 
       
    end
    %}
    

    %Maybe the code below is what we want?
    %gimbal_ang_vel(1) = gimbal_ang_vel(1) + gimbal_acceleration * sign(gimbal_error(1)) * delta_t; 
    %gimbal_ang_vel(2) = gimbal_ang_vel(2) + gimbal_acceleration * sign(gimbal_error(2)) * delta_t; 

    gimbal_ang_vel(1) = clip(gimbal_ang_vel(1), -gimbal_speed, gimbal_speed);
    gimbal_ang_vel(2) = clip(gimbal_ang_vel(2), -gimbal_speed, gimbal_speed);
    

    gimbal = gimbal + gimbal_ang_vel * delta_t;
    gimbal = clip(gimbal, -pi/12, pi/12);
    current_gimbal_accel = (gimbal_ang_vel - prev_gimbal_ang_vel) / delta_t;

    torque = diag([gimbal_top_I; gimbal_bottom_I; 0]) * current_gimbal_accel;

    info = {gimbal; gimbal_ang_vel; torque}; % cell array of gimbal info
end

%% creates a rotation matrix to turn an euler vector into the world frame with a quaternion attitude
function rotate = to_world_frame_quaternion(att)
    rotate = quaternion_to_rotation_matrix(att);
end

%% creates a rotation matrix to turn an euler vector into the body frame with a quaternion attitude
function rotate = to_body_frame_quaternion(att)
    rotate = quaternion_to_rotation_matrix(quaternion_inverse(att));
end

%% takes the inverse of a quaternion
function inv = quaternion_inverse(q)
    if norm(q) ~= 0
        inv = [q(1), -q(2), -q(3), -q(4)] / norm(q);
    else
        inv = [0; 0; 0; 0];
    end
end

%% multiplies two quaternions together
% quaternions are represented as: [w, x, y, z]
function q = quaternion_multiply(q1, q2)
    q = [q1(1)*q2(1)-q1(2)*q2(2)-q1(3)*q2(3)-q1(4)*q2(4);
        q1(1)*q2(2)+q1(2)*q2(1)+q1(3)*q2(4)-q1(4)*q2(3);
        q1(1)*q2(3)-q1(2)*q2(4)+q1(3)*q2(1)+q1(4)*q2(2);
        q1(1)*q2(4)+q1(2)*q2(3)-q1(3)*q2(2)+q1(4)*q2(1)];
end

%% turns a quaternion to an euler angle
% quaternions are represented as: [w, x, y, z]
% eulers are represented as: [x, y, z]
function euler = quaternion_to_euler(q)
        euler = [atan2(2 * (q(1) * q(2) + q(3) * q(4)), (1 - 2 * (q(2) * q(2) + q(3) * q(3))));
                 asin(2 * (q(1) * q(3) - q(4) * q(2)));
                 atan2(2 * (q(1) * q(4) + q(2) * q(3)), (1 - 2 * (q(3) * q(3) + q(4) * q(4))))];
end

%% turns an euler angle into a quaternion
% quaternions are represented as: [w, x, y, z]
% eulers are represented as: [x, y, z]
function q = euler_to_quaternion(euler)
        q = [cos(euler(1) / 2) * cos(euler(2) / 2) * cos(euler(3) / 2) + sin(euler(1) / 2) * sin(euler(2) / 2) * sin(euler(3) / 2);
            sin(euler(1) / 2) * cos(euler(2) / 2) * cos(euler(3) / 2) - cos(euler(1) / 2) * sin(euler(2) / 2) * sin(euler(3) / 2);
            cos(euler(1) / 2) * sin(euler(2) / 2) * cos(euler(3) / 2) + sin(euler(1) / 2) * cos(euler(2) / 2) * sin(euler(3) / 2);
            cos(euler(1) / 2) * cos(euler(2) / 2) * sin(euler(3) / 2) - sin(euler(1) / 2) * sin(euler(2) / 2) * cos(euler(3) / 2)];
end

%% turns a quaternion into a rotation matrix
function mat = quaternion_to_rotation_matrix(q)
    mat = [2*(q(1)*q(1)+q(2)*q(2))-1, 2*(q(2)*q(3)-q(1)*q(4)), 2*(q(2)*q(4)+q(1)*q(3));
        2*(q(2)*q(3)+q(1)*q(4)), 2*(q(1)*q(1)+q(3)*q(3))-1, 2*(q(3)*q(4)-q(1)*q(2));
        2*(q(2)*q(4)-q(1)*q(3)), 2*(q(3)*q(4)+q(1)*q(2)), 2*(q(1)*q(1)+q(4)*q(4))-1];
end

%% makes an extrinsic rotation matrix from a vector of euler angles
%euler in (x,y,z)
function mat = get_extrinsic_rotation_matrix(euler)
    mat = get_extrinsic_z_rotation(euler(3)) * get_extrinsic_y_rotation(euler(2)) * get_extrinsic_x_rotation(euler(1));
end

%% makes an extrinsic rotation matrix about the x axis from an angle value
function mat = get_extrinsic_x_rotation(x)
    mat = [1, 0, 0;
        0, cos(x), -sin(x);
        0, sin(x), cos(x)];
end

%% makes an extrinsic rotation matrix about the y axis from an angle value
function mat = get_extrinsic_y_rotation(y)
    mat = [cos(y), 0, sin(y);
         0, 1, 0;
         -sin(y), 0, cos(y)];
end

%% makes an extrinsic rotation matrix about the z axis from an angle value
function mat = get_extrinsic_z_rotation(z)
    mat = [cos(z), -sin(z), 0;
        sin(z), cos(z), 0;
        0, 0, 1];
end
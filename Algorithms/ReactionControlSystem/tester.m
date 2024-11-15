dt = 0.1;           % Time step (seconds)
t_end = 100;         % Total simulation time (seconds)
time_steps = 0:dt:t_end;  % Time vector

Kp_vals = 0.1:0.1:5;   % Range for proportional gain
Ki_vals = 0.1:0.1:5;   % Range for integral gain
Kd_vals = 0.1:0.1:5;   % Range for derivative gain

stability_weight = 0.7; % Heavier weight to stability
fuel_weight = 0.3;      % Lighter weight to fuel optimization

theta_dot_desired_vals = -2:0.4:2; % Set theta_dot_desired from -2 to 2 with 0.4 increments
results = table(); % Initialize an empty table for storing results

stability_tolerance = 0.01;  % Tolerance for stability (close to 0)
max_time = 500;               % Maximum time to reach stability (in seconds)

% Function to calculate the total impulse and stability penalty for a given PID setting
function [cost, stable, time_to_stabilize] = calculate_cost(Kp, Ki, Kd, time_steps, dt, stability_weight, fuel_weight, theta_dot_desired, stability_tolerance, max_time)
    theta = 0;
    theta_dot = 0.6; % Initial angular velocity
    integral = 0;
    previous_error = 0;
    F1 = 0; F2 = 0; F3 = 0; F4 = 0;
    r = 1.0;
    I = 10.0;
    thrust_max = 2;
    total_impulse = 0;
    
    % Initialize stability check variables
    time_to_stabilize = NaN;
    stable = false;
    
    for t = 1:length(time_steps)
        e_theta_dot = theta_dot_desired - theta_dot;
        integral = integral + e_theta_dot * dt;
        derivative = (e_theta_dot - previous_error) / dt;
        F_total = Kp * e_theta_dot + Kd * derivative + Ki * integral;

        if F_total < 0
            F2 = F2 - F_total / 2;
            F4 = F4 - F_total / 2;
            F1 = 0;
            F3 = 0;
        else
            F1 = F1 + F_total / 2;
            F3 = F3 + F_total / 2;
            F2 = 0;
            F4 = 0;
        end
        
        F1 = min(F1, thrust_max);
        F2 = min(F2, thrust_max);
        F3 = min(F3, thrust_max);
        F4 = min(F4, thrust_max);
        
        total_impulse = total_impulse + sqrt(F_total^2) * dt;
        
        net_torque = r * (F1 - F2 + F3 - F4);
        angular_acceleration = net_torque / I;
        theta_dot = theta_dot + angular_acceleration * dt;
        theta = theta + theta_dot * dt;

        previous_error = e_theta_dot;
        
        % Check stability condition (angular velocity and acceleration close to 0)
        if abs(theta_dot) < stability_tolerance && abs(angular_acceleration) < stability_tolerance && ~stable
            time_to_stabilize = t * dt;  % Record the time taken to stabilize
            stable = true;
        end
        
        % If the system reaches max_time, break early
        if t * dt >= max_time
            break;
        end
    end

    stability_penalty = abs(theta_dot) + abs(theta) + abs(angular_acceleration);
    stability_penalty = stability_penalty / length(time_steps);
    
    cost = stability_weight * stability_penalty + fuel_weight * total_impulse;

    % If the system didn't stabilize within max_time, set stable to false
    if ~stable
        time_to_stabilize = NaN;
    end
end

% Loop through different values of theta_dot_desired and find the optimal PID parameters
for theta_dot_desired = theta_dot_desired_vals
    best_cost = Inf;
    best_Kp = 0;
    best_Ki = 0;
    best_Kd = 0;
    time_to_stabilize_best = NaN;

    for Kp = Kp_vals
        for Ki = Ki_vals
            for Kd = Kd_vals
                [cost, stable, time_to_stabilize] = calculate_cost(Kp, Ki, Kd, time_steps, dt, stability_weight, fuel_weight, theta_dot_desired, stability_tolerance, max_time);
                
                if stable && cost < best_cost
                    best_cost = cost;
                    best_Kp = Kp;
                    best_Ki = Ki;
                    best_Kd = Kd;
                    time_to_stabilize_best = time_to_stabilize;
                end
            end
        end
    end
    
    % Add the best result for the current theta_dot_desired
    new_row = {theta_dot_desired, best_Kp, best_Ki, best_Kd, best_cost, time_to_stabilize_best};
    results = [results; new_row];
    disp(new_row)
end

% Final result table with column names
results.Properties.VariableNames = {'theta_dot_desired', 'Kp', 'Ki', 'Kd', 'Cost', 'Time_to_Stabilize'};
disp(results);

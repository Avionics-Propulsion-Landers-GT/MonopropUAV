dt = 0.1;
t_end = 20;
time_steps = 0:dt:t_end;

Kp_vals = 0.1:0.1:5;
Ki_vals = 0.1:0.1:5;
Kd_vals = 0.1:0.1:5;

best_cost = Inf;
best_Kp = 0;
best_Ki = 0;
best_Kd = 0;

stability_weight = 0.6; % CHANGE THIS DEPENDING ON PROPULSION RECOMMENDATIONS
fuel_weight = 0.4;

function [cost, stable] = calculate_cost(Kp, Ki, Kd, time_steps, dt, stability_weight, fuel_weight)
    theta = 0;
    theta_dot = 0.6;
    theta_dot_desired = 0;
    integral = 0;
    previous_error = 0;
    F1 = 0; F2 = 0; F3 = 0; F4 = 0;
    r = 1.0;
    I = 10.0;
    thrust_max = 2;
    total_impulse = 0;

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
    end

    stability_penalty = abs(theta_dot) + abs(theta) + abs(angular_acceleration);
    stability_penalty = stability_penalty / length(time_steps);
    
    cost = stability_weight * stability_penalty + fuel_weight * total_impulse;
    
    stable = stability_penalty < 0.01;
end

% GRID SEARCH
for Kp = Kp_vals
    for Ki = Ki_vals
        for Kd = Kd_vals
            [cost, stable] = calculate_cost(Kp, Ki, Kd, time_steps, dt, stability_weight, fuel_weight);
            disp([cost, stable])

            if stable && cost < best_cost
                best_cost = cost;
                best_Kp = Kp;
                best_Ki = Ki;
                best_Kd = Kd;
            end
        end
    end
end

fprintf('Best PID Parameters: Kp = %.2f, Ki = %.2f, Kd = %.2f\n', best_Kp, best_Ki, best_Kd);
fprintf('Total Cost (Stability + Fuel): %.2f\n', best_cost);

calculate_cost(best_Kp, best_Ki, best_Kd, time_steps, dt, stability_weight, fuel_weight);

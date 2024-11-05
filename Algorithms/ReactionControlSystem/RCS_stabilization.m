Kp = 1.0;
Kd = 0.2;
Ki = 0.5;

theta_dot = 0.6;
theta_dot_desired = 0;
theta = 0;

F1 = 0;
F2 = 0;
F3 = 0;
F4 = 0;

r = 1.0;
I = 10.0;
thrust_max = 2;

dt = 0.1;
t_end = 20;
time_steps = 0:dt:t_end;

theta_vals = zeros(size(time_steps));
theta_dot_vals = zeros(size(time_steps));
angular_acceleration_vals = zeros(size(time_steps));
F1_vals = zeros(size(time_steps));
F2_vals = zeros(size(time_steps));
F3_vals = zeros(size(time_steps));
F4_vals = zeros(size(time_steps));

previous_error = 0;
integral = 0;

for i = 1:length(time_steps)
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
    
    F1 = min(F1,thrust_max);
    F2 = min(F2,thrust_max);
    F3 = min(F3,thrust_max);
    F4 = min(F4,thrust_max);

    net_torque = r * (F1 - F2 + F3 - F4);
    angular_acceleration = net_torque / I;
    theta_dot = theta_dot + angular_acceleration * dt;
    theta = theta + theta_dot * dt;

    theta_vals(i) = theta;
    theta_dot_vals(i) = theta_dot;
    angular_acceleration_vals(i) = angular_acceleration;
    F1_vals(i) = F1;
    F2_vals(i) = F2;
    F3_vals(i) = F3;
    F4_vals(i) = F4;

    previous_error = e_theta_dot;
end

disp(theta_dot_vals(1:100))

figure;
subplot(5, 1, 1);
plot(time_steps, theta_vals, 'LineWidth', 2);
xlabel('time (s)');
ylabel('roll angle (rad)');
title('roll angle vs. time');
grid on;

subplot(5, 1, 2);
plot(time_steps, theta_dot_vals, 'LineWidth', 2);
xlabel('time (s)');
ylabel('angular Velocity (rad/s)');
title('angular velocity vs. time');
grid on;

subplot(5, 1, 3);
plot(time_steps, F1_vals, 'r', 'LineWidth', 2); hold on;
plot(time_steps, F2_vals, 'g', 'LineWidth', 2);
xlabel('time (s)');
ylabel('thrust (N)');
title('thruster forces vs. time');
legend('F1', 'F2');
grid on;

subplot(5, 1, 4);
plot(time_steps, F3_vals, 'b', 'LineWidth', 2); hold on;
plot(time_steps, F4_vals, 'm', 'LineWidth', 2);
xlabel('time (s)');
ylabel('thrust (N)');
title('thruster forces vs. Time');
legend('F3', 'F4');
grid on;

subplot(5, 1, 5);
plot(time_steps, angular_acceleration_vals, 'LineWidth', 2);
xlabel('time (s)');
ylabel('angular acceleration (rad/s^2)');
title('angular acceleration vs. time');
grid on;
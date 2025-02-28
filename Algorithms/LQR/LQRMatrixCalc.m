%% Initialization
syms psi theta phi psi_dot theta_dot phi_dot real; 
% angular state variables

% psi - about x 
% theta - about y
% phi - about z
% global reference frame is x y z

syms r_x r_y r_z v_x v_y v_z real;
% linear state variables

full_state = [r_x r_y r_z v_x v_y v_z psi theta phi psi_dot theta_dot phi_dot]';
position = [r_x r_y r_z]';
velocity = [v_x v_y v_z]';
angle = [psi theta phi]';
angular_vel = [psi_dot theta_dot phi_dot]';

% state vector and other useful vectors

syms a_x a_y a_z phi_dot_dot theta_dot_dot psi_dot_dot real;
% values to be used in the differentiated state vector

syms T a b a_dot b_dot a_dot_dot b_dot_dot real;
% input vector variables 

thrust = [T*cos(b)*sin(a) T*sin(b) -T*cos(b)*cos(a)]';
% thrust in the body frame, assuming +z is the direction of the attitude of
% the monoprop

full_input = [T a b a_dot b_dot a_dot_dot b_dot_dot];
% full input matrix, used to take partials

syms f cd area real
% density, coefficient of drag and reference area

syms rc_x rc_y rc_z real;
% distance from the COM to the COP.

rc = [rc_x rc_y rc_z]';

syms rt_x rt_y rt_z real;
% distance from the COM to the Thrust

rt = [rt_x rt_y rt_z];

syms Ixx Iyy Izz Ixy Ixz Iyz real;
% inertia values of the entire system

inertia = [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz];

syms Ixx_s Iyy_s Izz_s Ixy_s Ixz_s Iyz_s real
% inertia values of the "static" elements (non-TVC) in body frame

inertia_s = [Ixx_s Ixy_s Ixz_s; Ixy_s Iyy_s Iyz_s; Ixz_s Iyz_s Izz_s];
% inertia "tensor" of the static elements, in the body frame

syms Ixx_a Iyy_a Izz_a Ixy_a Ixz_a Iyz_a real;
% inertia values of the upper TVC element in reference to the gimbal axis
% in the body frame

inertia_a = [Ixx_a Ixy_a Ixz_a; Ixy_a Iyy_a Iyz_a; Ixz_a Iyz_a Izz_a];

syms Ixx_b Iyy_b Izz_b Ixy_b Ixz_b Iyz_b real;
% same as inertia_b but for the lower TVC element

inertia_b = [Ixx_b Ixy_b Ixz_b; Ixy_b Iyy_b Iyz_b; Ixz_b Iyz_b Izz_b];

%% Rotation, Coordinate Transformations
% Define rotation matrices
R_x = [1 0 0; 0 cos(psi) -sin(psi); 0 sin(psi) cos(psi)];
R_y = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
R_z = [cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0; 0 0 1];

% Composite rotation (from WORLD FRAME to BODY FRAME)
R_bf = simplify(R_z * R_y * R_x);
% Inverse (from BODY FRAME to WORLDFRAME)
R_wf = simplify(R_bf)';

%% Main Calculation - Angular Component
% convert initial necessary state variables into body frame

vel_BF = R_bf*velocity;
angular_vel_BF = R_bf*angular_vel;

% Drag Forces
Fd_x = (1/2)*f*cd*area*norm(vel_BF)*vel_BF(1);
Fd_y = (1/2)*f*cd*area*norm(vel_BF)*vel_BF(2);
Fd_z = (1/2)*f*cd*area*norm(vel_BF)*vel_BF(3);
Fd = [Fd_x Fd_y Fd_z];

% Torque from Drag Forces and Thrust, Body Frame
aero_torque = simplify(cross(rc, Fd)); 
thrust_torque = simplify(cross(rt, thrust));

torque_net_body = aero_torque + thrust_torque;
% torque_net_body is 1x3

ang_accel_BF = (inv(inertia_s))*(torque_net_body');
% ang_accel_WF = R_wf*(ang_accel_BF);

% Convert torque from world frame to body frame
torque_net_world = R_wf*(torque_net_body');
% Torque from gyroscopic precession, World Frame
% converting inertia tensors into the world frame
inertia_s_WF = R_wf*(inertia_s)*(R_wf');
inertia_a_WF = R_wf*(inertia_a)*(R_wf');
inertia_b_WF = R_wf*(inertia_b)*(R_wf');

gyro_torque_s = cross(angular_vel, (inertia_s_WF)*(angular_vel));
%gyroscopic torque of the system, excluding the a and b 
ang_vel_a_BF = [a_dot 0 0]';
ang_vel_b_BF = [0 b_dot 0]';
ang_vel_a_WF = R_wf*(ang_vel_a_BF) + angular_vel;
ang_vel_b_WF = R_wf*(ang_vel_b_BF) + angular_vel;
gyro_torque_a = cross(ang_vel_a_WF, (inertia_a_WF)*(ang_vel_a_WF));
gyro_torque_b = cross(ang_vel_b_WF, (inertia_b_WF)*(ang_vel_b_WF));

% torque caused on the TVC causing moment on the entire rigid body
ang_accel_a_BF = [a_dot_dot 0 0]';
ang_accel_b_BF = [0 b_dot_dot 0]';
m_torque_a = inertia_a*(ang_accel_a_BF);
m_torque_b = inertia_b*(ang_accel_b_BF);
m_torque_a_WF = R_wf*m_torque_a
m_torque_b_BF = R_wf*m_torque_b

torque_net_world = torque_net_world - (gyro_torque_s + gyro_torque_a + ...
    gyro_torque_b) + m_torque_a_WF + m_torque_b_BF;

ang_accel_WF = inv(inertia)*(torque_net_world);

A4 = jacobian(ang_accel_WF, (full_state'));
B4 = jacobian(ang_accel_WF, (full_input'));

%% Linear Acceleration

%% Initialization
syms vel m;
g = [0 0 -9.80665]; % gravity, world frame

thrust_WF = R_wf*thrust;
Fd_WF = R_wf*(Fd');
accel_WF = (1/m)*(thrust_WF - Fd_WF) + g';
% 3x1

A2 = jacobian((accel_WF'), (full_state'));
B2 = jacobian((accel_WF'), (full_input'));

%% Trivial Matrix Sections
A1 = [zeros([3,3]) eye(3) zeros([3,6])];
A3 = [zeros([3,9]) eye(3)];
B1 = [zeros([3,7])];
B3 = [zeros([3,7])];

A = [A1 ; A2 ; A3 ; A4]
B = [B1 ; B2; B3; B4]

%% Conversion to C++ (possibly??)

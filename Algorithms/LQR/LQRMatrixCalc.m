%% Initialization
syms psiVar thetaVar phiVar psiVar_dot thetaVar_dot phiVar_dot real; 
% angular state variables

% psiVar - about x 
% thetaVar - about y
% phiVar - about z
% global reference frame is x y z

syms r_x r_y r_z v_x v_y v_z real;
% linear state variables

full_state = [r_x r_y r_z v_x v_y v_z psiVar thetaVar phiVar psiVar_dot thetaVar_dot phiVar_dot]';
position = [r_x r_y r_z]';
velocity = [v_x v_y v_z]';
angle = [psiVar thetaVar phiVar]';
angular_vel = [psiVar_dot thetaVar_dot phiVar_dot]';

% state vector and other useful vectors

syms a_x a_y a_z phiVar_dot_dot thetaVar_dot_dot psiVar_dot_dot real;
% values to be used in the differentiated state vector

syms T a b a_dot b_dot a_dot_dot b_dot_dot real;
% input vector variables 

thrust = [T*cos(b)*sin(a) T*sin(b) -T*cos(b)*cos(a)]';
% thrust in the body frame, assuming +z is the direction of the attitude of
% the monoprop

full_input = [T a b a_dot b_dot a_dot_dot b_dot_dot];
% full input matrix, used to take partials

syms f cDrag areaVar real
% density, coefficient of drag and reference areaVar

syms rc_x rc_y rc_z real;
% distance from the (combined) COM to the COP.

rc = [rc_x rc_y rc_z]';

syms rt_x rt_y rt_z real;
% distance from the (combined) COM to the Thrust

rt = [rt_x rt_y rt_z]';

syms Ixx Iyy Izz Ixy Ixz Iyz real;
% inertia values of the entire system 

inertia = [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz];

syms Ixx_a Iyy_a Izz_a Ixy_a Ixz_a Iyz_a real;
% inertia values of the upper TVC element in reference to the +Z axis
% in the body frame

inertia_a = [Ixx_a Ixy_a Ixz_a; Ixy_a Iyy_a Iyz_a; Ixz_a Iyz_a Izz_a];

syms Ixx_b Iyy_b Izz_b Ixy_b Ixz_b Iyz_b real;
% same as inertia_b but for the lower TVC element (rel to +Z)

inertia_b = [Ixx_b Ixy_b Ixz_b; Ixy_b Iyy_b Iyz_b; Ixz_b Iyz_b Izz_b];


%% Rotation, Coordinate Transformations
% Define rotation matrices
R_x = [1 0 0; 0 cos(psiVar) -sin(psiVar); 0 sin(psiVar) cos(psiVar)];
R_y = [cos(thetaVar) 0 sin(thetaVar); 0 1 0; -sin(thetaVar) 0 cos(thetaVar)];
R_z = [cos(phiVar) -sin(phiVar) 0; sin(phiVar) cos(phiVar) 0; 0 0 1];

% Composite rotation (from WORLD FRAME to BODY FRAME)
R_bf = simplify(R_z * R_y * R_x);
% Inverse (from BODY FRAME to WORLDFRAME)
R_wf = simplify(R_bf)';

%% Main Calculation - Angular Component
% convert initial necessary state variables into body frame

vel_BF = R_bf*velocity;
angular_vel_BF = R_bf*angular_vel;

% Drag Forces
Fd_x = -(1/2)*f*cDrag*areaVar*norm(vel_BF)*vel_BF(1);
Fd_y = -(1/2)*f*cDrag*areaVar*norm(vel_BF)*vel_BF(2);
Fd_z = -(1/2)*f*cDrag*areaVar*norm(vel_BF)*vel_BF(3);
Fd = [Fd_x Fd_y Fd_z]';


% Torque from Drag Forces and Thrust, Body Frame
aero_torque = simplify(cross(rc, Fd)); 
thrust_torque = simplify(cross(rt, -thrust));


% torque caused on the TVC causing moment on the entire rigid body
ang_accel_a_BF = [a_dot_dot 0 0]';
ang_accel_b_BF = [0 b_dot_dot 0]';
m_torque_a = -inertia_a*(ang_accel_a_BF); % Questionable, minus sign?
m_torque_b = -inertia_b*(ang_accel_b_BF); % Questionable, minus sign?

torque_net_body = aero_torque + thrust_torque + m_torque_a + m_torque_b;
torque_net_world = R_wf*(torque_net_body);

ang_accel_WF = simplify(inv(inertia) * torque_net_world);

A4 = jacobian(ang_accel_WF, (full_state'));
B4 = jacobian(ang_accel_WF, (full_input'));

%% Linear Acceleration

%% Initialization
syms m;
g = [0 0 -9.80665]; % gravity, world frame

thrust_WF = -R_wf*thrust;
Fd_WF = R_wf*(Fd);
% Fl_WF = R_wf*(Fl);
Fg = m*g';
accel_WF = (1/m)*(thrust_WF + Fd_WF + Fg);
% 3xf

A2 = jacobian((accel_WF'), (full_state'));
B2 = jacobian((accel_WF'), (full_input'));

%% Trivial Matrix Sections
A1 = [zeros([3,3]) eye(3) zeros([3,6])];
A3 = [zeros([3,9]) eye(3)];
B1 = [zeros([3,7])];
B3 = [zeros([3,7])];

% Continuous time A and B matrices -> final output, input to c++ code. 
A = [A1 ; A2 ; A3 ; A4];
B = [B1 ; B2; B3; B4];

C = eye(12);
D = 0;

matlabFunction(A, 'File', 'calculateA', ...
    'Vars', {m, f, cDrag, areaVar, full_state, full_input, rc, rt, inertia, inertia_a, inertia_b}, ...
    'Optimize', true, 'Outputs', {'A_out'});

% disp('function created for A calculation');

matlabFunction(B, ...
    'File', 'calculateB', ...
    'Vars', {m, f, cDrag, areaVar, full_state, full_input, rc, rt, inertia, inertia_a, inertia_b}, ...
    'Optimize', true, ...
    'Outputs', {'B_out'});


% codegen calculateA -args {0, 0, 0, 0, zeros(12,1), zeros(1,7), zeros(3,1), zeros(1,3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3)} -lang:c++
% codegen calculateB -args {0, 0, 0, 0, zeros(12,1), zeros(1,7), zeros(3,1), zeros(1,3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3)} -lang:c++
% disp('cpp files created');

% converting to discrete time, using zero order hold and timestep t.
% syms t real; 
% sys_c = ss(A, B, C, D);
% sys_d = c2d(sys_c, t, 'zoh');
% A_d = sys_d.A
% B_d = sys_d.B

%% Conversion to C++ (????)

%% Numeric substution. (???)

%% Write symbolic matrices to txt file (???

%% COST FUNCTION (DISCRETE TIME, INFINITE HORIZON)

% define Q (state cost) and R (input cost) matrices
% Q = eye(12);
% R = eye(7);
% 
% [K, S, P] = dlqr(A_d, B_d, Q, R);
% this currently does not work because we dont have numeric A and B
% matrices
% must be called at every timestep because A and B are NOT invariant
% K - optimal gain matrix (F in the wikipedia LQR inf horizon disc time section)
% S - P - solution to the discrete algebraic riccati equation
% P - poles - I don't know what this means
% see: https://www.mathworks.com/help/control/ref/lti.dlqr.html
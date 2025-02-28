% Symbolic representation of state vector variables and input vector 
% variables: position (r_), velocity (v_), angle (an_), 
% angular velocity (anv_), thrust (t), gimbal angle (gan_), gimbal angle 
% velocity (gav_), gimbal angle acceleration (gac_)
syms rx ry rz vx vy vz anx any anz anvx anvy anvz t ganp gany gavp gavy gacp gacy

% General formula for finite-horizon, discrete time
    % J = xT_Hp * Q_Hp * x_Hp + sum(V)
    % Where Hp is the time horizon the optimal control sequence minimizing 
    % the performance; Capital T denotes transpose
    % V = subs(f, k, 0:(Hp-1))
    % f = xT_k * Q * x_K + uT_k * R * u_k + 2 * xT_k * N * u_k
    % u_k = -F_k * x_k
    % F_k = [(R + BT * P_{k+1} * B) ^ {-1}] * [BT * P_{k+1} * A + NT]
    % P_{k-1} = AT * P_k * A - (AT * P_k * B + N) * [(R + BT * P_k * B) ^
    % {-1}] * (BT * P_k * A + NT) + Q
    % P is unknown, nxn, and symmetric. A, B, Q, R are known, real 
    % coefficient matrices. Q and R are symmetric. We define A and B as
    % part of our state space model, and x and u are the state and input
    % vectors used. Q is the state cost matrix, R the control cost
    % matrix, and N the cross-term (error from a combination of both
    % control and state) cost matrix. N, Q, and R are defined by us, most
    % likely through trial and error.
    %
    % V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V
    % Q has positive entries along its diagonal corresponding to the cost
    % of deviating from each state. The rest of its entries are 0. Larger Q
    % value ==> higher penalty of error. R works similarly to Q, where
    % larger R values ==> higher penalty for using a certain input ==>
    % minimizes the use of that input. We can start with the identity
    % matrix for both Q and R then tune it from there.
    % 
    % At each timestep t, the optimal control input is calculated using a 
    % series of for loops (where we run each for loop ~N times) that spit 
    % out the u (i.e. control inputs) that corresponds to the minimal 
    % overall cost. LQR should take in 7 inputs: actual state (x), desired 
    % state (xf), Q, R, A, B, and dt.
    % 
    % For our desired state, when we are in the initial path following
    % stage, we want to make sure desired velocity is NOT 0. Only have
    % desired velocity be 0 when we are approaching the end of the path for
    % hovering. Desired angle I think should correspond to taking the
    % tangent along the path at that point (?) / is the angle in the
    % direction of the path. 
    % ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^
    %
    % Resources:
    % 1) https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator
    % 2) https://www.sjsu.edu/ae/docs/project-thesis/Alex.Ganbold-Su23.pdf
    % 3) https://lewisgroup.uta.edu/ee4314/lectures/Lecture%2024.pdf
    % 4) https://automaticaddison.com/linear-quadratic-regulator-lqr-with-python-code-example/

% Need to figure out how to translate our SSM into C++ 
% In MatLab specifically, we can use LQR(A, B, Q, R) to tune for K and P
% Might actually want to use infinite horizon instead of finite so we can
% solve for steady state (hovering - but also might be impossible to get a 
% real steady state bc gravity + wind make it "impossible" to be 
% fully steady?), and we don't have a definite time interval we are solving 
% for; we don't know how long exactly everything will take. 
%
% Overall procedure:
    % 1) We have already found A and B 
    % 2) Choose Q and R (will tune later, start both as identity matrices)
    % 3) Solve riccati equation for P
    % 4) Compute optimal gain matrix K (or F_K)
    % 5) Only 1 of the solutions of K will result in steady state
x   = [rx; ry; rz; vx; vy; vz; anx; any; anz; anvx; anvy; anvz];
xT  = transpose(x);
u   = [t; ganp; gany; gavp; gavy; gacp; gacy];
uT  = transpose(u);

% Python code from resource 4 for discrete-time, convert to MatLab
% LQR(x, xf, Q, R, A, B, dt):
    % x_error = x - xf
    % N = ## (arbitrary integer for number of loops to run)
    % P = [empty list of N + 1 elements]
    % P[N] = Q
    % for i = N to 1:
        % P[i-1] = discrete time algebraic riccati equation
    % K = [empty list of N elements]
    % u = [empty list of N elements]
    % for i = 0 to N - 1:
        % K[i] = (i think this refers to F_k in the fh, discrete time eq.)
    % for i = 0 to N - 1
        % u[i] = K[i] * x_error
            % We do N iterations of the loop until we get a stable value
            % for the optimal control. We assume optimal control is
            % established after the Nth iteration. We adjust N until we get
            % stable results.
    % u_star = u[N-1] %%% THIS IS THE OPTIMAL CONTROL INPUT
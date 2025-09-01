# The NONLINEAR MPC Algorithm 

# A python implementation of NONLINEAR Model Predictive Control purposed for active real time control.
# Written from scratch since libraries are too big
# Algorithm to be translated to Rust
# Hansel Zhang

# High Level Overview
# 1. Start from current state x_0 and warm-start control sequence U = [u_0, u_1, ..., u_N-1]
# 2. Simulate the dynamics under control inputs U to get nominal trajectory x_1, x_2, ..., x_N
# 3. Linearize dynamics along nominal trajectory by computing the Jacobians A_k, B_k for each time step k
# 4. Form a quadratic program (QP) that approximates the nonlinear optimal control via QUADRATIC cost and linear dynamics.
# 5. Solve the QP to get optimal control increments dU
# 6. Apply a line search on the true nonlinear cost to ensure there is a decrease in cost. Accept the new control inputs.
# 7. Use the first control input u_0 in the control sequence U and shift the sequence to act as a warm start for the next iteration.

import numpy as np
import matplotlib.pyplot as plt

# ---  DYNAMICS --- #

# To change the dynamics to represent a TVC rocket. For now it is just quadcopter dynamics as an example.
def dynamics(x, u):
    # 3D Quadcopter (6DOF) simplified nonlinear MIMO system
    # x = [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]
    # u = [u1, u2, u3, u4] (thrusts from 4 rotors)
    m = 1.0      # mass (kg)
    g = 9.81     # gravity (m/s^2)
    l = 0.25     # arm length (m)
    Ixx = 0.02   # moment of inertia x (kg*m^2)
    Iyy = 0.02   # moment of inertia y (kg*m^2)
    Izz = 0.04   # moment of inertia z (kg*m^2)
    dt = 0.05    # integration step

    # Unpacking states
    x_pos, y_pos, z_pos, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r = x
    u1, u2, u3, u4 = u

    # Clamp thrusts to be non-negative
    u1 = np.maximum(u1, 0.0)
    u2 = np.maximum(u2, 0.0)
    u3 = np.maximum(u3, 0.0)
    u4 = np.maximum(u4, 0.0)

    # Total thrust and torques (standard quadcopter configuration)
    thrust = u1 + u2 + u3 + u4
    tau_phi   = l * (u2 - u4)
    tau_theta = l * (u3 - u1)
    tau_psi   = 0.01 * (u1 - u2 + u3 - u4)  # simplified yaw moment

    # Rotation matrix from body to world
    cphi = np.cos(phi); sphi = np.sin(phi)
    cth = np.cos(theta); sth = np.sin(theta)
    cpsi = np.cos(psi); spsi = np.sin(psi)
    R = np.array([
        [cth * cpsi, sphi * sth * cpsi - cphi * spsi, cphi * sth * cpsi + sphi * spsi],
        [cth * spsi, sphi * sth * spsi + cphi * cpsi, cphi * sth * spsi - sphi * cpsi],
        [-sth,       sphi * cth,                      cphi * cth]
    ])

    # Acceleration in world frame
    acc = (R @ np.array([0, 0, thrust / m])) - np.array([0, 0, g])
    x_ddot, y_ddot, z_ddot = acc

    # Angular rates derivatives (Euler angles, small angle approx for simplicity)
    p_dot = (tau_phi - (Izz - Iyy) * q * r) / Ixx
    q_dot = (tau_theta - (Ixx - Izz) * p * r) / Iyy
    r_dot = (tau_psi - (Iyy - Ixx) * p * q) / Izz

    # Euler angle derivatives
    phi_dot = p + sphi * np.tan(theta) * q + cphi * np.tan(theta) * r
    theta_dot = cphi * q - sphi * r
    psi_dot = sphi / cth * q + cphi / cth * r

    # Euler integration
    x_pos_new = x_pos + dt * x_dot
    y_pos_new = y_pos + dt * y_dot
    z_pos_new = z_pos + dt * z_dot
    phi_new = phi + dt * phi_dot
    theta_new = theta + dt * theta_dot
    psi_new = psi + dt * psi_dot
    x_dot_new = x_dot + dt * x_ddot
    y_dot_new = y_dot + dt * y_ddot
    z_dot_new = z_dot + dt * z_ddot
    p_new = p + dt * p_dot
    q_new = q + dt * q_dot
    r_new = r + dt * r_dot

    x_new = np.array([
        x_pos_new, y_pos_new, z_pos_new,
        phi_new, theta_new, psi_new,
        x_dot_new, y_dot_new, z_dot_new,
        p_new, q_new, r_new
    ])

    return x_new

def compute_jacobian(x, u, eps=None):
    x = np.asarray(x, dtype=float)
    u = np.asarray(u, dtype=float)
    n = x.size
    m = u.size

    if eps is None:
        # scale with magnitude to avoid cancellation
        eps_x = 1e-6 * (1.0 + np.linalg.norm(x))
        eps_u = 1e-6 * (1.0 + np.linalg.norm(u))
    else:
        eps_x = eps_u = float(eps)

    A = np.zeros((n, n))
    B = np.zeros((n, m))

    f0 = dynamics(x, u)

    # central difference for states
    for i in range(n):
        dx = np.zeros(n); dx[i] = eps_x
        f_plus = dynamics(x + dx, u)
        f_minus = dynamics(x - dx, u)
        A[:, i] = (f_plus - f_minus) / (2 * eps_x)

    # central difference for inputs
    for j in range(m):
        du = np.zeros(m); du[j] = eps_u
        f_plus = dynamics(x, u + du)
        f_minus = dynamics(x, u - du)
        B[:, j] = (f_plus - f_minus) / (2 * eps_u)

    return A, B

def rollout(x0, U):
    xs = [np.array(x0, dtype=float)]
    x = xs[0].copy()
    for k in range(len(U)):
        x = dynamics(x, U[k])   # discrete next-state map (what your dynamics returns)
        xs.append(x.copy())
    return np.array(xs)   # shape (N+1, n)

def linearize_trajectory(xs, U):
    A_seq, B_seq = [], []
    for k in range(len(U)):
        A_k, B_k = compute_jacobian(xs[k], U[k])
        A_seq.append(A_k)
        B_seq.append(B_k)
    return A_seq, B_seq

# --- BUILD PREDICTION MATRICES --- #

def build_Su(A_seq, B_seq):
    # Building the time-varying prediction matrix Su

    n = A_seq[0].shape[0]
    m = B_seq[0].shape[1]
    N = len(A_seq)
    Su = np.zeros((n*N, m*N))

    # For each row block k (predict x_{k+1}), and each control j <= k,
    # block = (A_{k-1} * A_{k-2} * ... * A_{j+1}) * B_j  (product is identity when j==k)
    for k in range(N):
        for j in range(k+1):
            # compute Phi = A_{k-1} * ... * A_{j+1}
            Phi = np.eye(n)
            if j + 1 <= k:  # if there is at least one A to multiply
                for t in range(j+1, k):
                    Phi = Phi @ A_seq[t]
            # if j == k, Phi = I; block = B_seq[j]
            block = Phi @ B_seq[j]
            Su[k*n:(k+1)*n, j*m:(j+1)*m] = block
    return Su

# --- BUILD QP INCREMENT --- #
def assemble_qp_increment(xs, U, xref_traj, A_seq, B_seq, Q, R, QN, u_min, u_max):
    # Assemble the QP increment for the NMPC.
    # H = Su^T * Q_bar * Su + Rbar
    # g = Su^T * Q_bar * r 
    N = len(U)
    n = xs.shape[1]
    m = U.shape[1] if U.ndim > 1 else 1

    # stack Qbar
    Q_blocks = [Q]*(N-1) + [QN]
    Q_bar = np.block([[Q_blocks[i] if i==j else np.zeros_like(Q) for j in range(N)] for i in range(N)])
    R_bar = np.kron(np.eye(N), R)

    # residual r = [x1 - xref1; ... ; xN - xrefN]
    r = (xs[1:] - xref_traj[1:]).reshape(-1)


    Su = build_Su(A_seq, B_seq)
    H = Su.T @ Q_bar @ Su + R_bar
    g = Su.T @ Q_bar @ r


    # bounds on delta U
    Ustack = U.reshape(-1)
    U_max = np.tile(u_max, N)
    U_min = np.tile(u_min, N)

    # print("Ustack shape: ", Ustack.shape)
    # print("U_min shape: ", U_min.shape)
    # print("U_max shape: ", U_max.shape)

    dU_min = U_min - Ustack
    dU_max = U_max - Ustack

    H = 0.5*(H + H.T) + 1e-8*np.eye(H.shape[0])  # ensure symmetry and positive definiteness

    return H, g, dU_min, dU_max, Su, Q_bar, R_bar

    # --- SOLVE QP PROBLEM --- #

# OPTION A: PROJECTED GRADIENT DESCENT ('pgd')
def solve_qp_box(H, f, u_min, u_max, N, u_init=None, max_iter=50, alpha=0.01):
    """
    Solve min 0.5 U^T H U + f^T U
    subject to U_min <= U <= U_max
    using simple projected gradient descent with backtracking line search.
    """
    n = f.size
    U_min = u_min
    U_max = u_max

    if u_init is None:
        U = np.zeros(n)
    else:
        U = u_init.copy().reshape(-1)
        U = np.minimum(np.maximum(U, U_min), U_max)  # project to bounds

    for _ in range(max_iter):
        grad = H @ U + f
        print("Gradient Norm:", np.linalg.norm(grad))
        alpha_ls = alpha
        cost_old = 0.5 * U.T @ H @ U + f.T @ U
        while alpha_ls > 1e-6:
            U_new = U - alpha_ls * grad
            U_new = np.minimum(np.maximum(U_new, U_min), U_max)  # project
            cost_new = 0.5 * U_new.T @ H @ U_new + f.T @ U_new
            if cost_new < cost_old:
                U = U_new
                break
            alpha_ls *= 0.5

    return U

# OPTION B: QUADRATIC PROGRAMMING SOLVER (BOX CONSTRAINTS ONLY) ('active_set')
def solve_qp_active_set(H, f, u_min, u_max, N, U_init=None, max_iter=100):
    """
    Solve min 0.5 U^T H U + f^T U
    subject to U_min <= U <= U_max
    using an active set method.
    """

    # U_max = np.tile(u_max, N)
    # U_min = np.tile(u_min, N)
    U_max = u_max #u_max and u_min are already stacked
    U_min = u_min

    n = H.shape[0]
    if U_init is None:
        U = np.zeros(n)
    else:
         U = np.minimum(np.maximum(U_init, np.tile(u_min, n // len(u_min))), 
                       np.tile(u_max, n // len(u_max)))  # Project to bounds

    G = np.vstack((np.eye(n), -np.eye(n)))
    h = np.hstack((U_max, -U_min))

    # initialize active set (constraints that are right)
    active = []
    tol = 1e-8
    for i in range(max_iter):

        # identify active constraints
        active = [j for j in range(len(h)) if abs(G[j] @ U - h[j]) < tol]

        # solve equality-constrained QP
        if active:
            G_A = G[active, :]

            KKT = np.block([
                [H, G_A.T],
                [G_A, np.zeros((len(active), len(active)))]
            ])

            rhs = -np.hstack((f, G_A @ U - h[active]))

            # Solve the KKT system
            sol = np.linalg.solve(KKT, rhs)

            U_new = sol[:n]
        else:
            # If no active constraints, solve unconstrained QP
            U_new = np.linalg.solve(H, -f)

        # Project the solution onto the feasible set
        U_new = np.minimum(np.maximum(U_new, U_min), U_max)

        # Check for convergence
        if np.linalg.norm(U_new - U) < 1e-6:
            return U_new
        U = U_new

    return U

# DEFINE TRUE COST
def true_cost(x0, U, xref_traj, Q, R, QN):
    xs_try = rollout(x0, [U[k:k+1] for k in range(len(U))]) if isinstance(U, np.ndarray) and U.ndim==1 else rollout(x0, U)
    xs_try = np.array(xs_try)
    # sum stage costs
    cost = 0.0
    for k in range(len(U)):
        ex = xs_try[k+1] - xref_traj[k+1]
        cost += ex.T @ Q @ ex
        uk = U[k] if isinstance(U[k], np.ndarray) else np.array([U[k]])
        cost += uk.T @ R @ uk
    # terminal
    eN = xs_try[-1] - xref_traj[-1]
    cost += eN.T @ QN @ eN
    return float(cost)

# NMPC STEP
def nmpc_step(x0, U_init, xref_traj, Q, R, QN, u_min, u_max, sqp_iters=3, alpha_pgd=0.1):
    N = len(U_init)
    m = U_init.shape[1] if U_init.ndim==2 else 1
    # flatten warm start into shape (N, m)
    U = U_init.copy()
    for it in range(sqp_iters):
        xs = rollout(x0, [U[k] for k in range(N)])
        xs = np.array(xs)  # shape (N+1, n)
        A_seq, B_seq = linearize_trajectory(xs, [U[k] for k in range(N)])
        H, g, dUmin, dUmax, Su, Qbar, Rbar = assemble_qp_increment(xs, U, xref_traj, A_seq, B_seq, Q, R, QN, u_min, u_max)
        # solve for delta U (stacked)
        dU0 = np.zeros_like(g)  # warm start zero increments
        # dU = solve_qp_active_set(H, g, dUmin, dUmax, N, U_init=dU0, max_iter=20)
        dU = solve_qp_box(H, g, dUmin, dUmax, N, u_init=dU0, max_iter=20)
        # line-search on nonlinear cost
        flatU = U.reshape(-1)
        J0 = true_cost(x0, U, xref_traj, Q, R, QN)
        step = 1.0
        while step > 1e-3:
            U_try_flat = flatU + step * dU
            U_try = U_try_flat.reshape(U.shape)
            # project nominal U_try to bounds
            U_try = np.minimum(np.maximum(U_try, np.tile(u_min, (N,1))), np.tile(u_max, (N,1)))
            Jtry = true_cost(x0, U_try, xref_traj, Q, R, QN)
            if Jtry < J0:
                U = U_try
                break
            step *= 0.5
        # convergence
        if np.linalg.norm(dU, ord=np.inf) < 1e-4:
            break
    return U, xs

# --- MAIN MPC FUNCTION --- #
if __name__ == "__main__":
    
    # QUADCOPTER PROBLEM SETUP
    # problem sizes
    n = 12  # [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]
    m = 4   # [u1, u2, u3, u4] (thrusts from 4 rotors)
    N = 15
    T = 30
    dt = 0.1
    iters = int(T / dt)

    # initial state: at origin, level, stationary
    x = np.zeros(n)

    # hover at set point
    xref = np.zeros(n)
    xref[0] = np.random.uniform(-4, 4) 
    xref[1] = np.random.uniform(-4, 4)   
    xref[2] = np.random.uniform(-4, 4)  
    xref_traj = np.tile(xref, (N+1, 1))

    # warm start: hover thrust (each rotor supports 1/4 of weight)
    m_quadcopter = 1.0
    g = 9.81
    hover_thrust = m_quadcopter * g / 4.0
    U_warm = np.ones((N, m)) * hover_thrust

    # costs: penalize position, angles, velocities
    Q = np.diag([100.0, 100.0, 200.0, 10.0, 10.0, 10.0, 5.0, 5.0, 10.0, 1.0, 1.0, 1.0])
    R = np.eye(m) * 0.1
    QN = Q * 1.0

    # bounds on thrusts (N)
    u_min = np.zeros(m)
    u_max = np.ones(m) * 20.0

    xs_hist = []
    us_hist = []

    for k in range(iters):
        # sqp iterations typically 2 or 3. Has been reduced to 1 for speed.
        U_opt, xs = nmpc_step(x, U_warm, xref_traj, Q, R, QN, u_min, u_max, sqp_iters=1, alpha_pgd=0.05)
        u_apply = U_opt[0].copy()
        # apply first control
        x = dynamics(x, u_apply)
        # warm start shift
        U_warm = np.vstack([U_opt[1:], U_opt[-1:]])
        xs_hist.append(x.copy())
        us_hist.append(u_apply.copy())
        print(f"t={k*dt:.2f} x={x} u={u_apply}")

    xs_hist = np.array(xs_hist)
    us_hist = np.array(us_hist).reshape(-1, m)

    # Plot all states and control inputs with labels and titles

    time = np.arange(xs_hist.shape[0]) * dt

    # Plot positions
    plt.figure(figsize=(12, 8))
    plt.subplot(3, 1, 1)
    plt.plot(time, xs_hist[:, 0], label='x (m)')
    plt.plot(time, xs_hist[:, 1], label='y (m)')
    plt.plot(time, xs_hist[:, 2], label='z (m)')
    plt.legend()
    plt.grid()
    plt.title("Quadcopter Position")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")

    # Plot angles (convert to degrees)
    plt.subplot(3, 1, 2)
    plt.plot(time, np.rad2deg(xs_hist[:, 3]), label='phi (deg)')
    plt.plot(time, np.rad2deg(xs_hist[:, 4]), label='theta (deg)')
    plt.plot(time, np.rad2deg(xs_hist[:, 5]), label='psi (deg)')
    plt.legend()
    plt.grid()
    plt.title("Quadcopter Angles")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")

    # Plot control inputs (thrusts)
    plt.subplot(3, 1, 3)
    for i in range(us_hist.shape[1]):
        plt.plot(time, us_hist[:, i], label=f'u{i+1} (thrust)')
    plt.legend()
    plt.grid()
    plt.title("Control Inputs (Rotor Thrusts)")
    plt.xlabel("Time (s)")
    plt.ylabel("Thrust (N)")

    plt.tight_layout()
    plt.show()

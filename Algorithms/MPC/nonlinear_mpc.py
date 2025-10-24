# The NONLINEAR MPC Algorithm 

# A python implementation of NONLINEAR Model Predictive Control purposed for active real time control.
# Written from scratch since most solver libraries are not available in Rust.
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
# import rocketdynamics as rd
import rocketdynamics_plus as rd

# --- LINEARIZATION --- #

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

    f0 = rd.dynamics(x, u)

    # central difference for states
    for i in range(n):
        dx = np.zeros(n); dx[i] = eps_x
        f_plus = rd.dynamics(x + dx, u)
        f_minus = rd.dynamics(x - dx, u)
        A[:, i] = (f_plus - f_minus) / (2 * eps_x)

    # central difference for inputs
    for j in range(m):
        du = np.zeros(m); du[j] = eps_u
        f_plus = rd.dynamics(x, u + du)
        f_minus = rd.dynamics(x, u - du)
        B[:, j] = (f_plus - f_minus) / (2 * eps_u)

    return A, B

def rollout(x0, U):
    xs = [np.array(x0, dtype=float)]
    x = xs[0].copy()
    for k in range(len(U)):
        x = rd.dynamics(x, U[k])   # discrete next-state map (what your dynamics returns)
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
        # print("Gradient Norm:", np.linalg.norm(grad))
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
def nmpc_step(x0, U_init, xref_traj, Q, R, QN, u_min, u_max, sqp_iters=1, alpha_pgd=0.1):
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
    
    # TVC ROCKET PROBLEM SETUP
    # problem sizes
    n = 17  # [x, y, z, qw, qx, qy, qz, gimbal_x, gimbal_y, gimbal_x_rate, gimbal_y_rate, vx, vy, vz, wx, wy, wz]
    m = 3   # [gimbal_theta, gimbal_phi, thrust]
    N = 10
    T = 30.0
    dt = 0.1
    iters = int(T / dt)

    # dynamics parameters are defined in rocketdynamics.dynamics()
    """
    Rocket dynamics with gimbal control
    
    Parameters:
    x: state vector [rx, ry, rz, qw, qx, qy, qz, gimbal_x, gimbal_y, 
                     gimbal_x_rate, gimbal_y_rate, vx, vy, vz, wx, wy, wz]
    u: control input [gimbal_x_cmd, gimbal_y_cmd, thrust]
    dt: time step (s)
    """
    # initial state: at origin, level, stationary, quaternion [0,0,0,1]
    x = np.zeros(n)
    x[6] = 1.0  # qw = 1 (unit quaternion)

    # hover at set point
    xref = np.zeros(n)
    xref[0] = 2
    xref[1] = -3
    xref[2] = 10
    xref[6] = 1.0  # reference orientation: level (unit quaternion)
    # Create a reference trajectory that changes halfway through the simulation
    xref_traj = np.tile(xref, (N+1, 1))

    # warm start: hover thrust (thrust = mass * gravity, gimbal angles = 0)
    m_rocket = 1.0
    g = 9.81
    hover_thrust = m_rocket * g
    U_warm = np.zeros((N, m))
    U_warm[:, 2] = hover_thrust  # thrust
    # gimbal_theta and gimbal_phi are zero (upright)

    # costs: penalize position, orientation, velocities, angular rates
    Q = np.diag([
        70.0, 70.0, 200.0,   # position x, y, z
        200.0, 200.0, 200.0, 200.0, # quaternion qx, qy, qz, qw
        50.0, 50.0, 50.0,        # linear velocities x_dot, y_dot, z_dot
        5.0, 5.0, 5.0,          # angular velocities wx, wy, wz
        2, 2, 5, 5      # gimbal_x, gimbal_y, gimbal_x_rate, gimbal_y_rate
    ])
    R = np.diag([400.0, 400, 0.1])  # penalize gimbal angles and thrust

    
    QN = Q * 1.0

    # bounds on control inputs
    gimbal_limit = np.deg2rad(10)  # +/- 10 degrees
    thrust_min = 1.0 # prevent flame out
    thrust_max = 20.0
    u_min = np.array([-gimbal_limit, -gimbal_limit, thrust_min])
    u_max = np.array([gimbal_limit, gimbal_limit, thrust_max])

    xs_hist = []
    us_hist = []

    t_switch = 15  # time step to switch reference

    for k in range(iters):
        
        if k*dt > t_switch:  # t_switch is the time you want to change reference
            xref[0] = 0
            xref[1] = 0
            xref[2] = 0
            xref_traj = np.tile(xref, (N+1, 1))
            # change costs. Thrust has to be weaker to allow for descent, whilst it had to be greater than weight force for ascent, so the penalty scheme must change.
            Q = np.diag([
                70.0, 70.0, 200.0,   # position x, y, z
                200.0, 200.0, 200.0, 200.0, # quaternion qx, qy, qz, qw
                50.0, 50.0, 50.0,        # linear velocities x_dot, y_dot, z_dot
                5.0, 5.0, 5.0,          # angular velocities wx, wy, wz
                2, 2, 5, 5      # gimbal_x, gimbal_y, gimbal_x_rate, gimbal_y_rate
            ])
            R = np.diag([500.0, 500.0, 0.1])  # penalize gimbal angles and thrust
        
        # sqp iterations typically 2 or 3. Has been reduced to 1 for speed (sacrificing accuracy).
        U_opt, xs = nmpc_step(x, U_warm, xref_traj, Q, R, QN, u_min, u_max, sqp_iters=1, alpha_pgd=0.05)
        u_apply = U_opt[0].copy()
        # apply first control
        x = rd.dynamics(x, u_apply)
        # warm start shift
        U_warm = np.vstack([U_opt[1:], U_opt[-1:]])
        xs_hist.append(x.copy())
        us_hist.append(u_apply.copy())
        print(f"t={k*dt:.2f} x={x[:3]} u={u_apply}")

    xs_hist = np.array(xs_hist)
    us_hist = np.array(us_hist).reshape(-1, m)

    # Plot all states and control inputs with labels and titles

    time = np.arange(xs_hist.shape[0]) * dt

    # Plot positions
    plt.figure(figsize=(12, 10))
    plt.subplot(4, 1, 1)
    plt.plot(time, xs_hist[:, 0], label='x (m)')
    plt.plot(time, xs_hist[:, 1], label='y (m)')
    plt.plot(time, xs_hist[:, 2], label='z (m)')
    plt.legend()
    plt.grid()
    plt.title("Rocket Position")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")

    # Plot quaternion components
    plt.subplot(4, 1, 2)
    plt.plot(time, xs_hist[:, 3], label='qx')
    plt.plot(time, xs_hist[:, 4], label='qy')
    plt.plot(time, xs_hist[:, 5], label='qz')
    plt.plot(time, xs_hist[:, 6], label='qw')
    plt.legend()
    plt.grid()
    plt.title("Rocket Quaternion (Attitude)")
    plt.xlabel("Time (s)")
    plt.ylabel("Quaternion")

    # Plot linear and angular velocities
    plt.subplot(4, 1, 3)
    plt.plot(time, xs_hist[:, 7], label='x_dot (m/s)')
    plt.plot(time, xs_hist[:, 8], label='y_dot (m/s)')
    plt.plot(time, xs_hist[:, 9], label='z_dot (m/s)')
    plt.plot(time, xs_hist[:, 10], label='wx (rad/s)')
    plt.plot(time, xs_hist[:, 11], label='wy (rad/s)')
    plt.plot(time, xs_hist[:, 12], label='wz (rad/s)')
    plt.legend()
    plt.grid()
    plt.title("Rocket Velocities")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity")

    # Plot control inputs: gimbal angles and thrust
    plt.subplot(4, 1, 4)
    plt.plot(time, np.rad2deg(us_hist[:, 0]), label='Gimbal Theta (deg)')
    plt.plot(time, np.rad2deg(us_hist[:, 1]), label='Gimbal Phi (deg)')
    plt.plot(time, us_hist[:, 2], label='Thrust (N)')
    plt.legend()
    plt.grid()
    plt.title("Control Inputs")
    plt.xlabel("Time (s)")
    plt.ylabel("Input")

    plt.tight_layout()
    plt.show()

'''
    # --- ANIMATION --- #
    import matplotlib.animation as animation

    def quaternion_to_rotation_matrix(q):
        qx, qy, qz, qw = q
        R = np.array([
            [1 - 2*qy**2 - 2*qz**2,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw,     1 - 2*qx**2 - 2*qz**2,     2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw,         2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
        ])
        return R

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-10, 10])
    ax.set_ylim([-10, 10])
    ax.set_zlim([0, 25])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Rocket Animation')
    ax.set_box_aspect([1,1,1])

    rocket_body, = ax.plot([], [], [], 'o-', lw=3, color='blue', label='Rocket')
    setpoint, = ax.plot([], [], [], 'rx', markersize=12, label='Setpoint')
    traj, = ax.plot([], [], [], 'g--', lw=1, label='Trajectory')

    def set_axes_equal(ax):
        #Set 3D plot axes to equal scale.
        x_limits = ax.get_xlim3d()
        y_limits = ax.get_ylim3d()
        z_limits = ax.get_zlim3d()
        x_range = abs(x_limits[1] - x_limits[0])
        x_middle = np.mean(x_limits)
        y_range = abs(y_limits[1] - y_limits[0])
        y_middle = np.mean(y_limits)
        z_range = abs(z_limits[1] - z_limits[0])
        z_middle = np.mean(z_limits)
        plot_radius = 0.5 * max([x_range, y_range, z_range])
        ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
        ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
        ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

    def init():
        rocket_body.set_data([], [])
        rocket_body.set_3d_properties([])
        setpoint.set_data([], [])
        setpoint.set_3d_properties([])
        traj.set_data([], [])
        traj.set_3d_properties([])
        return rocket_body, setpoint, traj

    def draw_rocket(pos, quat, length=2.0):
        # Draw a line from pos in the direction of the rocket's body z-axis
        R = quaternion_to_rotation_matrix(quat)
        z_axis = R @ np.array([0, 0, 1])
        start = pos
        end = pos + z_axis * length
        return np.vstack([start, end])

    def animate(i):
        pos = xs_hist[i, 0:3]
        quat = xs_hist[i, 3:7]
        rocket_line = draw_rocket(pos, quat)
        rocket_body.set_data(rocket_line[:, 0], rocket_line[:, 1])
        rocket_body.set_3d_properties(rocket_line[:, 2])
        setpoint.set_data([xref[0]], [xref[1]])
        setpoint.set_3d_properties([xref[2]])
        traj.set_data(xs_hist[:i+1, 0], xs_hist[:i+1, 1])
        traj.set_3d_properties(xs_hist[:i+1, 2])
        plt.draw()
        return rocket_body, setpoint, traj

    ani = animation.FuncAnimation(
        fig, animate, frames=len(xs_hist), interval=dt * 1000, blit=False, init_func=init
    )

    ax.legend()
    plt.show()
'''
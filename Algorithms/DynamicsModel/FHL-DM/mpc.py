import numpy as np
import matplotlib.pyplot as plt
import sympy as sp
from sympy.utilities.lambdify import lambdify
from matplotlib import rcParams
import casadi as ca
import do_mpc
from params import RocketParams

# ------------------------------------------------------------------
# 1. Define the dynamics symbolically in SymPy. Copied from ABSymbolicCalc.py which may become deprecated.

def calculate_f_nonlinear_sym():
    # Define symbols
    theta, phi = sp.symbols('theta phi', real=True)
    theta_d, phi_d = sp.symbols('theta_dot phi_dot', real=True)

    r_x, r_y, r_z = sp.symbols('r_x r_y r_z', real=True)
    v_x, v_y, v_z = sp.symbols('v_x v_y v_z', real=True)

    T, R1, R2, a, b = sp.symbols('T R1 R2 a b', real=True)
    rho, C_d, A_ref, m, g = sp.symbols('rho C_d A_ref m g', real=True)

    # Rotation matrices: Only need Ry and Rz now (no psi)
    R_y = sp.Matrix([
        [sp.cos(theta), 0, sp.sin(theta)],
        [0, 1, 0],
        [-sp.sin(theta), 0, sp.cos(theta)]
    ])
    R_z = sp.Matrix([
        [sp.cos(phi), -sp.sin(phi), 0],
        [sp.sin(phi),  sp.cos(phi), 0],
        [0, 0, 1]
    ])
    R_wb = R_y.T * R_z.T  # World to body frame

    # World-frame velocity
    v_w = sp.Matrix([v_x, v_y, v_z])
    v_b = R_wb * v_w

    # Drag in body frame
    vel_mag_b = sp.sqrt(v_b.dot(v_b) + 1e-8)
    F_d_b = -0.5 * rho * C_d * A_ref * vel_mag_b * v_b

    # Gravity in world frame
    F_g_w = sp.Matrix([0, 0, -m * g])
    F_g_b = R_wb * F_g_w

    # Thrust (body frame)
    F_thrust_b = sp.Matrix([
        T * sp.cos(b) * sp.sin(a),
        T * sp.sin(b),
        -T * sp.cos(b) * sp.cos(a)
    ])

    # Net force in body frame
    F_net_b = F_thrust_b + F_d_b + F_g_b
    a_b = F_net_b / m

    # Final state derivative in body frame
    f_sym = sp.Matrix([
        v_x, v_y, v_z,
        *a_b,
        theta_d, phi_d,
        0, 0  # angular accels stubbed
    ])
    x_sym = sp.Matrix([r_x, r_y, r_z, v_x, v_y, v_z, theta, phi, theta_d, phi_d])
    u_sym = sp.Matrix([T, R1, R2, a, b])

    return f_sym, x_sym, u_sym


def initialize_mpc_with_physics_model(params: RocketParams):
    print("Initializing MPC (physics-based)...")

    model = do_mpc.model.Model('continuous')

    # State: [x, y, z, vx, vy, vz, theta, phi, theta_dot, phi_dot]
    x = model.set_variable('_x', 'x', shape=(10, 1))
    u = model.set_variable('_u', 'u', shape=(5, 1))  # [T, R1, R2, a, b]

    # Extract state and input variables
    pos = x[0:3]
    vel = x[3:6]
    theta, phi = x[6], x[7]
    theta_d, phi_d = x[8], x[9]
    R1, R2 = u[1], u[2]
    a, b = u[3], u[4]
    thrust_mag = u[0]

    # Rotation matrix (YX extrinsic only — yaw removed)
    cp, sp = ca.cos(phi), ca.sin(phi)
    ct, st = ca.cos(theta), ca.sin(theta)

    Ry = ca.vertcat(
        ca.horzcat(ct, 0, st),
        ca.horzcat(0,  1, 0),
        ca.horzcat(-st, 0, ct)
    )
    Rx = ca.vertcat(
        ca.horzcat(1, 0, 0),
        ca.horzcat(0, cp, -sp),
        ca.horzcat(0, sp,  cp)
    )
    R_bf = ca.mtimes([Rx, Ry])
    R_wf = R_bf.T

    # Gimbal (extrinsic X then Y)
    Ra = ca.vertcat(
        ca.horzcat(1, 0, 0),
        ca.horzcat(0, ca.cos(a), -ca.sin(a)),
        ca.horzcat(0, ca.sin(a),  ca.cos(a))
    )
    Rb = ca.vertcat(
        ca.horzcat(ca.cos(b), 0, ca.sin(b)),
        ca.horzcat(0, 1, 0),
        ca.horzcat(-ca.sin(b), 0, ca.cos(b))
    )
    R_gimbal = ca.mtimes([Rb, Ra])
    ez = ca.DM([0, 0, 1])
    F_thrust_b = thrust_mag * ca.mtimes(R_gimbal, ez)

    # Drag
    rho, A, Cd = params.air_density, params.A_z, params.Cd_z
    vel_b = ca.mtimes(R_bf, vel)
    v_b_norm = ca.sqrt(ca.fmax(ca.dot(vel_b, vel_b), 1e-8))
    F_drag_b = -0.5 * rho * Cd * A * v_b_norm * vel_b

    # Net force
    F_gravity_w = ca.DM([0, 0, -params.m * 9.80665])
    F_gravity_b = ca.mtimes(R_bf, F_gravity_w)  # transform gravity to body frame

    F_net_b = F_thrust_b + F_drag_b + F_gravity_b
    accel_b = F_net_b / params.m
    accel = ca.mtimes(R_wf, accel_b)  # transform back to world frame

    # Torques
    tau_rcs = ca.DM([0, 0, params.rcs_offset * (R1 - R2)])
    r_thrust = ca.DM(params.thrust_offset + params.gimbal_offset)
    r_drag = ca.DM(params.COP_offset)
    T_thrust = ca.cross(r_thrust, F_thrust_b)
    T_drag = ca.cross(r_drag, F_drag_b)
    torque_b = tau_rcs + T_thrust + T_drag
    torque_b = ca.vertcat(torque_b[0], torque_b[1], 0.0)  # Remove yaw torque

    # Angular acceleration: pitch (y) and roll (x) only
    ang_accel = ca.mtimes(params.inv_I, torque_b)[0:2]  # x, y only

    # Assemble dx/dt
    dxdt = ca.vertcat(
        vel,
        accel,
        ca.vertcat(theta_d, phi_d),
        ang_accel
    )

    model.set_rhs('x', dxdt)

    # Debug: validate dynamics
    print(">> Evaluating dxdt at test state for debug...")
    x_bad = ca.DM([0.0, 0.0, -50.0, 0.0, 0.0, -13.0, 0.0, 0.0, 0.0, 0.0])
    u_test = ca.DM([1.0, 0.0, 0.0, 0.0, 0.0])
    try:
        rhs_func = ca.Function('rhs_func', [model.x, model.u], [dxdt])
        result = rhs_func(x_bad, u_test)
        print("✅ dxdt evaluated successfully.")
        print("Result:", np.array(result).flatten())
    except Exception as e:
        print("❌ ERROR evaluating dxdt:", e)

    model.set_expression('y', x)
    model.setup()

    # MPC controller
    mpc_ctrl = do_mpc.controller.MPC(model)
    mpc_ctrl.set_param(n_horizon=10, t_step=params.dt, state_discretization='collocation')

    Q = np.diag([1.0, 1.0, 100.0, 0.5, 0.5, 50.0, 1.0, 1.0, 1.0, 1.0])
    R_vec = np.array([1e-4, 0.1, 0.1, 1.0, 1.0])
    R = np.diag(R_vec)
    x_ref = ca.DM([0, 0, 1] + [0]*7)
    err = x - x_ref
    mpc_ctrl.set_objective(
        lterm=ca.mtimes([err.T, Q, err]) + ca.mtimes([u.T, R, u]),
        mterm=ca.mtimes([err.T, Q, err])
    )

    mpc_ctrl.bounds['lower','_u','u'] = np.array([0.0, 0.0, 0.0, -np.pi/12, -np.pi/12])
    mpc_ctrl.bounds['upper','_u','u'] = np.array([15.0, 1.0, 1.0,  np.pi/12,  np.pi/12])
    mpc_ctrl.set_rterm(u=R_vec)
    mpc_ctrl.setup()

    return mpc_ctrl




def run_mpc(f_sym_dt, x, u):
    print("Running MPC with f_sym_dt, debug")
    return np.array(f_sym_dt(x.reshape(12,1), u)).flatten()
    
if __name__ == "__main__":
    # Example usage
    f_sym, x_sym, u_sym = calculate_f_nonlinear_sym()
    mpc = initialize_mpc(x_sym, u_sym, f_sym)
    print("Integrated testing completed.")
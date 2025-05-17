'''THIS FILE IS BASED ON THE MPC JUPYTER NOTEBOOK. ALL CHANGES MUST BE MADE THERE FIRST.'''
'''THIS IS INTEGRATED WITH THE FHL DYNAMICS MODEL/SIMULATOR.'''
'''AUTHOR: JUSTIN E.'''

import numpy as np
import matplotlib.pyplot as plt
import sympy as sp
from sympy.utilities.lambdify import lambdify
from matplotlib import rcParams
import casadi as ca
import do_mpc

'''Helpers and constants should be defined here.'''

# Inertia Tensors (numeric, gimbals unused)
I = np.matrix([[1.0, 0.0, 0.0],
               [0.0, 1.0, 0.0],
               [0.0, 0.0, 1.0]])
I_s = np.matrix([[1.0, 0.0, 0.0],
               [0.0, 1.0, 0.0],
               [0.0, 0.0, 1.0]])
I_a = np.matrix([[0.5, 0.0, 0.0],
               [0.0, 0.5, 0.0],
               [0.0, 0.0, 0.5]])
I_b = np.matrix([[0.5, 0.0, 0.0],
               [0.0, 0.5, 0.0],
               [0.0, 0.0, 0.5]])

# COP / COT / COM Offsets (numeric)
# TODO: add gimbal offset, make rc and rt time dependent (since COM is moving on the rocket due to fuel consumption)
rc = np.array([0.0, 0.0, 0.1])  # cop offset
rt = np.array([0.0, 0.0, -0.1])  # cot offset
r_rcs = 0.05  # rcs offset

g = 9.81  # gravity
m_0 = 1.0  # starting mass. 
m_min = 0.1 # minimum mass.
rho = 1.225  # air density
A = 0.1  # cross-sectional area
alpha = 0.005 # mass flow rate coefficient (determine empirically, given by propulsion team)

def initialize_mpc():
    # First, create the continuous time state space model.
    # define equations of motion
    # x_d = f(x,u), where f is some nonlinear function. 

    model = do_mpc.model.Model('continuous')

    # Linear states
    r = model.set_variable('_x', 'r', shape=(3,1))
    r_dot = model.set_variable('_x', 'r_dot', shape=(3,1))
    # Angular states
    omega = model.set_variable('_x', 'omega', shape=(3,1))
    omega_dot = model.set_variable('_x', 'omega_dot', shape=(3,1))
    # Mass state
    m = model.set_variable('_x', 'm', shape=(1,1))
    # Control inputs
    T = model.set_variable('_u', 'T')
    a = model.set_variable('_u', 'a')
    b = model.set_variable('_u', 'b')
    R1 = model.set_variable('_u', 'R1') # clockwise - positive, only one direction
    R2 = model.set_variable('_u', 'R2') # counterclockwise - negative, only one direction

    # Mass flow rate
    rhs_m = -alpha*T

    # Rotational Motion
    # first, create rotation matrices from euler angles
    cphi, sphi   = ca.cos(omega[2]),   ca.sin(omega[2]) # roll
    cth,  sth    = ca.cos(omega[0]), ca.sin(omega[0]) # pitch
    cpsi, spsi   = ca.cos(omega[1]),   ca.sin(omega[1]) # yaw

    R_z = ca.blockcat([[ cphi, -sphi, 0],
                    [ sphi,  cphi, 0],
                    [    0,     0, 1]])

    R_y = ca.blockcat([[ cth, 0, sth],
                    [   0, 1,   0],
                    [-sth, 0, cth]])

    R_x = ca.blockcat([[1, 0,    0],
                    [0, cpsi, -spsi],
                    [0, spsi,  cpsi]])

    R_bf = ca.mtimes(R_z, R_y)
    R_bf = ca.mtimes(R_bf, R_x)
    R_wf = R_bf.T

    zero_err = np.full(3, 1e-6) # small number to avoid division by zero

    model.set_expression('AoA', ca.acos(ca.dot(r_dot+zero_err, np.array([0, 0, 1]))/ca.norm_2(r_dot+zero_err))) # angle of attack in radians
    AoA = model.aux['AoA']
    model.set_expression('C_dx', -0.449*(ca.cos(3.028*AoA*180/np.pi)) + 0.463)
    model.set_expression('C_dy', -0.449*(ca.cos(3.028*AoA*180/np.pi)) + 0.463)
    model.set_expression('C_dz', -0.376*(ca.cos(5.675*AoA*180/np.pi)) + 1.854)
    C_dx = model.aux['C_dx']
    C_dy = model.aux['C_dy']
    C_dz = model.aux['C_dz']

    # first, calculate torques in body.
    #v velocities in body frame, needed for drag calculation.
    r_dot_bf = ca.mtimes(R_bf, r_dot)

    # TODO: update relevant A and C_d values for each axis.
    F_d_bf = ca.vertcat(
        -0.5*rho*A*C_dx*(ca.fabs(r_dot_bf[0]))*r_dot_bf[0],
        -0.5*rho*A*C_dy*(ca.fabs(r_dot_bf[1]))*r_dot_bf[1],
        -0.5*rho*A*C_dz*(ca.fabs(r_dot_bf[2]))*r_dot_bf[2]
    )

    F_t_bf = ca.vertcat(
        T*ca.cos(b)*ca.sin(a),
        T*ca.sin(a),
        T*ca.cos(b)*ca.cos(a)
    )

    F_g_wf = ca.vertcat(
        0,
        0,
        -m*g
    )
    F_g_bf = ca.mtimes(R_bf, F_g_wf) 

    # Net force and linear in body frame
    F_net_bf = F_t_bf + F_d_bf + F_g_bf

    # Acceleration in world frame
    F_net_wf = ca.mtimes(R_wf, F_net_bf)
    rhs_r_dot = ca.mtimes(1/m, F_net_wf)

    # Net torque in body frame
    tau_d_bf = ca.cross(rc, F_d_bf)
    tau_t_bf = ca.cross(rt, F_t_bf)
    tau_rcs_bf = r_rcs*(R1 - R2)
    tau_net_bf = tau_d_bf + tau_t_bf + tau_rcs_bf
    # Net torque in world frame
    tau_net_wf = ca.mtimes(R_wf, tau_net_bf)
    I_wf = ca.mtimes(R_wf, I)
    I_wf = ca.mtimes(I_wf, R_wf.T)
    I_wf_inv = ca.inv(I_wf)

    # angular acceleration in world frame
    rhs_omega_dot = ca.mtimes(I_wf_inv, tau_net_wf)

    #Translational Velocity
    rhs_r = ca.vertcat(
        r_dot[0],                                                            # ẋ = v_x
        r_dot[1],                                                            # ẏ = v_y
        r_dot[2],                                                            # ż = v_z
    )

    # Rotational Velocity
    rhs_omega = ca.vertcat(
        omega[0],                                                            # φ̇ = ω_x
        omega[1],                                                            # θ̇ = ω_y
        omega[2],                                                            # ψ̇ = ω_z
    )


    model.set_rhs('r', rhs_r)
    model.set_rhs('r_dot', rhs_r_dot)
    model.set_rhs('omega', rhs_omega)
    model.set_rhs('omega_dot', rhs_omega_dot)
    model.set_rhs('m', rhs_m)

    # defining reference variables
    r_ref = model.set_variable('_tvp', 'r_ref', shape=(3,1))
    r_dot_ref = model.set_variable('_tvp', 'r_dot_ref', shape=(3,1))
    omega_ref = model.set_variable('_tvp', 'omega_ref', shape=(3,1))
    omega_dot_ref = model.set_variable('_tvp', 'omega_dot_ref', shape=(3,1))
    m_ref = model.set_variable('_tvp', 'm_ref', shape=(1,1))

    model.setup()

    # 2. Create the MPC controller itself.
    mpc = do_mpc.controller.MPC(model)
    setup_mpc = {
        # automatically discretized with collocation
        'n_horizon': 5,
        't_step': 0.1,
        'n_robust': 0, # default to 0 if not specified. I don't know what this does.
        'store_full_solution': True,
    }
    mpc.set_param(**setup_mpc)

    # Define cost function.
    # Cost Function. Will be defined as a quadratic function
    # error between desired and actual state as well as control effort.
    x_ref = r_ref[0]  # desired x position
    x_dot_ref = r_dot_ref[0]  # desired x velocity
    y_ref = r_ref[1]  # desired y position
    y_dot_ref = r_dot_ref[1]  # desired y velocity
    z_ref = r_ref[2]  # desired altitude
    z_dot_ref = r_dot_ref[2]  # desired vertical velocity
    m_ref = m_0  # desired mass (use least amount of fuel possible)

    phi_ref = omega_ref[0]  # desired pitch angle
    theta_ref = omega_ref[1]  # desired yaw angle
    psi_ref = omega_ref[2]  # desired roll angle
    phi_dot_ref = omega_dot_ref[0]  # desired pitch rate
    theta_dot_ref = omega_dot_ref[1]  # desired yaw rate
    psi_dot_ref = omega_dot_ref[2]  # desired roll rate

    dx = r[0] - x_ref
    dx_dot = r_dot[0] - x_dot_ref
    dy = r[1] - y_ref
    dy_dot = r_dot[1] - y_dot_ref
    dz = r[2] - z_ref
    dz_dot = r_dot[2] - z_dot_ref

    dphi = omega[0] - phi_ref
    dphi_dot = omega_dot[0] - phi_dot_ref
    dtheta = omega[1] - theta_ref
    dtheta_dot = omega_dot[1] - theta_dot_ref
    dpsi = omega[2] - psi_ref
    dpsi_dot = omega_dot[2] - psi_dot_ref

    dm = m - m_ref

    err_vec = ca.vertcat(
        dx,
        dx_dot,
        dy,
        dy_dot,
        dz,
        dz_dot,
        dphi,
        dphi_dot,
        dtheta,
        dtheta_dot,
        dpsi,
        dpsi_dot,
        dm
    )
    # Cost matrix for state.
    Q = np.diag([1.0, 1.0, 2.5, # xyz position state penalty
                1.0, 1.0, 3.0, # xyz velocity state penalty
                6.0, 6.0, 2.0, # pitch, yaw, roll angle penalty
                3.0, 3.0, 2.0, # pitch, yaw, roll rate penalty
                0.01 # mass penalty. Low because if its too high its not gonna work.
                ])

    # m_term is mayer term and l_term is lagrange term. 
    # TODO: figure out what these are.
    m_term = ca.mtimes(err_vec.T, Q)  # quadratic term
    m_term = ca.mtimes(m_term, err_vec)  # quadratic term
    l_term = m_term

    mpc.set_objective(mterm=m_term, lterm=l_term)
    mpc.set_rterm(T=0.1, a=0.2, b=0.2, R1 = 0.1, R2 = 0.1)  # control effort penalty

    tvp_template = mpc.get_tvp_template()

    # define general desired state(s)
    # to be followed at (mostly) all times
    r_dot_ref_num = np.array([0.0, 0.0, 0.0])
    omega_ref_num = np.array([0.0, 0.0, 0.0])
    omega_dot_ref_num = np.array([0.0, 0.0, 0.0])

    # define ascent phase desired state(s)
    r_ref_num_hi_hover = np.array([0.0, 0.0, 5.0])
    x_ref_num_ascent = np.concatenate((r_ref_num_hi_hover, np.array([0.0,0.0,1.0]), omega_ref_num, omega_dot_ref_num, m_ref/2), axis=None)
    # reference velocity is 1 m/s in the z direction for ascent phase.

    # define hi hover phase desired state(s)
    x_ref_num_hi_hover = np.concatenate((r_ref_num_hi_hover, r_dot_ref_num, omega_ref_num, omega_dot_ref_num, m_min), axis=None)

    # define descent phase desired state(s)
    # TAKE PRECAUTION, DO NOT SET Z VELOCITY TO BE TOO LOW, IT WILL CRASH INTO THE GROUND.
    r_ref_num_lo_hover = np.array([0.0, 0.0, 0.1])
    x_ref_num_descent = np.concatenate((r_ref_num_lo_hover, np.array([0.0,0.0,-0.5]), omega_ref_num, omega_dot_ref_num, m_min), axis=None)

    # define lo hover phase desired state(s)
    x_ref_num_lo_hover = np.concatenate((np.array([0.0,0.0,0.0]), r_dot_ref_num, omega_ref_num, omega_dot_ref_num, m_min), axis=None)

    def tvp_fun(t_now):
        # define the reference trajectory here
        # Ascend for 10 seconds, hover for 5, descend in 7.5, then hover for the remainder.
        # Simulation time should be longer than this for best results.
        if t_now < 10:
            x_ref_num = x_ref_num_ascent
        elif t_now < 15:
            x_ref_num = x_ref_num_hi_hover
        elif t_now < 22.5:
            x_ref_num = x_ref_num_descent
        else:
            x_ref_num = x_ref_num_lo_hover
        tvp_template['_tvp', :] = x_ref_num
        # tvp_template['_tvp', :] = x_ref_num_hi_hover
        return tvp_template

    mpc.set_tvp_fun(tvp_fun)

    mpc.bounds['lower', '_x', 'r'] = -np.inf
    mpc.bounds['upper', '_x','r'] = np.inf
    mpc.bounds['lower', '_x', 'r_dot'] = -np.inf
    mpc.bounds['upper', '_x', 'r_dot'] = np.inf
    mpc.bounds['lower', '_x', 'omega'] = -np.inf
    mpc.bounds['upper', '_x', 'omega'] = np.inf
    mpc.bounds['lower', '_x', 'omega_dot'] = -np.inf
    mpc.bounds['upper', '_x', 'omega_dot'] = np.inf
    mpc.bounds['lower', '_x', 'm'] = m_min  # mass cannot be less than minimum mass (dry mass)
    mpc.bounds['upper', '_x', 'm'] = m_0  # mass cannot be higher than initial mass (wet mass)
    mpc.bounds['lower', '_u', 'a'] = -np.pi/12  # gimbal angle cannot be too low
    mpc.bounds['upper', '_u', 'a'] = np.pi/12  # gimbal angle cannot be too high
    mpc.bounds['lower', '_u', 'b'] = -np.pi/12  # gimbal angle cannot be too low
    mpc.bounds['upper', '_u', 'b'] = np.pi/12  # gimbal angle cannot be too high
    mpc.bounds['upper', '_u', 'T'] = 15.0  # thrust cannot be too high
    mpc.bounds['lower', '_u', 'T'] = 0.0  # thrust cannot be lower than burnout thrust
    mpc.bounds['lower', '_u', 'R1'] = 0  # rcs cannot be negative
    mpc.bounds['upper', '_u', 'R1'] = 1  # max rcs1 thrust
    mpc.bounds['lower', '_u', 'R2'] = 0  # rcs cannot be negative
    mpc.bounds['upper', '_u', 'R2'] = 1  # max rcs2 thrust

    mpc.settings.supress_ipopt_output()
    # Scaling can be done, will not be done here because I don't know how it works or if its needed.
    mpc.setup()

    return mpc
    

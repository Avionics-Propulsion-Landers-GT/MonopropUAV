# -------------------------------------------------------
# Greek identifiers (ψ, θ, φ, …) replaced with ASCII names
# Justin E, Future Hovering Lander, 2025.


import sympy as sp

def calculate_state_matrices_sym():
    """Return symbolic LQR A, B matrices (12 × 12 and 12 × 7)."""
    # ------------------------------------------------------------------
    # 1. State symbols (angular + translational)
    # ------------------------------------------------------------------
    psi,   theta,   phi   = sp.symbols('psi theta phi', real=True)
    psi_d, theta_d, phi_d = sp.symbols('psi_dot theta_dot phi_dot', real=True)
    psi_dd, theta_dd, phi_dd = sp.symbols('psi_dd theta_dd phi_dd', real=True)

    r_x, r_y, r_z = sp.symbols('r_x r_y r_z', real=True)
    v_x, v_y, v_z = sp.symbols('v_x v_y v_z', real=True)

    # Inputs: thrust magnitude + gimbal (a, b) & derivatives
    T, a, b, a_d, b_d, a_dd, b_dd = sp.symbols('T a b a_dot b_dot a_ddot b_ddot', real=True)

    # Aerodynamic / physical constants
    rho, C_d, A_ref = sp.symbols('rho cDrag areaVar', real=True)
    m = sp.symbols('m', real=True)

    # Offsets
    r_cx, r_cy, r_cz = sp.symbols('rc_x rc_y rc_z', real=True)
    r_tx, r_ty, r_tz = sp.symbols('rt_x rt_y rt_z', real=True)

    # Inertia tensor components
    Ixx,Iyy,Izz,Ixy,Ixz,Iyz = sp.symbols('Ixx Iyy Izz Ixy Ixz Iyz', real=True)
    Ixx_s,Iyy_s,Izz_s,Ixy_s,Ixz_s,Iyz_s = sp.symbols('Ixx_s Iyy_s Izz_s Ixy_s Ixz_s Iyz_s', real=True)
    Ixx_a,Iyy_a,Izz_a,Ixy_a,Ixz_a,Iyz_a = sp.symbols('Ixx_a Iyy_a Izz_a Ixy_a Ixz_a Iyz_a', real=True)
    Ixx_b,Iyy_b,Izz_b,Ixy_b,Ixz_b,Iyz_b = sp.symbols('Ixx_b Iyy_b Izz_b Ixy_b Ixz_b Iyz_b', real=True)

    # ------------------------------------------------------------------
    # 2. Assemble state & input vectors
    # ------------------------------------------------------------------
    full_state = sp.Matrix([
        r_x, r_y, r_z,
        v_x, v_y, v_z,
        psi, theta, phi,
        psi_d, theta_d, phi_d
    ])

    velocity_wf  = sp.Matrix([v_x, v_y, v_z])
    ang_vel_wf   = sp.Matrix([psi_d, theta_d, phi_d])

    full_input = sp.Matrix([T, a, b, a_d, b_d, a_dd, b_dd])

    # Offsets & inertia matrices
    r_c = sp.Matrix([r_cx, r_cy, r_cz])
    r_t = sp.Matrix([r_tx, r_ty, r_tz])

    I      = sp.Matrix([[Ixx,  Ixy,  Ixz],
                        [Ixy,  Iyy,  Iyz],
                        [Ixz,  Iyz,  Izz]])
    I_s    = sp.Matrix([[Ixx_s, Ixy_s, Ixz_s],
                        [Ixy_s, Iyy_s, Iyz_s],
                        [Ixz_s, Iyz_s, Izz_s]])
    I_a    = sp.Matrix([[Ixx_a, Ixy_a, Ixz_a],
                        [Ixy_a, Iyy_a, Iyz_a],
                        [Ixz_a, Iyz_a, Izz_a]])
    I_b    = sp.Matrix([[Ixx_b, Ixy_b, Ixz_b],
                        [Ixy_b, Iyy_b, Iyz_b],
                        [Ixz_b, Iyz_b, Izz_b]])

    # ------------------------------------------------------------------
    # 3. Rotation matrices (world→body, Z‑Y‑X extrinsic)
    # ------------------------------------------------------------------
    R_x = sp.Matrix([[1, 0, 0],
                     [0, sp.cos(psi), -sp.sin(psi)],
                     [0, sp.sin(psi),  sp.cos(psi)]])

    R_y = sp.Matrix([[ sp.cos(theta), 0, sp.sin(theta)],
                     [              0, 1,              0],
                     [-sp.sin(theta), 0, sp.cos(theta)]])

    R_z = sp.Matrix([[ sp.cos(phi), -sp.sin(phi), 0],
                     [ sp.sin(phi),  sp.cos(phi), 0],
                     [           0,            0, 1]])

    R_bf = sp.simplify(R_z * R_y * R_x)  # body←world
    R_wf = R_bf.T                        # world←body

    # ------------------------------------------------------------------
    # 4. Forces (body frame)
    # ------------------------------------------------------------------
    thrust_b = sp.Matrix([
        T*sp.cos(b)*sp.sin(a),
        T*sp.sin(b),
       -T*sp.cos(b)*sp.cos(a)
    ])

    vel_b   = R_bf * velocity_wf
    vel_mag = sp.sqrt(vel_b.dot(vel_b))

    Fd_b = (sp.Rational(1,2)*rho*C_d*A_ref*vel_mag) * vel_b

    # ------------------------------------------------------------------
    # 5. Torques & angular acceleration
    # ------------------------------------------------------------------
    aero_tau_b   = r_c.cross(Fd_b)
    thrust_tau_b = r_t.cross(thrust_b)
    tau_net_b    = aero_tau_b + thrust_tau_b

    tau_net_w = R_wf * tau_net_b

    # Gyro + TVC terms
    I_s_w = R_wf*I_s*R_wf.T
    I_a_w = R_wf*I_a*R_wf.T
    I_b_w = R_wf*I_b*R_wf.T

    gyro_tau_s = ang_vel_wf.cross(I_s_w*ang_vel_wf)

    ang_vel_a_b = sp.Matrix([a_d, 0, 0])
    ang_vel_b_b = sp.Matrix([0, b_d, 0])
    ang_vel_a_w = R_wf*ang_vel_a_b + ang_vel_wf
    ang_vel_b_w = R_wf*ang_vel_b_b + ang_vel_wf
    gyro_tau_a  = ang_vel_a_w.cross(I_a_w*ang_vel_a_w)
    gyro_tau_b  = ang_vel_b_w.cross(I_b_w*ang_vel_b_w)

    ang_acc_a_b = sp.Matrix([a_dd, 0, 0])
    ang_acc_b_b = sp.Matrix([0, b_dd, 0])
    m_tau_a_w   = R_wf * (I_a*ang_acc_a_b)
    m_tau_b_w   = R_wf * (I_b*ang_acc_b_b)

    tau_net_w = (tau_net_w
                 - (gyro_tau_s + gyro_tau_a + gyro_tau_b)
                 + m_tau_a_w + m_tau_b_w)

    ang_accel_w = I.inv() * tau_net_w

    # Jacobians (angular part)
    A4 = ang_accel_w.jacobian(full_state)
    B4 = ang_accel_w.jacobian(full_input)

    # ------------------------------------------------------------------
    # 6. Translational acceleration
    # ------------------------------------------------------------------
    g_w = sp.Matrix([0, 0, -9.80665])
    thrust_w = R_wf * thrust_b
    Fd_w     = R_wf * Fd_b
    accel_w  = (thrust_w - Fd_w)/m + g_w

    A2 = accel_w.T.jacobian(full_state)
    B2 = accel_w.T.jacobian(full_input)

    # ------------------------------------------------------------------
    # 7. Assemble full A, B
    # ------------------------------------------------------------------
    A1 = sp.Matrix.hstack(sp.zeros(3), sp.eye(3), sp.zeros(3,6))
    A3 = sp.Matrix.hstack(sp.zeros(3,9), sp.eye(3))

    B1 = sp.zeros(3,7)
    B3 = sp.zeros(3,7)

    A = sp.Matrix.vstack(A1, A2, A3, A4)
    B = sp.Matrix.vstack(B1, B2, B3, B4)

    return A, B

def numeric__matrices(subs: dict[str,float], as_numpy: bool = True):
    """Substitute numeric values into (A,B).

    Parameters
    ----------
    subs : mapping {symbol_name: value}
        Keys may be strings or actual SymPy symbols.
    as_numpy : if True, return *numpy* arrays; else SymPy matrices.
    """
    import numpy as np

    A_sym, B_sym = calculate_state_matrices_sym()
    A_num = A_sym.subs(subs)
    B_num = B_sym.subs(subs)

    if as_numpy:
        return np.array(A_num.evalf(), dtype=float), np.array(B_num.evalf(), dtype=float)
    return A_num.evalf(), B_num.evalf()

# Quick smoke‑test
if __name__ == "__main__":
    A_sym, B_sym = calculate_state_matrices_sym()
    print("A shape:", A_sym.shape)
    print("B shape:", B_sym.shape)

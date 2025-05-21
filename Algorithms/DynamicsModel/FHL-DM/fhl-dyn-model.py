"""
Rocket Physics Integrator (Python)
----------------------------------
A translation of the C++ dynamics model/simulator, stripped of state‑estimation logic.  
It propagates a rigid body (rocket) forward in time under gravity, thrust, and aerodynamic drag.

TODO: ADD NOISE, WIND, AND CREATE STATE ESTIMATION FILTERS.

Dependencies
============
* numpy
* scipy
* do-mpc, in the mpc.py file
* mpc.py

Author: Propulsive Landers @ GT, for the Future Hovering Lander.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Tuple, List

# imports mpc.py from the same directory. Add other py files as needed.
import sys
from pathlib import Path

# Add the directory containing mpc.py to the Python path
sys.path.append(str(Path(__file__).parent))

import mpc  # needed for mpc controller
from mpc import *

import numpy as np
from scipy.spatial.transform import Rotation as R

# -----------------------------------------------------------------------------
# Helper functions (vector + quaternion)
# -----------------------------------------------------------------------------

# m_0 = 1.0 # initial mass, in kg. 

def extrinsic_rotation_matrix(euler_xyz: np.ndarray) -> np.ndarray:
    # """Return *world←body* rotation for extrinsic **Z‑Y‑X** (yaw‑pitch‑roll)."""
    # idk what this shit does anymore
    yaw, pitch, roll = euler_xyz  # x, y, z
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rx = np.array([[1, 0, 0],
                   [0, cy, -sy],
                   [0, sy, cy]])
    Ry = np.array([[cp, 0, sp],
                   [0, 1, 0],
                   [-sp, 0, cp]])
    Rz = np.array([[cr, -sr, 0],
                   [sr, cr, 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx


# -----------------------------------------------------------------------------
# Core dataclasses
# -----------------------------------------------------------------------------

@dataclass
class RocketParams:
    """Physical and aerodynamic constants (subset of the C++ struct)."""
    # NOTE: Mass / inertia - these must match with the MPC.py file!!!
    # TODO: Find a way to import the values from mpc.py.
    m_static: float = 0.9  # kg
    m_gimbal_top: float = 0.05
    m_gimbal_bottom: float = 0.05
    m_0: float = m_static + m_gimbal_top + m_gimbal_bottom 
    I_body: np.ndarray = field(default_factory=lambda: np.diag([0.5, 0.5, 0.5]))
    # these are not used
    I_gimbal_top: np.ndarray = field(default_factory=lambda: np.diag([0.5, 0.5, 0.5]))
    I_gimbal_bottom: np.ndarray = field(default_factory=lambda: np.diag([0.5, 0.5, 0.5]))

    # Geometry offsets (set to zero by default)
    thrust_offset: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, -0.1]))
    COP_offset: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.05]))
    gimbal_offset: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # Aero coefficients (Declared here, will be updated each step via AoA curve fit)
    Cd_x: float = 0.1
    Cd_y: float = 0.1
    Cd_z: float = 0.1
    A_x: float = 0.1  # m²
    A_y: float = 0.1
    A_z: float = 0.1

    rcs_offset: float = 0.05  # m

    alpha = 0.001

    air_density: float = 1.225  # kg / m³

    # Thrust envelope (N) and timestep (s)
    T_max: float = 15.0
    T_min: float = 0.0
    dt: float = 0.1 # s. Matching with mpc.py would be good but idk what happens if they don't match.

    @property
    def m(self) -> float:
        return self.m_static + self.m_gimbal_top + self.m_gimbal_bottom

    @property
    def inv_I(self) -> np.ndarray:
        return np.linalg.inv(self.I_body)


@dataclass
class State:
    """Translational + rotational states + mass."""
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    accel: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # Orientation & angular motion (body‑frame)
    att: R = field(default_factory=lambda: R.identity())  # body→world
    ang_vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    ang_accel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    m: np.float = m_0


# I don't think we need this
# def state_to_vec(s: State, *, to_dm: bool = False):
#     """
#     Convert `State` → 12×1 NumPy (or CasADi DM) vector in the order
#     [ r_x, r_y, r_z,  v_x, v_y, v_z,  ψ, θ, φ,  ψ̇, θ̇, φ̇ ].
#     """
#     # Euler angles MUST match the convention in your model:
#     # R_bf = R_z(phi) * R_y(theta) * R_x(psi)  ⇒  extrinsic Z‑Y‑X
#     psi, theta, phi = s.att.as_euler('zyx', degrees=False)  
#     x_vec = np.concatenate([
#         s.pos,                    # r_x, r_y, r_z
#         s.vel,                    # v_x, v_y, v_z
#         [psi, theta, phi],        # ψ, θ, φ
#         s.ang_vel,                 # ψ̇, θ̇, φ̇
#         s.m
#     ])
#     if to_dm:
#         return ca.DM(x_vec).reshape((-1, 1))  # (12,1) DM column
#     return x_vec                               # (12,) NumPy


# -----------------------------------------------------------------------------
# Force / torque models
# -----------------------------------------------------------------------------

# Wind dict and wind grabber function.
wind_dict = {
    # as of 5/18/2025 at ~10pm, this would only affect the ascent phase of our path.
    0.0: [0.0, 0.0, 0.0],
    1.0: [3.0, 0.0, 0.0],
    3.0: [0.0, 3.0, 0.0],
    5.0: [0.0, 0.0,-0.5],
    8.0: [0.0, 0.0, 0.0],
}

t_pts  = np.fromiter(wind_dict.keys(),  dtype=float)
uvw    = np.vstack(list(wind_dict.values()))          # shape (N,3)

def wind_at(t):
    """Piece-wise linear interpolation with NumPy only."""
    u = np.interp(t, t_pts, uvw[:,0])
    v = np.interp(t, t_pts, uvw[:,1])
    w = np.interp(t, t_pts, uvw[:,2])
    return np.array([u, v, w]) # functions expect ndarray input.

def thrust_body(params: RocketParams, F_mag: float, thrust_gimbal_xyz: np.ndarray, R1: float, R2: float) -> Tuple[np.ndarray, np.ndarray]:
    """Return thrust force/torque in the **body** frame.

    * ``thrust_gimbal_xyz`` – extrinsic‑XYZ angles (rad) describing how the
      engine nozzle is pointed relative to the body *Z* axis.
    """
    # R_gimbal = extrinsic_rotation_matrix(thrust_gimbal_xyz)
    # ez = np.array([0.0, 0.0, 1.0])
    # F_b = F_mag * (R_gimbal @ ez)  # N
    F_b = np.array(
        [F_mag*(np.cos(thrust_gimbal_xyz[1]))*(np.sin(thrust_gimbal_xyz[0])),
        F_mag*(np.sin(thrust_gimbal_xyz[1])),
        F_mag*(np.cos(thrust_gimbal_xyz[1]))*(np.cos(thrust_gimbal_xyz[0]))]
    )

    # Assume torque about COM arises from gimbal and structural offset
    r_b = params.thrust_offset 
    T_b = np.cross(r_b, F_b)
    T_rcs = np.array([0.0, 0.0, params.rcs_offset * (R1 - R2)])
    T_b += T_rcs
    
    print(T_b)
    return F_b, T_b


def drag_body(params: RocketParams, att: R, vel_wf: np.ndarray, v_wind_wf: np.array) -> Tuple[np.ndarray, np.ndarray]:
    """Simple axis‑aligned quadratic drag model (body frame)."""
    vel_rel_wf = vel_wf - v_wind_wf
    vel_rel_b = att.apply(vel_rel_wf)  # world → body
    v_mag = np.linalg.norm(vel_rel_b)

    if v_mag == 0.0:
        return np.zeros(3), np.zeros(3)

    diag = np.array([params.Cd_x * params.A_x,
                     params.Cd_y * params.A_y,
                     params.Cd_z * params.A_z])
    F_b = -0.5 * params.air_density * v_mag * vel_rel_b * diag
    T_b = np.cross(params.COP_offset, F_b)
    return F_b, T_b
    # return np.zeros(3), np.zeros(3)


# -----------------------------------------------------------------------------
# Integrator step
# -----------------------------------------------------------------------------

def integrate_step(params: RocketParams,
                   state: State,
                   F_b: np.ndarray,
                   T_b: np.ndarray,
                   F_mag: np.float,
                   v_wind_wf: np.ndarray = np.zeros(3)) -> State:
    """Advance one Euler step (explicit)."""
    dt = params.dt

    
    # Forces: convert body‑frame thrust/drag to world
    F_w = state.att.inv().apply(F_b)

    # Gravity in world frame
    F_grav = np.array([0.0, 0.0, -state.m * 9.80665])

    # --- Translational dynamics -------------------------------------------------
    accel_w = (F_grav + F_w) / state.m
    vel_w = state.vel + accel_w * dt
    pos_w = state.pos + vel_w * dt

    # --- Rotational dynamics ----------------------------------------------------
    # ang_accel_b = params.inv_I @ (T_b - np.cross(state.att.inv().apply(state.ang_vel), params.I_body @ state.att.inv().apply(state.ang_vel)))
    ang_accel_b = params.inv_I @ (T_b - np.cross(state.ang_vel, params.I_body @ state.ang_vel))
    # ang_accel_b = params.inv_I @ T_b
    # R_bf = extrinsic_rotation_matrix(state.att.as_euler("xyz"))
    # R_wf = R_bf.T

    # R_bf = extrinsic_rotation_matrix(state.att.as_euler('xyz'))
    # R_bf_quat = state.att.as_matrix()
    # print(R_bf)
    # print(R_bf_quat)
    ang_accel_w = state.att.inv().apply(ang_accel_b)
    # ang_vel_b = state.att.apply(state.ang_vel) + ang_accel_b * dt
    ang_vel_b = state.ang_vel + ang_accel_b * dt
    # ang_vel_w = state.ang_vel + ang_accel_w * dt

    # --- Mass Reduction ------------------
    mass = state.m - params.alpha*F_mag*dt # mass loss due to propellant consumption

    # Propagate orientation via incremental rotation vector (Rodrigues/exp‑map)
    dR = R.from_rotvec(ang_vel_b * dt)  # body→body'
    att_new = state.att * dR  # body→world update

    # Pack new state -------------------------------------------------------------
    return State(
        pos=pos_w,
        vel=vel_w,
        accel=accel_w,
        att=att_new,
        ang_vel=ang_vel_b,
        ang_accel=ang_accel_b,
        m = mass
    )


# -----------------------------------------------------------------------------
# Simulation driver (open‑loop thrust profile)
# -----------------------------------------------------------------------------

def simulate(params: RocketParams, controller,
             t_end: float = 30.0,
             out_csv: str | Path = "simulation_results.csv") -> None:
    """Run an closed‑loop ascent/hover/descent profile defined in MPC file and dump CSV."""
    n = int(np.round(t_end / params.dt)) + 1
    times: List[float] = []
    poses: List[np.ndarray] = []
    vels: List[np.ndarray] = []
    mass: List[float] = []
    attitude: List[np.ndarray] = []
    command: List[np.ndarray] = []

    # Initial state: add slight attitude offset to see if quaternion math works
    state = State(att=R.from_euler('xyz', [0.0, 0.0, 0.0]))

    # t = 0.0 # initialize time, wind is a time varying velocity vector field.

    # NOTE: I think i should simulate wind with an acceleration vector field.

    for i in range(n):
        t = i * params.dt
        print(t)
        
        # FOR THE FUTURE: ATT QUATERNION DEFINED AS WORLD TO BODY.
        # TO GET WORLD FRAME VECTORS, INVERT THE ATTITUDE QUATERNION.
        ang_vel_wf = state.att.inv().apply(state.ang_vel)
        x_meas = np.array([state.pos[0], state.pos[1], state.pos[2], state.vel[0], state.vel[1], state.vel[2], state.att.as_euler('xyz')[0], state.att.as_euler('xyz')[1], state.att.as_euler('xyz')[2], ang_vel_wf[0], ang_vel_wf[1], ang_vel_wf[2], state.m])
        # print(x_meas)
        u_cmd = controller.make_step(x_meas).flatten()
        # print(u_cmd)
        F_mag = u_cmd[0]
        # thrust_gimbal = np.ndarray([float(u_cmd[1]), float(u_cmd[2])])  # gimbal angles
        thrust_gimbal = np.zeros(3)
        thrust_gimbal[0] = u_cmd[1]  # gimbal angle A. Temporarily set to zero.
        thrust_gimbal[1] = u_cmd[2]  # gimbal angle B. 
        R1 = u_cmd[3]  # RCS1
        R2 = u_cmd[4]  # RCS2

        # Update aero coefficients based on AoA 
        # TODO: currently uses monoprop data, update to FHL data when possible.
        velocity_mag = np.linalg.norm(state.vel)
        if velocity_mag > 0.0:
            # cosine alpha is not calculated this way in mpc.py. May cause problems later.
            cos_alpha = np.dot(state.vel / velocity_mag, np.array([0.0, 0.0, np.sign(state.vel[2])])) # switch 1 with np.sign(state.vel[2]) later
            AoA = np.arccos(cos_alpha)
            params.Cd_x = -0.449 * np.cos(3.028 * np.degrees(AoA)) + 0.463
            params.Cd_y = params.Cd_x
            params.Cd_z = -0.376 * np.cos(5.675 * np.degrees(AoA)) + 1.854
        

        # Body‑frame forces
        v_wind = wind_at(t)
        # v_wind = np.zeros(3)

        F_thrust_b, T_thrust_b = thrust_body(params, F_mag, thrust_gimbal, R1, R2)
        F_drag_b, T_drag_b = drag_body(params, state.att, state.vel, v_wind)

        F_total_b = F_thrust_b + F_drag_b
        T_total_b = T_thrust_b + T_drag_b

        # Integrate one step
        # print("integrating step")
        state = integrate_step(params, state, F_total_b, T_total_b, F_mag)

        # History ---------------------------------------------------
        times.append(t)
        poses.append(state.pos.copy())
        vels.append(state.vel.copy())
        mass.append(state.m)
        attitude.append(state.att.as_euler('xyz'))
        command.append(np.array([F_mag, thrust_gimbal[0], thrust_gimbal[1], R1, R2]))

    # -------------------------------------------------------------------------
    # Dump CSV
    # -------------------------------------------------------------------------
    data = np.column_stack([
        np.array(times),
        np.array(poses),
        np.array(vels),
        np.array(mass),
        np.array(attitude),
        np.array(command),
    ])
    header = "time,x,y,z,vx,vy,vz,m,att_x,att_y,att_z,thrust,gimbal_a,gimbal_b,R1,R2"
    np.savetxt(out_csv, data, delimiter=",", header=header, comments="")
    print(f"Simulation complete → {out_csv} ({len(times)} samples).")


# -----------------------------------------------------------------------------
# Main guard
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    # f_sym, x_sym, u_sym = calculate_f_nonlinear_sym()
    params = RocketParams()
    mpc = initialize_mpc()

    # x = np.array([[0,0,0,0,0,0,0,0,0,1,0,0,0,params.m_0]]).T # has attitude quaternion.
    x_mpc = np.array([[0,0,0,0,0,0,0,0,0,0,0,0,params.m_0]]).T
    mpc.x0 = x_mpc
    mpc.set_initial_guess()
    u = np.zeros(5)
    simulate(params, mpc)
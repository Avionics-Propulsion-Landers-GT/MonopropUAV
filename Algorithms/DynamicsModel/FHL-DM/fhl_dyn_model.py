import numpy as np
from dataclasses import dataclass, field
from scipy.spatial.transform import Rotation as R
from pathlib import Path
import sys
from params import RocketParams


# Add local path for mpc.py
sys.path.append(str(Path(__file__).parent))
from mpc import initialize_mpc_with_physics_model, calculate_f_nonlinear_sym


@dataclass
class State:
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    accel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    att: R = field(default_factory=lambda: R.identity())
    ang_vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    ang_accel: np.ndarray = field(default_factory=lambda: np.zeros(3))

def extrinsic_rotation_matrix_yx(pitch, roll):
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rx @ Ry

def thrust_body(params, F_mag, gimbal_angles, R1, R2):
    R_gimbal = extrinsic_rotation_matrix_yx(*gimbal_angles)
    ez = np.array([0.0, 0.0, 1.0])
    F_b = F_mag * (R_gimbal @ ez)
    r_b = params.thrust_offset + params.gimbal_offset
    T_b = np.cross(r_b, F_b)
    T_rcs = np.array([params.rcs_offset * (R1 - R2), 0.0, 0.0])
    return F_b, T_b + T_rcs

def drag_body(params, att: R, vel_wf: np.ndarray):
    vel_b = att.inv().apply(vel_wf)
    v_mag = np.linalg.norm(vel_b)
    if v_mag == 0.0:
        return np.zeros(3), np.zeros(3)
    F_b = -0.5 * params.air_density * params.Cd_z * params.A_z * v_mag * vel_b
    T_b = np.cross(params.COP_offset, F_b)
    return F_b, T_b

def integrate_step(params, state, F_b, T_b):
    dt = params.dt
    F_w = state.att.apply(F_b)
    F_grav = np.array([0.0, 0.0, -params.m * 9.80665])
    accel_w = (F_grav + F_w) / params.m
    vel_w = state.vel + accel_w * dt
    pos_w = state.pos + vel_w * dt
    ang_accel_b = params.inv_I @ T_b
    ang_vel_b = state.ang_vel + ang_accel_b * dt
    dR = R.from_rotvec(np.array([ang_vel_b[0], ang_vel_b[1], 0.0]) * dt)
    att_new = state.att * dR
    return State(pos=pos_w, vel=vel_w, accel=accel_w, att=att_new, ang_vel=ang_vel_b, ang_accel=ang_accel_b)

def simulate(params, controller, t_end=5.0, out_csv="simulation_results.csv"):
    n = int(t_end / params.dt) + 1
    data = []
    state = State(att=R.from_euler("yx", [0, 0]))
    for i in range(n):
        t = i * params.dt
        _, theta, phi = state.att.as_euler("xyz", degrees=False)
        x_meas = np.array([*state.pos, *state.vel, theta, phi, *state.ang_vel[:2]])
        if i == 0:
            controller.x0 = x_meas.reshape(-1, 1)
            controller.set_initial_guess()
        u_cmd = controller.make_step(x_meas).flatten()
        F_mag, R1, R2, a, b = u_cmd
        F_thrust_b, T_thrust_b = thrust_body(params, F_mag, [a, b], R1, R2)
        F_drag_b, T_drag_b = drag_body(params, state.att, state.vel)
        F_total_b = F_thrust_b + F_drag_b
        T_total_b = T_thrust_b + T_drag_b
        state = integrate_step(params, state, F_total_b, T_total_b)
        data.append([t, *state.pos, *state.vel, theta, phi, F_mag, a, b])
    np.savetxt(out_csv, np.array(data), delimiter=",", header="time,x,y,z,vx,vy,vz,theta,phi,thrust,gimbal_a,gimbal_b", comments="")

if __name__ == "__main__":
    f_sym, x_sym, u_sym = calculate_f_nonlinear_sym()
    params = RocketParams()
    mpc_ctrl = initialize_mpc_with_physics_model(params)
    mpc_ctrl.x0 = np.zeros(10)
    mpc_ctrl.set_initial_guess()
    simulate(params, mpc_ctrl)

"""
Rocket Physics Integrator (Python)
----------------------------------
A bare‑bones translation of the C++ dynamics core, stripped of any
control‑law or state‑estimation logic.  It propagates a rigid body
(rocket) forward in time under gravity, thrust, and aerodynamic drag.

TODO: CREATE CONTROL LOOP AND STATE ESTIMATION CONNECTION.

Dependencies
============
* numpy
* scipy

Author: Propulsive Landers @ GT, for the Future Hovering Lander.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Tuple, List

import numpy as np
from scipy.spatial.transform import Rotation as R

# -----------------------------------------------------------------------------
# Helper functions (vector + quaternion)
# -----------------------------------------------------------------------------

def extrinsic_rotation_matrix(euler_xyz: np.ndarray) -> np.ndarray:
    """Return *world←body* rotation for extrinsic **Z‑Y‑X** (yaw‑pitch‑roll)."""
    roll, pitch, yaw = euler_xyz  # x, y, z
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rx = np.array([[1, 0, 0],
                   [0, cr, -sr],
                   [0, sr, cr]])
    Ry = np.array([[cp, 0, sp],
                   [0, 1, 0],
                   [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0],
                   [sy, cy, 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx


# -----------------------------------------------------------------------------
# Core dataclasses
# -----------------------------------------------------------------------------

@dataclass
class RocketParams:
    """Physical and aerodynamic constants (subset of the C++ struct)."""
    # Mass / inertia
    m_static: float = 0.6  # kg
    m_gimbal_top: float = 0.05
    m_gimbal_bottom: float = 0.05
    I_body: np.ndarray = field(default_factory=lambda: np.diag([0.00940, 0.00940, 0.00014]))

    # Geometry offsets (set to zero by default)
    COM_offset: np.ndarray = field(default_factory=lambda: np.zeros(3))
    COP_offset: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.01]))
    gimbal_offset: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # Aero coefficients (will be updated each step via AoA curve fit)
    Cd_x: float = 0.1
    Cd_y: float = 0.1
    Cd_z: float = 0.1
    A_x: float = 0.7  # m²
    A_y: float = 0.7
    A_z: float = 0.3

    air_density: float = 1.225  # kg / m³

    # Thrust envelope (N) and timestep (s)
    T_max: float = 100.0
    T_min: float = 0.0
    dt: float = 1e-3

    @property
    def m(self) -> float:
        return self.m_static + self.m_gimbal_top + self.m_gimbal_bottom

    @property
    def inv_I(self) -> np.ndarray:
        return np.linalg.inv(self.I_body)


@dataclass
class State:
    """Translational + rotational states."""
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    accel: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # Orientation & angular motion (body‑frame)
    att: R = field(default_factory=lambda: R.identity())  # body→world
    ang_vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    ang_accel: np.ndarray = field(default_factory=lambda: np.zeros(3))


# -----------------------------------------------------------------------------
# Force / torque models
# -----------------------------------------------------------------------------

def thrust_body(params: RocketParams, F_mag: float, thrust_gimbal_xyz: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Return thrust force/torque in the **body** frame.

    * ``thrust_gimbal_xyz`` – extrinsic‑XYZ angles (rad) describing how the
      engine nozzle is pointed relative to the body *Z* axis.
    """
    R_gimbal = extrinsic_rotation_matrix(thrust_gimbal_xyz)
    ez = np.array([0.0, 0.0, 1.0])
    F_b = F_mag * (R_gimbal @ ez)  # N

    # Assume torque about COM arises from gimbal and structural offset
    r_b = params.COM_offset + params.gimbal_offset
    T_b = np.cross(r_b, F_b)
    return F_b, T_b


def drag_body(params: RocketParams, att: R, vel_wf: np.ndarray, v_wind_wf: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Simple axis‑aligned quadratic drag model (body frame)."""
    vel_rel_wf = vel_wf - v_wind_wf
    vel_rel_b = att.inv().apply(vel_rel_wf)  # world → body
    v_mag = np.linalg.norm(vel_rel_b)

    if v_mag == 0.0:
        return np.zeros(3), np.zeros(3)

    diag = np.array([params.Cd_x * params.A_x,
                     params.Cd_y * params.A_y,
                     params.Cd_z * params.A_z])
    F_b = -0.5 * params.air_density * v_mag * vel_rel_b * diag
    T_b = np.cross(params.COP_offset, F_b)
    return F_b, T_b


# -----------------------------------------------------------------------------
# Integrator step
# -----------------------------------------------------------------------------

def integrate_step(params: RocketParams,
                   state: State,
                   F_b: np.ndarray,
                   T_b: np.ndarray,
                   v_wind_wf: np.ndarray = np.zeros(3)) -> State:
    """Advance one Euler step (explicit)."""
    dt = params.dt

    # Forces: convert body‑frame thrust/drag to world
    F_w = state.att.apply(F_b)

    # Gravity in world frame
    F_grav = np.array([0.0, 0.0, -params.m * 9.80665])

    # --- Translational dynamics -------------------------------------------------
    accel_w = (F_grav + F_w) / params.m
    vel_w = state.vel + accel_w * dt
    pos_w = state.pos + vel_w * dt

    # --- Rotational dynamics ----------------------------------------------------
    ang_accel_b = params.inv_I @ T_b
    ang_vel_b = state.ang_vel + ang_accel_b * dt

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
    )


# -----------------------------------------------------------------------------
# Simulation driver (open‑loop thrust profile)
# -----------------------------------------------------------------------------

def simulate(params: RocketParams,
             t_end: float = 40.0,
             out_csv: str | Path = "simulation_results.csv") -> None:
    """Run an open‑loop ascent/hover/descent profile and dump CSV."""
    n = int(np.round(t_end / params.dt)) + 1
    times: List[float] = []
    poses: List[np.ndarray] = []
    vels: List[np.ndarray] = []
    attitude: List[np.ndarray] = []
    command: List[np.ndarray] = []

    # Initial state: add slight attitude offset to see if quaternion math works
    state = State(att=R.from_euler("xyz", [0, 0, 0.0]))

    v_wind = np.zeros(3)  # no wind

    for i in range(n):
        t = i * params.dt

        # -----------------------------------
        # Basic thrust schedule (open loop):
        #   0–10 s  : full thrust up (15 N)
        #   10–20 s : hover (~mg)
        #   20–30 s : engine off, free fall
        #   30–40 s : retro‑thrust to slow
        # -----------------------------------
        if t < 10.0:
            F_mag = 15.0
        elif t < 20.0:
            F_mag = params.m * 9.80665
        elif t < 30.0:
            F_mag = 0.0
        else:
            F_mag = 5.0

        thrust_gimbal = np.zeros(3)  # keep nozzle aligned with body Z

        # Update aero coefficients based on AoA 
        # TODO: currently uses monoprop data, update to FHL data when possible.
        velocity_mag = np.linalg.norm(state.vel)
        if velocity_mag > 0.0:
            cos_alpha = np.clip(np.dot(state.vel / velocity_mag, np.array([0.0, 0.0, np.sign(state.vel[2])])), -1.0, 1.0)
            AoA = np.arccos(cos_alpha)
            params.Cd_x = -0.449 * np.cos(3.028 * np.degrees(AoA)) + 0.463
            params.Cd_y = params.Cd_x
            params.Cd_z = -0.376 * np.cos(5.675 * np.degrees(AoA)) + 1.854

        # Body‑frame forces
        F_thrust_b, T_thrust_b = thrust_body(params, F_mag, thrust_gimbal)
        F_drag_b, T_drag_b = drag_body(params, state.att, state.vel, v_wind)

        F_total_b = F_thrust_b + F_drag_b
        T_total_b = T_thrust_b + T_drag_b

        # Integrate one step
        state = integrate_step(params, state, F_total_b, T_total_b)

        # History ---------------------------------------------------
        times.append(t)
        poses.append(state.pos.copy())
        vels.append(state.vel.copy())
        attitude.append(state.att.as_euler("xyz"))
        command.append(np.array([F_mag, thrust_gimbal[0], thrust_gimbal[1]]))

    # -------------------------------------------------------------------------
    # Dump CSV
    # -------------------------------------------------------------------------
    data = np.column_stack([
        np.array(times),
        np.array(poses),
        np.array(vels),
        np.array(attitude),
        np.array(command),
    ])
    header = "time,x,y,z,vx,vy,vz,att_x,att_y,att_z,thrust,gimbal_a,gimbal_b"
    np.savetxt(out_csv, data, delimiter=",", header=header, comments="")
    print(f"Simulation complete → {out_csv} ({len(times)} samples).")


# -----------------------------------------------------------------------------
# Main guard
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    params = RocketParams()
    simulate(params)
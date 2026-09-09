"""
Shared helper for the controls exercises. You do not need to edit this file.

It does three jobs:

1. Finds the MonopropUAV repo and puts Algorithms/MPC on sys.path, because
   nonlinear_mpc.py does a bare ``import rocketdynamics_plus``.
2. Holds the index constants for the repo's 17-state rocket model so nobody
   has to remember that qw is x[6] and gimbal_phi is x[13].
3. Provides load_dynamics(mass) to build a plant with a different mass without
   editing the repo file, and metrics() for settle time, overshoot and so on.

Where the repo is looked for, in order:
  a) a path passed in explicitly (the --repo flag on ex3),
  b) this file living inside the repo at Tutorials/Controls/exercises/,
  c) the environment variable MONOPROP_REPO,
  d) a sibling checkout: ../../../MonopropUAV relative to this folder.
"""
from __future__ import annotations

import os
import re
import sys
import types
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
CLONE_URL = "https://github.com/Avionics-Propulsion-Landers-GT/MonopropUAV.git"


# --------------------------------------------------------------------------- #
# Repo location
# --------------------------------------------------------------------------- #
def _looks_like_repo(root: Path) -> bool:
    return (root / "Algorithms" / "MPC" / "nonlinear_mpc.py").exists()


def find_repo_root(explicit: str | os.PathLike | None = None) -> Path:
    """Return the MonopropUAV root or raise FileNotFoundError with a fix."""
    if explicit:
        # An explicit path is trusted and not silently replaced by a guess.
        # Accept either the repo root or the Algorithms/MPC folder itself.
        p = Path(explicit).expanduser().resolve()
        for c in (p, p.parents[1] if p.name == "MPC" and len(p.parents) > 1 else None):
            if c is not None and _looks_like_repo(c):
                return c
        raise FileNotFoundError(
            f"--repo / MONOPROP_REPO points at {p}, but Algorithms/MPC/nonlinear_mpc.py is not "
            f"there. Pass the root of your MonopropUAV clone (git clone {CLONE_URL})."
        )
    candidates = []
    # HERE = .../exercises; parents[0] = Controls, [1] = Tutorials, [2] = the folder above Tutorials
    if len(HERE.parents) > 2:
        candidates.append(HERE.parents[2])                      # <repo>/Tutorials/Controls/exercises
    if os.environ.get("MONOPROP_REPO"):
        candidates.append(Path(os.environ["MONOPROP_REPO"]).expanduser().resolve())
    if len(HERE.parents) > 2:
        candidates.append(HERE.parents[2] / "MonopropUAV")    # sibling checkout next to Tutorials/

    for c in candidates:
        if _looks_like_repo(c):
            return c

    tried = "\n  ".join(str(c) for c in candidates) or "(none)"
    raise FileNotFoundError(
        "Could not find the MonopropUAV repo (Algorithms/MPC/nonlinear_mpc.py).\n"
        f"Looked in:\n  {tried}\n"
        "Fix one of:\n"
        f"  git clone {CLONE_URL}   next to the Tutorials folder, or\n"
        "  set MONOPROP_REPO=<path to your clone>, or\n"
        "  pass --repo <path to your clone> on the command line."
    )


def find_mpc_dir(explicit=None) -> Path:
    return find_repo_root(explicit) / "Algorithms" / "MPC"


def add_mpc_to_path(explicit=None) -> Path:
    """Insert Algorithms/MPC at the front of sys.path and return it."""
    mpc = find_mpc_dir(explicit)
    s = str(mpc)
    if s not in sys.path:
        sys.path.insert(0, s)
    return mpc


# --------------------------------------------------------------------------- #
# Index constants for rocketdynamics_plus.dynamics(x, u)
# --------------------------------------------------------------------------- #
# State (17). Quaternion is SCALAR-LAST: [qx, qy, qz, qw], so level is x[6] = 1.
IX, IY, IZ = 0, 1, 2
IQX, IQY, IQZ, IQW = 3, 4, 5, 6
IVX, IVY, IVZ = 7, 8, 9
IWX, IWY, IWZ = 10, 11, 12
IGPHI, IGTHETA, IGPHI_RATE, IGTHETA_RATE = 13, 14, 15, 16
NX = 17

STATE_NAMES = ["x", "y", "z", "qx", "qy", "qz", "qw",
               "vx", "vy", "vz", "wx", "wy", "wz",
               "g_phi", "g_theta", "g_phi_rate", "g_theta_rate"]

# Input (3). Note the gimbal order is theta, phi here but phi, theta in the state.
U_THETA, U_PHI, U_THRUST = 0, 1, 2
NU = 3
INPUT_NAMES = ["theta_cmd", "phi_cmd", "thrust"]

DT = 0.1          # hard-coded inside dynamics(); it is a one-step map, not an ODE
MASS = 1.0
G = 9.81
U_HOVER = np.array([0.0, 0.0, MASS * G])
GIMBAL_LIMIT = np.deg2rad(10.0)
U_MIN = np.array([-GIMBAL_LIMIT, -GIMBAL_LIMIT, 1.0])     # 1 N = flame-out floor
U_MAX = np.array([GIMBAL_LIMIT, GIMBAL_LIMIT, 20.0])

# The 14 states LQR can use: drop qz (5), qw (6) and wz (12).
KEEP14 = [0, 1, 2, 3, 4, 7, 8, 9, 10, 11, 13, 14, 15, 16]
KEEP14_NAMES = [STATE_NAMES[i] for i in KEEP14]


def hover_state() -> np.ndarray:
    x = np.zeros(NX)
    x[IQW] = 1.0
    return x


# --------------------------------------------------------------------------- #
# Plant with a different mass, without editing the repo file
# --------------------------------------------------------------------------- #
def load_dynamics(mass: float = 1.0, explicit_repo=None) -> types.ModuleType:
    """
    Return a module that behaves like rocketdynamics_plus but with the given
    mass. Works by reading the repo file, replacing the ``m = 1`` line, and
    executing the copy. Everything else (inertia, dt, servo gains) is unchanged.
    """
    src_path = find_mpc_dir(explicit_repo) / "rocketdynamics_plus.py"
    src = src_path.read_text(encoding="utf-8")
    new_src, n = re.subn(r"^(\s*)m\s*=\s*1\b[^\n]*$",
                         rf"\1m = {float(mass)!r}  # mass (kg), patched by _repo.load_dynamics",
                         src, count=1, flags=re.MULTILINE)
    if n != 1:
        raise RuntimeError(f"Could not find the mass line in {src_path}")
    mod = types.ModuleType(f"rocketdynamics_plus_m{mass}")
    mod.__file__ = str(src_path)
    exec(compile(new_src, str(src_path), "exec"), mod.__dict__)
    return mod


# --------------------------------------------------------------------------- #
# Metrics shared by all three exercises
# --------------------------------------------------------------------------- #
def metrics(t, y, y_ref, band=0.1, t_start=0.0):
    """
    Step-response numbers for a 1-D signal y(t) chasing a constant y_ref.

    rise_time          first time |y - y_ref| < band (after t_start), or nan
    overshoot          how far y went past y_ref in the direction of the step (>= 0)
    settle_time        last time y left the +/- band, measured from t_start; nan if never inside
    steady_state_error mean(y - y_ref) over the final 10 percent of the run
    iae                integral of |error| over time (np.trapezoid), same idea as
                       _compute_iae in Algorithms/MTV Sim/pid_tuning_runner.py
    """
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    sel = t >= t_start
    t, y = t[sel], y[sel]
    err = y - y_ref
    out = {}
    if not np.all(np.isfinite(y)):
        return dict(rise_time=np.nan, overshoot=np.nan, settle_time=np.nan,
                    steady_state_error=np.nan, iae=np.nan)

    inside = np.abs(err) < band
    out["rise_time"] = float(t[inside][0] - t_start) if inside.any() else np.nan

    direction = np.sign(y_ref - y[0]) if abs(y_ref - y[0]) > 1e-9 else 1.0
    out["overshoot"] = float(max(0.0, np.max(direction * err)))

    if inside.any() and inside[-1]:
        outside_idx = np.where(~inside)[0]
        last_out = t[outside_idx[-1]] if outside_idx.size else t[0]
        out["settle_time"] = float(last_out - t_start)
    else:
        out["settle_time"] = np.nan

    tail = max(1, int(0.1 * len(t)))
    out["steady_state_error"] = float(np.mean(err[-tail:]))
    out["iae"] = float(np.trapezoid(np.abs(err), t))
    return out


def fmt_metrics(m: dict) -> str:
    def f(v, unit=""):
        return "   nan" if not np.isfinite(v) else f"{v:6.2f}{unit}"
    return (f"rise {f(m['rise_time'], ' s')}  overshoot {f(m['overshoot'], ' m')}  "
            f"settle {f(m['settle_time'], ' s')}  ss err {f(m['steady_state_error'], ' m')}  "
            f"IAE {f(m['iae'])}")

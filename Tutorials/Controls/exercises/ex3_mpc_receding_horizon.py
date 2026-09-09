"""
Exercise 3: run the MPC, then change one thing.

You write the receding-horizon loop around nmpc_step from the repo's
Algorithms/MPC/nonlinear_mpc.py, change the constraints and the reference,
and put the MPC and your LQR from exercise 2 on the same plot with the same
limits. The point is to see constraints being respected instead of clipped,
and to see what the MPC still cannot do (it has no integrator).

Speed warning. One nmpc_step is about 40 ms in Python, so 60 steps is a few
seconds but the repo's full 300-step demo is close to a minute with plotting.
Keep --steps small while you experiment and only go long for the final plots.
Do not grid-search weights with a long run.

    python ex3_mpc_receding_horizon.py                    # 60 steps, N = 10, demo reference
    python ex3_mpc_receding_horizon.py --steps 300        # the full repo demo (slow)
    python ex3_mpc_receding_horizon.py --thrust-max 12    # TODO 1
    python ex3_mpc_receding_horizon.py --profile step     # TODO 2 (after you write it)
    python ex3_mpc_receding_horizon.py --compare-lqr      # TODO 3
    python ex3_mpc_receding_horizon.py --mass 1.05        # stretch: heavier plant, same MPC model
    python ex3_mpc_receding_horizon.py --repo C:/path/to/MonopropUAV

What to do
  0. Run it. Read the per-step log: solve time, applied command, position.
  1. TODO 1: lower thrust_max from 20 N to 12 N (--thrust-max 12). Watch the
     thrust sit on the new ceiling for a while and the rocket still get there.
     Write down what happened to the settle time and to the max gimbal angle.
  2. TODO 2: implement the "step" profile in make_reference: hold (2, -3, 10)
     and at t = 3 s move the x reference to 0. The whole horizon must carry
     the new reference, row by row, not just row 0 (row 0 is ignored by
     nmpc_step; the residual uses rows 1..N).
  3. TODO 3: run --compare-lqr. It builds K from your student_weights() in
     ex2_lqr_tvc.py and runs it on the same scenario with the same clipping.
     Which one would you fly? What would you change in Q and R to make the
     MPC faster?
  4. Stretch: --mass 1.05 makes the plant heavier than the MPC's model. The
     MPC hovers low and does not notice. Explain why, then add the integral
     reference shift that Algorithms/MPC/src/main.rs already has a hook for
     (z_integral += (z_ref - z) dt; xref_traj[:, 2] += ki_z * z_integral).

Things about the repo code worth knowing before you touch it
  - State order is [x, y, z, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz, g_phi,
    g_theta, g_phi_rate, g_theta_rate]; qw = x[6] = 1 is level. The comments
    at the top of nonlinear_mpc.py's __main__ say something else; the code is
    right and the comments are stale.
  - Inputs are [theta_cmd, phi_cmd, thrust]. Gimbal order is swapped between
    the input (theta, phi) and the state (phi, theta).
  - dt = 0.1 s is hard-coded inside dynamics(). nmpc_step's alpha_pgd
    argument is not used. xref_traj must be (N+1, 17).
  - The cost penalizes absolute thrust, u'Ru, not (u - u_hover). With
    R_thrust = 0.1 the closed loop settles about 5 cm low. That is not a bug
    in your loop.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

os.environ.setdefault("MPLBACKEND", "Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import _repo  # noqa: E402
from _repo import (NX, NU, DT, IX, IY, IZ, IQW, IQX, IQY, U_THRUST, U_HOVER,  # noqa: E402
                   U_MIN, U_MAX, KEEP14)

OUT_DIR = HERE / "outputs"
DEFAULT_REPO_GUESS = "../../../MonopropUAV/Algorithms/MPC"    # relative to this file

# Weights and bounds copied from nonlinear_mpc.py __main__ (17-state order).
Q = np.diag([70.0, 70.0, 200.0,
             200.0, 200.0, 200.0, 200.0,
             50.0, 50.0, 50.0,
             5.0, 5.0, 5.0,
             2.0, 2.0, 5.0, 5.0])
R = np.diag([400.0, 400.0, 0.1])
QN = Q.copy()

HOLD_POINT = np.array([2.0, -3.0, 10.0])


# --------------------------------------------------------------------------- #
# Import the repo solver, or explain what is missing
# --------------------------------------------------------------------------- #
def import_repo(repo_arg):
    explicit = repo_arg
    if explicit is None and (HERE / DEFAULT_REPO_GUESS).exists():
        explicit = HERE / DEFAULT_REPO_GUESS
    try:
        mpc_dir = _repo.add_mpc_to_path(explicit)
    except FileNotFoundError as e:
        print("Cannot import the MPC solver.\n" + str(e))
        return None, None, None
    import nonlinear_mpc as nm
    import rocketdynamics_plus as rd
    print(f"using solver from {mpc_dir}")
    return nm, rd, mpc_dir


# --------------------------------------------------------------------------- #
# Reference trajectories
# --------------------------------------------------------------------------- #
def level_ref(pos):
    x = np.zeros(NX)
    x[[IX, IY, IZ]] = pos
    x[IQW] = 1.0
    return x


def make_reference(t, N, dt, profile="demo"):
    """
    Return xref_traj with shape (N+1, 17): row k is the reference at t + k*dt.
    qw must be 1 in every row (level attitude).

    "hold"  constant (2, -3, 10)
    "demo"  the repo demo: (2, -3, 10), then the origin once t > 15 s
    "step"  TODO 2: hold (2, -3, 10); at t = 3 s the x reference goes to 0
    """
    if profile == "hold":
        return np.tile(level_ref(HOLD_POINT), (N + 1, 1))

    if profile == "demo":
        rows = []
        for k in range(N + 1):
            tk = t + k * dt
            rows.append(level_ref(HOLD_POINT if tk <= 15.0 else np.zeros(3)))
        return np.array(rows)

    if profile == "step":
        # TODO 2: build the rows like the "demo" branch does, with the x
        # reference switching from 2 to 0 when the row's time passes 3 s.
        # Until you do, this falls back to "hold" so the file still runs.
        if not make_reference._warned:
            print("  [make_reference] 'step' is not implemented yet; using 'hold' (TODO 2)")
            make_reference._warned = True
        return np.tile(level_ref(HOLD_POINT), (N + 1, 1))

    raise ValueError(f"unknown profile {profile!r}")


make_reference._warned = False


# --------------------------------------------------------------------------- #
# The receding-horizon loop. Read every line; this is the whole idea of MPC.
# --------------------------------------------------------------------------- #
def receding_horizon(nm, plant, x0, ref_fn, steps, N, u_min, u_max, sqp_iters=1,
                     verbose=True):
    """
    Each tick: build the reference over the horizon, solve for N commands
    starting from the warm start, apply only the first one to the plant, then
    shift the solution one step to warm-start the next tick.

    Returns a dict with t, x (steps+1, 17), u (steps, 3), pred (list of the
    predicted xs per tick), solve_ms, violations (count of commands outside
    the box; should be 0 because the solver projects onto the box).
    """
    x = np.array(x0, float)
    U_warm = np.zeros((N, NU))
    U_warm[:, U_THRUST] = U_HOVER[U_THRUST]     # never start the solver from zero thrust
    hist = dict(t=[0.0], x=[x.copy()], u=[], pred=[], solve_ms=[], violations=0)
    for k in range(steps):
        t = k * DT
        xref_traj = ref_fn(t, N, DT)                              # (N+1, 17)
        t0 = time.perf_counter()
        U_opt, xs_pred = nm.nmpc_step(x, U_warm, xref_traj, Q, R, QN, u_min, u_max,
                                      sqp_iters=sqp_iters)
        ms = 1000 * (time.perf_counter() - t0)
        u = U_opt[0].copy()
        if np.any(u < u_min - 1e-9) or np.any(u > u_max + 1e-9):
            hist["violations"] += 1
        x = plant.dynamics(x, u)                                  # apply only the first command
        U_warm = np.vstack([U_opt[1:], U_opt[-1:]])               # shift, repeat the last one
        hist["t"].append(t + DT); hist["x"].append(x.copy()); hist["u"].append(u)
        hist["pred"].append(xs_pred); hist["solve_ms"].append(ms)
        if verbose and (k % 10 == 0 or k == steps - 1):
            print(f"  t={t:5.1f}  solve {ms:5.1f} ms  u=[{np.rad2deg(u[0]):6.2f} deg, "
                  f"{np.rad2deg(u[1]):6.2f} deg, {u[2]:5.2f} N]  pos=({x[IX]:6.3f}, {x[IY]:6.3f}, {x[IZ]:6.3f})")
    for key in ("t", "x", "u", "solve_ms"):
        hist[key] = np.array(hist[key])
    return hist


def summarize(hist, ref_pos, label, u_max):
    pos_err = np.linalg.norm(hist["x"][:, :3] - ref_pos, axis=1)
    tail = hist["t"] >= hist["t"][-1] - 2.0
    inside = pos_err < 0.1
    if inside.any() and inside[-1]:
        out = np.where(~inside)[0]
        settle = hist["t"][out[-1]] + DT if out.size else 0.0
    else:
        settle = math.nan
    u = hist["u"]
    print(f"\n{label}")
    print(f"  final position ({hist['x'][-1, IX]:.3f}, {hist['x'][-1, IY]:.3f}, {hist['x'][-1, IZ]:.3f}) "
          f"vs reference ({ref_pos[0]:.1f}, {ref_pos[1]:.1f}, {ref_pos[2]:.1f}); error {pos_err[-1]:.3f} m")
    print(f"  settle (pos err < 0.1 m): {'not within this run' if not math.isfinite(settle) else f'{settle:.1f} s'}"
          f"   RMS pos err over the last 2 s: {np.sqrt(np.mean(pos_err[tail] ** 2)):.3f} m")
    print(f"  max tilt |qx|,|qy| {np.max(np.abs(hist['x'][:, [IQX, IQY]])):.3f}   "
          f"max |gimbal| {np.rad2deg(np.max(np.abs(u[:, :2]))):.2f} deg   "
          f"thrust {u[:, 2].min():.2f}..{u[:, 2].max():.2f} N   "
          f"steps on the thrust ceiling {int(np.sum(u[:, 2] >= u_max[2] - 1e-6))}")
    if "solve_ms" in hist and len(hist["solve_ms"]):
        print(f"  solve time per step: mean {hist['solve_ms'].mean():.1f} ms, max {hist['solve_ms'].max():.1f} ms;"
              f"  commands outside the box: {hist['violations']}")
    if "sat" in hist:
        print(f"  clipped steps (LQR): {int(np.sum(hist['sat']))}")


# --------------------------------------------------------------------------- #
# Plots
# --------------------------------------------------------------------------- #
def compare(runs, ref_pos, u_min, u_max, title, save, show=False):
    """runs: list of (label, hist). Positions, gimbal and thrust on shared axes."""
    fig, ax = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    styles = ["-", "--", ":"]
    for (label, h), ls in zip(runs, styles):
        t, x, u = h["t"], h["x"], h["u"]
        tu = t[:len(u)]
        for i, name in enumerate("xyz"):
            ax[0].plot(t, x[:, i], ls, color=f"C{i}", label=f"{name} ({label})")
        ax[1].plot(tu, np.rad2deg(u[:, 0]), ls, color="C0", label=f"theta ({label})")
        ax[1].plot(tu, np.rad2deg(u[:, 1]), ls, color="C1", label=f"phi ({label})")
        ax[2].plot(tu, u[:, 2], ls, color="C0", label=f"thrust ({label})")
    for i in range(3):
        ax[0].axhline(ref_pos[i], color=f"C{i}", lw=0.8, alpha=0.5)
    for s in (np.rad2deg(u_min[0]), np.rad2deg(u_max[0])):
        ax[1].axhline(s, color="tab:red", ls="--", lw=1)
    for s in (u_min[2], u_max[2]):
        ax[2].axhline(s, color="tab:red", ls="--", lw=1)
    ax[0].set_ylabel("position (m)"); ax[1].set_ylabel("gimbal (deg)"); ax[2].set_ylabel("thrust (N)")
    ax[2].set_xlabel("time (s)"); ax[0].set_title(title)
    ax[1].set_ylim(-12, 12); ax[2].set_ylim(0, 22)
    for a in ax:
        a.grid(alpha=0.3); a.legend(fontsize=7, ncol=3, loc="upper right")
    fig.tight_layout()
    Path(save).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save, dpi=120); print(f"saved {save}")
    if show:
        plt.show()
    plt.close(fig)


def plot_predictions(hist, save, every=10, show=False):
    """The realised path in x-z with the solver's predicted xs drawn every few ticks."""
    fig, ax = plt.subplots(figsize=(7, 6))
    for k in range(0, len(hist["pred"]), every):
        p = hist["pred"][k]
        ax.plot(p[:, IX], p[:, IZ], color="C1", alpha=0.35, lw=1)
    ax.plot(hist["x"][:, IX], hist["x"][:, IZ], "C0", lw=2, label="realised")
    ax.plot([], [], "C1", alpha=0.5, label="predicted horizon (every %d ticks)" % every)
    ax.set_xlabel("x (m)"); ax.set_ylabel("z (m)"); ax.grid(alpha=0.3); ax.legend(fontsize=8)
    ax.set_title("what the solver expected vs what happened")
    fig.tight_layout(); fig.savefig(save, dpi=120); print(f"saved {save}"); plt.close(fig)
    if show:
        plt.show()


# --------------------------------------------------------------------------- #
# TODO 3 helper: LQR from exercise 2 on the same scenario
# --------------------------------------------------------------------------- #
def run_lqr_baseline(nm, plant, x0, ref_pos, steps, u_min, u_max):
    import ex2_lqr_tvc as ex2
    A, B = ex2.linearize_hover(nm, plant)
    A14, B14 = ex2.reduce_model(A, B, KEEP14)
    Qs, Rs = ex2.student_weights()
    K = ex2.lqr_gain(A14, B14, Qs, Rs)
    xref = level_ref(ref_pos)
    hist = ex2.simulate(ex2.lqr_controller(K, KEEP14, xref), x0, steps * DT, plant,
                        clip=True, u_min=u_min, u_max=u_max)
    return hist


# --------------------------------------------------------------------------- #
def main(argv=None):
    ap = argparse.ArgumentParser(description="Exercise 3: receding-horizon MPC with the repo solver")
    ap.add_argument("--repo", default=None, metavar="PATH",
                    help=f"MonopropUAV clone or its Algorithms/MPC folder (default guess: {DEFAULT_REPO_GUESS})")
    ap.add_argument("--steps", type=int, default=60, help="closed-loop steps of 0.1 s (default 60)")
    ap.add_argument("--N", type=int, default=10, help="horizon length (default 10)")
    ap.add_argument("--thrust-max", type=float, default=U_MAX[2], help="thrust ceiling in N (TODO 1)")
    ap.add_argument("--profile", choices=["demo", "hold", "step"], default="demo")
    ap.add_argument("--compare-lqr", action="store_true", help="TODO 3: run ex2's LQR on the same scenario")
    ap.add_argument("--mass", type=float, default=1.0, help="plant mass (kg); the MPC model keeps 1.0")
    ap.add_argument("--sqp-iters", type=int, default=1)
    ap.add_argument("--out", default=str(OUT_DIR), metavar="DIR")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args(argv)

    nm, rd, _ = import_repo(args.repo)
    if nm is None:
        return 2
    plant = rd if args.mass == 1.0 else _repo.load_dynamics(args.mass, args.repo)
    if args.mass != 1.0:
        print(f"plant mass {args.mass} kg; the MPC still believes 1.0 kg")

    u_min = U_MIN.copy()
    u_max = U_MAX.copy(); u_max[U_THRUST] = args.thrust_max
    x0 = _repo.hover_state()
    out = Path(args.out)
    ref_fn = lambda t, N, dt: make_reference(t, N, dt, args.profile)   # noqa: E731
    ref_end = ref_fn(args.steps * DT, 0, DT)[0, :3]                    # reference at the end of the run

    print(f"receding horizon: {args.steps} steps x {DT} s, N = {args.N}, sqp_iters = {args.sqp_iters}, "
          f"thrust {u_min[2]:.0f}..{u_max[2]:.0f} N, gimbal +/-{np.rad2deg(u_max[0]):.0f} deg, profile '{args.profile}'")
    t0 = time.perf_counter()
    hist = receding_horizon(nm, plant, x0, ref_fn, args.steps, args.N, u_min, u_max, args.sqp_iters)
    print(f"loop wall time {time.perf_counter() - t0:.1f} s")
    summarize(hist, ref_end, "MPC", u_max)

    tag = f"_tmax{args.thrust_max:g}" if args.thrust_max != U_MAX[2] else ""
    tag += f"_m{args.mass:g}" if args.mass != 1.0 else ""
    runs = [("MPC", hist)]
    if args.compare_lqr:
        try:
            h_lqr = run_lqr_baseline(nm, plant, x0, ref_end, args.steps, u_min, u_max)
            summarize(h_lqr, ref_end, "LQR (ex2 student_weights, clipped)", u_max)
            runs.append(("LQR", h_lqr))
        except Exception as e:  # keep the MPC result even if ex2 is unfinished
            print(f"\nLQR baseline skipped: {type(e).__name__}: {e}")
    compare(runs, ref_end, u_min, u_max,
            f"{args.profile} reference, thrust max {u_max[2]:.0f} N", out / f"ex3_{args.profile}{tag}.png", args.show)
    plot_predictions(hist, out / f"ex3_{args.profile}{tag}_predictions.png", show=args.show)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

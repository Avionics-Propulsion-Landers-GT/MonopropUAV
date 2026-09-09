"""
Exercise 2: one gain matrix for the whole vehicle (LQR on the repo's TVC model).

Default mode uses the repo's own 17-state rocket, Algorithms/MPC/
rocketdynamics_plus.py, and the repo's own finite-difference linearization,
nonlinear_mpc.compute_jacobian. You will find out why the Riccati solver
refuses the raw model, what Q and R mean in numbers, and where LQR falls over.

    python ex2_lqr_tvc.py               # needs the MonopropUAV repo (see README)
    python ex2_lqr_tvc.py --no-clip     # also show what the first K asks for
    python ex2_lqr_tvc.py --planar      # self-contained 2-D rocket, no repo needed

The script runs these steps and prints as it goes:
  1. Linearize at hover. B is printed entry by entry.
     TODO 1: write one comment per nonzero B entry saying which input moves
             which state (see linearize_hover below).
  2. Try scipy.linalg.solve_discrete_are on all 17 states. It fails. Compute
     the controllability rank (14 of 17).
     TODO 2: fill in UNCONTROLLABLE_EXPLANATION: which three states can the
             two-axis gimbal plus thrust never reach, and why.
  3. Drop those states (KEEP14) and solve for K with the repo's MPC weights.
     K and the closed-loop eigenvalue magnitudes are printed.
  4. Simulate a 0.3 m step with the inputs clipped to +/-10 deg and 1..20 N.
  5. Simulate the repo's own step to (2, -3, 10) with the same K. It diverges
     under clipping. With --no-clip you see how many degrees it wanted.
  6. TODO 3: fill in student_weights() with Q and R of your own (Bryson's rule
             is the recipe) so the big step settles under clipping. The grader
             at the end checks that run.

Stretch ideas are at the bottom of the file.

--planar runs the guide page's planar rocket (x, z, theta, vx, vz, omega;
inputs thrust and gimbal) with 50 kg, 350..1000 N and a 10 deg gimbal. It is
self-contained: numerical linearization of the continuous ODE, discretization
with the matrix exponential, DARE, closed-loop sim with actuator lags and
clipping. It shows the full recipe on a small model before you apply it to the
17-state one, and it works without the repo.

The time step matters. The repo's dynamics(x, u) is a one-step map with
dt = 0.1 s baked in, so compute_jacobian already returns DISCRETE A, B:
use solve_discrete_are on them directly, no c2d step. The planar model is a
continuous ODE, so there we discretize first. Mixing the two up is the most
common bug in this exercise.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path

import numpy as np
import scipy.linalg as sla

os.environ.setdefault("MPLBACKEND", "Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import _repo  # noqa: E402
from _repo import (KEEP14, KEEP14_NAMES, STATE_NAMES, INPUT_NAMES, U_HOVER, U_MIN, U_MAX,  # noqa: E402
                   IX, IY, IZ, IQX, IQY, IQZ, IQW, IWZ, IGPHI, IGTHETA, U_THRUST)

OUT_DIR = HERE / "outputs"


# =========================================================================== #
# Generic LQR helpers. Shipped complete; short enough to read in full.
# =========================================================================== #
def lqr_gain(A, B, Q, R):
    """
    Discrete-time LQR. Returns K such that u = -K x minimizes
    sum x'Qx + u'Ru for x_{k+1} = A x_k + B u_k.
    Raises numpy.linalg.LinAlgError when the Riccati equation has no finite
    solution, which is what happens when some state cannot be controlled.
    """
    P = sla.solve_discrete_are(A, B, Q, R)
    K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    return K


def controllability_rank(A, B) -> int:
    """Rank of [B, AB, A^2 B, ..., A^(n-1) B]. Full rank n means controllable."""
    n = A.shape[0]
    blocks = [B]
    for _ in range(n - 1):
        blocks.append(A @ blocks[-1])
    return int(np.linalg.matrix_rank(np.hstack(blocks)))


def reduce_model(A, B, keep):
    """Keep only the listed state indices."""
    keep = list(keep)
    return A[np.ix_(keep, keep)], B[keep, :]


def bryson_weights(max_state_dev, max_input):
    """
    Bryson's rule: Q_ii = 1 / (largest acceptable deviation of state i)^2,
    R_jj = 1 / (largest acceptable input j)^2. Both arguments are lists.
    The result is a starting point, not the answer; you then scale rows up
    or down by factors of 3 to 10 until the response looks right.
    """
    Q = np.diag([1.0 / d ** 2 for d in max_state_dev])
    R = np.diag([1.0 / u ** 2 for u in max_input])
    return Q, R


def closed_loop_eig_mags(A, B, K):
    return np.sort(np.abs(np.linalg.eigvals(A - B @ K)))[::-1]


def print_gain_matrix(K, state_names, input_names):
    """Print K transposed: one row per state, one column per input."""
    print(f"  {'state':<14s}" + "".join(f"{n:>12s}" for n in input_names))
    for i, name in enumerate(state_names):
        print(f"  {name:<14s}" + "".join(f"{K[j, i]:12.4f}" for j in range(K.shape[0])))


def print_eigs(mags):
    stable = np.all(mags < 1.0)
    print("  closed-loop |eig|: " + " ".join(f"{m:.3f}" for m in mags))
    print(f"  max |eig| = {mags[0]:.4f} -> {'stable (all inside the unit circle)' if stable else 'UNSTABLE'}")


# =========================================================================== #
# The 17-state repo model
# =========================================================================== #
def load_repo(explicit=None):
    """Import nonlinear_mpc and rocketdynamics_plus, or explain how to get them."""
    try:
        _repo.add_mpc_to_path(explicit)
        import nonlinear_mpc as nm
        import rocketdynamics_plus as rd
    except FileNotFoundError as e:
        print(e)
        print("\nRun with --planar for the self-contained 2-D version meanwhile.")
        return None, None
    return nm, rd


def linearize_hover(nm, plant):
    """
    A, B = compute_jacobian(x_hover, u_hover) on the given plant module.
    compute_jacobian reads the module-level name nm.rd, so swap it in and out.
    """
    saved = nm.rd
    nm.rd = plant
    try:
        A, B = nm.compute_jacobian(_repo.hover_state(), U_HOVER)
    finally:
        nm.rd = saved
    return A, B


def describe_B(B, tol=1e-9):
    print("  nonzero entries of B (one-step effect of each input on each state):")
    for i in range(B.shape[0]):
        for j in range(B.shape[1]):
            if abs(B[i, j]) > tol:
                print(f"    B[{i:2d},{j}] = {B[i, j]:8.4f}   {INPUT_NAMES[j]:>9s} -> {STATE_NAMES[i]}")
    # TODO 1: for each line printed above, write a comment here explaining it.
    # Two of them are worth a second look: the gimbal commands do not appear
    # in the attitude rows at all (why?), and theta_cmd shows up in wx while
    # phi_cmd shows up in wy (compare with the torque line in
    # rocketdynamics_plus.py and decide whether that is what you expected).
    #
    #   B[ 9,2] = 0.1   thrust -> vz         ... your explanation ...
    #   ...


# TODO 2: replace this string with your answer after step 2 prints the rank.
UNCONTROLLABLE_EXPLANATION = (
    "TODO: which three of the 17 states can the two gimbal angles plus thrust "
    "never move independently, and why? Hint: one quaternion component is fixed "
    "by the other three; one body axis gets no torque from a two-axis gimbal."
)


# The repo's MPC weights, from nonlinear_mpc.py __main__, in the 17-state order.
MPC_Q17 = np.diag([70.0, 70.0, 200.0,          # x, y, z
                   200.0, 200.0, 200.0, 200.0,  # qx, qy, qz, qw
                   50.0, 50.0, 50.0,            # vx, vy, vz
                   5.0, 5.0, 5.0,               # wx, wy, wz
                   2.0, 2.0, 5.0, 5.0])         # g_phi, g_theta, g_phi_rate, g_theta_rate
MPC_R = np.diag([400.0, 400.0, 0.1])            # theta_cmd, phi_cmd, thrust


def mpc_weights_14():
    return MPC_Q17[np.ix_(KEEP14, KEEP14)], MPC_R.copy()


# =========================================================================== #
# TODO 3: your weights for the 14-state model. Order of the 14 states:
#   x, y, z, qx, qy, vx, vy, vz, wx, wy, g_phi, g_theta, g_phi_rate, g_theta_rate
# Order of the 3 inputs: theta_cmd (rad), phi_cmd (rad), thrust (N).
#
# Bryson's rule: pick the largest deviation you are willing to tolerate for
# each state and the largest input you are willing to use, then weight by
# 1 / value^2. Write the value you chose and why next to each entry.
# For small tilts qx and qy are about half the tilt angle in radians, so a
# 0.05 quaternion deviation means roughly 6 degrees of lean.
# =========================================================================== #
def student_weights():
    """Return (Q14, R). The placeholder just returns the repo's MPC weights."""
    # max_state_dev = [
    #     1.0, 1.0, 1.0,       # x, y, z            (m)      ...why...
    #     0.05, 0.05,          # qx, qy             (-)      ...
    #     1.0, 1.0, 1.0,       # vx, vy, vz         (m/s)    ...
    #     0.5, 0.5,            # wx, wy             (rad/s)  ...
    #     0.17, 0.17,          # g_phi, g_theta     (rad)    ...
    #     1.0, 1.0,            # gimbal rates       (rad/s)  ...
    # ]
    # max_input = [0.17, 0.17, 10.0]   # theta_cmd, phi_cmd (rad), thrust (N)
    # return bryson_weights(max_state_dev, max_input)
    return mpc_weights_14()


# --------------------------------------------------------------------------- #
# Closed-loop simulation on the 17-state model. Shipped complete.
# --------------------------------------------------------------------------- #
def lqr_controller(K, keep, xref, u_hover=U_HOVER):
    """u = u_hover - K (x[keep] - xref[keep]). No clipping here; the sim clips."""
    keep = np.asarray(keep)
    xref_k = np.asarray(xref)[keep]

    def ctrl(x):
        return u_hover - K @ (np.asarray(x)[keep] - xref_k)
    return ctrl


def simulate(ctrl, x0, T, plant, clip=True, u_min=U_MIN, u_max=U_MAX):
    """
    Run the closed loop with the repo's dynamics for T seconds (dt = 0.1).
    Returns a dict: t (n+1), x (n+1, 17), u (n, 3) applied, u_raw (n, 3) asked
    for, sat (n,) True when clipping changed the command.
    """
    n = int(round(T / _repo.DT))
    x = np.array(x0, float)
    xs, us, raws, sats = [x.copy()], [], [], []
    for _ in range(n):
        u_raw = np.asarray(ctrl(x), float)
        u = np.clip(u_raw, u_min, u_max) if clip else u_raw
        sats.append(bool(np.any(u != u_raw)))
        x = plant.dynamics(x, u)
        if not np.all(np.isfinite(x)) or abs(x[IZ]) > 1e6:
            break
        xs.append(x.copy()); us.append(u); raws.append(u_raw)
    xs = np.array(xs)
    return dict(t=np.arange(len(xs)) * _repo.DT, x=xs, u=np.array(us).reshape(-1, 3),
                u_raw=np.array(raws).reshape(-1, 3), sat=np.array(sats[:len(us)], bool))


def step_metrics(hist, xref, band=0.1):
    """Settle time of the position error norm, max tilt, saturation count, and so on."""
    pos_err = np.linalg.norm(hist["x"][:, :3] - np.asarray(xref)[:3], axis=1)
    inside = pos_err < band
    if inside.any() and inside[-1]:
        out = np.where(~inside)[0]
        settle = hist["t"][out[-1]] + _repo.DT if out.size else 0.0
    else:
        settle = math.nan
    gimbal_deg = np.rad2deg(np.abs(hist["u_raw"][:, :2])) if len(hist["u_raw"]) else np.zeros((1, 2))
    return dict(
        settle_time=settle,
        final_pos_err=float(pos_err[-1]),
        max_tilt=float(np.max(np.abs(hist["x"][:, [IQX, IQY]]))),
        saturated_steps=int(hist["sat"].sum()),
        max_gimbal_deg=float(np.max(gimbal_deg)),
        thrust_range=(float(hist["u"][:, U_THRUST].min()) if len(hist["u"]) else math.nan,
                      float(hist["u"][:, U_THRUST].max()) if len(hist["u"]) else math.nan),
        finished=bool(len(hist["u"]) == round(hist["t"][-1] / _repo.DT)),
    )


def print_step_metrics(m):
    st = "never" if not math.isfinite(m["settle_time"]) else f"{m['settle_time']:.1f} s"
    print(f"  settle (pos err < 0.1 m): {st:>8s}   final pos err {m['final_pos_err']:.3f} m   "
          f"max |qx|,|qy| {m['max_tilt']:.3f}")
    print(f"  saturated steps {m['saturated_steps']:4d}   max gimbal asked {m['max_gimbal_deg']:.1f} deg   "
          f"thrust applied {m['thrust_range'][0]:.1f}..{m['thrust_range'][1]:.1f} N")


def plot_hist(hist, xref, title, save, show=False):
    t, x, u = hist["t"], hist["x"], hist["u"]
    tu = t[:len(u)]
    fig, ax = plt.subplots(4, 1, figsize=(9, 10), sharex=True)
    for i, name in enumerate("xyz"):
        ax[0].plot(t, x[:, i], label=name)
        ax[0].axhline(xref[i], color=f"C{i}", ls=":", lw=1)
    ax[0].set_ylabel("position (m)"); ax[0].legend(fontsize=8); ax[0].set_title(title)
    ax[1].plot(t, x[:, IQX], label="qx"); ax[1].plot(t, x[:, IQY], label="qy")
    ax[1].set_ylabel("tilt quaternion"); ax[1].legend(fontsize=8)
    if len(u):
        ax[2].plot(tu, np.rad2deg(u[:, 0]), label="theta cmd (applied)")
        ax[2].plot(tu, np.rad2deg(u[:, 1]), label="phi cmd (applied)")
        ax[2].plot(tu, np.rad2deg(hist["u_raw"][:, 0]), "C0:", lw=1, label="theta asked")
        ax[2].plot(tu, np.rad2deg(hist["u_raw"][:, 1]), "C1:", lw=1, label="phi asked")
    for s in (-10, 10):
        ax[2].axhline(s, color="tab:red", ls="--", lw=1)
    ax[2].set_ylabel("gimbal (deg)"); ax[2].legend(fontsize=8)
    ax[2].set_ylim(-15, 15)
    if len(u):
        ax[3].plot(tu, u[:, 2], label="thrust (applied)")
        ax[3].plot(tu, hist["u_raw"][:, 2], "C0:", lw=1, label="thrust asked")
    for s in (U_MIN[2], U_MAX[2]):
        ax[3].axhline(s, color="tab:red", ls="--", lw=1)
    ax[3].set_ylabel("thrust (N)"); ax[3].set_xlabel("time (s)"); ax[3].legend(fontsize=8)
    ax[3].set_ylim(-2, 25)
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    Path(save).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save, dpi=120); print(f"  saved {save}")
    if show:
        plt.show()
    plt.close(fig)


def plot_gimbal_compare(runs, save, show=False):
    """runs: list of (label, hist). One panel, |gimbal asked| in degrees, 10 deg line."""
    fig, ax = plt.subplots(figsize=(9, 4))
    for label, h in runs:
        if len(h["u_raw"]):
            g = np.rad2deg(np.max(np.abs(h["u_raw"][:, :2]), axis=1))
            ax.plot(h["t"][:len(g)], g, label=label)
    ax.axhline(10, color="tab:red", ls="--", lw=1, label="gimbal stop (10 deg)")
    ax.set_yscale("symlog", linthresh=10)
    ax.set_ylabel("|gimbal command asked| (deg, symlog)"); ax.set_xlabel("time (s)")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    ax.set_title("What each K asked the gimbal for")
    fig.tight_layout(); fig.savefig(save, dpi=120); print(f"  saved {save}"); plt.close(fig)
    if show:
        plt.show()


def grade_big_step(m):
    checks = [
        ("run finished without blowing up", m["finished"] and math.isfinite(m["final_pos_err"])),
        ("final position error below 0.10 m", m["final_pos_err"] < 0.10),
        ("settles within 15 s under clipping",
         math.isfinite(m["settle_time"]) and m["settle_time"] <= 15.0),
        ("max tilt |qx|,|qy| below 0.3 (about 35 deg)", m["max_tilt"] < 0.3),
    ]
    ok = True
    print("\nGrader (big step with your student_weights, clipped)")
    for name, passed in checks:
        ok &= bool(passed)
        print(f"  [{'PASS' if passed else 'FAIL'}] {name}")
    print(f"  => {'PASS' if ok else 'FAIL'}"
          + ("" if ok else "   (expected until you fill in student_weights)"))
    return ok


def run_repo_model(args):
    nm, rd = load_repo(args.repo)
    if nm is None:
        return 2
    plant = rd
    xh = _repo.hover_state()

    print("\nStep 1: linearize at hover (compute_jacobian, DISCRETE A and B, dt = 0.1 s)")
    A, B = linearize_hover(nm, plant)
    print(f"  A is {A.shape}, max |eig(A)| = {np.max(np.abs(np.linalg.eigvals(A))):.3f} "
          "(1.0 = integrators: position and attitude do not decay by themselves)")
    describe_B(B)

    print("\nStep 2: LQR on all 17 states")
    try:
        lqr_gain(A, B, MPC_Q17, MPC_R)
        print("  unexpectedly succeeded; check your scipy version")
    except (np.linalg.LinAlgError, ValueError) as e:
        print(f"  solve_discrete_are raised {type(e).__name__}: {e}")
        print("  The solver is telling you some state cannot be driven by the inputs, so the")
        print("  cost of holding it would be infinite. Check with the controllability rank.")
    r17 = controllability_rank(A, B)
    print(f"  controllability rank of the 17-state model: {r17} of 17")
    print(f"  your explanation: {UNCONTROLLABLE_EXPLANATION}")

    print("\nStep 3: drop qz, qw, wz -> 14 states, repo MPC weights")
    A14, B14 = reduce_model(A, B, KEEP14)
    print(f"  controllability rank of the 14-state model: {controllability_rank(A14, B14)} of 14")
    Q14, R = mpc_weights_14()
    K_mpc = lqr_gain(A14, B14, Q14, R)
    print("  K (shown transposed: 14 state rows x 3 input columns)")
    print_gain_matrix(K_mpc, KEEP14_NAMES, INPUT_NAMES)
    print_eigs(closed_loop_eig_mags(A14, B14, K_mpc))

    OUT_DIR.mkdir(exist_ok=True)
    print("\nStep 4: small step, x -> 0.3 m, clipped")
    xref_small = xh.copy(); xref_small[IX] = 0.3
    h_small = simulate(lqr_controller(K_mpc, KEEP14, xref_small), xh, args.T, plant, clip=True)
    print_step_metrics(step_metrics(h_small, xref_small))
    plot_hist(h_small, xref_small, "MPC weights, 0.3 m step, clipped", OUT_DIR / "ex2_small_step.png", args.show)

    print("\nStep 5: the repo's step (2, -3, 10) with the same K, clipped")
    xref_big = xh.copy(); xref_big[[IX, IY, IZ]] = [2.0, -3.0, 10.0]
    h_big_mpc = simulate(lqr_controller(K_mpc, KEEP14, xref_big), xh, args.T, plant, clip=True)
    m = step_metrics(h_big_mpc, xref_big)
    print_step_metrics(m)
    if not m["finished"]:
        print("  the run was cut short because the state blew up")
    plot_hist(h_big_mpc, xref_big, "MPC weights, (2,-3,10) step, clipped", OUT_DIR / "ex2_big_step_mpc_weights.png", args.show)
    runs = [("MPC weights, clipped", h_big_mpc)]
    if args.no_clip:
        h_free = simulate(lqr_controller(K_mpc, KEEP14, xref_big), xh, args.T, plant, clip=False)
        mf = step_metrics(h_free, xref_big)
        print("  same K without clipping:")
        print_step_metrics(mf)
        print("  It 'works' by asking for a gimbal angle no servo has. That is the number to remember.")
        runs.append(("MPC weights, no clipping", h_free))

    print("\nStep 6: your weights (student_weights) on the big step, clipped")
    Qs, Rs = student_weights()
    K_student = lqr_gain(A14, B14, Qs, Rs)
    print_gain_matrix(K_student, KEEP14_NAMES, INPUT_NAMES)
    print_eigs(closed_loop_eig_mags(A14, B14, K_student))
    h_big_student = simulate(lqr_controller(K_student, KEEP14, xref_big), xh, args.T, plant, clip=True)
    ms = step_metrics(h_big_student, xref_big)
    print_step_metrics(ms)
    plot_hist(h_big_student, xref_big, "student weights, (2,-3,10) step, clipped", OUT_DIR / "ex2_big_step_student.png", args.show)
    runs.append(("student weights, clipped", h_big_student))
    plot_gimbal_compare(runs, OUT_DIR / "ex2_gimbal_compare.png", args.show)
    return 0 if grade_big_step(ms) else 1


# =========================================================================== #
# --planar: the guide's 2-D rocket, self-contained
# =========================================================================== #
PLANAR = dict(m=50.0, L=0.8, I=25.0, thrust_min=350.0, thrust_max=1000.0,
              delta_max=np.deg2rad(10.0), tau=0.20, tau_servo=0.05, dt_ctrl=0.02, dt_sim=0.005)
PLANAR_STATES = ["x", "z", "theta", "vx", "vz", "omega"]
PLANAR_INPUTS = ["thrust", "delta"]
G = 9.81


def planar_f(x, u, p=PLANAR):
    """
    Continuous-time ODE x_dot = f(x, u). theta is tilt from vertical, positive
    toward +x. Positive gimbal delta pushes the base toward +x and torques the
    nose toward -x. u = [thrust (N), delta (rad)].
    """
    _, _, th, vx, vz, om = x
    T, d = u
    ang = th + d
    return np.array([vx, vz, om,
                     T * np.sin(ang) / p["m"],
                     T * np.cos(ang) / p["m"] - G,
                     -p["L"] * T * np.sin(d) / p["I"]])


def linearize_fd(f, x0, u0, eps=1e-6):
    """Central finite differences, same idea as the repo's compute_jacobian."""
    n, m = len(x0), len(u0)
    A, B = np.zeros((n, n)), np.zeros((n, m))
    for i in range(n):
        dx = np.zeros(n); dx[i] = eps
        A[:, i] = (f(x0 + dx, u0) - f(x0 - dx, u0)) / (2 * eps)
    for j in range(m):
        du = np.zeros(m); du[j] = eps
        B[:, j] = (f(x0, u0 + du) - f(x0, u0 - du)) / (2 * eps)
    return A, B


def discretize(A, B, dt):
    """Exact zero-order-hold discretization via the matrix exponential."""
    n, m = A.shape[0], B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n], M[:n, n:] = A * dt, B * dt
    E = sla.expm(M)
    return E[:n, :n], E[:n, n:]


# TODO (planar): these are Bryson weights for 1 m, 1 m, 0.2 rad, 1 m/s, 1 m/s,
# 0.5 rad/s and 200 N. The gimbal weight was then raised from 1/0.17^2 = 33 to
# 1000 because with 33 a 5 m offset asks for 45 deg of gimbal. Try 33 and see.
def student_weights_planar():
    Q, R = bryson_weights([1.0, 1.0, 0.2, 1.0, 1.0, 0.5], [200.0, 0.17])
    R[1, 1] = 1000.0
    return Q, R


def simulate_planar(K, x0, target, T_end, p=PLANAR):
    """Truth sim at 200 Hz with thrust lag, servo lag and clipping; LQR at 50 Hz."""
    u_hover = np.array([p["m"] * G, 0.0])
    n = int(round(T_end / p["dt_sim"]))
    div = int(round(p["dt_ctrl"] / p["dt_sim"]))
    x = np.array(x0, float)
    Tact, dact = u_hover[0], 0.0
    u = u_hover.copy(); u_raw = u_hover.copy()
    log = dict(t=np.zeros(n), x=np.zeros((n, 6)), u=np.zeros((n, 2)), u_raw=np.zeros((n, 2)))
    for k in range(n):
        if k % div == 0:
            u_raw = u_hover - K @ (x - target)
            u = np.array([np.clip(u_raw[0], p["thrust_min"], p["thrust_max"]),
                          np.clip(u_raw[1], -p["delta_max"], p["delta_max"])])
        Tact += (u[0] - Tact) * (1 - math.exp(-p["dt_sim"] / p["tau"]))
        dact += (u[1] - dact) * (1 - math.exp(-p["dt_sim"] / p["tau_servo"]))
        xd = planar_f(x, [Tact, dact], p)
        x[3:] += xd[3:] * p["dt_sim"]           # velocities first
        x[:3] += x[3:] * p["dt_sim"]            # then positions with the new velocities
        if x[1] <= 0:
            x[1] = 0.0; x[4] = max(x[4], 0.0)
        log["t"][k], log["x"][k], log["u"][k], log["u_raw"][k] = (k + 1) * p["dt_sim"], x, u, u_raw
    return log


def run_planar(args):
    p = PLANAR
    x_hover = np.zeros(6); u_hover = np.array([p["m"] * G, 0.0])
    print("\nPlanar rocket: 50 kg, L = 0.8 m, I = 25 kg m^2, thrust 350..1000 N, gimbal +/-10 deg")
    print("Step 1: linearize the continuous ODE at hover by finite differences")
    Ac, Bc = linearize_fd(planar_f, x_hover, u_hover)
    with np.printoptions(precision=3, suppress=True):
        print("  A (continuous) =\n" + str(Ac)); print("  B (continuous) =\n" + str(Bc))
    print(f"Step 2: discretize at dt = {p['dt_ctrl']} s (matrix exponential)")
    Ad, Bd = discretize(Ac, Bc, p["dt_ctrl"])
    print(f"  controllability rank: {controllability_rank(Ad, Bd)} of 6")
    print("Step 3: LQR")
    Q, R = student_weights_planar()
    print("  Q diag = " + str(np.diag(Q)) + "   R diag = " + str(np.diag(R)))
    K = lqr_gain(Ad, Bd, Q, R)
    print("  K (transposed: state rows, input columns). Note the block structure:")
    print("  thrust only looks at z and vz; the gimbal only at x, theta, vx, omega.")
    print_gain_matrix(K, PLANAR_STATES, PLANAR_INPUTS)
    print_eigs(closed_loop_eig_mags(Ad, Bd, K))

    print("Step 4: closed loop from 5 m off to the side at 10 m, actuator lags and clipping on")
    x0 = np.array([5.0, 10.0, 0, 0, 0, 0]); target = np.array([0.0, 10.0, 0, 0, 0, 0])
    log = simulate_planar(K, x0, target, args.T)
    t, X, U, Ur = log["t"], log["x"], log["u"], log["u_raw"]
    xabs = np.abs(X[:, 0]); inside = xabs < 0.1
    settle = t[np.where(~inside)[0][-1]] if (~inside).any() and inside[-1] else (math.nan if not inside[-1] else 0.0)
    peak_tilt = np.rad2deg(np.max(np.abs(X[:, 2]))); peak_cmd = np.rad2deg(np.max(np.abs(Ur[:, 1])))
    print(f"  |x| < 0.1 m after {settle:.2f} s   peak tilt {peak_tilt:.1f} deg   "
          f"peak gimbal asked {peak_cmd:.1f} deg   thrust {U[:, 0].min():.0f}..{U[:, 0].max():.0f} N   "
          f"altitude dip {10 - X[:, 1].min():.3f} m")

    OUT_DIR.mkdir(exist_ok=True)
    fig, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    ax[0].plot(t, X[:, 0], label="x"); ax[0].plot(t, X[:, 1], label="z"); ax[0].set_ylabel("m"); ax[0].legend(fontsize=8)
    ax[0].set_title("planar LQR, 5 m lateral step")
    ax[1].plot(t, np.rad2deg(X[:, 2]), label="tilt theta")
    ax[1].plot(t, np.rad2deg(U[:, 1]), label="gimbal applied"); ax[1].plot(t, np.rad2deg(Ur[:, 1]), ":", label="gimbal asked")
    for s in (-10, 10):
        ax[1].axhline(s, color="tab:red", ls="--", lw=1)
    ax[1].set_ylabel("deg"); ax[1].legend(fontsize=8)
    ax[2].plot(t, U[:, 0], label="thrust applied"); ax[2].plot(t, Ur[:, 0], ":", label="thrust asked")
    for s in (p["thrust_min"], p["thrust_max"]):
        ax[2].axhline(s, color="tab:red", ls="--", lw=1)
    ax[2].set_ylabel("N"); ax[2].set_xlabel("time (s)"); ax[2].legend(fontsize=8)
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout(); out = OUT_DIR / "ex2_planar.png"; fig.savefig(out, dpi=120); print(f"  saved {out}")
    if args.show:
        plt.show()
    plt.close(fig)

    checks = [("|x| settles below 0.1 m within 8 s", math.isfinite(settle) and settle <= 8.0),
              ("peak tilt below 20 deg", peak_tilt < 20.0),
              ("gimbal never asked past its 10 deg stop", peak_cmd <= 10.0 + 1e-6)]
    ok = True
    print("\nGrader (planar)")
    for name, passed in checks:
        ok &= bool(passed); print(f"  [{'PASS' if passed else 'FAIL'}] {name}")
    print(f"  => {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


# =========================================================================== #
def main(argv=None):
    ap = argparse.ArgumentParser(description="Exercise 2: LQR on the TVC rocket")
    ap.add_argument("--planar", action="store_true", help="self-contained 2-D rocket instead of the repo model")
    ap.add_argument("--repo", default=None, metavar="PATH", help="path to the MonopropUAV clone (or its Algorithms/MPC)")
    ap.add_argument("--no-clip", action="store_true", help="also run the big step without clipping")
    ap.add_argument("--T", type=float, default=None, help="simulation length in seconds")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args(argv)
    if args.planar:
        args.T = args.T or 15.0
        return run_planar(args)
    args.T = args.T or 30.0
    return run_repo_model(args)


if __name__ == "__main__":
    raise SystemExit(main())


# =========================================================================== #
# Stretch ideas (pick one, describe what you found in RESULTS.md)
#   A. Error clipping: clip the position error to +/-0.5 m before multiplying
#      by K. It rescues the MPC-weight K on the big step, slowly. Why is this a
#      hack, and what does MPC do instead?
#   B. Re-linearize every step (compute_jacobian at x_k, u_k, re-solve the
#      DARE) and compare with the fixed K. A 14-state DARE takes milliseconds.
#   C. Reproduce K with the 50-iteration Riccati recursion in
#      Sensing&Controls/Controllers/LQR.py and show it converges to the DARE K.
#   D. The 8-state planar cut of the repo model: indices [0,2,4,7,9,11,14,16],
#      inputs theta_cmd and thrust. Linearize and simulate it on
#      rocketdynamics.py (set nm.rd = rocketdynamics), then run the same K on
#      rocketdynamics_plus.py and explain the sideways drift.
# =========================================================================== #

# Controls take-home exercises

Three Python files that go with the "Control, from the ground up" page. They
get harder in order, but start wherever you like. Budget about two hours for
the first and three for each of the others, more if you do the stretch goals.

| file | what you build | needs the repo? |
|---|---|---|
| `ex1_hover_pid.py` | PID plus gravity feedforward on a 1-D hover rocket | no (stretch only) |
| `ex2_lqr_tvc.py` | LQR on the repo's 17-state TVC rocket | yes (`--planar` works without it) |
| `ex3_mpc_receding_horizon.py` | the receding-horizon loop around the repo's `nmpc_step` | yes |

`_repo.py` is a shared helper (finds the repo, index constants, metrics). You do
not edit it. Plots land in `outputs/`, which is gitignored.

## Setup

1. Python 3.12 or newer. `python --version` to check.
2. A virtual environment, then three packages:

   ```
   python -m venv .venv
   .venv\Scripts\activate          # Windows;  source .venv/bin/activate on macOS/Linux
   pip install numpy scipy matplotlib
   ```

3. For exercises 2 and 3, a clone of the team repo:

   ```
   git clone https://github.com/Avionics-Propulsion-Landers-GT/MonopropUAV.git
   ```

   The scripts look for it (a) inside the repo if these files were committed
   under `Tutorials/Controls/exercises/`, (b) in the environment variable
   `MONOPROP_REPO`, (c) as a sibling folder `MonopropUAV` next to `Tutorials/`.
   Or pass `--repo <path to your clone>`. If it is not found the script says
   so and stops; nothing else is required (no CoolProp, no MATLAB, no Rust).

Everything runs headless and writes PNGs. Set `MPLBACKEND=Agg` if matplotlib
tries to open a window on your machine (the scripts set it themselves unless
you pass `--show`).

## Running

### Exercise 1

```
python ex1_hover_pid.py                                # placeholder: hovers, ignores the step, FAIL
python ex1_hover_pid.py --kp 2 --no-ff                 # P only: watch it bounce
python ex1_hover_pid.py --kp 2 --kd 3 --no-ff          # PD, no feedforward: parks m g / kp low
python ex1_hover_pid.py --kp 2 --kd 3                  # PD + feedforward
python ex1_hover_pid.py --kp 2 --ki 0.5 --kd 3         # PID + feedforward (close, but retune to pass)
python ex1_hover_pid.py --kp 2 --ki 0.5 --kd 3 --mass-error 5
python ex1_hover_pid.py --kp 2 --ki 0.5 --kd 3 --gust
```

The default plant is the page's rocket: 1 kg, thrust 1..20 N, a 0.2 s thrust lag,
hovering at 10 m with a step to 15 m at t = 1 s. Start from the gains you found
on the page (`--kp 2 --ki 0.5 --kd 3`). They get you close, but the grader is
stricter than the sandbox, so expect to retune: more Kp and Kd, less Ki.
`--plant repo` is the same 1 kg toy with no lag and the repo's 0.1 s step;
`--repo-model` overlays the repo's 17-state dynamics on it, and the two traces
should be identical for pure vertical flight.

The grader wants: settle inside 0.25 m within 6 s, overshoot under 15 percent
of the step, steady-state error under 0.15 m over the last 2 s, and with `--gust`
an excursion under 1 m and recovery within 6 s. It is meant to be passable with
a plain PID plus feedforward; if you find yourself adding tricks, ask in #gnc.

### Exercise 2

```
python ex2_lqr_tvc.py                # 17-state model: DARE failure, rank, K, small step, big step, grader
python ex2_lqr_tvc.py --no-clip      # also print how many degrees the first K asks for
python ex2_lqr_tvc.py --planar       # the page's 6-state planar rocket, no repo needed
```

Three TODOs in the file: explain each nonzero entry of B, name the three
uncontrollable states, and fill in `student_weights()` so the (2, -3, 10) step
settles with the inputs clipped. The grader checks the last one.

### Exercise 3

```
python ex3_mpc_receding_horizon.py                 # 60 steps, about 3 s
python ex3_mpc_receding_horizon.py --thrust-max 12 # TODO 1
python ex3_mpc_receding_horizon.py --profile step  # TODO 2, after you write it
python ex3_mpc_receding_horizon.py --compare-lqr   # TODO 3, uses your ex2 weights
python ex3_mpc_receding_horizon.py --mass 1.05     # stretch: heavier plant, same MPC model
python ex3_mpc_receding_horizon.py --steps 300     # the full repo demo, about a minute
```

One solver step is about 35 ms in Python. Keep `--steps` small while you
experiment.

## What to submit

Same flow as the git tutorial: fork, branch, edit, pull request.

1. Fork `Avionics-Propulsion-Landers-GT/MonopropUAV` (or branch directly if you
   have write access) and create a branch named `controls-tutorial/<your-github-handle>`.
2. Put your work in `Tutorials/Controls/submissions/<your-github-handle>/`:
   the three `.py` files copied from here and filled in, the PNGs you want
   reviewed, and a `RESULTS.md` with the metric tables the scripts print and
   three to five sentences of "what surprised me". Do not edit anything under
   `Algorithms/` in the same PR; repo fixes from the stretch goals go in a
   separate PR.
3. Every script must run from its own folder with no absolute paths and no
   `plt.show()` unless `--show` is passed. No `__pycache__`, no `.venv`.
4. Open a PR against `main` titled `Controls tutorial: <your name>` and ask in
   #gnc for a review.

A good warm-up PR before any of this: fix the stale state-order comments in
`Algorithms/MPC/rocketdynamics.py` (line 6) and `nonlinear_mpc.py` (lines 288,
300-302, 332). Small, real, and a second pass through the git flow.

## FAQ

**The import fails with `No module named rocketdynamics_plus`.**
`nonlinear_mpc.py` does a bare `import rocketdynamics_plus`, so
`Algorithms/MPC` has to be on `sys.path`. `_repo.add_mpc_to_path()` does that;
importing `nonlinear_mpc` by file path alone will not work. Never import from
`Algorithms/MTV Sim`: the folder name has a space, it needs CoolProp and an
executable at a hard-coded path on the author's machine, and
`pid_tuning_runner.py` points at a file that is spelled differently on disk.
Read its `PIDController` class, do not run it.

**Which Riccati solver, and do I discretize?**
The repo's `dynamics(x, u)` is a one-step map with `dt = 0.1` baked in, so
`compute_jacobian` already returns discrete A and B. Use
`scipy.linalg.solve_discrete_are` on them directly. No `c2d`, no
`solve_continuous_are`. The planar model in `ex2 --planar` is a continuous ODE,
so there you discretize first (`scipy.linalg.expm`). The MATLAB files under
`Algorithms/LQR` call `c2d` because their A and B are continuous; different
world. Also: the servo sub-model inside `dynamics()` is only stable for
`dt` up to about 0.14 s. Do not copy the model and raise `dt`.

**The DARE says `Failed to find a finite solution`.**
That is exercise 2, step 2. The 17-state model has controllability rank 14.
`qw` is fixed by the other three quaternion components, and a two-axis gimbal
produces no yaw torque, so `qz` and `wz` cannot be moved. Drop indices 5, 6, 12
(`_repo.KEEP14`).

**Quaternion order and normalization.**
Scalar last: `[qx, qy, qz, qw]` at indices 3..6, level is `x[6] = 1`. The
comments in `nonlinear_mpc.py` say otherwise; the code is right. If you start
from `np.zeros(17)` and forget `x[6] = 1`, `dynamics()` silently replaces the
zero quaternion with identity, so the bug hides until you look at a plot. For
small tilts `qx` and `qy` are about half the tilt angle in radians, which is
why the LQR can use them directly as the attitude error against a level
reference. For a non-level reference you need the error quaternion, not
`q - q_ref`.

**Gimbal order.**
Inputs are `[theta_cmd, phi_cmd, thrust]` but the state stores `g_phi` at 13
and `g_theta` at 14 (rates at 15, 16 in the same order). Also, in
`rocketdynamics_plus.py` the theta servo's reaction torque acts on body x while
theta's thrust torque acts on body y (`B[10, 0] = -2.5`). A planar controller
built on `_plus` drifts sideways; use `rocketdynamics.py` for planar work or
keep the full 14-state model.

**The MPC hovers a few centimetres low even with a perfect model.**
Its cost penalizes absolute thrust, `u'Ru`, not `u - u_hover`, and it has no
integrator. Five centimetres at 1 kg; 0.7 m with a 5 percent heavier plant.
That is the stretch goal in exercise 3, not a bug in your loop.

**Saturation and units.**
The plant only enforces `thrust >= 0`. Clip to 1..20 N and plus or minus 10
degrees in the loop, in radians (`np.deg2rad(10) = 0.1745`), and plot the
gimbal in degrees. The 1 kg toy, the 80 kg Rust demo and the 1 kN Monarch do
not share gains or weights; compare shapes, not numbers.

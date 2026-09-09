"""
Exercise 1: hold altitude with PID and gravity feedforward.

This is the 1-D hover rocket from the guide page, in Python. The plant is a
mass with an engine that has a maximum thrust, a minimum thrust (a hybrid
flames out below it), and a first-order lag between the commanded thrust and
the thrust you actually get. The file already contains the rocket, the
scenario, the plotting and a small grader. You write the controller.

Default plant (matches the web sims):  1 kg, 1..20 N, 0.2 s thrust lag, so
                                        the gains you found on the page
                                        (kp 2, ki 0.5, kd 3) carry over.
Optional plant --plant repo:            the same 1 kg toy with no lag and the
                                        repo's 0.1 s explicit Euler step, so
                                        the stretch goal can compare against
                                        Algorithms/MPC/rocketdynamics_plus.py.

What to do
  1. Run the file once as it is. The placeholder controller commands the
     rocket's weight, so it hovers wherever it starts and ignores the step.
  2. Fill in controller() below: PID on the altitude error plus gravity
     feedforward. Use the state dict to remember the integral between calls.
  3. Run P only, then PD, then PD + feedforward, then PID + feedforward
     (see the README for the gain values on each plant). Read the settle
     time, overshoot and steady-state error the script prints.
  4. Run again with --mass-error 5 and with --gust. Watch the gap come back
     without I, and close with it.
  5. Stretch: --repo-model runs your controller against the repo's 17-state
     rocketdynamics_plus.dynamics (needs --plant repo, which uses the same
     0.1 s step and the same explicit Euler as the repo). For pure vertical
     flight the two traces should be identical.

Run examples
  python ex1_hover_pid.py
  python ex1_hover_pid.py --kp 2 --kd 3 --no-ff
  python ex1_hover_pid.py --mass-error 5
  python ex1_hover_pid.py --gust --out outputs/ex1_gust.png
  python ex1_hover_pid.py --plant repo --kp 2 --kd 3 --ki 0.5 --repo-model

Every run writes a PNG (default outputs/ex1_hover.png) and never opens a
window unless you pass --show.
"""
from __future__ import annotations

import argparse
import math
import os
from pathlib import Path

import numpy as np

os.environ.setdefault("MPLBACKEND", "Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent

# --------------------------------------------------------------------------- #
# Default gains. These are what the controller reads unless you override them
# on the command line with --kp/--ki/--kd. Starting values are deliberately
# zero so the placeholder run does nothing clever.
# --------------------------------------------------------------------------- #
KP = 0.0      # N per m of error
KI = 0.0      # N per (m * s) of accumulated error
KD = 0.0      # N per (m/s) of climb rate


# =========================================================================== #
# TODO: this is the only function you must write.
# =========================================================================== #
def controller(t, z, vz, target, dt, state) -> float:
    """
    Return the thrust command in newtons.

    t       time (s)
    z       measured altitude (m)
    vz      measured climb rate (m/s). Think of it as the EKF velocity estimate.
            Using it for the D term is the same as differentiating the
            measurement, and it has no derivative kick on setpoint steps.
    target  altitude setpoint (m)
    dt      time since the last call (s)
    state   a dict that survives between calls. It arrives pre-filled with
              state["kp"], state["ki"], state["kd"]    gains (from the CLI)
              state["mass"], state["g"]                what the controller
                                                       believes about the rocket
              state["thrust_min"], state["thrust_max"] engine limits
              state["use_ff"]                          False when --no-ff is set
            Put anything you want to remember in it (the integral, for example).

    Plan:
      error = target - z
      P = kp * error
      D = kd * (0 - vz)                      # d(error)/dt for a constant target
      I: state["integral"] += error * dt, then ki * integral. Two rules that
         every flight PID has: clamp the integral so ki * integral can never
         exceed the engine range, and do not integrate while the command is
         saturated in the same direction as the error (anti-windup).
      feedforward = mass * g if state["use_ff"] else 0
      return feedforward + P + I + D          # the plant clips it to the limits
    """
    # ---- placeholder: just hold the weight. Replace everything below. ------
    return state["mass"] * state["g"]
    # -----------------------------------------------------------------------


# --------------------------------------------------------------------------- #
# The plant. Shipped complete; read it, do not edit it.
# --------------------------------------------------------------------------- #
PLANTS = {
    # mass kg, thrust limits N, thrust lag s, physics step s, controller every
    # n physics steps, and which Euler flavour the integrator uses.
    "web":  dict(mass=1.0, thrust_min=1.0, thrust_max=20.0, tau=0.20,
                 dt=0.005, ctrl_div=2, explicit_euler=False),   # the guide's hover rocket, 100 Hz control
    "repo": dict(mass=1.0, thrust_min=1.0, thrust_max=20.0, tau=0.0,
                 dt=0.1, ctrl_div=1, explicit_euler=True),      # Algorithms/MPC toy: dt 0.1 and plain
                                                                # explicit Euler, exactly like dynamics()
}
G = 9.81


class Hover1D:
    """Vertical rocket: m z'' = T - m g + F_dist, T lags the command."""

    def __init__(self, mass=1.0, g=G, thrust_min=1.0, thrust_max=20.0,
                 dt=0.005, thrust_tau=0.2, z0=0.0, thrust0=None, explicit_euler=False):
        self.m, self.g = mass, g
        self.thrust_min, self.thrust_max = thrust_min, thrust_max
        self.dt, self.tau = dt, thrust_tau
        self.explicit_euler = explicit_euler
        self.z, self.v = z0, 0.0
        self.T = mass * g if thrust0 is None else thrust0   # actual thrust
        self.t = 0.0
        self.saturated = False

    def step(self, thrust_cmd: float, disturbance: float = 0.0):
        """Advance one physics step. Returns (z, v)."""
        Tc = min(max(thrust_cmd, self.thrust_min), self.thrust_max)
        self.saturated = Tc != thrust_cmd
        if self.tau > 0:
            # exact first-order lag for a command held over dt
            self.T += (Tc - self.T) * (1.0 - math.exp(-self.dt / self.tau))
        else:
            self.T = Tc
        a = (self.T - self.m * self.g + disturbance) / self.m
        if self.explicit_euler:
            # plain explicit Euler: position uses the OLD velocity (what the repo model does)
            self.z += self.v * self.dt
            self.v += a * self.dt
        else:
            # semi-implicit Euler: velocity first, then position with the new velocity
            self.v += a * self.dt
            self.z += self.v * self.dt
        if self.z <= 0.0:                   # the ground
            self.z = 0.0
            if self.v < 0.0:
                self.v = 0.0
        self.t += self.dt
        return self.z, self.v


# --------------------------------------------------------------------------- #
# Scenario, metrics, grader, plotting. Shipped complete.
# --------------------------------------------------------------------------- #
T_END = 30.0
T_STEP = 1.0            # setpoint jumps at this time
GUST_N_FRACTION = -0.10 # gust force as a fraction of weight (downdraft)
GUST_WINDOW = (10.0, 13.0)
SETTLE_BAND = 0.25      # m


def run_sim(ctrl, plant: Hover1D, z_target, z_start, ctrl_state,
            gust=False, t_end=T_END, t_step=T_STEP, ctrl_div=2):
    """Run the closed loop; the controller runs every ctrl_div physics steps."""
    n = int(round(t_end / plant.dt))
    log = {k: np.zeros(n) for k in ("t", "z", "v", "T", "cmd", "ref", "sat", "dist")}
    ctrl_dt = plant.dt * ctrl_div
    cmd = plant.m * plant.g
    gust_force = GUST_N_FRACTION * ctrl_state["mass"] * G
    for k in range(n):
        t = k * plant.dt                 # k * dt, not an accumulated sum: 10 * 0.1 == 1.0 exactly
        ref = z_target if t >= t_step else z_start
        dist = gust_force if (gust and GUST_WINDOW[0] <= t < GUST_WINDOW[1]) else 0.0
        if k % ctrl_div == 0:
            cmd = float(ctrl(t, plant.z, plant.v, ref, ctrl_dt, ctrl_state))
            if not math.isfinite(cmd):
                raise ValueError(f"controller returned {cmd} at t={t:.2f}")
        z, v = plant.step(cmd, dist)
        log["t"][k], log["z"][k], log["v"][k] = (k + 1) * plant.dt, z, v     # state after the step
        log["T"][k], log["cmd"][k], log["ref"][k] = plant.T, cmd, ref
        log["sat"][k], log["dist"][k] = plant.saturated, dist
    return log


def analyze(log, z_target, z_start, gust=False):
    """Settle time, overshoot, RMS error, steady-state error, gust dip."""
    t, z = log["t"], log["z"]
    after = t >= T_STEP
    err = z[after] - z_target
    step_size = abs(z_target - z_start)
    direction = 1.0 if z_target >= z_start else -1.0
    out = {"step_size": step_size}
    out["rms_error"] = float(np.sqrt(np.mean(err ** 2)))
    out["overshoot"] = float(max(0.0, np.max(direction * err)))
    out["overshoot_pct"] = 100.0 * out["overshoot"] / step_size if step_size > 0 else 0.0

    # settle: last time the altitude was outside the band, measured from the step.
    # If a gust is on, measure settling on the window before the gust.
    t_lim = GUST_WINDOW[0] if gust else t[-1] + 1
    sel = after & (t < t_lim)
    outside = np.abs(z[sel] - z_target) > SETTLE_BAND
    if outside.any() and outside[-1]:
        out["settle_time"] = math.nan
    elif outside.any():
        out["settle_time"] = float(t[sel][np.where(outside)[0][-1]] - T_STEP)
    else:
        out["settle_time"] = 0.0

    tail = t >= t[-1] - 2.0
    out["ss_error"] = float(np.mean(z[tail] - z_target))
    out["sat_fraction"] = float(np.mean(log["sat"][after]))

    if gust:
        g0, g1 = GUST_WINDOW
        during = (t >= g0) & (t < g1 + 6.0)
        out["gust_dip"] = float(np.max(np.abs(z[during] - z_target)))
        back = np.where((t >= g1) & (np.abs(z - z_target) <= SETTLE_BAND))[0]
        # recovered when inside the band and stays inside afterwards
        recover = math.nan
        for i in back:
            if np.all(np.abs(z[i:] - z_target) <= SETTLE_BAND):
                recover = float(t[i] - g1)
                break
        out["gust_recovery"] = recover
    return out


def grade(res, gust=False, from_pad=False):
    """Print PASS/FAIL lines. Returns True if everything passed."""
    if from_pad:
        print("\nGrader: not run for --from-pad. A 10 m climb from rest saturates any sane gain set,")
        print("so compare the numbers above with your hover-start run instead.")
        return True
    settle_limit = 6.0
    checks = [
        ("settles within %.0f s (band +/- %.2f m)" % (settle_limit, SETTLE_BAND),
         math.isfinite(res["settle_time"]) and res["settle_time"] <= settle_limit,
         "%.2f s" % res["settle_time"] if math.isfinite(res["settle_time"]) else "never"),
        ("overshoot below 15 percent of the step",
         res["overshoot_pct"] < 15.0,
         "%.2f m = %.1f %%" % (res["overshoot"], res["overshoot_pct"])),
        ("steady-state error within 0.15 m (last 2 s)",
         abs(res["ss_error"]) <= 0.15,
         "%+.3f m" % res["ss_error"]),
    ]
    if gust:
        checks.append(("gust: altitude excursion below 1.0 m",
                       res["gust_dip"] < 1.0, "%.2f m" % res["gust_dip"]))
        checks.append(("gust: back inside the band within 6 s of the gust ending",
                       math.isfinite(res["gust_recovery"]) and res["gust_recovery"] <= 6.0,
                       "%.2f s" % res["gust_recovery"] if math.isfinite(res["gust_recovery"]) else "never"))
    ok = True
    print("\nGrader")
    for name, passed, value in checks:
        ok &= bool(passed)
        print(f"  [{'PASS' if passed else 'FAIL'}] {name:<58s} {value}")
    print(f"  => {'PASS' if ok else 'FAIL'}")
    return ok


def plot_run(log, res, plant_params, title, save, extra=None, show=False):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6.5), sharex=True)
    ax1.plot(log["t"], log["ref"], "k--", lw=1, label="setpoint")
    ax1.plot(log["t"], log["z"], lw=1.6, label="altitude")
    if extra is not None:
        ax1.plot(extra["t"], extra["z"], ":", lw=1.6, label=extra["label"])
    ax1.fill_between(log["t"], log["ref"] - SETTLE_BAND, log["ref"] + SETTLE_BAND,
                     color="0.85", label="settle band")
    if np.any(log["dist"] != 0):
        on = log["dist"] != 0
        ax1.axvspan(log["t"][on][0], log["t"][on][-1], color="tab:red", alpha=0.08, label="gust")
    ax1.set_ylabel("altitude (m)")
    ax1.set_title(title)
    ax1.legend(loc="lower right", fontsize=8)
    ax1.grid(alpha=0.3)

    tmin, tmax = plant_params[1], plant_params[2]
    ax2.plot(log["t"], log["cmd"], lw=1, alpha=0.6, label="commanded thrust")
    ax2.plot(log["t"], log["T"], lw=1.6, label="actual thrust (lagged)")
    ax2.axhline(tmin, color="tab:red", lw=1, ls="--", label="flame-out floor")
    ax2.axhline(tmax, color="tab:red", lw=1, ls="--")
    ax2.axhline(plant_params[0] * G, color="0.5", lw=1, ls=":", label="hover thrust")
    ax2.set_ylabel("thrust (N)")
    ax2.set_xlabel("time (s)")
    ax2.legend(loc="upper right", fontsize=8)
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    save = Path(save)
    save.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save, dpi=120)
    print(f"saved {save}")
    if show:
        plt.show()
    plt.close(fig)


# --------------------------------------------------------------------------- #
# Stretch: the same controller on the repo's 17-state model (vertical only)
# --------------------------------------------------------------------------- #
def run_on_repo_model(ctrl, ctrl_state, z_target, z_start, t_end=T_END, t_step=T_STEP):
    """
    Drive Algorithms/MPC/rocketdynamics_plus.dynamics with u = [0, 0, thrust].
    Its dt is 0.1 s and lives inside dynamics(), so the controller is called
    at 10 Hz here. Returns a dict with t, z or None if the repo is not found.
    """
    try:
        import _repo
        _repo.add_mpc_to_path()
        import rocketdynamics_plus as rd
    except (FileNotFoundError, ImportError) as e:
        print(f"\n[repo model skipped] {e}")
        return None
    x = _repo.hover_state()
    x[_repo.IZ] = z_start
    n = int(round(t_end / _repo.DT))
    t_log, z_log = np.zeros(n), np.zeros(n)
    for k in range(n):
        t = k * _repo.DT
        ref = z_target if t >= t_step else z_start
        cmd = float(ctrl(t, x[_repo.IZ], x[_repo.IVZ], ref, _repo.DT, ctrl_state))
        cmd = min(max(cmd, ctrl_state["thrust_min"]), ctrl_state["thrust_max"])
        x = rd.dynamics(x, np.array([0.0, 0.0, cmd]))
        t_log[k], z_log[k] = t + _repo.DT, x[_repo.IZ]
    # Question for you: what happens if you start from x = zeros(17) and forget x[6] = 1?
    return {"t": t_log, "z": z_log, "label": "repo rocketdynamics_plus (dt 0.1)"}


# --------------------------------------------------------------------------- #
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--plant", choices=PLANTS, default="web",
                    help="web = the page's 1 kg rocket with a 0.2 s thrust lag (default); repo = the same toy with no lag and the repo's 0.1 s step")
    ap.add_argument("--kp", type=float, default=KP)
    ap.add_argument("--ki", type=float, default=KI)
    ap.add_argument("--kd", type=float, default=KD)
    ap.add_argument("--no-ff", action="store_true", help="turn gravity feedforward off")
    ap.add_argument("--mass-error", type=float, default=0.0, metavar="PCT",
                    help="real mass is this many percent heavier than the controller thinks")
    ap.add_argument("--gust", action="store_true",
                    help="downdraft of 10 percent of the weight from t=10 to 13 s")
    ap.add_argument("--from-pad", action="store_true",
                    help="start on the ground and climb to the target (harder)")
    ap.add_argument("--target", type=float, default=None, help="altitude setpoint (m)")
    ap.add_argument("--repo-model", action="store_true",
                    help="stretch: also run on the repo's rocketdynamics_plus and overlay")
    ap.add_argument("--out", default=None, metavar="PATH", help="PNG path")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args(argv)

    P = PLANTS[args.plant]
    mass_nom, tmin, tmax, tau = P["mass"], P["thrust_min"], P["thrust_max"], P["tau"]
    mass_true = mass_nom * (1.0 + args.mass_error / 100.0)
    z_start = 0.0 if args.from_pad else 10.0
    z_target = args.target if args.target is not None else (10.0 if args.from_pad else 15.0)

    ctrl_state = {
        "kp": args.kp, "ki": args.ki, "kd": args.kd,
        "mass": mass_nom, "g": G, "thrust_min": tmin, "thrust_max": tmax,
        "use_ff": not args.no_ff,
    }
    plant = Hover1D(mass=mass_true, thrust_min=tmin, thrust_max=tmax, thrust_tau=tau,
                    dt=P["dt"], explicit_euler=P["explicit_euler"],
                    z0=z_start, thrust0=(mass_nom * G if not args.from_pad else tmin))

    print(f"plant {args.plant}: mass {mass_true:.2f} kg (controller assumes {mass_nom:.2f}), "
          f"thrust {tmin:.0f}..{tmax:.0f} N, lag {tau:.2f} s, controller at {1 / (P['dt'] * P['ctrl_div']):.0f} Hz")
    print(f"gains kp={args.kp} ki={args.ki} kd={args.kd} feedforward={'off' if args.no_ff else 'on'}"
          f"{'  gust on' if args.gust else ''}")
    print(f"scenario: z {z_start:.1f} m -> {z_target:.1f} m at t = {T_STEP:.0f} s, {T_END:.0f} s run")

    base_state = dict(ctrl_state)          # a clean copy for the optional second run
    log = run_sim(controller, plant, z_target, z_start, ctrl_state, gust=args.gust,
                  ctrl_div=P["ctrl_div"])
    res = analyze(log, z_target, z_start, gust=args.gust)

    print("\nResults")
    print(f"  RMS error after step   {res['rms_error']:.3f} m")
    print(f"  overshoot              {res['overshoot']:.3f} m ({res['overshoot_pct']:.1f} % of a {res['step_size']:.1f} m step)")
    st = res["settle_time"]
    print(f"  settle time (+/-{SETTLE_BAND} m)  {'never' if not math.isfinite(st) else f'{st:.2f} s'}")
    print(f"  steady-state error     {res['ss_error']:+.3f} m")
    print(f"  saturated              {100 * res['sat_fraction']:.0f} % of the time")
    if args.gust:
        rec = res["gust_recovery"]
        print(f"  gust excursion         {res['gust_dip']:.2f} m, recovered "
              f"{'never' if not math.isfinite(rec) else f'{rec:.2f} s after it ended'}")
    if args.no_ff and args.ki == 0:
        print("  note: without feedforward or an I term the rocket parks m*g/kp below the target")

    ok = grade(res, gust=args.gust, from_pad=args.from_pad)

    extra = None
    if args.repo_model:
        if args.plant != "repo":
            print("\n--repo-model only makes sense with --plant repo (1 kg, 1..20 N, no lag)")
        else:
            extra = run_on_repo_model(controller, dict(base_state), z_target, z_start)
            if extra is not None:
                zi = np.interp(extra["t"], log["t"], log["z"])
                print(f"\nrepo model vs toy: max |dz| = {np.max(np.abs(zi - extra['z'])):.2e} m "
                      f"(should be ~0: same dt, same integrator, thrust-only input)")

    tag = "_".join(filter(None, [
        "ex1_hover", args.plant if args.plant != "web" else "",
        "noff" if args.no_ff else "", f"mass{args.mass_error:g}" if args.mass_error else "",
        "gust" if args.gust else "", "pad" if args.from_pad else ""]))
    out = args.out or (HERE / "outputs" / f"{tag}.png")
    title = (f"{args.plant} plant  kp={args.kp:g} ki={args.ki:g} kd={args.kd:g}  "
             f"FF {'off' if args.no_ff else 'on'}  mass err {args.mass_error:+g} %")
    plot_run(log, res, (mass_nom, tmin, tmax), title, out, extra=extra, show=args.show)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

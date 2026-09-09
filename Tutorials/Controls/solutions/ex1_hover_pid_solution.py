"""
Solution key for exercise 1 (leads only). Imports the student file and swaps in
a finished controller, so run it exactly like the exercise:

  python ex1_hover_pid_solution.py [--mass-error 5] [--gust] [--plant repo] ...

Gains scale with the mass the controller believes (kp = 6 m, ki = 0.1 m,
kd = 8 m), so 6 / 0.1 / 8 on the 1 kg rocket. Stiffer than the page's teaching
gains (2 / 0.5 / 3) because the grader wants a 6 s settle and under 15 percent
overshoot on a 5 m step, and the integrator's time constant is roughly kp / ki.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "exercises"))
import ex1_hover_pid as ex  # noqa: E402


def controller(t, z, vz, target, dt, state):
    kp, ki, kd = state["kp"], state["ki"], state["kd"]
    error = target - z
    P = kp * error
    D = kd * (0.0 - vz)                       # derivative of the measurement
    ff = state["mass"] * state["g"] if state["use_ff"] else 0.0

    integ = state.get("integral", 0.0)
    I = ki * integ
    unsat = ff + P + I + D
    sat = 1 if unsat > state["thrust_max"] else (-1 if unsat < state["thrust_min"] else 0)
    if not ((sat == 1 and error > 0) or (sat == -1 and error < 0)):   # anti-windup
        integ += error * dt
    if ki > 0:                                                         # integral clamp
        lim = (state["thrust_max"] - state["thrust_min"]) / ki
        integ = max(-lim, min(lim, integ))
    state["integral"] = integ
    return ff + P + ki * integ + D


ex.controller = controller

if __name__ == "__main__":
    argv = sys.argv[1:]
    if not any(a.startswith(("--kp", "--ki", "--kd")) for a in argv):
        plant = "repo" if "repo" in argv else "web"
        m = ex.PLANTS[plant]["mass"]
        argv += ["--kp", str(6 * m), "--ki", str(0.1 * m), "--kd", str(8 * m)]
    raise SystemExit(ex.main(argv))

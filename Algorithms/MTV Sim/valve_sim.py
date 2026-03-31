"""
Simulates the angular movement of a Valworx 60° V-port ball valve driven by
Reefs Triple7 servos (2:1 servo-to-valve ratio)

Outputs:
  - Matplotlib plot of chamber pressure, servo angle, and valve open % vs. time
  - CSV log in ./logs/<YYYY-MM-DD_HH-MM>.csv with raw simulation data
"""

import csv
import os
import math
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

# Servo travel: 0° – 180°  (fully closed → fully open)
# Valve travel: 0° – 90°   (2:1 ratio: valve_angle = servo_angle / 2)
SERVO_MIN_DEG = 0.0
SERVO_MAX_DEG = 180.0
VALVE_MAX_DEG = 90.0        # 100% open


def valve_open_fraction(servo_angle_deg: float) -> float:
    valve_angle = servo_angle_deg / 2.0           # 2:1 mechanical ratio
    fraction = valve_angle / VALVE_MAX_DEG        # normalise to 0–1
    return max(0.0, min(1.0, fraction))

# Polynomial fit for 1/2" 60° V-port ball valve:
#   Cv = Ax^2 + Bx + C,   x = open fraction
CV_A =  8.1790
CV_B = -3.3599
CV_C =  0.2728


def cv_from_open_fraction(x: float) -> float:
    """Return Cv for the 60° V-port valve at open fraction x ∈ [0, 1].
    Clamped to 0 at the low end.
    """
    cv = CV_A * x**2 + CV_B * x + CV_C
    return max(0.0, cv)


def delta_pressure(cv: float, chamber_pressure: float, dt: float) -> float:
    """Convert current Cv and chamber pressure to ΔP for one time step.
    Dummy function at the moment, need more data to create a real model
    """
    return cv * 0.01 * dt

def _target_at_time(t: float, servo_schedule: dict) -> float:
    """Return the commanded servo angle at time t.

    Takes the most-recent entry in servo_schedule whose key ≤ t.
    If t precedes all defined times, returns 0° (fully closed).
    """
    past_times = sorted(k for k in servo_schedule if k <= t)
    if not past_times:
        return 0.0
    return float(servo_schedule[past_times[-1]])


def simulate(
    initial_pressure_bar: float,
    tick_s: float,
    servo_speed_rad_s: float,
    servo_schedule: dict,
    total_time_s: float,
    pressure_loss_pct_per_s: float = 0.0,
) -> tuple[list, list, list, list]:
    """Run the valve-chamber pressure simulation.

    At every tick the servo steps toward its current target (limited by
    servo_speed), the resulting valve open fraction determines Cv, and Cv
    together with the current chamber pressure determine ΔP.

    Args:
        initial_pressure_bar:     Starting chamber pressure (bar).
        tick_s:                   Data-collection / integration time step (s).
        servo_speed_rad_s:        Maximum servo angular velocity (rad/s).
        servo_schedule:           Dict mapping {time_s: servo_angle_deg}.
                                  Between defined times the target is unchanged.
        total_time_s:             Simulation end time (s).
        pressure_loss_pct_per_s:  Passive pressure loss percent

    Returns:
        Four lists: (times_s, pressures_bar, servo_angles_deg, open_percents)
    """
    servo_speed_deg_s = servo_speed_rad_s * (180.0 / math.pi)

    times:         list[float] = []
    pressures:     list[float] = []
    servo_angles:  list[float] = []
    open_percents: list[float] = []

    chamber_pressure = initial_pressure_bar
    servo_pos = 0.0          # start fully closed

    n_steps = int(total_time_s / tick_s) + 1

    for i in range(n_steps):
        t = i * tick_s

        target = _target_at_time(t, servo_schedule)
        delta = target - servo_pos
        max_step = servo_speed_deg_s * tick_s

        if abs(delta) <= max_step:
            servo_pos = target
        else:
            servo_pos += math.copysign(max_step, delta)

        servo_pos = max(SERVO_MIN_DEG, min(SERVO_MAX_DEG, servo_pos))

        x  = valve_open_fraction(servo_pos)
        cv = cv_from_open_fraction(x)
        dp = delta_pressure(cv, chamber_pressure, tick_s)
        leak = chamber_pressure * (pressure_loss_pct_per_s / 100.0) * tick_s
        chamber_pressure += dp - leak

        times.append(t)
        pressures.append(chamber_pressure)
        servo_angles.append(servo_pos)
        open_percents.append(x * 100.0)

    return times, pressures, servo_angles, open_percents


def save_log(
    times: list,
    pressures: list,
    servo_angles: list,
    open_percents: list,
) -> str:
    """Write raw simulation data to ./logs/<YYYY-MM-DD_HH-MM>.csv."""
    os.makedirs("logs", exist_ok=True)
    filename = datetime.now().strftime("%Y-%m-%d_%H-%M") + ".csv"
    filepath = os.path.join("logs", filename)

    with open(filepath, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "chamber_pressure_bar", "servo_angle_deg", "valve_open_pct"])
        for t, p, s, o in zip(times, pressures, servo_angles, open_percents):
            writer.writerow([
                round(t, 4),
                round(p, 6),
                round(s, 4),
                round(o, 4),
            ])

    print(f"Log saved → {filepath}")
    return filepath


def plot_results(
    times: list,
    pressures: list,
    servo_angles: list,
    open_percents: list,
) -> None:
    """Display a three-panel plot of pressure, servo angle, and valve open %."""
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    axes[0].plot(times, pressures, color="royalblue", linewidth=1.8)
    axes[0].set_ylabel("Chamber Pressure (bar)")
    axes[0].set_title("Valve-Chamber Pressure Simulation")
    axes[0].grid(True, linestyle="--", alpha=0.5)

    axes[1].plot(times, servo_angles, color="darkorange", linewidth=1.5)
    axes[1].set_ylabel("Servo Angle (°)")
    axes[1].set_ylim(-5, 130)
    axes[1].grid(True, linestyle="--", alpha=0.5)

    axes[2].plot(times, open_percents, color="seagreen", linewidth=1.5)
    axes[2].set_ylabel("Valve Open (%)")
    axes[2].set_ylim(-2, 105)
    axes[2].set_xlabel("Time (s)")
    axes[2].grid(True, linestyle="--", alpha=0.5)

    plt.tight_layout()
    plt.show()


def main():

    # Starting chamber pressure (bar)
    initial_pressure_bar: float = 1.0

    # How often a data point is collected / integration step (seconds)
    tick_s: float = 0.05

    # Servo angular speed (rad/s)
    servo_speed_rad_s: float = 1.0

    # percent of current chamber pressure lost per second (0-100)
    pressure_loss_pct_per_s: float = 0

    # Servo schedule: {time_in_seconds: target_servo_angle_in_degrees}
    # The servo moves toward the commanded angle at servo_speed_rad_s.
    # Between two defined times the target remains at the earlier value.
    servo_schedule: dict = {
        0.0:   0.0,   # t=0 s  → servo at   0° (valve fully closed)
        2.0:  60.0,   # t=2 s  → servo to  90° (valve at 50% open)
        5.0: 120.0,   # t=5 s  → servo to 180° (valve fully open)
        8.0:   0.0,   # t=8 s  → close valve
    }


    total_time_s = max(servo_schedule.keys()) + 3.0   # 3 s buffer after last event
    
    times, pressures, servo_angles, open_percents = simulate(
        initial_pressure_bar=initial_pressure_bar,
        tick_s=tick_s,
        servo_speed_rad_s=servo_speed_rad_s,
        servo_schedule=servo_schedule,
        total_time_s=total_time_s,
        pressure_loss_pct_per_s=pressure_loss_pct_per_s,
    )

    save_log(times, pressures, servo_angles, open_percents)
    plot_results(times, pressures, servo_angles, open_percents)


if __name__ == "__main__":
    main()

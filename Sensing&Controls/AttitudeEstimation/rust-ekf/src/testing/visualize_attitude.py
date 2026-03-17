#!/usr/bin/env python3
"""
Visualize and compare rocket attitude from EKF output and flight-data ground truth.

EKF CSV format (attitude_output.csv):
    time,roll,pitch,yaw,omega_x,omega_y,omega_z

Flight-data CSV format (flight_data.csv):
    time,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,q_x,q_y,q_z,q_w

Modes
-----
plot     – static time-series overlay (default when both CSVs are present)
animate  – 3-D animation; shows both rockets when flight data is available
both     – plot window first, then animation
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

@dataclass
class AttitudeSample:
    time: float
    roll: float
    pitch: float
    yaw: float
    omega_x: float
    omega_y: float
    omega_z: float


# ---------------------------------------------------------------------------
# Parsers
# ---------------------------------------------------------------------------

def parse_attitude_csv(path: Path) -> List[AttitudeSample]:
    """Read EKF output CSV (Euler angles + body rates)."""
    samples: List[AttitudeSample] = []
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        required = {"time", "roll", "pitch", "yaw", "omega_x", "omega_y", "omega_z"}
        missing = required.difference(set(reader.fieldnames or []))
        if missing:
            raise ValueError(f"CSV missing required columns: {sorted(missing)}")
        for row in reader:
            if not row["time"].strip():
                continue
            samples.append(
                AttitudeSample(
                    time=float(row["time"]),
                    roll=float(row["roll"]),
                    pitch=float(row["pitch"]),
                    yaw=float(row["yaw"]),
                    omega_x=float(row["omega_x"]),
                    omega_y=float(row["omega_y"]),
                    omega_z=float(row["omega_z"]),
                )
            )
    if not samples:
        raise ValueError(f"No data rows found in {path}.")
    return samples


def _quat_to_euler_zyx(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """Convert quaternion to ZYX Euler angles (roll, pitch, yaw) in radians."""
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    sin_pitch = 2.0 * (qw * qy - qz * qx)
    sin_pitch = max(-1.0, min(1.0, sin_pitch))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return roll, pitch, yaw


def parse_flight_data_csv(path: Path) -> List[AttitudeSample]:
    """
    Read flight_data CSV, convert quaternion columns to ZYX Euler, and
    use gyro columns as body rates — producing AttitudeSample objects
    comparable to EKF output.
    """
    samples: List[AttitudeSample] = []
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        required = {"time", "q_x", "q_y", "q_z", "q_w", "gyro_x", "gyro_y", "gyro_z"}
        missing = required.difference(set(reader.fieldnames or []))
        if missing:
            raise ValueError(f"Flight-data CSV missing required columns: {sorted(missing)}")
        for row in reader:
            if not row["time"].strip():
                continue
            qx, qy, qz, qw = (
                float(row["q_x"]),
                float(row["q_y"]),
                float(row["q_z"]),
                float(row["q_w"]),
            )
            roll, pitch, yaw = _quat_to_euler_zyx(qx, qy, qz, qw)
            samples.append(
                AttitudeSample(
                    time=float(row["time"]),
                    roll=roll,
                    pitch=pitch,
                    yaw=yaw,
                    omega_x=float(row["gyro_x"]),
                    omega_y=float(row["gyro_y"]),
                    omega_z=float(row["gyro_z"]),
                )
            )
    if not samples:
        raise ValueError(f"No data rows found in {path}.")
    return samples


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def world_to_body_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX Euler rotation matrix (R_wb)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def make_rocket_body() -> Tuple[np.ndarray, np.ndarray]:
    length = 2.0
    fin_span = 0.35
    tail = np.array([0.0, 0.0, -0.5 * length])
    nose = np.array([0.0, 0.0, 0.5 * length])
    centerline = np.stack([tail, nose], axis=0)
    fins = np.array(
        [
            [fin_span, 0.0, tail[2]],
            [-fin_span, 0.0, tail[2]],
            [0.0, fin_span, tail[2]],
            [0.0, -fin_span, tail[2]],
        ]
    )
    return centerline, fins


def transform_points(points: np.ndarray, body_to_world: np.ndarray) -> np.ndarray:
    return (body_to_world @ points.T).T


# ---------------------------------------------------------------------------
# Comparison plot
# ---------------------------------------------------------------------------

def plot_comparison(
    ekf: List[AttitudeSample],
    truth: Optional[List[AttitudeSample]],
) -> None:
    """Static time-series comparison of EKF estimates vs. flight-data ground truth."""
    t_ekf = np.array([s.time for s in ekf])
    roll_ekf = np.degrees([s.roll for s in ekf])
    pitch_ekf = np.degrees([s.pitch for s in ekf])
    yaw_ekf = np.degrees([s.yaw for s in ekf])
    ox_ekf = np.array([s.omega_x for s in ekf])
    oy_ekf = np.array([s.omega_y for s in ekf])
    oz_ekf = np.array([s.omega_z for s in ekf])

    has_truth = truth is not None and len(truth) > 0
    if has_truth:
        t_gt = np.array([s.time for s in truth])
        roll_gt = np.degrees([s.roll for s in truth])
        pitch_gt = np.degrees([s.pitch for s in truth])
        yaw_gt = np.degrees([s.yaw for s in truth])
        ox_gt = np.array([s.omega_x for s in truth])
        oy_gt = np.array([s.omega_y for s in truth])
        oz_gt = np.array([s.omega_z for s in truth])

    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    fig.suptitle("EKF Estimate vs. Flight-Data Ground Truth", fontsize=14, fontweight="bold")

    ekf_kw = dict(color="#1f77b4", linewidth=1.5, label="EKF estimate")
    gt_kw = dict(color="#d62728", linewidth=1.2, linestyle="--", label="Flight data (truth)", alpha=0.85)

    # Euler angles column
    for ax, ekf_vals, gt_vals, ylabel in zip(
        axes[:, 0],
        [roll_ekf, pitch_ekf, yaw_ekf],
        ([roll_gt, pitch_gt, yaw_gt] if has_truth else [None, None, None]),
        ["Roll (deg)", "Pitch (deg)", "Yaw (deg)"],
    ):
        ax.plot(t_ekf, ekf_vals, **ekf_kw)
        if has_truth and gt_vals is not None:
            ax.plot(t_gt, gt_vals, **gt_kw)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc="upper right")

    # Body-rate column
    for ax, ekf_vals, gt_vals, ylabel in zip(
        axes[:, 1],
        [ox_ekf, oy_ekf, oz_ekf],
        ([ox_gt, oy_gt, oz_gt] if has_truth else [None, None, None]),
        [r"$\omega_x$ (rad/s)", r"$\omega_y$ (rad/s)", r"$\omega_z$ (rad/s)"],
    ):
        ax.plot(t_ekf, ekf_vals, **ekf_kw)
        if has_truth and gt_vals is not None:
            ax.plot(t_gt, gt_vals, **gt_kw)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc="upper right")

    for ax in axes[-1, :]:
        ax.set_xlabel("Time (s)")

    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# 3-D animation
# ---------------------------------------------------------------------------

def _add_rocket_artists(ax, body_color: str, fin_color: str, path_color: str):
    """Create and return the line artists for one rocket model."""
    rocket_line, = ax.plot([], [], [], lw=3.0, color=body_color)
    fin_lines = [ax.plot([], [], [], lw=2.0, color=fin_color)[0] for _ in range(4)]
    nose_path, = ax.plot([], [], [], lw=1.0, color=path_color, alpha=0.7)
    return rocket_line, fin_lines, nose_path


def run_animation(
    ekf: List[AttitudeSample],
    truth: Optional[List[AttitudeSample]],
    playback_speed: float,
    step: int,
    show_omega: bool,
) -> None:
    indices = list(range(0, len(ekf), max(1, step)))
    ekf_ds = [ekf[i] for i in indices]

    # Align truth to the same indices / timestamps as EKF if present.
    truth_ds: Optional[List[AttitudeSample]] = None
    if truth is not None and len(truth) > 0:
        # Build a time-keyed lookup; pick the nearest sample to each EKF timestamp.
        truth_times = np.array([s.time for s in truth])
        truth_ds = []
        for s in ekf_ds:
            idx = int(np.argmin(np.abs(truth_times - s.time)))
            truth_ds.append(truth[idx])

    centerline_b, fins_b = make_rocket_body()
    tail_b = centerline_b[0]

    fig = plt.figure(figsize=(10, 9))
    ax = fig.add_subplot(111, projection="3d")
    title = "Rocket Attitude — EKF (blue) vs. Flight Data (red)" if truth_ds else "Rocket Attitude (EKF)"
    ax.set_title(title, fontsize=11)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    lim = 2.3
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim, lim)
    ax.set_box_aspect((1, 1, 1))

    ax.plot([0, 1.0], [0, 0], [0, 0], color="tab:red",   alpha=0.25)
    ax.plot([0, 0], [0, 1.0], [0, 0], color="tab:green", alpha=0.25)
    ax.plot([0, 0], [0, 0], [0, 1.0], color="tab:blue",  alpha=0.25)

    # EKF rocket – blue
    ekf_rocket, ekf_fins, ekf_path = _add_rocket_artists(ax, "#1f77b4", "#aec7e8", "#9467bd")
    # Truth rocket – red (only created when data is present)
    gt_rocket = gt_fins = gt_path_line = None
    if truth_ds:
        gt_rocket, gt_fins, gt_path_line = _add_rocket_artists(ax, "#d62728", "#f5a89a", "#ff7f0e")

    timestamp_text = ax.text2D(0.02, 0.97, "", transform=ax.transAxes, fontsize=9)
    ekf_text       = ax.text2D(0.02, 0.93, "", transform=ax.transAxes, fontsize=8, color="#1f77b4")
    gt_text        = ax.text2D(0.02, 0.89, "", transform=ax.transAxes, fontsize=8, color="#d62728")
    omega_text     = ax.text2D(0.02, 0.85, "", transform=ax.transAxes, fontsize=8, color="grey")

    ekf_nose_hist: List[np.ndarray] = []
    gt_nose_hist:  List[np.ndarray] = []
    omega_quiver = None

    def _pose_rocket(sample, rocket_line, fin_lines, nose_hist, nose_path_line):
        r_wb = world_to_body_rotation(sample.roll, sample.pitch, sample.yaw)
        r_bw = r_wb.T
        cl_w = transform_points(centerline_b, r_bw)
        fins_w = transform_points(fins_b, r_bw)
        tail_w = transform_points(tail_b[None, :], r_bw)[0]
        rocket_line.set_data_3d(cl_w[:, 0], cl_w[:, 1], cl_w[:, 2])
        for i, tip in enumerate(fins_w):
            seg = np.stack([tail_w, tip], axis=0)
            fin_lines[i].set_data_3d(seg[:, 0], seg[:, 1], seg[:, 2])
        nose_hist.append(cl_w[1])
        nh = np.array(nose_hist)
        nose_path_line.set_data_3d(nh[:, 0], nh[:, 1], nh[:, 2])
        return r_bw

    def init():
        ekf_rocket.set_data_3d([], [], [])
        for fl in ekf_fins:
            fl.set_data_3d([], [], [])
        ekf_path.set_data_3d([], [], [])
        artists = [ekf_rocket, *ekf_fins, ekf_path]
        if truth_ds:
            gt_rocket.set_data_3d([], [], [])
            for fl in gt_fins:
                fl.set_data_3d([], [], [])
            gt_path_line.set_data_3d([], [], [])
            artists += [gt_rocket, *gt_fins, gt_path_line]
        return artists

    def update(frame_idx: int):
        nonlocal omega_quiver
        es = ekf_ds[frame_idx]
        r_bw = _pose_rocket(es, ekf_rocket, ekf_fins, ekf_nose_hist, ekf_path)

        timestamp_text.set_text(f"t = {es.time:.2f} s")
        ekf_text.set_text(
            f"[EKF]  r/p/y = {math.degrees(es.roll):.1f}°, "
            f"{math.degrees(es.pitch):.1f}°, {math.degrees(es.yaw):.1f}°"
        )

        artists = [ekf_rocket, *ekf_fins, ekf_path, timestamp_text, ekf_text, gt_text, omega_text]

        if truth_ds:
            gs = truth_ds[frame_idx]
            _pose_rocket(gs, gt_rocket, gt_fins, gt_nose_hist, gt_path_line)
            gt_text.set_text(
                f"[DATA] r/p/y = {math.degrees(gs.roll):.1f}°, "
                f"{math.degrees(gs.pitch):.1f}°, {math.degrees(gs.yaw):.1f}°"
            )
            artists += [gt_rocket, *gt_fins, gt_path_line]

        if show_omega:
            if omega_quiver is not None:
                omega_quiver.remove()
            omega_body = np.array([es.omega_x, es.omega_y, es.omega_z])
            omega_world = r_bw @ omega_body
            scale = 0.6
            omega_quiver = ax.quiver(
                0.0, 0.0, 0.0,
                omega_world[0] * scale, omega_world[1] * scale, omega_world[2] * scale,
                color="#1f77b4", linewidth=2.0,
            )
            omega_text.set_text(
                f"ω = ({es.omega_x:.3f}, {es.omega_y:.3f}, {es.omega_z:.3f}) rad/s"
            )
            artists.append(omega_quiver)

        return artists

    dt = max(1e-3, ekf_ds[1].time - ekf_ds[0].time) if len(ekf_ds) > 1 else 0.02
    interval_ms = 1000.0 * dt / max(playback_speed, 1e-6)

    anim = FuncAnimation(
        fig, update,
        frames=len(ekf_ds),
        init_func=init,
        interval=interval_ms,
        blit=False,
        repeat=False,
    )
    _ = anim  # keep reference alive until plt.show()

    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    here = Path(__file__).parent
    parser = argparse.ArgumentParser(
        description=(
            "Visualize and compare rocket attitude from EKF output and "
            "flight-data ground truth."
        )
    )
    parser.add_argument(
        "--ekf-input",
        type=Path,
        default=here / "attitude_output.csv",
        help="Path to EKF output CSV  (default: attitude_output.csv)",
    )
    parser.add_argument(
        "--flight-data",
        type=Path,
        default=here / "flight_data.csv",
        help="Path to flight-data CSV  (default: flight_data.csv)",
    )
    parser.add_argument(
        "--no-flight-data",
        action="store_true",
        help="Ignore flight-data CSV even if it exists",
    )
    parser.add_argument(
        "--mode",
        choices=["plot", "animate", "both"],
        default="plot",
        help=(
            "plot    – static time-series comparison (default)  |  "
            "animate – 3-D animation  |  "
            "both    – plot then animate"
        ),
    )
    parser.add_argument(
        "--playback-speed",
        type=float,
        default=1.0,
        help="Animation playback speed multiplier",
    )
    parser.add_argument(
        "--step",
        type=int,
        default=1,
        help="Use every Nth sample (faster animation rendering)",
    )
    parser.add_argument(
        "--hide-omega",
        action="store_true",
        help="Hide angular-rate vector arrow in animation",
    )
    args = parser.parse_args()

    ekf = parse_attitude_csv(args.ekf_input)

    truth: Optional[List[AttitudeSample]] = None
    if not args.no_flight_data and args.flight_data.exists():
        truth = parse_flight_data_csv(args.flight_data)
    elif not args.no_flight_data:
        print(f"[warn] flight-data file not found: {args.flight_data}  (running EKF-only)")

    if args.mode in ("plot", "both"):
        plot_comparison(ekf, truth)

    if args.mode in ("animate", "both"):
        run_animation(
            ekf=ekf,
            truth=truth,
            playback_speed=args.playback_speed,
            step=args.step,
            show_omega=not args.hide_omega,
        )


if __name__ == "__main__":
    main()

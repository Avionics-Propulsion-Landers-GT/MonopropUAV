#!/usr/bin/env python3
"""
Animate rocket attitude from EKF CSV output.

Input CSV format (from attitude_test):
time,roll,pitch,yaw,omega_x,omega_y,omega_z
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation


@dataclass
class AttitudeSample:
    time: float
    roll: float
    pitch: float
    yaw: float
    omega_x: float
    omega_y: float
    omega_z: float


def parse_attitude_csv(path: Path) -> List[AttitudeSample]:
    samples: List[AttitudeSample] = []
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        required = {
            "time",
            "roll",
            "pitch",
            "yaw",
            "omega_x",
            "omega_y",
            "omega_z",
        }
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
        raise ValueError("No data rows found in CSV.")
    return samples


def world_to_body_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX Euler matrix matching the EKF model convention."""
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
    """
    Return model points in body frame:
    - centerline endpoints (tail -> nose)
    - fin endpoints connected to tail
    """
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


def run_animation(
    samples: List[AttitudeSample],
    playback_speed: float,
    step: int,
    show_omega: bool,
) -> None:
    indices = list(range(0, len(samples), max(1, step)))
    downsampled = [samples[i] for i in indices]

    centerline_b, fins_b = make_rocket_body()
    tail_b = centerline_b[0]

    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title("Rocket Attitude Visualization")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    lim = 2.3
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-lim, lim)
    ax.set_box_aspect((1, 1, 1))

    # World frame reference axes.
    ax.plot([0, 1.0], [0, 0], [0, 0], color="tab:red", alpha=0.35)
    ax.plot([0, 0], [0, 1.0], [0, 0], color="tab:green", alpha=0.35)
    ax.plot([0, 0], [0, 0], [0, 1.0], color="tab:blue", alpha=0.35)

    rocket_line, = ax.plot([], [], [], lw=3.0, color="black")
    fin_lines = [ax.plot([], [], [], lw=2.0, color="tab:orange")[0] for _ in range(4)]
    nose_path_line, = ax.plot([], [], [], lw=1.25, color="tab:purple", alpha=0.8)
    omega_quiver = None

    timestamp_text = ax.text2D(0.02, 0.95, "", transform=ax.transAxes)
    rpy_text = ax.text2D(0.02, 0.91, "", transform=ax.transAxes)
    omega_text = ax.text2D(0.02, 0.87, "", transform=ax.transAxes)

    nose_path: List[np.ndarray] = []

    def init():
        rocket_line.set_data_3d([], [], [])
        for fin_line in fin_lines:
            fin_line.set_data_3d([], [], [])
        nose_path_line.set_data_3d([], [], [])
        return [rocket_line, *fin_lines, nose_path_line]

    def update(frame_idx: int):
        nonlocal omega_quiver
        sample = downsampled[frame_idx]

        r_wb = world_to_body_rotation(sample.roll, sample.pitch, sample.yaw)
        r_bw = r_wb.T

        centerline_w = transform_points(centerline_b, r_bw)
        fins_w = transform_points(fins_b, r_bw)
        tail_w = transform_points(tail_b[None, :], r_bw)[0]

        rocket_line.set_data_3d(centerline_w[:, 0], centerline_w[:, 1], centerline_w[:, 2])

        for i, fin_tip in enumerate(fins_w):
            seg = np.stack([tail_w, fin_tip], axis=0)
            fin_lines[i].set_data_3d(seg[:, 0], seg[:, 1], seg[:, 2])

        nose_path.append(centerline_w[1])
        nose_path_np = np.array(nose_path)
        nose_path_line.set_data_3d(
            nose_path_np[:, 0], nose_path_np[:, 1], nose_path_np[:, 2]
        )

        timestamp_text.set_text(f"t = {sample.time:.2f} s")
        rpy_text.set_text(
            "roll/pitch/yaw (deg) = "
            f"{math.degrees(sample.roll):.1f}, "
            f"{math.degrees(sample.pitch):.1f}, "
            f"{math.degrees(sample.yaw):.1f}"
        )
        omega_text.set_text(
            "omega (rad/s) = "
            f"{sample.omega_x:.3f}, {sample.omega_y:.3f}, {sample.omega_z:.3f}"
        )

        if show_omega:
            if omega_quiver is not None:
                omega_quiver.remove()
            omega_body = np.array([sample.omega_x, sample.omega_y, sample.omega_z])
            omega_world = r_bw @ omega_body
            scale = 0.6
            omega_quiver = ax.quiver(
                0.0,
                0.0,
                0.0,
                omega_world[0] * scale,
                omega_world[1] * scale,
                omega_world[2] * scale,
                color="tab:red",
                linewidth=2.0,
            )

        artists = [rocket_line, *fin_lines, nose_path_line, timestamp_text, rpy_text, omega_text]
        if omega_quiver is not None:
            artists.append(omega_quiver)
        return artists

    if len(downsampled) > 1:
        dt = max(1e-3, downsampled[1].time - downsampled[0].time)
    else:
        dt = 0.02
    interval_ms = 1000.0 * dt / max(playback_speed, 1e-6)

    animation = FuncAnimation(
        fig,
        update,
        frames=len(downsampled),
        init_func=init,
        interval=interval_ms,
        blit=False,
        repeat=False,
    )

    plt.tight_layout()
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visualize rocket attitude from attitude_output.csv"
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=Path(__file__).with_name("attitude_output.csv"),
        help="Path to attitude_output.csv",
    )
    parser.add_argument(
        "--playback-speed",
        type=float,
        default=1.0,
        help="Playback speed multiplier (e.g., 2.0 = 2x)",
    )
    parser.add_argument(
        "--step",
        type=int,
        default=1,
        help="Use every Nth sample (for faster rendering)",
    )
    parser.add_argument(
        "--hide-omega",
        action="store_true",
        help="Hide angular-rate vector arrow",
    )
    args = parser.parse_args()

    samples = parse_attitude_csv(args.input)
    run_animation(
        samples=samples,
        playback_speed=args.playback_speed,
        step=args.step,
        show_omega=not args.hide_omega,
    )


if __name__ == "__main__":
    main()

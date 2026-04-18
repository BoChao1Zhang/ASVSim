import argparse
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def plot_trajectory_file(npz_path: str | Path, output_path: str | Path | None = None) -> Path:
    npz_path = Path(npz_path)
    output_path = Path(output_path) if output_path is not None else npz_path.with_suffix(".png")

    data = np.load(npz_path, allow_pickle=True)
    trajectory = np.asarray(data.get("trajectory", np.empty((0, 2))), dtype=np.float32)
    waypoints = np.asarray(data.get("waypoints", np.empty((0, 2))), dtype=np.float32)
    obstacles = np.asarray(data.get("obstacles", np.empty((0, 3))), dtype=np.float32)
    end_reason_array = data.get("end_reason", np.asarray(["unknown"]))
    end_reason = str(end_reason_array[0]) if len(end_reason_array) else "unknown"

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_title(f"Trajectory: {end_reason}")

    if len(trajectory) > 0:
        ax.plot(trajectory[:, 0], trajectory[:, 1], color="#1f77b4", linewidth=2.0, label="trajectory")
        ax.scatter(trajectory[0, 0], trajectory[0, 1], color="#2ca02c", s=60, label="start")
        ax.scatter(trajectory[-1, 0], trajectory[-1, 1], color="#d62728", s=60, label="end")

    if len(waypoints) > 0:
        ax.plot(waypoints[:, 0], waypoints[:, 1], color="#ff7f0e", linestyle="--", marker="o", label="waypoints")

    if len(obstacles) > 0:
        static_mask = obstacles[:, 2] < 0.5
        dynamic_mask = ~static_mask
        if np.any(static_mask):
            ax.scatter(obstacles[static_mask, 0], obstacles[static_mask, 1], color="#8c564b", marker="x", s=80, label="static obstacles")
        if np.any(dynamic_mask):
            ax.scatter(obstacles[dynamic_mask, 0], obstacles[dynamic_mask, 1], color="#9467bd", marker="^", s=80, label="dynamic obstacles")

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.2)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def main():
    parser = argparse.ArgumentParser(description="Plot trajectory PNGs from evaluation NPZ dumps")
    parser.add_argument("--npz", type=str, default=None, help="Single trajectory npz path")
    parser.add_argument("--trajectory-dir", type=str, default=None, help="Directory containing trajectory npz files")
    args = parser.parse_args()

    if args.npz is None and args.trajectory_dir is None:
        raise ValueError("Provide either --npz or --trajectory-dir")

    if args.npz is not None:
        output = plot_trajectory_file(args.npz)
        print(f"Wrote {output}")
        return

    trajectory_dir = Path(args.trajectory_dir).resolve()
    for npz_path in sorted(trajectory_dir.glob("*.npz")):
        output = plot_trajectory_file(npz_path)
        print(f"Wrote {output}")


if __name__ == "__main__":
    main()

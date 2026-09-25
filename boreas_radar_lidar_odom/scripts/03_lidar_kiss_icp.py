#!/usr/bin/env python3
"""Step 4: KISS-ICP LiDAR baseline on Boreas .bin scans."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from kiss_icp.config import load_config
from kiss_icp.kiss_icp import KissICP
from tqdm import tqdm

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))

from boreas_io import (  # noqa: E402
    list_sensor_files,
    load_lidar,
    se3_to_tum_row,
    seq_root,
    time_from_stem,
    write_tum,
)


def relative_timestamps(raw_t: np.ndarray) -> np.ndarray:
    """Normalize per-point times in a scan to [0, 1] for deskew."""
    t = raw_t.astype(np.float64)
    lo, hi = float(np.min(t)), float(np.max(t))
    if hi <= lo:
        return np.zeros_like(t)
    return (t - lo) / (hi - lo)


def run_kiss_icp_python(files: list[Path], out_traj: Path) -> None:
    config = load_config(None)
    config.data.max_range = 100.0
    odom = KissICP(config)

    traj = []
    for path in tqdm(files, desc="KISS-ICP"):
        cloud = load_lidar(path)
        pts = cloud[:, :3].astype(np.float64)
        stamps = relative_timestamps(cloud[:, 5])
        t = time_from_stem(path.stem)
        odom.register_frame(pts, stamps)
        T = np.asarray(odom.last_pose, dtype=np.float64)
        traj.append(se3_to_tum_row(t, T))

    write_tum(out_traj, np.vstack(traj))
    print(f"Wrote {out_traj} ({len(traj)} poses)")


def main() -> None:
    ap = argparse.ArgumentParser(description="KISS-ICP LiDAR odometry on Boreas")
    ap.add_argument("--data", type=Path, default=ROOT / "boreas-data")
    ap.add_argument("--out", type=Path, default=ROOT / "out")
    ap.add_argument("--max-frames", type=int, default=800, help="0 = all")
    ap.add_argument("--stride", type=int, default=1)
    args = ap.parse_args()

    root = seq_root(args.data)
    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    out_traj = out / "lidar_trajectory.txt"

    files = list_sensor_files(root, "lidar", "bin")[:: max(1, args.stride)]
    if args.max_frames and args.max_frames > 0:
        files = files[: args.max_frames]
    print(f"LiDAR KISS-ICP on {len(files)} frames from {root}")

    run_kiss_icp_python(files, out_traj)

    try:
        import matplotlib.pyplot as plt

        tum = np.loadtxt(out_traj)
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.plot(tum[:, 1], tum[:, 2], "-b", lw=1.2, label="KISS-ICP")
        ax.set_aspect("equal")
        ax.set_title("LiDAR KISS-ICP trajectory")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(out / "lidar_trajectory.png", dpi=140)
        print(f"Wrote {out / 'lidar_trajectory.png'}")
    except Exception as e:  # noqa: BLE001
        print(f"Plot skipped: {e}")


if __name__ == "__main__":
    main()

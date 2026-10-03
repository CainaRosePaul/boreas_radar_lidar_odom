#!/usr/bin/env python3
"""Step 2: Load & inspect one radar scan, one LiDAR scan, and GT poses."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))

from boreas_io import (  # noqa: E402
    list_sensor_files,
    load_lidar,
    load_poses_csv,
    load_radar,
    radar_to_xy,
    seq_root,
)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", type=Path, default=ROOT / "boreas-data")
    ap.add_argument("--out", type=Path, default=ROOT / "out")
    ap.add_argument("--frame-idx", type=int, default=50)
    args = ap.parse_args()

    root = seq_root(args.data)
    out = args.out
    out.mkdir(parents=True, exist_ok=True)

    radar_files = list_sensor_files(root, "radar", "png")
    lidar_files = list_sensor_files(root, "lidar", "bin")
    idx = min(args.frame_idx, len(radar_files) - 1, len(lidar_files) - 1)

    radar_path = radar_files[idx]
    # nearest lidar by timestamp stem
    r_t = int(radar_path.stem)
    lidar_path = min(lidar_files, key=lambda p: abs(int(p.stem) - r_t))

    az, fft, valid, res = load_radar(radar_path)
    pts_radar = radar_to_xy(az, fft, res, top_k=5)
    lidar = load_lidar(lidar_path)

    _, tum_radar = load_poses_csv(root / "applanix" / "radar_poses.csv")
    _, tum_lidar = load_poses_csv(root / "applanix" / "lidar_poses.csv")

    print(f"Sequence root: {root}")
    print(f"Radar frames: {len(radar_files)} | LiDAR frames: {len(lidar_files)}")
    print(f"Radar sample: {radar_path.name}  shape={fft.shape}  res={res} m/bin")
    print(f"LiDAR sample: {lidar_path.name}  points={lidar.shape[0]}")
    print(f"Radar keypoints (top-5/az): {pts_radar.shape[0]}")
    print(f"GT radar poses: {tum_radar.shape[0]} | GT lidar poses: {tum_lidar.shape[0]}")

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    axes[0, 0].imshow(fft, aspect="auto", cmap="hot", origin="lower")
    axes[0, 0].set_title(f"Radar polar (az × range)\n{radar_path.name}")
    axes[0, 0].set_xlabel("range bin")
    axes[0, 0].set_ylabel("azimuth index")

    axes[0, 1].scatter(pts_radar[:, 0], pts_radar[:, 1], s=1, c="crimson", alpha=0.4)
    axes[0, 1].set_aspect("equal")
    axes[0, 1].set_title("Radar Cartesian keypoints (top-5/az)")
    axes[0, 1].set_xlabel("x [m]")
    axes[0, 1].set_ylabel("y [m]")
    axes[0, 1].grid(True, alpha=0.3)

    # BEV lidar (z slice)
    zmask = np.abs(lidar[:, 2]) < 2.5
    xy = lidar[zmask]
    axes[1, 0].scatter(xy[:, 0], xy[:, 1], s=0.2, c=xy[:, 3], cmap="viridis", alpha=0.5)
    axes[1, 0].set_aspect("equal")
    axes[1, 0].set_title(f"LiDAR BEV (|z|<2.5 m)\n{lidar_path.name}")
    axes[1, 0].set_xlabel("x [m]")
    axes[1, 0].set_ylabel("y [m]")
    axes[1, 0].set_xlim(-60, 60)
    axes[1, 0].set_ylim(-60, 60)

    axes[1, 1].plot(tum_radar[:, 1], tum_radar[:, 2], label="radar GT", lw=1.5)
    axes[1, 1].plot(tum_lidar[:, 1], tum_lidar[:, 2], "--", label="lidar GT", lw=1.0, alpha=0.7)
    axes[1, 1].set_aspect("equal")
    axes[1, 1].set_title("Applanix ground-truth path (ENU)")
    axes[1, 1].set_xlabel("easting [m]")
    axes[1, 1].set_ylabel("northing [m]")
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    fig.suptitle("Boreas inspect — radar vs LiDAR vs GT", fontsize=14)
    fig.tight_layout()
    out_path = out / "inspect_side_by_side.png"
    fig.savefig(out_path, dpi=140)
    print(f"Wrote {out_path}")


if __name__ == "__main__":
    main()

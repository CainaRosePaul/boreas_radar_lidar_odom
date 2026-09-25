#!/usr/bin/env python3
"""Step 3: 2D radar odometry — top-K (fake CFAR) keypoints + Open3D ICP."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
from tqdm import tqdm

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))

from boreas_io import (  # noqa: E402
    list_sensor_files,
    load_radar,
    radar_to_xy,
    se3_to_tum_row,
    seq_root,
    time_from_stem,
    write_tum,
)


def to_pcd(xyz: np.ndarray) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    return pcd


def icp_step(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    threshold: float,
    init: np.ndarray | None = None,
) -> np.ndarray:
    if init is None:
        init = np.eye(4)
    result = o3d.pipelines.registration.registration_icp(
        source,
        target,
        threshold,
        init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50),
    )
    return result.transformation.copy()


def main() -> None:
    ap = argparse.ArgumentParser(description="Radar CFAR-ish keypoints + ICP odometry")
    ap.add_argument("--data", type=Path, default=ROOT / "boreas-data")
    ap.add_argument("--out", type=Path, default=ROOT / "out")
    ap.add_argument("--max-frames", type=int, default=400, help="0 = all frames")
    ap.add_argument("--top-k", type=int, default=5)
    ap.add_argument("--icp-thresh", type=float, default=2.0)
    ap.add_argument("--stride", type=int, default=1)
    args = ap.parse_args()

    root = seq_root(args.data)
    out = args.out
    out.mkdir(parents=True, exist_ok=True)

    files = list_sensor_files(root, "radar", "png")[:: max(1, args.stride)]
    if args.max_frames and args.max_frames > 0:
        files = files[: args.max_frames]
    print(f"Radar odometry on {len(files)} frames from {root}")

    prev_pcd = None
    T_world = np.eye(4, dtype=np.float64)
    traj = []

    for i, path in enumerate(tqdm(files, desc="radar ICP")):
        az, fft, _, res = load_radar(path)
        xyz = radar_to_xy(az, fft, res, top_k=args.top_k)
        if xyz.shape[0] < 50:
            # skip degenerate scan but keep time alignment with identity motion
            t = time_from_stem(path.stem)
            traj.append(se3_to_tum_row(t, T_world))
            continue

        pcd = to_pcd(xyz)
        t = time_from_stem(path.stem)

        if prev_pcd is None:
            traj.append(se3_to_tum_row(t, T_world))
            prev_pcd = pcd
            continue

        # Estimate T_prev_curr: maps current → previous
        T_rel = icp_step(pcd, prev_pcd, threshold=args.icp_thresh)
        T_world = T_world @ T_rel
        traj.append(se3_to_tum_row(t, T_world))
        prev_pcd = pcd

    tum = np.vstack(traj)
    out_traj = out / "radar_trajectory.txt"
    write_tum(out_traj, tum)
    print(f"Wrote {out_traj} ({len(tum)} poses)")

    # quick XY plot
    try:
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(7, 7))
        ax.plot(tum[:, 1], tum[:, 2], "-r", lw=1.2, label="radar ICP")
        ax.set_aspect("equal")
        ax.set_title("Radar odometry trajectory")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(out / "radar_trajectory.png", dpi=140)
        print(f"Wrote {out / 'radar_trajectory.png'}")
    except Exception as e:  # noqa: BLE001
        print(f"Plot skipped: {e}")


if __name__ == "__main__":
    main()

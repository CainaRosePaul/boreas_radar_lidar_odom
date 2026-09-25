#!/usr/bin/env python3
"""Step 5: Export GT in TUM format and evaluate radar/LiDAR with evo (APE + RPE)."""

from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))

from boreas_io import load_poses_csv, seq_root, write_tum  # noqa: E402


def evo_bin(name: str) -> str:
    cand = Path(sys.executable).parent / name
    if cand.exists():
        return str(cand)
    found = shutil.which(name)
    if not found:
        raise FileNotFoundError(name)
    return found


def run_evo(args: list[str]) -> str:
    cmd = args
    print(">>", " ".join(cmd))
    # evo may prompt to overwrite plot files if plot is enabled in ~/.evo/settings.json
    proc = subprocess.run(cmd, capture_output=True, text=True, input="n\n" * 8)
    out = (proc.stdout or "") + "\n" + (proc.stderr or "")
    out = re.sub(r"\x1b\[[0-9;]*m", "", out)
    print(out)
    # Accept success, or failure only if no RMSE was printed (overwrite cancel after stats is ok)
    if proc.returncode != 0 and parse_rmse(out) is None:
        raise RuntimeError(f"evo failed ({proc.returncode})")
    return out


def parse_rmse(text: str) -> float | None:
    for line in text.splitlines():
        if re.search(r"\brmse\b", line, re.I):
            nums = re.findall(r"[-+]?\d*\.\d+|\d+", line)
            if nums:
                return float(nums[-1])
    return None


def slice_gt_to_est(gt_tum: np.ndarray, est_tum: np.ndarray, max_dt: float = 0.05) -> np.ndarray:
    et = est_tum[:, 0]
    keep = []
    for row in gt_tum:
        j = int(np.searchsorted(et, row[0]))
        best = None
        for k in (j - 1, j):
            if 0 <= k < len(et) and (best is None or abs(et[k] - row[0]) < abs(et[best] - row[0])):
                best = k
        if best is not None and abs(et[best] - row[0]) <= max_dt:
            keep.append(row)
    return np.vstack(keep) if keep else gt_tum[:0].reshape(0, 8)


def path_length(tum: np.ndarray) -> float:
    d = np.diff(tum[:, 1:4], axis=0)
    return float(np.sum(np.linalg.norm(d, axis=1)))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", type=Path, default=ROOT / "boreas-data")
    ap.add_argument("--out", type=Path, default=ROOT / "out")
    ap.add_argument("--rpe-delta-m", type=float, default=10.0)
    args = ap.parse_args()

    root = seq_root(args.data)
    out = args.out
    out.mkdir(parents=True, exist_ok=True)

    _, gt_radar = load_poses_csv(root / "applanix" / "radar_poses.csv")
    _, gt_lidar = load_poses_csv(root / "applanix" / "lidar_poses.csv")
    write_tum(out / "groundtruth_radar.txt", gt_radar)
    write_tum(out / "groundtruth_lidar.txt", gt_lidar)
    write_tum(out / "groundtruth.txt", gt_lidar)

    metrics: dict = {"rpe_delta_m": args.rpe_delta_m}

    for name, est_path, gt_full in [
        ("radar", out / "radar_trajectory.txt", gt_radar),
        ("lidar", out / "lidar_trajectory.txt", gt_lidar),
    ]:
        if not est_path.exists():
            print(f"Missing {est_path}, skip")
            continue
        est_tum = np.loadtxt(est_path)
        gt_slice = slice_gt_to_est(gt_full, est_tum, max_dt=0.08)
        gt_path = out / f"groundtruth_{name}_synced.txt"
        write_tum(gt_path, gt_slice)

        # Metrics only here (plots already saved via CLI / overlay below).
        # Avoid --save_plot: evo prompts interactively if files exist.
        ape_txt = run_evo(
            [
                evo_bin("evo_ape"),
                "tum",
                str(gt_path),
                str(est_path),
                "-a",
                "--t_max_diff",
                "0.1",
            ]
        )
        rpe_txt = run_evo(
            [
                evo_bin("evo_rpe"),
                "tum",
                str(gt_path),
                str(est_path),
                "-a",
                "--t_max_diff",
                "0.1",
                "--delta",
                str(args.rpe_delta_m),
                "--delta_unit",
                "m",
            ]
        )
        ate = parse_rmse(ape_txt)
        rpe = parse_rmse(rpe_txt)
        plen = path_length(gt_slice)
        drift_pct = (100.0 * rpe / args.rpe_delta_m) if rpe is not None else None
        metrics[name] = {
            "ate_rmse_m": ate,
            "rpe_rmse_m": rpe,
            "drift_pct_rpe": drift_pct,
            "path_length_m": plen,
            "n_est": int(est_tum.shape[0]),
            "n_gt_synced": int(gt_slice.shape[0]),
        }

    # Overlay trajectories (start-aligned XY for visual compare)
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(gt_lidar[:, 1], gt_lidar[:, 2], "k-", lw=2, label="GT (full lidar poses)", alpha=0.35)
    if (out / "radar_trajectory.txt").exists():
        r = np.loadtxt(out / "radar_trajectory.txt")
        g = np.loadtxt(out / "groundtruth_radar_synced.txt")
        ax.plot(g[:, 1], g[:, 2], "k--", lw=1.5, label="GT (radar window)")
        ax.plot(
            r[:, 1] - r[0, 1] + g[0, 1],
            r[:, 2] - r[0, 2] + g[0, 2],
            "r-",
            lw=1.2,
            label="radar ICP",
        )
    if (out / "lidar_trajectory.txt").exists():
        l = np.loadtxt(out / "lidar_trajectory.txt")
        g = np.loadtxt(out / "groundtruth_lidar_synced.txt")
        ax.plot(
            l[:, 1] - l[0, 1] + g[0, 1],
            l[:, 2] - l[0, 2] + g[0, 2],
            "b-",
            lw=1.2,
            label="KISS-ICP",
        )
    ax.set_aspect("equal")
    ax.set_title("Trajectories (start-aligned XY)")
    ax.set_xlabel("easting / x [m]")
    ax.set_ylabel("northing / y [m]")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "traj_overlay.png", dpi=140)

    (out / "metrics.json").write_text(json.dumps(metrics, indent=2))
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()

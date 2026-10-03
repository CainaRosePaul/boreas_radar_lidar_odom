"""Shared Boreas loaders (radar PNG, lidar BIN, Applanix poses → TUM)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image

# Navtech CIR204 → CIR304 resolution change (unix seconds). After this: 0.04381 m/bin.
RADAR_RESOLUTION_UPGRADE_TIME = 1632182400.0
RADAR_OFFSET_M = -0.31
ENCODER_SIZE = 5600


def seq_root(data_dir: Path | str) -> Path:
    p = Path(data_dir)
    if (p / "radar").is_dir() and (p / "lidar").is_dir():
        return p
    # allow parent that contains the sequence folder
    cands = sorted(p.glob("boreas-*"))
    if len(cands) == 1:
        return cands[0]
    raise FileNotFoundError(f"Expected Boreas sequence root under {p}")


def time_from_stem(stem: str) -> float:
    """Filename stem (µs) → seconds."""
    t = float(stem)
    if len(stem) >= 16:
        return t * 1e-6
    return t


def list_sensor_files(root: Path, sensor: str, ext: str) -> list[Path]:
    d = root / sensor
    files = sorted(d.glob(f"*.{ext}"))
    if not files:
        raise FileNotFoundError(f"No {sensor}/*.{ext} under {root}")
    return files


def load_lidar(path: Path | str) -> np.ndarray:
    """Velodyne Alpha-Prime: float32 (N, 6) = x,y,z,intensity,laser_id,time."""
    pts = np.fromfile(str(path), dtype=np.float32).reshape((-1, 6))
    return pts


def load_radar(path: Path | str) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """
    Decode Navtech polar PNG (Oxford/Boreas layout).

    Returns
    -------
    azimuths : (M,) radians
    fft_data : (M, R) power in [0, 1]
    valid : (M,) bool
    resolution : metres per range bin
    """
    path = Path(path)
    t = time_from_stem(path.stem)
    resolution = 0.04381 if t > RADAR_RESOLUTION_UPGRADE_TIME else 0.0596

    raw = np.array(Image.open(path).convert("L"))
    timestamps = raw[:, :8].copy().view(np.int64).reshape(-1)
    azimuths = (
        raw[:, 8:10].copy().view(np.uint16).astype(np.float64).reshape(-1)
        / float(ENCODER_SIZE)
        * 2.0
        * np.pi
    ).astype(np.float32)
    valid = (raw[:, 10] == 255).reshape(-1)
    fft = raw[:, 11:].astype(np.float32) / 255.0
    # blank near-range (sensor blind zone ~2.5 m)
    min_range = int(round(2.5 / resolution))
    fft[:, :min_range] = 0.0
    return azimuths, fft, valid, resolution


def radar_to_xy(
    azimuths: np.ndarray,
    fft: np.ndarray,
    resolution: float,
    top_k: int = 5,
    power_thresh: float = 0.0,
) -> np.ndarray:
    """
    Fake-CFAR: keep the strongest `top_k` range bins per azimuth, convert to Cartesian.

    Returns (N, 3) points with z=0 for Open3D ICP.
    """
    m, r = fft.shape
    # argpartition for top-k per row
    if top_k >= r:
        idxs = np.tile(np.arange(r), (m, 1))
    else:
        part = np.argpartition(-fft, top_k, axis=1)[:, :top_k]
        # sort those by power
        row = np.arange(m)[:, None]
        vals = fft[row, part]
        order = np.argsort(-vals, axis=1)
        idxs = part[row, order]

    az_all = np.asarray(azimuths, dtype=np.float64).reshape(m)
    row = np.arange(m)[:, None]
    powers = fft[row, idxs]
    mask = powers > power_thresh
    az = np.repeat(az_all[:, None], top_k, axis=1)[mask]
    bins = idxs[mask].astype(np.float64)
    ranges = np.maximum(bins * resolution + RADAR_OFFSET_M, 0.0)
    x = (ranges * np.cos(az)).reshape(-1)
    y = (ranges * np.sin(az)).reshape(-1)
    z = np.zeros_like(x)
    return np.column_stack([x, y, z]).astype(np.float64)


def yaw_pitch_roll_to_rot(y: float, p: float, r: float) -> np.ndarray:
    """Boreas convention: C = roll(r) @ pitch(p) @ yaw(y)."""
    cy, sy = np.cos(y), np.sin(y)
    cp, sp = np.cos(p), np.sin(p)
    cr, sr = np.cos(r), np.sin(r)
    yaw_m = np.array([[cy, sy, 0], [-sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    pitch_m = np.array([[cp, 0, -sp], [0, 1, 0], [sp, 0, cp]], dtype=np.float64)
    roll_m = np.array([[1, 0, 0], [0, cr, sr], [0, -sr, cr]], dtype=np.float64)
    return roll_m @ pitch_m @ yaw_m


def rot_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    """Rotation matrix → quaternion [qx, qy, qz, qw]."""
    t = np.trace(R)
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    return q / np.linalg.norm(q)


def load_poses_csv(path: Path | str) -> tuple[np.ndarray, np.ndarray]:
    """
    Load Boreas applanix/*_poses.csv.

    Returns times (N,) seconds, poses (N, 8) as TUM: t x y z qx qy qz qw
    """
    data = np.genfromtxt(str(path), delimiter=",", names=True)
    # GPSTime is µs integer stored as float
    t = data["GPSTime"].astype(np.float64) * 1e-6
    x = data["easting"].astype(np.float64)
    y = data["northing"].astype(np.float64)
    z = data["altitude"].astype(np.float64)
    roll = data["roll"].astype(np.float64)
    pitch = data["pitch"].astype(np.float64)
    heading = data["heading"].astype(np.float64)

    n = len(t)
    tum = np.zeros((n, 8), dtype=np.float64)
    tum[:, 0] = t
    tum[:, 1] = x
    tum[:, 2] = y
    tum[:, 3] = z
    for i in range(n):
        R = yaw_pitch_roll_to_rot(heading[i], pitch[i], roll[i])
        tum[i, 4:8] = rot_to_quat_xyzw(R)
    return t, tum


def write_tum(path: Path | str, tum: np.ndarray) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        for row in tum:
            f.write(
                f"{row[0]:.6f} {row[1]:.6f} {row[2]:.6f} {row[3]:.6f} "
                f"{row[4]:.6f} {row[5]:.6f} {row[6]:.6f} {row[7]:.6f}\n"
            )


def se3_to_tum_row(t: float, T: np.ndarray) -> np.ndarray:
    q = rot_to_quat_xyzw(T[:3, :3])
    return np.array([t, T[0, 3], T[1, 3], T[2, 3], q[0], q[1], q[2], q[3]], dtype=np.float64)

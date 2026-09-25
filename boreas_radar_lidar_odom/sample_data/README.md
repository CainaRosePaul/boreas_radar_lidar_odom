# Sample Boreas subset (offline viewing)

Includes **5 radar PNGs**, **5 LiDAR `.bin` scans**, matching **Applanix** pose rows, and **calib**.

## Quick view

```bash
cd boreas_radar_lidar_odom
source .venv/bin/activate   # or: python3 -m venv .venv && pip install -r requirements.txt
python scripts/01_inspect.py --data sample_data --out out_sample --frame-idx 0
```

Open `out_sample/inspect_side_by_side.png`.

## What’s inside

| Path | Contents |
|------|----------|
| `radar/*.png` | Navtech polar scans |
| `lidar/*.bin` | Velodyne float32 `(N,6)` |
| `applanix/*_poses.csv` | GT rows for these frames |
| `calib/` | Extrinsics |

For full odometry (~400+ frames), run `bash scripts/00_download.sh`.

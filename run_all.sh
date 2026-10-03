#!/usr/bin/env bash
# End-to-end runner for Boreas radar vs LiDAR odometry mini-project.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PY="${ROOT}/.venv/bin/python"
export MPLBACKEND=Agg

MAX_RADAR="${MAX_RADAR:-400}"
MAX_LIDAR="${MAX_LIDAR:-800}"

echo "== Step 2: inspect =="
"$PY" "$ROOT/scripts/01_inspect.py" --data "$ROOT/boreas-data" --out "$ROOT/out"

echo "== Step 3: radar odometry (max ${MAX_RADAR}) =="
"$PY" "$ROOT/scripts/02_radar_odometry.py" --data "$ROOT/boreas-data" --out "$ROOT/out" --max-frames "$MAX_RADAR"

echo "== Step 4: KISS-ICP LiDAR (max ${MAX_LIDAR}) =="
"$PY" "$ROOT/scripts/03_lidar_kiss_icp.py" --data "$ROOT/boreas-data" --out "$ROOT/out" --max-frames "$MAX_LIDAR"

echo "== Step 5: evo eval =="
"$PY" "$ROOT/scripts/04_eval_evo.py" --data "$ROOT/boreas-data" --out "$ROOT/out"

echo "Done. See $ROOT/out/metrics.json and $ROOT/README.md"

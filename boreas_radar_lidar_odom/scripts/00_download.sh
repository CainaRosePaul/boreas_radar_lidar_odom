#!/usr/bin/env bash
# Download a small Boreas subset (applanix + calib + first N lidar + all radar is huge;
# prefer the curated sync below after a full lidar sync was pruned).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SEQ="${SEQ:-boreas-2021-09-02-11-42}"
DEST="${ROOT}/boreas-data"

mkdir -p "$DEST"
echo "Syncing applanix + calib + radar for ${SEQ} ..."
aws s3 sync "s3://boreas/${SEQ}/applanix/" "$DEST/applanix/" --no-sign-request
aws s3 sync "s3://boreas/${SEQ}/calib/" "$DEST/calib/" --no-sign-request
aws s3 sync "s3://boreas/${SEQ}/radar/" "$DEST/radar/" --no-sign-request \
  --exclude "cart/*" --exclude "mask/*"

# LiDAR is large (~5MB/scan). Download only the first MAX_LIDAR scans by name order.
MAX_LIDAR="${MAX_LIDAR:-1000}"
mkdir -p "$DEST/lidar"
echo "Listing lidar keys..."
mapfile -t KEYS < <(aws s3 ls "s3://boreas/${SEQ}/lidar/" --no-sign-request | awk '{print $4}' | sort | head -n "$MAX_LIDAR")
echo "Downloading ${#KEYS[@]} lidar scans..."
for k in "${KEYS[@]}"; do
  [[ -f "$DEST/lidar/$k" ]] && continue
  aws s3 cp "s3://boreas/${SEQ}/lidar/$k" "$DEST/lidar/$k" --no-sign-request >/dev/null
done
echo "Done. Sizes:"
du -sh "$DEST"/* 2>/dev/null || true

# Radar & LiDAR Odometry on the Boreas Dataset

Comparing 2D Navtech radar odometry with a KISS-ICP LiDAR baseline against Applanix GNSS/INS ground truth on the [Boreas](https://www.boreas.utias.utoronto.ca/) autonomous-driving dataset.

The project is motivated by the complementary properties of radar and LiDAR. LiDAR provides dense and accurate 3D geometry, while radar is typically more robust to precipitation but produces much sparser and noisier measurements.

The current evaluation uses sequence:

`boreas-2021-09-02-11-42`

The same pipeline can also be run on winter Boreas sequences to investigate how radar and LiDAR odometry behave under more challenging weather conditions.

**Stack:** Python · Open3D · KISS-ICP · evo

---

## Overview

The pipeline:

1. Loads radar, LiDAR, calibration, and Applanix GNSS/INS data.
2. Converts Navtech polar radar scans into 2D Cartesian keypoints.
3. Estimates frame-to-frame radar motion using ICP.
4. Runs KISS-ICP as a LiDAR odometry baseline.
5. Converts trajectories to TUM format.
6. Evaluates both methods against Applanix ground truth using evo.

---

## Results

Sequence: `boreas-2021-09-02-11-42`

| Method | RPE drift @ 10 m | ATE RMSE |
|---|---:|---:|
| Radar (top-5 peaks/azimuth + ICP) | **4.5%** | **3.16 m** |
| LiDAR (KISS-ICP) | **1.3%** | **0.29 m** |

On this sequence, LiDAR clearly outperforms the simple radar baseline. The goal of the radar implementation is not to outperform KISS-ICP, but to build and evaluate the complete odometry pipeline from raw Navtech radar observations.

### Project summary

> Built 2D radar odometry from Navtech radar using top-K peak extraction and frame-to-frame ICP on a Boreas sequence, and benchmarked it against a KISS-ICP LiDAR baseline using Applanix GNSS/INS ground truth. Radar drift: **4.5%** vs. LiDAR: **1.3%** over 10 m; ATE: **3.16 m** vs. **0.29 m**.

---

## Motivation

Radar and LiDAR provide different sensing characteristics:

| Sensor | Strength | Limitation |
|---|---|---|
| **LiDAR** | Dense and accurate 3D geometry | Performance can degrade due to precipitation, spray, or difficult atmospheric conditions |
| **Radar** | Robust long-range sensing and less affected by precipitation | Sparse, noisy, and harder to match between frames |
| **Applanix GNSS/INS** | Accurate trajectory reference | Used as ground truth rather than as an odometry input |

Odometry estimates how the vehicle moves from one sensor observation to the next.

The relative motions are accumulated into a trajectory and then compared against the Applanix GNSS/INS trajectory.

The evaluated September sequence does not contain heavy snow, and LiDAR performs substantially better. The pipeline supports other Boreas sequences such as `2020-12-*` and `2021-01-*`, which can be used for future winter-condition experiments.

---

## Repository Structure

```text
boreas_radar_lidar_odom/
├── README.md
├── requirements.txt
├── run_all.sh
├── sample_data/
│   ├── radar/
│   ├── lidar/
│   ├── applanix/
│   ├── calib/
│   ├── preview/
│   └── README.md
├── scripts/
│   ├── 00_download.sh
│   ├── boreas_io.py
│   ├── 01_inspect.py
│   ├── 02_radar_odometry.py
│   ├── 03_lidar_kiss_icp.py
│   └── 04_eval_evo.py
├── out/
│   ├── inspect_side_by_side.png
│   ├── radar_trajectory.txt
│   ├── lidar_trajectory.txt
│   ├── traj_overlay.png
│   ├── radar_ape_map.png
│   ├── lidar_ape_map.png
│   └── metrics.json
└── boreas-data/

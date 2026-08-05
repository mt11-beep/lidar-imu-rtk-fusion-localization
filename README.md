# LiDAR-IMU-RTK Fusion Localization

## Contribution

This project focuses on improving the robustness of LiDAR-IMU localization in GNSS-challenged environments.

Main contributions:

- Designed RTK quality evaluation strategy based on fix status and covariance information.
- Developed RTK outlier rejection module based on temporal consistency and position jump detection.
- Proposed adaptive RTK factor weighting strategy for factor graph optimization.
- Integrated RTK constraints into FAST-LIO2 based localization framework.

## Overview

This repository presents a LiDAR-IMU-RTK fusion localization pipeline built on a FAST-LIO2-style LiDAR-inertial odometry front end. The system adds RTK global position constraints, quality-aware RTK filtering, ENU local frame construction, outlier rejection, and adaptive GPS factor weighting for high-precision robot localization in GNSS-degraded environments.

The repository is a public algorithm showcase. It keeps the core RTK fusion logic and removes unrelated robot drivers, third-party source trees, maps, bags, hardware files, and private project configuration.

## Features

- LiDAR-Inertial Odometry
- RTK coordinate transformation
- ENU local frame construction
- RTK quality evaluation
- Outlier rejection
- Adaptive weighting strategy
- Factor graph based optimization
- Real robot deployment

## System Architecture

Architecture diagram placeholder:

![System Architecture](docs/architecture.png)

The front end estimates local motion from LiDAR and IMU data. RTK measurements are converted from WGS84 to a local ENU frame, filtered by status and covariance, checked for abnormal jumps, and added to the factor graph as position constraints with adaptive noise.

## Directory Structure

```text
lidar-imu-rtk-fusion-localization-public/
├── README.md
├── LICENSE
├── docs/
│   ├── architecture.png
│   ├── experiment.png
│   └── demo_placeholder.md
├── config/
│   └── example.yaml
├── launch/
│   └── fusion_demo.launch.py
├── src/
│   └── rtk_fusion/
│       ├── rtk_types.hpp
│       ├── rtk_filter.cpp
│       ├── rtk_coordinate.cpp
│       ├── gps_factor.cpp
│       ├── adaptive_weight.cpp
│       └── jump_detection.cpp
├── scripts/
│   └── evaluation.py
└── .gitignore
```

`src/rtk_fusion` contains the core RTK processing and fusion code: NavSatFix quality filtering, WGS84-to-ENU conversion, abnormal jump detection, adaptive noise modeling, and GTSAM GPS factor insertion.

## Environment

- Ubuntu 22.04
- ROS2 Humble
- C++
- FAST-LIO2
- GTSAM
- GeographicLib

## Hardware

Tested on:

- 3D LiDAR
- IMU
- RTK GNSS receiver
- Mobile robot platform

## Experiments

Test scenarios:

- Open environment
- Orchard environment

Evaluation metrics:

- RMSE
- Maximum error
- Drift

Experiment figure placeholder:

![Experiment](docs/experiment.png)

## Demo

Video demo placeholder. Add a public demo link after removing private maps, rosbag data, and hardware-specific information.

## Notes

The full FAST-LIO2, ROS driver, Nav2, robot description, and hardware bring-up source trees are intentionally excluded. Where the RTK code depends on FAST-LIO2 or GTSAM runtime state, this repository keeps only the integration interface and comments needed to explain the algorithm.

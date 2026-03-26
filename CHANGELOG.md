# Changelog

All notable changes to Anibot are documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

---

## [Unreleased]

### In Progress
- Wheel odometry + laser odometry EKF fusion (`robot_localization`)
- EKF covariance matrix tuning and validation
- IMU integration (MPU6050 or similar)

---

## [0.1.0] — 2025

### Added
- ROS2 Jazzy differential drive stack on Raspberry Pi 5
- Quadrature encoder odometry via Arduino Mega (28 PPR × 4, gear ratio 19.2:1)
- RPLIDAR A1M8 integration via `rplidar_ros`
- SLAM mapping with `slam_toolbox` (lifelong mode)
- Autonomous navigation with Nav2 (DWB local planner)
- AMCL localization on pre-saved maps
- `anibot_full.launch.py` — full Nav2 launch
- `slam_full.launch.py` — SLAM mapping launch
- Docker-based RViz2 remote visualization from laptop (`docker_jazzy.sh`)
- `anibot_setup.sh` — one-shot dependency installer
- Calibrated wheel base: `0.355 m` (360° spin test, 1.1° residual error)
- Left motor scale compensation: `left_scale = 0.82`
- `swap_motors = True` software fix for physical L/R wiring swap
- PWM deadband: `PWM_MIN = 70` to prevent stall

### Fixed
- `slam_toolbox` crash in ROS2 Jazzy → added `lifecycle_manager` + `bond_timeout: 0.0`
- TF tree errors → added `joint_state_publisher` to launch
- AMCL non-convergence → documented 2D Pose Estimate initialization step
- Motor stall at low PWM → enforced `PWM_MIN = 70` deadband

---

## [0.2.0] — Planned
- Sensor fusion: wheel odom + laser odom via EKF
- IMU integration
- Full 3-source EKF (wheel + laser + IMU)

## [0.3.0] — Planned
- Web monitoring dashboard
- Dynamic obstacle avoidance tuning
- Autonomous dock return

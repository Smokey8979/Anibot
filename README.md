# 🤖 Anibot — ROS2 Autonomous Navigation Robot

<p align="center">
  <img src="assets/anibot_demo.gif" alt="Anibot Demo" width="600"/>
</p>

<p align="center">
  <a href="https://github.com/Smokey8979/Anibot/blob/main/LICENSE"><img src="https://img.shields.io/badge/license-MIT-blue.svg" alt="License"/></a>
  <img src="https://img.shields.io/badge/ROS2-Jazzy-brightgreen?logo=ros" alt="ROS2 Jazzy"/>
  <img src="https://img.shields.io/badge/Platform-Raspberry%20Pi%205-red?logo=raspberrypi" alt="RPi5"/>
  <img src="https://img.shields.io/badge/Nav2-Enabled-orange" alt="Nav2"/>
  <img src="https://img.shields.io/badge/SLAM-Toolbox-yellow" alt="SLAM"/>
  <img src="https://img.shields.io/badge/Status-Active%20Dev-blueviolet" alt="Status"/>
  <img src="https://img.shields.io/github/last-commit/Smokey8979/Anibot" alt="Last Commit"/>
</p>

> ⚠️ **Active development — BETA. Architecture and params may change.**

Anibot is a compact **ROS2 differential drive robot** built around a **Raspberry Pi 5**, capable of **autonomous navigation**, **real-time SLAM mapping**, and **sensor-fused odometry** using wheel encoders and LiDAR. It runs the full Nav2 stack with a calibrated EKF for reliable localization.

---

## 📋 Table of Contents

- [Overview](#-overview)
- [System Architecture](#-system-architecture)
- [Hardware](#-hardware)
- [Software Stack](#-software-stack)
- [Odometry & Sensor Fusion](#-odometry--sensor-fusion)
- [Calibrated Parameters](#-calibrated-parameters)
- [Workspace Structure](#-workspace-structure)
- [Setup & Usage](#️-setup--usage)
- [Features & Status](#-features--status)
- [Roadmap](#-roadmap)
- [Known Issues & Fixes](#-known-issues--fixes)
- [Contributing](#-contributing)
- [Contact](#-contact)

---

## 🚀 Overview

Anibot implements a complete mobile robotics stack on affordable hardware:

- **Autonomous navigation** with Nav2 (DWB controller)
- **SLAM mapping** with slam_toolbox (lifelong mode)
- **Sensor-fused odometry** — wheel encoders + LiDAR via EKF (`robot_localization`)
- **AMCL localization** on saved maps
- **Docker-based RViz** for remote visualization from a laptop
- **Arduino Mega** motor controller with quadrature encoder tracking at 20 Hz

---

## 🧠 System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Raspberry Pi 5                        │
│  ┌────────────┐   ┌──────────────┐   ┌───────────────┐  │
│  │ RPLIDAR A1 │──▶│  /scan topic │──▶│  slam_toolbox │  │
│  └────────────┘   └──────────────┘   └───────────────┘  │
│                                             │            │
│  ┌────────────┐   ┌──────────────┐   ┌─────▼─────────┐  │
│  │Arduino Mega│──▶│ /odom (wheel)│──▶│ robot_local.  │  │
│  │Cytron MDD3A│   └──────────────┘   │ (EKF fusion)  │  │
│  └────────────┘                      └───────┬───────┘  │
│       ▲                                      │           │
│  ┌────┴───────┐   ┌──────────────┐   ┌───────▼───────┐  │
│  │ /cmd_vel   │◀──│     Nav2     │◀──│  /odom_fused  │  │
│  └────────────┘   └──────────────┘   └───────────────┘  │
└─────────────────────────────────────────────────────────┘
                          │ SSH / ROS2 Domain 42
                ┌─────────▼─────────┐
                │  Laptop (Docker)  │
                │  RViz2 + Nav2 UI  │
                └───────────────────┘
```

---

## 🔧 Hardware

| Component | Model | Notes |
|---|---|---|
| **Compute** | Raspberry Pi 5 | Ubuntu 24.04, ROS2 Jazzy |
| **Microcontroller** | Arduino Mega 2560 | Motor control + encoder reading |
| **Motor Driver** | Cytron MDD3A | Dual channel, PWM control |
| **LiDAR** | RPLIDAR A1M8 | 360° scan, ~6m range |
| **Drive** | Differential Drive | 2WD with caster |
| **Encoders** | Quadrature (×4) | 28 PPR per motor, gear ratio 19.2:1 |

**Arduino Pin Mapping:**

| Signal | Pin |
|---|---|
| M1A (Right Motor) | 9 |
| M1B (Right Motor) | 10 |
| M2A (Left Motor) | 5 |
| M2B (Left Motor) | 6 |
| ENC_L_A | 2 (INT) |
| ENC_L_B | 3 (INT) |
| ENC_R_A | 18 (INT) |
| ENC_R_B | 19 (INT) |

---

## 💾 Software Stack

| Layer | Technology |
|---|---|
| OS | Ubuntu 24.04 LTS (RPi5) |
| Middleware | ROS2 Jazzy |
| Navigation | Nav2 (DWB Local Planner) |
| Mapping | slam_toolbox (lifelong) |
| Localization | AMCL + EKF (`robot_localization`) |
| Sensor Fusion | `robot_localization` (wheel odom + laser odom) |
| Visualization | RViz2 (via Docker on laptop) |
| Serial Comm | Python `pyserial` @ 115200 baud |

---

## 🔀 Odometry & Sensor Fusion

Anibot fuses **wheel odometry** and **laser odometry** using an **Extended Kalman Filter (EKF)** via the `robot_localization` package.

```
/odom          (wheel encoders)  ──┐
                                    ├──▶ EKF ──▶ /odometry/filtered
/laser_odom    (LiDAR scan match) ──┘
```

- **Wheel odom** provides high-frequency velocity estimates (20 Hz) but drifts over time
- **Laser odom** corrects long-term drift using scan matching
- **EKF output** `/odometry/filtered` is fed to Nav2 as the primary odometry source

> 🧪 **Fusion testing in progress** — actively tuning EKF covariance matrices and validating against ground truth.

---

## ⚙️ Calibrated Parameters

These values are calibrated through physical testing (360° spin test, straight-line test):

```yaml
# Wheel & Drive
wheel_radius:       0.041 m
wheel_base:         0.355 m     # Calibrated — 360° spin, 1.1° residual error
encoder_ppr:        28          # × 4 quadrature = 112 counts/rev
gear_ratio:         19.2

# Scale correction (motor asymmetry compensation)
left_scale:         0.82
right_scale:        1.00
swap_motors:        true        # Physical L/R wiring swap — corrected in software

# PWM limits
max_pwm:            150
pwm_min:            70          # Deadband — below this motor stalls
max_linear_speed:   0.20 m/s

# Nav2 controller
desired_linear_vel: 0.18 m/s
rotate_angular_vel: 0.42 rad/s
xy_goal_tolerance:  0.20 m
yaw_goal_tolerance: 0.25 rad

# AMCL
particles_min:      1000
particles_max:      5000
z_hit:              0.95
max_beams:          180
```

---

## 📦 Workspace Structure

```
anibot_ws/
├── src/
│   └── interface/          # Main ROS2 package
│       ├── interface/       # Python nodes
│       │   ├── odometery.py     # Wheel odometry node
│       │   └── ...
│       ├── launch/
│       │   ├── anibot_full.launch.py    # Nav2 full launch
│       │   └── slam_full.launch.py     # SLAM launch
│       ├── config/          # Nav2, EKF, AMCL params
│       ├── maps/            # Saved maps (.yaml + .pgm)
│       └── CMakeLists.txt
├── assets/                  # Demo GIFs, images
├── anibot_setup.sh          # One-shot dependency installer
├── docker_jazzy.sh          # RViz2 Docker launcher (laptop)
├── .gitignore
└── README.md
```

---

## ⚙️ Setup & Usage

### 1. Install Dependencies

```bash
chmod +x anibot_setup.sh
./anibot_setup.sh
```

### 2. Build Workspace

```bash
cd ~/anibot_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch SLAM (Mapping)

```bash
ros2 launch interface slam_full.launch.py
```

### 4. Save Map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map \
  --ros-args -p save_map_timeout:=10.0
```

### 5. Launch Autonomous Navigation

```bash
ros2 launch interface anibot_full.launch.py map:=$HOME/maps/my_map.yaml
```

### 6. Teleoperation

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.21 -p turn:=0.43
```

### 7. RViz2 on Laptop (Docker)

```bash
# On laptop — sets up ROS2 Domain 42, software OpenGL
./docker_jazzy.sh
```

> Make sure `ROS_DOMAIN_ID=42` is set on both RPi and laptop.

---

## ✨ Features & Status

| Feature | Status |
|---|---|
| Differential drive control | ✅ Working |
| Quadrature encoder odometry | ✅ Working |
| RPLIDAR A1M8 integration | ✅ Working |
| SLAM mapping (slam_toolbox) | ✅ Working |
| Autonomous navigation (Nav2) | ✅ Working |
| Map saving & reloading | ✅ Working |
| Teleoperation | ✅ Working |
| Docker RViz (laptop) | ✅ Working |
| Wheel + Laser odom EKF fusion | 🧪 Testing |
| IMU integration | 🔜 Planned |
| Autonomous docking | 🔜 Planned |
| Web monitoring dashboard | 🔜 Planned |

---

## 🗺️ Roadmap

- [x] Basic diff drive + encoder odometry
- [x] LiDAR scan integration
- [x] SLAM mapping
- [x] Nav2 autonomous navigation
- [x] Docker RViz remote visualization
- [ ] **Sensor fusion: Wheel odom + Laser odom (EKF)** ← *in progress*
- [ ] IMU integration (MPU6050 or similar)
- [ ] Full 3-source EKF (wheel + laser + IMU)
- [ ] Dynamic obstacle avoidance tuning
- [ ] Web-based robot monitoring dashboard
- [ ] Autonomous charging dock return
- [ ] Multi-floor mapping

---

## 🐛 Known Issues & Fixes

| Issue | Fix Applied |
|---|---|
| Left motor runs faster than right | `left_scale=0.82` software compensation |
| Motors physically wired L/R swapped | `swap_motors=True` in odometry node |
| SLAM node crashes in ROS2 Jazzy | `lifecycle_manager` + `bond_timeout: 0.0` |
| Motors stall at low PWM | `PWM_MIN=70` deadband enforced |
| Arduino resets mid-session causing odom jump | Triple reset on startup + clean encoder state |
| `joint_state_publisher` missing causing TF errors | Added to launch file |
| AMCL not converging | Set 2D Pose Estimate manually in RViz first |
| Wheel base drift in turns | Calibrated to `0.355m` via 360° spin test |

---

## 🤝 Contributing

Currently a solo project — contributions not open yet. Once the sensor fusion and IMU integration are stable, I'll open issues for community input. Feel free to fork and experiment!

---

## 📬 Contact

**Omkarthik Dhudu**
- GitHub: [@Smokey8979](https://github.com/Smokey8979)
- LinkedIn: [Omkarthik Dhudu](https://linkedin.com/in/omkarthikdhudu-a54ab91b9)
- Email: sdhudu@gmail.com

---

## ⭐ Acknowledgements

- [ROS2 Community](https://docs.ros.org/en/jazzy/)
- [Nav2 Developers](https://nav2.org/)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [robot_localization](https://github.com/cra-ros-pkg/robot_localization)
- Open-source robotics ecosystem 🙏

---

<p align="center">
  <i>Built with too much coffee and not enough sleep ☕</i>
</p>

# 🎥 Robust Stereo Camera & IMU Calibration Workspace (via Kalibr Docker)

[![Docker Build](https://img.shields.io/badge/docker-build-blue.svg?logo=docker&logoColor=white)](https://github.com/Nags016/kalibr-calibration-setup)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Kalibr](https://img.shields.io/badge/ETH%20Zurich-Kalibr-red.svg)](https://github.com/ethz-asl/kalibr)

An out-of-the-box, containerized pipeline to perform **Sensor Calibration** for multi-camera and Visual-Inertial systems. This workspace compiles and runs **ETH Zurich's Kalibr** framework inside a Docker container (ROS Noetic base), allowing you to solve camera intrinsics, stereo extrinsics, and camera-IMU spatial-temporal alignment with **zero local dependencies** and **one-click automation scripts**.

---

## 🚀 Why Choose This Workspace?

Fusing cameras and IMUs for state estimation (VIO, SLAM, visual odometry) requires extremely precise sensor calibration. Minor errors in lens modeling, stereo baseline, or camera-IMU timeshift will cause estimators (like OpenVINS, VINS-Mono, or OKVIS) to diverge or drift.

This repository eliminates the hassle of dependency hell:
*   **Dockerized ROS Noetic Environment:** No ROS installation required on the host system.
*   **Automated Manual Initialization:** Eliminates Kalibr startup errors when target grids are not fully visible in the first frames.
*   **GUI / X11 Forwarding Support:** Allows real-time visualization of corners and calibration graph reports inside Docker.
*   **Single-Script Entrypoint:** Run build, intrinsics, and extrinsics phases with simple CLI commands.

---

## 🛠️ Calibration Pipeline Overview

```
                   +------------------------+
                   |  Prepare Aprilgrid     |
                   |  (calibration target)  |
                   +-----------+------------+
                               |
                               v
                   +------------------------+
                   |  Record ROS Bags       |
                   |  (cam_calib & imu_calib|
                   +-----------+------------+
                               |
            +------------------+------------------+
            |                                     |
            v                                     v
+-----------------------+             +-----------------------+
|  Phase 1: Camera-Only |             |  Phase 2: Camera-IMU  |
|  * Intrinsics (fx,fy) |             |  * Extrinsics (T_c_i) |
|  * Distortion (k,p)   |             |  * Timeshift (d_t)    |
|  * Stereo Baseline    |             |                       |
+-----------+-----------+             +-----------+-----------+
            |                                     ^
            | (Generates camchain.yaml)           |
            +-------------------------------------+
```

---

## 📋 Prerequisites

*   **Docker:** Installed and running.
*   **Host System:** Linux/X11 environment (required for GUI report displays).
*   **Calibration Target:** Aprilgrid target (recommended) or checkerboard.

---

## ⏱️ Quick Start in 5 Steps

### 1. Clone the Workspace
```bash
git clone https://github.com/Nags016/kalibr-calibration-setup.git
cd kalibr-calibration-setup
```

### 2. Put Your Rosbags in Place
Place your recorded `.bag` files inside the `bags/` directory.
```
bags/
├── cam_calib.bag   # Stereo camera intrinsics data (slow, smooth sweeps)
└── imu_calib.bag   # Dynamic camera-IMU extrinsics data (high 6-DoF excitation)
```

> [!IMPORTANT]
> The scripts assume your bags use the standard ROS 1 `.bag` format. If you recorded in ROS 2, convert them first using `rosbags-convert --src <ros2_bag_dir> --dst bags/<name>.bag`.

### 3. Build the Image
```bash
./run_calibration.sh build
```
This pulls the ROS Noetic environment and compiles the Kalibr workspace from source.

### 4. Run Stereo Camera Intrinsics (Phase 1)
```bash
./run_calibration.sh cam
```
*   **Solves:** Camera focal lengths, principal centers, distortion parameters, and the relative stereo baseline matrix ($T_{c1\_c0}$).
*   **Outputs:** Generated under `calibration_results/cam_calib/` (includes pdf reports and `cam_calib-camchain.yaml`).

### 5. Run Camera-IMU Extrinsics (Phase 2)
```bash
./run_calibration.sh imu
```
*   **Solves:** 4x4 spatial transformation matrix ($T_{cam\_imu}$) and temporal clock delay (`timeshift`).
*   **Outputs:** Generated under `calibration_results/imu_calib/` (includes `imu_calib-results-imucam.txt` and `imu_calib-camchain-imucam.yaml`).

---

## 🎯 Recording the Perfect Bag File

A calibration is only as good as the raw data you feed it. Follow these strict rules when recording your calibration data:

### Phase 1: Camera-Only (`cam_calib.bag`)
*   **Lighting:** Ensure bright, diffuse, and uniform lighting across the grid. Avoid reflections and shadows.
*   **Camera settings:** Lock auto-exposure to prevent frame rate drops. Disable any IR projectors/emitters.
*   **Motion:** Move the camera slowly and smoothly. Cover all areas of the image plane (corners and edges) to accurately model radial lens distortion. Never let the target go out of frame.

### Phase 2: Camera-IMU (`imu_calib.bag`)
*   **Warm-up:** Power on the camera/IMU for at least 20 minutes before recording to let the IMU sensor reach thermal equilibrium.
*   **Static periods:** Start the recording with the sensor completely stationary for 5 seconds, and end it with another 5 seconds of stationary data.
*   **Excitation:** Exciting the IMU's accelerometers and gyroscopes is critical. Rotate the sensor quickly and sharply around all three axes (roll, pitch, yaw) while translating. Without sufficient excitation, the solver cannot distinguish between sensor bias and spatial offset.

---

## ⚙️ Configuration Setup

Configure your target and sensor noise values before running the pipeline:

1.  **Calibration Target (`config/aprilgrid.yaml`):** Edit this file to match your target grid dimensions, tag size (in meters), and spacing factor.
2.  **IMU Intrinsics (`config/imu.yaml`):** Provide the noise characteristics (noise density and random walk) of your IMU sensor.
    > [!TIP]
    > To obtain highly accurate values for your IMU noise parameters, analyze a 3–4 hour stationary dataset using the **IMU Allan Variance Calibration Suite** (see repository [imu-allan-variance-calibration](https://github.com/Nags016/imu-allan-variance-calibration)).

---

## 🛠️ Command Reference

The runner script supports the following management commands:

| Command | Action |
|:---|:---|
| `./run_calibration.sh build` | Compiles the Kalibr Docker image from source. |
| `./run_calibration.sh cam` | Runs camera intrinsics and stereo calibration. |
| `./run_calibration.sh imu` | Runs camera-IMU spatial-temporal calibration. |
| `./run_calibration.sh shell` | Launches an interactive bash shell inside the container. |

---

## 📊 Evaluation Criteria: What is a Good Calibration?

Once the calibration completes, inspect the output text report to verify your quality metrics:

### Camera Reprojection Error (Phase 1 & 2)
*   **Standard:** Standard deviation should be **`< 0.5 pixels`** (ideally **`< 0.25 pixels`**).
*   **If too high:** Check for motion blur in the camera bag, poor lighting, or incorrect target board specifications.

### Stereo Baseline Accuracy
*   Verify the calculated baseline ($t_x$ in $T_{c1\_c0}$) against the physical lens separation of your camera rig. It should match within **`0.5 mm`**.

### Camera-to-IMU Timeshift
*   Ensure that the time delay converges to a small, constant offset (typically **`< 15 ms`**). Large or fluctuating timeshifts indicate USB bus lag or missing timestamp synchronization in the camera driver.

---

## 📄 License
This project is released under the **MIT License**. See the `LICENSE` file for details.

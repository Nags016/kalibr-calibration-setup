# Kalibr Calibration Setup — Intel RealSense D455

Camera + IMU calibration using [Kalibr](https://github.com/ethz-asl/kalibr) via Docker (ROS Noetic).  
Just clone, add your bag files, and run. No ROS install needed on your machine.

---

## Prerequisites

- [Docker](https://docs.docker.com/engine/install/) installed and running
- Your own ROS bag files (see below)
- ~5 GB free disk space (for the Docker image)

---

## Step 1 — Clone the repo

```bash
git clone https://github.com/Nags016/kalibr-calibration-setup.git
cd kalibr-calibration-setup
```

---

## Step 2 — Add your bag files

Place your `.bag` files inside the `bags/` folder:

```
bags/
├── cam_calib.bag     ← stereo camera recording (slow smooth motion, ~3–5 min)
└── imu_calib.bag     ← stereo + IMU recording (excite all 6 DOF, ~2–3 min)
```

**Recording tips:**
- `cam_calib.bag`: Move slowly, cover all corners of the FOV, avoid motion blur
- `imu_calib.bag`: Rotate and translate along each axis, keep AprilGrid fully visible
- Use 30 Hz camera rate. Use consistent, diffuse lighting.

**Expected ROS topics in your bags:**

| Topic | Description |
|-------|-------------|
| `/d455/infra1/image_rect_raw` | Left infrared camera |
| `/d455/infra2/image_rect_raw` | Right infrared camera |
| `/d455/imu` | IMU at 200 Hz |

> If your topics are different, edit the topic names at the top of `run_calibration.sh`.

---

## Step 3 — Build the Docker image

```bash
./run_calibration.sh build
```

This builds Kalibr from source inside a ROS Noetic container. Takes **10–15 min** on first run.

---

## Step 4 — Run camera calibration

```bash
./run_calibration.sh cam
```

Results saved to `calibration_results/cam_calib/`.

---

## Step 5 — Run camera + IMU calibration

```bash
./run_calibration.sh imu
```

> Must run Step 4 first. Results saved to `calibration_results/imu_calib/`.

---

## All commands

```bash
./run_calibration.sh build    # Build Docker image
./run_calibration.sh cam      # Camera-only calibration
./run_calibration.sh imu      # Camera + IMU calibration
./run_calibration.sh shell    # Open interactive shell inside container
```

---

## Folder Structure

```
kalibr-calibration-setup/
├── run_calibration.sh              # Main entry point — run this
├── config/
│   ├── aprilgrid.yaml              # AprilGrid target config (6×6, 30mm tags)
│   └── imu.yaml                    # IMU noise parameters (D455)
├── bags/                           # ← Put your .bag files here (not in git)
├── calibration_results/
│   ├── cam_calib/                  # Camera intrinsics + stereo extrinsics
│   └── imu_calib/                  # Camera–IMU extrinsics + timeshift
└── docker/
    ├── Dockerfile                  # Kalibr on ROS Noetic
    └── docker-compose.yml          # Optional compose setup
```

---

## Calibration Results (from this repo — 30 Hz run)

### Camera Intrinsics

**cam0** (`/d455/infra1/image_rect_raw`)
```
fx: 386.862  fy: 387.199
cx: 318.890  cy: 245.122
distortion (radtan): [0.00689, -0.02057, 0.00181, -0.00180]
reprojection error: ±0.51 / ±0.73 px
```

**cam1** (`/d455/infra2/image_rect_raw`)
```
fx: 385.996  fy: 387.152
cx: 322.796  cy: 245.735
distortion (radtan): [-0.00928, 0.01224, 0.00160, 0.00002]
reprojection error: ±0.46 / ±0.72 px
```

### Stereo Baseline
```
translation: [-0.09495, -0.00023, -0.00020] m  (~95 mm baseline)
rotation (quaternion xyzw): [-0.00065, 0.00462, -0.00038, 1.00000]
```

### Camera–IMU Extrinsics (T_cam0_imu)
```
[[ 0.99997  -0.00762   0.00184   0.03279]
 [ 0.00763   0.99997  -0.00253  -0.01156]
 [-0.00182   0.00254   1.00000  -0.02120]
 [ 0.        0.        0.        1.     ]]

timeshift (t_imu = t_cam + shift): -0.00949 s
```

### IMU Noise Parameters
```
accelerometer_noise_density: 0.014   m/s²/√Hz
accelerometer_random_walk:   0.0004  m/s³/√Hz
gyroscope_noise_density:     0.0008  rad/s/√Hz
gyroscope_random_walk:       2.2e-5  rad/s²/√Hz
```

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `permission denied` on script | Run `chmod +x run_calibration.sh` |
| Calibration fails to initialize | Already handled — `KALIBR_MANUAL_FOCAL_LENGTH_INIT=1` is set in Docker |
| Wrong topics in bag | Edit topic names at the top of `run_calibration.sh` |
| `xhost` error | Run `xhost +local:docker` before the script |
| Docker not found | Install Docker: https://docs.docker.com/engine/install/ |

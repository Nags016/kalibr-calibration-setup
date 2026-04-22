# Kalibr Calibration Setup — Intel RealSense D455

Camera + IMU calibration using [Kalibr](https://github.com/ethz-asl/kalibr) in Docker (ROS Noetic / Ubuntu 20.04).

---

## Folder Structure

```
kalibr_setup/
├── run_calibration.sh              # Main entry point
├── config/
│   ├── aprilgrid.yaml              # Calibration target config
│   └── imu.yaml                    # IMU noise parameters (D455)
├── bags/
│   ├── cam_calib.bag               # Place your camera bag here
│   └── imu_calib.bag               # Place your IMU+camera bag here
├── calibration_results/
│   ├── cam_calib/
│   │   ├── cam_calib-camchain.yaml         # Camera intrinsics + stereo extrinsics
│   │   ├── cam_calib-results-cam.txt       # Human-readable results
│   │   └── cam_calib-report-cam.pdf        # Visual report
│   └── imu_calib/
│       ├── imu_calib-camchain-imucam.yaml  # T_cam_imu transform + timeshift
│       ├── imu_calib-imu.yaml              # IMU calibration output
│       ├── imu_calib-results-imucam.txt    # Human-readable results
│       └── imu_calib-report-imucam.pdf     # Visual report
└── docker/
    ├── Dockerfile                  # Kalibr build on ROS Noetic
    └── docker-compose.yml          # Optional compose setup
```

---

## Quick Start

### 1. Build the Docker image
```bash
./run_calibration.sh build
```
This clones and builds Kalibr inside a ROS Noetic container. Takes ~10–15 min on first run.

### 2. Place your bag files
```
bags/cam_calib.bag    ← stereo camera recording (slow, smooth motion, ~3–5 min)
bags/imu_calib.bag    ← stereo + IMU recording (excite all 6 DOF, ~2–3 min)
```

### 3. Run camera calibration
```bash
./run_calibration.sh cam
```
Output goes to `calibration_results/cam_calib/`.

### 4. Run camera + IMU calibration
```bash
./run_calibration.sh imu
```
Requires camera calibration to be done first. Output goes to `calibration_results/imu_calib/`.

### 5. Interactive shell (for debugging)
```bash
./run_calibration.sh shell
```

---

## Sensor Configuration

| Parameter | Value |
|-----------|-------|
| Camera | Intel RealSense D455 (stereo infrared) |
| Topics | `/d455/infra1/image_rect_raw`, `/d455/infra2/image_rect_raw` |
| IMU topic | `/d455/imu` |
| Resolution | 640 × 480 |
| Camera model | Pinhole + Radtan distortion |
| IMU rate | 200 Hz |
| Target | AprilGrid 6×6, tag size 30mm, spacing ratio 0.3 |

---

## Calibration Results (30 Hz run — best quality)

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

### Stereo Baseline (T_cam1_cam0)
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

### Residuals (IMU calibration)
```
Reprojection error:   mean 0.161 px,  std 0.133 px
Gyroscope error:      mean 0.00436 rad/s
Accelerometer error:  mean 0.0272 m/s²
```

---

## Tips for Good Calibration

- **Camera bag**: Move slowly and smoothly, cover all corners of the field of view. Avoid motion blur.
- **IMU bag**: Excite all 6 degrees of freedom — rotate and translate along each axis. Keep the target fully visible.
- **Lighting**: Use consistent, diffuse lighting. Avoid reflections on the AprilGrid.
- **Recording rate**: 30 Hz camera works well. Higher rates (60 Hz+) can cause sync issues.
- If calibration fails to initialize, the `KALIBR_MANUAL_FOCAL_LENGTH_INIT=1` env var is already set in the Docker entrypoint.

---

## Requirements

- Docker (with BuildKit)
- `xhost` for GUI display (only needed if viewing reports inside container)
- ~5 GB disk space for the Docker image

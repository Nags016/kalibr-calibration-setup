# Kalibr Calibration Guide
## Intel RealSense D455 + BMI085 IMU → OpenVINS (GPS-Denied Flight)
### Team aeroKLE — IRoC-U 2026

---

## Table of Contents

1. [Why Calibration Matters](#1-why-calibration-matters)
2. [What We Are Calibrating](#2-what-we-are-calibrating)
3. [Hardware and Software Requirements](#3-hardware-and-software-requirements)
4. [Step 1 — Install Kalibr via Docker](#4-step-1--install-kalibr-via-docker)
5. [Step 2 — Generate the April Tag Target](#5-step-2--generate-the-april-tag-target)
6. [Step 3 — Prepare IMU Noise Parameters](#6-step-3--prepare-imu-noise-parameters)
7. [Step 4 — Configure and Verify RealSense Launch](#7-step-4--configure-and-verify-realsense-launch)
8. [Step 5 — Record Camera Calibration Bag](#8-step-5--record-camera-calibration-bag)
9. [Step 6 — Run Camera Intrinsics + Extrinsics Calibration](#9-step-6--run-camera-intrinsics--extrinsics-calibration)
10. [Step 7 — Record Camera-IMU Calibration Bag](#10-step-7--record-camera-imu-calibration-bag)
11. [Step 8 — Run Camera-IMU Calibration](#11-step-8--run-camera-imu-calibration)
12. [Step 9 — Update OpenVINS Config Files](#12-step-9--update-openvins-config-files)
13. [Step 10 — Verify Calibration Quality](#13-step-10--verify-calibration-quality)
14. [Common Errors and Fixes](#14-common-errors-and-fixes)
15. [Quick Checklist](#15-quick-checklist)
16. [Calibration Quality Reference Table](#16-calibration-quality-reference-table)

---

## 1. Why Calibration Matters

### The direct link between calibration and your drift problem

Your drone drifts in a **random direction every flight**. This is not a software bug. It is a calibration accuracy problem combined with IMU bias non-convergence.

Here is exactly what happens when calibration is wrong:

```
Wrong timeshift_cam_imu (even by 5ms):

  IMU says:    "I rotated left at t=1.000s"
  Camera says: "I see leftward motion at t=1.005s"
  
  OpenVINS thinks: "IMU and camera disagree — I must be rotating AND translating"
  Result:          Phantom velocity in a random direction
  
  Random direction because: the bias direction changes each boot
  Integrates over time:     small error → grows to meters of drift
```

**Correct calibration eliminates this.** With reprojection error below 0.3px and timeshift within ±5ms, OpenVINS gets consistent results every boot, and random-direction drift stops.

---

## 2. What We Are Calibrating

There are **three separate calibrations**, each building on the previous.

### 2.1 Camera Intrinsics (per camera)

Answers: *"What are the optical properties of each lens?"*

```
Parameters solved:
  fx, fy        — focal lengths in pixels
  cx, cy        — principal point (optical centre) in pixels
  k1, k2        — radial distortion coefficients
  p1, p2        — tangential distortion coefficients

Why it matters:
  If fx is wrong by even 2px, every feature position OpenVINS computes
  is slightly wrong. Over 15 frames per second, small position errors
  integrate into large drift.
```

### 2.2 Stereo Extrinsics (camera-to-camera)

Answers: *"Exactly how far apart are infra1 and infra2, and at what rotation?"*

```
Parameters solved:
  Baseline    — physical distance between the two IR cameras (~94.95mm on D455)
  Rotation    — any angular misalignment between the two cameras
  
Why it matters:
  OpenVINS uses stereo geometry to compute metric scale (real-world distances).
  If the baseline is off by 1mm, scale is off by ~1%.
  At 9m arena width, that is 9cm of accumulated error per pass.
```

### 2.3 Camera-IMU Extrinsics and Timeshift

Answers: *"What is the physical rotation between the camera and the IMU, and what is the time delay between their measurements?"*

```
Parameters solved:
  T_cam_imu        — 4×4 rigid body transform: rotation + translation
  timeshift_cam_imu — time delay in seconds (camera timestamp − IMU timestamp)
  
Why it matters:
  The IMU is physically inside the D455 body.
  The camera lens is offset from the IMU by several centimetres.
  OpenVINS must know this exact offset to correctly fuse the two sensors.
  
  The timeshift matters because:
  USB introduces variable latency on image timestamps.
  IMU timestamps are hardware-generated (accurate).
  The difference must be measured precisely.
  
  Your previous calibrated value: timeshift = -0.00949s
  If this is wrong by 10ms → random drift every flight (your exact symptom)
```

---

## 3. Hardware and Software Requirements

### 3.1 Physical Items

| Item | Specification | Why |
|------|--------------|-----|
| Calibration board | April Tag 6×6, printed at exact scale on rigid forex board | Warped boards corrupt calibration — paper warps, forex does not |
| Board size | Minimum 80cm × 80cm | Must fill camera FOV at 1m distance |
| Tripod or stand | To hold board fixed | Board must not move at all during recording |
| Lighting | Consistent, no shadows, no flicker | Shadows cause missed corner detection |
| Ruler | For verifying print scale | One tag must measure exactly as specified |
| Open space | ~3m × 3m minimum | You need room to move the drone around the board |

### 3.2 Software Requirements

```bash
# Check what you already have
docker --version          # Need Docker installed
ros2 --version            # Need ROS2 Jazzy
python3 --version         # Need Python 3.8+

# Required Python package for bag conversion
pip3 install rosbags

# Verify rosbags works
rosbags-convert --help
```

### 3.3 Important: IR Projector Must Be OFF

The D455 has a structured-light IR projector that fires infrared dots onto the scene to assist depth. When it is on:

```
Infra1 and Infra2 images show bright white blobs all over them.
These blobs move with the projector, not with the scene.
OpenVINS tracks these blobs as features.
When blobs appear/disappear → phantom motion → drift.

You MUST disable the projector for all VIO operation and ALL calibration.
The flag is: depth_module.emitter_enabled:=0
```

---

## 4. Step 1 — Install Kalibr via Docker

Docker is the recommended installation method. It avoids dependency conflicts with ROS versions.

```bash
# Install Docker (skip if already installed)
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER

# IMPORTANT: Log out and log back in after this
# Otherwise Docker commands will fail with permission error

# Verify Docker works
docker run hello-world

# Pull Kalibr image
docker pull stereolabs/kalibr:latest

# Verify Kalibr works
docker run -it stereolabs/kalibr kalibr_calibrate_cameras --help
# Should print: "usage: kalibr_calibrate_cameras ..."
```

If the Docker pull fails (network issues on Pi 5), build from source:

```bash
# Alternative: build from source (takes 20-30 minutes)
git clone https://github.com/ethz-asl/kalibr.git
cd kalibr
docker build -t kalibr .

# Then replace "stereolabs/kalibr" with "kalibr" in all commands below
```

---

## 5. Step 2 — Generate the April Tag Target

### 5.1 Why April Tag Instead of Checkerboard

```
Checkerboard problems:
  → Can only be detected when ALL corners are fully visible
  → Fails if any corner is near image edge or occluded
  → Corner labelling is ambiguous (which corner is #1?)
  → Works poorly at steep angles

April Tag advantages:
  → Each tag has a unique binary ID — never confused with another
  → Detected even when partially visible or at steep angles
  → Kalibr was specifically designed and optimised for April Tags
  → More stable calibration with fewer images required
```

### 5.2 Calculate Your Target Dimensions

```
Formula:
  Board total width  = (tagCols × tagSize) + ((tagCols-1) × tagSize × tagSpacing) + 2×border
  Board total height = (tagRows × tagSize) + ((tagRows-1) × tagSize × tagSpacing) + 2×border

With tagCols=6, tagRows=6, tagSize=0.088m, tagSpacing=0.3, border=0.02m:
  Total width  = (6 × 88mm) + (5 × 26.4mm) + 40mm = 700mm → print on 80cm wide board
  Total height = same → 700mm → print on 80cm tall board

At 1.0m flight altitude, D455 infra camera sees approximately:
  Width:  640px / 434px focal length × 1.0m ≈ 1.47m
  Height: 480px / 434px focal length × 1.0m ≈ 1.11m

Your 70cm × 70cm board fills ~48% of the frame at 1m → good coverage
```

### 5.3 Create Target Configuration File

```bash
mkdir -p ~/calibration
cat > ~/calibration/april_6x6.yaml << 'EOF'
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.088
tagSpacing: 0.3
EOF
```

### 5.4 Generate and Print the Target

```bash
mkdir -p ~/calibration/output

docker run -it \
  -v ~/calibration/output:/output \
  stereolabs/kalibr \
  kalibr_create_target_pdf \
  --type aprilgrid \
  --nx 6 --ny 6 \
  --tsize 0.088 \
  --tspace 0.3 \
  --outfile /output/april_6x6_target.pdf

# PDF is now at ~/calibration/output/april_6x6_target.pdf
# Open and print
evince ~/calibration/output/april_6x6_target.pdf
```

### 5.5 Verify Print Scale

After printing, **measure with a physical ruler**:

```
Measure one complete April Tag square (dark border to dark border).
It must be exactly 88mm.

If it is smaller: your printer scaled it down → reprint at 100% scale
If it is larger:  your printer scaled it up → reprint at 100% scale

Go into printer settings → Page Scaling → set to "None" or "Actual Size"
```

### 5.6 Mount the Board

```
Mount on forex board (PVC foam board, 5mm thick):
  1. Print on A0 paper (or tile multiple A4 sheets and tape perfectly flat)
  2. Apply spray adhesive to forex board
  3. Lay paper on board, smooth out ALL bubbles
  4. Let dry completely
  5. Check with straight edge — board must be perfectly flat
  6. Mount on tripod at comfortable working height (~1.2m from floor)
```

---

## 6. Step 3 — Prepare IMU Noise Parameters

### 6.1 Why IMU Parameters Matter for Kalibr

Kalibr uses IMU noise values to decide how much to trust the IMU relative to the camera during optimisation. If noise values are wrong:

```
Too low (underestimate noise):
  Kalibr over-trusts IMU → small IMU errors dominate → wrong timeshift
  
Too high (overestimate noise):
  Kalibr under-trusts IMU → poor observability → optimisation fails
```

### 6.2 BMI085 Specification Values

The Pixhawk 6C uses BMI085 IMU. These are the datasheet values — use them directly:

```bash
cat > ~/calibration/imu_params.yaml << 'EOF'
# BMI085 IMU Noise Parameters
# Source: Bosch BMI085 datasheet (verified against AGENTS.md)
# These values are for the IMU inside Intel RealSense D455

rostopic: /d455/imu
update_rate: 200.0

# Accelerometer
# In-run bias stability: 40 µg → ≈ 0.000392 m/s²
# Velocity random walk: derived from noise density
accelerometer_noise_density: 0.00207      # m/s²/√Hz  (BMI085 spec: 200 µg/√Hz)
accelerometer_random_walk:   0.000413     # m/s³/√Hz  (BMI085 spec)

# Gyroscope
# In-run bias stability: 2 °/h
# Angular random walk: derived from noise density
gyroscope_noise_density:     0.000205     # rad/s/√Hz  (BMI085 spec: 0.014 °/s/√Hz)
gyroscope_random_walk:       0.0000111    # rad/s²/√Hz (BMI085 spec)
EOF
```

### 6.3 Optional: Measure IMU Noise Experimentally

This gives more accurate values than datasheet specs. Only do this if you have 3+ hours and the calibration is still failing:

```bash
# Install imu_utils
cd ~/ros2_jazzy/src
git clone https://github.com/gaowenliang/imu_utils.git
cd ~/ros2_jazzy
colcon build --packages-select imu_utils
source install/setup.bash

# Place drone on vibration-free surface (not on table with fans)
# Record IMU for exactly 3 hours
ros2 bag record /d455/imu -o ~/calibration/imu_noise_bag &
sleep 10800  # 3 hours
kill %1

# Process with imu_utils
ros2 run imu_utils imu_an \
  --imu_topic /d455/imu \
  --imu_frequency 200 \
  --max_time_min 180

# Read output and update imu_params.yaml with measured values
```

---

## 7. Step 4 — Configure and Verify RealSense Launch

### 7.1 Why These Specific Launch Parameters

Every parameter below has a specific reason:

```bash
# Create calibration launch script
cat > ~/calibration/launch_for_calibration.sh << 'EOF'
#!/bin/bash
# RealSense launch configured specifically for Kalibr calibration
# DO NOT use the normal flight launch script for calibration

source ~/ros2_jazzy/install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=/ \
  camera_name:=d455 \
  enable_color:=false \          # Color not needed, saves USB bandwidth
  enable_depth:=false \          # Depth not needed, and competing for IR
  enable_gyro:=true \            # IMU required for camera-IMU calibration
  enable_accel:=true \           # IMU required
  depth_module.infra_profile:=640x480x15 \   # 640x480 at 15fps — matches flight config
  depth_module.emitter_enabled:=0 \          # CRITICAL: IR projector OFF
  gyro_fps:=200 \                # 200Hz IMU — must match imu_params.yaml update_rate
  accel_fps:=200 \               # 200Hz
  unite_imu_method:=2 \          # Combine gyro+accel into single /imu topic
  enable_sync:=true \            # Hardware timestamp synchronisation between cameras
  enable_infra1:=true \          # Left IR camera → cam0 in Kalibr
  enable_infra2:=true            # Right IR camera → cam1 in Kalibr
EOF

chmod +x ~/calibration/launch_for_calibration.sh
```

### 7.2 Verify All Topics Before Recording

Run the launch script in one terminal, then verify in another:

```bash
# Terminal 1
~/calibration/launch_for_calibration.sh

# Terminal 2 — verify topics
source ~/ros2_jazzy/install/setup.bash

# Check rates — all must pass before you start recording
ros2 topic hz /d455/infra1/image_rect_raw
# Expected: 14.5–15.5 Hz — FAIL if outside this range

ros2 topic hz /d455/infra2/image_rect_raw
# Expected: 14.5–15.5 Hz — FAIL if outside this range

ros2 topic hz /d455/imu
# Expected: 195–205 Hz — FAIL if outside this range

# Check image content — must see CLEAN images with NO white blobs
ros2 run rqt_image_view rqt_image_view &
# Select topic: /d455/infra1/image_rect_raw
# You should see: clear greyscale IR image of the scene
# You must NOT see: white circular blobs scattered across the image
# If you see blobs: IR projector is still ON — check emitter_enabled:=0
```

### 7.3 Check Timestamp Synchronisation

```bash
# Verify camera and IMU timestamps are in sync
ros2 topic echo /d455/infra1/image_rect_raw --field header.stamp --no-arr | head -5
ros2 topic echo /d455/imu --field header.stamp --no-arr | head -5

# The seconds values should be within 0.1s of each other
# If they differ by more than 1 second: sync is broken, restart the camera
```

---

## 8. Step 5 — Record Camera Calibration Bag

### 8.1 What This Bag Is For

This bag is used to calibrate **camera intrinsics** (lens parameters) and **stereo extrinsics** (the baseline between infra1 and infra2).

The key requirement: show the board from **many different angles and distances** while holding each position still.

### 8.2 Physical Setup

```
Layout:
  
  [Board on tripod]  ← FIXED, does not move at all
        ↑
      1.0m
        ↑
  [You holding drone] ← You move this around the board
  
Board height: 1.2m from floor (eye level when holding drone)
Your position: start 1.0m in front of board
Room needed: 2m radius around the board (you will move sideways, up, down)
```

### 8.3 Motion Sequence for Camera Calibration

```
The board must stay FULLY VISIBLE in BOTH infra1 and infra2 at all times.
Move to each position SLOWLY (2-3 seconds), then HOLD STILL for 5-8 seconds.
Do not rush. Quality matters more than quantity.

Position 1:  Face board straight on at 1.0m.              HOLD 10s
Position 2:  Tilt drone nose DOWN 25°.                    HOLD 7s
Position 3:  Tilt drone nose UP 25°.                      HOLD 7s
Position 4:  Return to straight. HOLD 5s
Position 5:  Tilt drone RIGHT side down (roll) 25°.       HOLD 7s
Position 6:  Tilt drone LEFT side down (roll) 25°.        HOLD 7s
Position 7:  Return to straight. HOLD 5s
Position 8:  Yaw drone 30° clockwise.                     HOLD 7s
Position 9:  Yaw drone 30° counter-clockwise.             HOLD 7s
Position 10: Return to straight. HOLD 5s
Position 11: Step CLOSER to board — 0.7m distance.        HOLD 7s
Position 12: Step FARTHER from board — 1.5m distance.     HOLD 7s
Position 13: Return to 1.0m. HOLD 5s
Position 14: Move drone LEFT 0.4m (board now right-of-centre). HOLD 7s
Position 15: Move drone RIGHT 0.4m (board now left-of-centre). HOLD 7s
Position 16: Move drone UP 0.3m.                          HOLD 7s
Position 17: Move drone DOWN 0.3m.                        HOLD 7s
Position 18: Return to centre 1.0m. HOLD 10s

Total duration: approximately 4-5 minutes
```

### 8.4 Record the Bag

```bash
mkdir -p ~/calibration/bags

# Terminal 2 (while RealSense is running in Terminal 1)
ros2 bag record \
  /d455/infra1/image_rect_raw \
  /d455/infra2/image_rect_raw \
  /d455/imu \
  --output ~/calibration/bags/camera_calib_$(date +%Y%m%d_%H%M%S)

# Perform the motion sequence above
# Press Ctrl+C when done
```

### 8.5 Verify the Bag

```bash
# Check the bag was recorded correctly
ros2 bag info ~/calibration/bags/camera_calib_*/

# What you must see:
# Duration:     240–300s (4–5 minutes)
# Topic: /d455/infra1/image_rect_raw   Messages: ~3600  (240 × 15Hz)
# Topic: /d455/infra2/image_rect_raw   Messages: ~3600
# Topic: /d455/imu                     Messages: ~48000 (240 × 200Hz)

# If any topic has 0 messages: recording failed, delete and re-record
```

### 8.6 Convert Bag to ROS1 Format

Kalibr requires ROS1 bag format. Convert it:

```bash
# Install rosbags converter if not already installed
pip3 install rosbags

# Convert (replace the folder name with your actual bag folder name)
rosbags-convert \
  ~/calibration/bags/camera_calib_*/ \
  --dst ~/calibration/bags/camera_calib.bag

# Verify conversion
ls -lh ~/calibration/bags/camera_calib.bag
# Should show a .bag file, size approximately 200-400 MB
```

---

## 9. Step 6 — Run Camera Intrinsics + Extrinsics Calibration

### 9.1 What This Step Solves

```
Input:  Bag with infra1 + infra2 images showing the April Tag board
Output: 
  → fx, fy, cx, cy for each camera
  → k1, k2, p1, p2 distortion for each camera
  → Stereo baseline and rotation (T_cn_cnm1)
  → camera_calib-camchain.yaml (input for next step)
```

### 9.2 Run Kalibr Camera Calibration

```bash
mkdir -p ~/calibration/results

docker run -it \
  -v ~/calibration/bags:/bags \
  -v ~/calibration/results:/results \
  -v ~/calibration:/target \
  stereolabs/kalibr \
  kalibr_calibrate_cameras \
  --bag /bags/camera_calib.bag \
  --topics /d455/infra1/image_rect_raw /d455/infra2/image_rect_raw \
  --models pinhole-radtan pinhole-radtan \
  --target /target/april_6x6.yaml \
  --bag-from-to 5 290 \
  --show-extraction \
  --verbose \
  --output-results /results/camera_calib
```

### 9.3 What Each Flag Means

| Flag | Value | Reason |
|------|-------|--------|
| `--topics` | infra1 then infra2 | Order matters — first = cam0, second = cam1. Must match OpenVINS config. |
| `--models pinhole-radtan` | pinhole-radtan for both | Pinhole: standard lens model. Radtan: radial+tangential distortion. Correct for D455 passive stereo (projector OFF). Do NOT use equidist. |
| `--bag-from-to 5 290` | Skip first/last 5s | Camera pipeline was still stabilising at start of recording. |
| `--show-extraction` | (flag only) | Opens a window showing detected April Tags in each frame. Watch this during processing. |
| `--verbose` | (flag only) | Prints optimisation progress. Helps diagnose if it is failing. |

### 9.4 Watching the Extraction Window

When `--show-extraction` opens a window, you should see:

```
GOOD: Green dots at all four corners of each visible April Tag
      Most frames show full board detection
      Corner positions look geometrically consistent across frames

BAD:  Red X marks where detection failed
      More than 30% of frames show no detection → bad lighting or board too small
      Corner positions jump around erratically → board is warped
```

If more than 30% of frames fail detection, stop the process and check:
- Is lighting even across the board? (no shadows)
- Did the board move during recording?
- Is the board flat (no warping)?
- Was the camera in focus at all distances?

### 9.5 Interpreting the Output

```
Look for this in the terminal output:

Calibration complete.
Reprojection error:
  cam0: mean = X.XXpx, max = X.XXpx
  cam1: mean = X.XXpx, max = X.XXpx

Stereo baseline: XX.XXmm

Quality interpretation:
  Mean reprojection < 0.15px  → Excellent. Proceed.
  Mean reprojection 0.15-0.30px → Good. Acceptable.
  Mean reprojection 0.30-0.50px → Marginal. Try again with better board.
  Mean reprojection > 0.50px  → Failed. Do not use. Fix board and re-record.

Stereo baseline should be: 94.0mm – 96.0mm for D455
  Outside this range → something is wrong with topic order or board
```

### 9.6 Output Files

```bash
ls ~/calibration/results/

# camera_calib-camchain.yaml     ← CRITICAL: input for Step 8 (IMU-cam calibration)
# camera_calib-report.pdf        ← Open this. Visual quality report.
# camera_calib-results-cam.txt   ← Detailed numerical results

# Open the PDF report and check:
# - Corner detection images look clean
# - Residual plots show random scatter (not systematic pattern)
# - Systematic pattern in residuals = board is warped
evince ~/calibration/results/camera_calib-report.pdf
```

---

## 10. Step 7 — Record Camera-IMU Calibration Bag

### 10.1 Why This Bag Is Different

Camera calibration needed **static poses** — hold still and let the camera observe the board from many angles.

Camera-IMU calibration needs **dynamic continuous motion** — the IMU must be excited so Kalibr can observe the IMU response and match it to the visual motion.

```
Physics behind this:

When you translate the drone left at 0.3 m/s:
  Camera sees: board moves right at some pixel rate
  IMU sees:    acceleration spike in the left direction

Kalibr matches these two signals in time.
The time offset that gives the best match = timeshift_cam_imu

If the motion is too slow: IMU barely responds → poor observability → wrong timeshift
If the motion is too fast: motion blur in images → features cannot be tracked
Target speed: each motion takes 1.5–2.0 seconds
```

### 10.2 Motion Sequence for Camera-IMU Calibration

```
IMPORTANT: Move CONTINUOUSLY. Do not hold still between movements.
Only hold still at the very beginning and end for IMU bias reference.
Keep the board in frame the ENTIRE time.

Phase 1 — Static reference (10 seconds, completely still)
Phase 2 — X-axis translation (left-right, 30cm amplitude, 1.5s per pass)
  Repeat 5 times: left → right → left → right → left
Phase 3 — Z-axis translation (up-down, 20cm amplitude, 1.5s per pass)
  Repeat 5 times: up → down → up → down → up
Phase 4 — Y-axis translation (toward/away from board, 20cm amplitude, 2s per pass)
  Repeat 4 times: forward → back → forward → back
Phase 5 — Roll rotation (tilt left-right, ±20°, 1.5s per pass)
  Repeat 5 times
Phase 6 — Pitch rotation (tilt forward-back, ±20°, 1.5s per pass)
  Repeat 5 times
Phase 7 — Yaw rotation (spin left-right, ±25°, 2s per pass)
  Repeat 4 times
Phase 8 — Figure-8 combination motion (translation + rotation together)
  Repeat 4 times, each figure-8 takes about 4 seconds
Phase 9 — Static reference (10 seconds, completely still)

Total duration: approximately 3 minutes
```

### 10.3 Record the Bag

```bash
# NOTE: Only infra1 is needed for IMU-cam. Stereo baseline already known from Step 5.
ros2 bag record \
  /d455/infra1/image_rect_raw \
  /d455/imu \
  --output ~/calibration/bags/imu_cam_calib_$(date +%Y%m%d_%H%M%S)

# Perform the motion sequence above
# Press Ctrl+C when done
```

### 10.4 Verify and Convert the Bag

```bash
ros2 bag info ~/calibration/bags/imu_cam_calib_*/

# What you must see:
# Duration: 180–240s (3–4 minutes)
# Topic: /d455/infra1/image_rect_raw   Messages: ~2700  (180s × 15Hz)
# Topic: /d455/imu                     Messages: ~36000 (180s × 200Hz)

# Convert to ROS1 format
rosbags-convert \
  ~/calibration/bags/imu_cam_calib_*/ \
  --dst ~/calibration/bags/imu_cam_calib.bag

ls -lh ~/calibration/bags/imu_cam_calib.bag
# Typical size: 100–250 MB
```

---

## 11. Step 8 — Run Camera-IMU Calibration

### 11.1 What This Step Solves

```
Input:
  → IMU-cam motion bag (infra1 + imu)
  → camera_calib-camchain.yaml from Step 6
  → imu_params.yaml from Step 3

Output:
  → T_cam_imu: 4×4 rigid body transform (rotation + translation)
  → timeshift_cam_imu: time delay in seconds
  → imu_cam_calib-results-imu.txt with all values
  → imu_cam_calib-report.pdf
```

### 11.2 Run Kalibr Camera-IMU Calibration

```bash
docker run -it \
  -v ~/calibration/bags:/bags \
  -v ~/calibration/results:/results \
  -v ~/calibration:/target \
  stereolabs/kalibr \
  kalibr_calibrate_imu_camera \
  --bag /bags/imu_cam_calib.bag \
  --cam /results/camera_calib-camchain.yaml \
  --imu /target/imu_params.yaml \
  --target /target/april_6x6.yaml \
  --bag-from-to 5 175 \
  --verbose \
  --output-results /results/imu_cam_calib
```

### 11.3 What Good Output Looks Like

```
The terminal will print the calibration results at the end.
Look for this section:

Transformation (cam0 to imu0):
-----------------------
T_ci:  (imu0 to cam0):
  [[  0.9999,  0.0012, -0.0087,  0.0038],
   [ -0.0013,  0.9999, -0.0145, -0.0001],
   [  0.0087,  0.0146,  0.9999,  0.1438],
   [  0.      0.      0.      1.     ]]

timeshift cam0 to imu0: -0.00384 [s]

Quality checks:
  timeshift must be in range: -0.030s to +0.030s
    Outside this: USB timing issue or motion too slow in bag
  
  Rotation matrix diagonal values must be close to 1.0
    Values below 0.99: camera and IMU badly misaligned in recording
  
  Translation Z value (~0.14m): distance from IMU to camera lens centre
    Should be plausible given your physical camera mount position

  Reprojection error printed during optimisation should decrease
    If it increases: optimisation is diverging, re-record bag with faster motion
```

### 11.4 If Optimisation Fails

```
Error: "Spline fit failed"
  Cause: Motion was too slow, IMU not excited enough
  Fix: Re-record bag with faster, more exaggerated movements

Error: "No corners detected"
  Cause: board moved, bad lighting, or IR projector on during recording
  Fix: Check emitter_enabled:=0, fix lighting, re-record

Error: "Time shift > 0.1s, calibration unreliable"
  Cause: USB latency is very high or IMU rate is wrong
  Fix: 
    1. Connect D455 directly to Pi 5 USB 3.0 port (not through hub)
    2. Verify IMU is actually at 200Hz: ros2 topic hz /d455/imu
    3. Try recording with a shorter, faster motion bag

Error: "Singular matrix during optimisation"
  Cause: Not enough variety in the motion — one axis was not excited
  Fix: Re-record bag, make sure you do ALL phases (X, Y, Z, roll, pitch, yaw)
```

---

## 12. Step 9 — Update OpenVINS Config Files

### 12.1 Understanding the Config File Structure

OpenVINS uses two main config files that you update with Kalibr results:

```
~/ros2_jazzy/src/open_vins/config/rs_d455/
  kalibr_imucam_chain.yaml    ← Camera intrinsics + T_cam_imu + timeshift
  kalibr_imu_chain.yaml       ← IMU noise parameters
  estimator_config.yaml       ← OpenVINS algorithm parameters (not changed by Kalibr)
```

### 12.2 Update kalibr_imucam_chain.yaml

Open your calibration results:

```bash
cat ~/calibration/results/imu_cam_calib-results-imu.txt
# Read the T_cam_imu matrix and timeshift values

cat ~/calibration/results/camera_calib-camchain.yaml
# Read the intrinsics (fx, fy, cx, cy) and distortion (k1, k2, p1, p2) for each camera
```

Now update the live config file:

```bash
# Open the live config file
nano ~/ros2_jazzy/src/open_vins/config/rs_d455/kalibr_imucam_chain.yaml
```

Replace with your calibration output values:

```yaml
# kalibr_imucam_chain.yaml
# Generated by Kalibr — replace ALL values below with YOUR calibration output
# Last calibrated: [DATE]
# Reprojection error cam0: [YOUR VALUE]px, cam1: [YOUR VALUE]px
# Timeshift: [YOUR VALUE]s

cameras:

- camera:
    camera_model: pinhole
    distortion_coeffs: [k1, k2, p1, p2]    # ← FROM camera_calib-camchain.yaml cam0
    distortion_model: radtan
    image_height: 480
    image_width: 640
    intrinsics: [fx, fy, cx, cy]            # ← FROM camera_calib-camchain.yaml cam0
    rostopic: /d455/infra1/image_rect_raw

  T_cam_imu:                                # ← FROM imu_cam_calib-results-imu.txt
  - [r00, r01, r02, t0]                     #   Copy the 4x4 matrix exactly
  - [r10, r11, r12, t1]
  - [r20, r21, r22, t2]
  - [0.0, 0.0, 0.0, 1.0]

  timeshift_cam_imu: X.XXXXX               # ← FROM imu_cam_calib-results-imu.txt
  camera_model: pinhole
  distortion_model: radtan

- camera:
    camera_model: pinhole
    distortion_coeffs: [k1, k2, p1, p2]    # ← FROM camera_calib-camchain.yaml cam1
    distortion_model: radtan
    image_height: 480
    image_width: 640
    intrinsics: [fx, fy, cx, cy]            # ← FROM camera_calib-camchain.yaml cam1
    rostopic: /d455/infra2/image_rect_raw

  T_cam_imu:                                # ← cam1 T_cam_imu from calibration
  - [r00, r01, r02, t0]
  - [r10, r11, r12, t1]
  - [r20, r21, r22, t2]
  - [0.0, 0.0, 0.0, 1.0]

  timeshift_cam_imu: X.XXXXX               # ← same as cam0 (same USB path)
  camera_model: pinhole
  distortion_model: radtan
```

### 12.3 Update kalibr_imu_chain.yaml

```bash
nano ~/ros2_jazzy/src/open_vins/config/rs_d455/kalibr_imu_chain.yaml
```

```yaml
# kalibr_imu_chain.yaml
# BMI085 values — verified against datasheet
# Update with measured values if you ran imu_utils

%YAML:1.0

accelerometer_noise_density: 0.00207
accelerometer_random_walk: 0.000413
gyroscope_noise_density: 0.000205
gyroscope_random_walk: 0.0000111
rostopic: /d455/imu
update_rate: 200.0
```

### 12.4 Sync Backup Copies

Always keep the backup copies in sync:

```bash
cp ~/ros2_jazzy/src/open_vins/config/rs_d455/kalibr_imucam_chain.yaml \
   ~/ISRO/kalibr_imucam_chain.yaml

cp ~/ros2_jazzy/src/open_vins/config/rs_d455/kalibr_imu_chain.yaml \
   ~/ISRO/kalibr_imu_chain.yaml

echo "Backup synced on $(date)" >> ~/ISRO/calibration_log.txt
```

### 12.5 Rebuild OpenVINS

OpenVINS reads config files at launch, not compile time — no rebuild needed for YAML changes. But if you changed any source code (UpdaterZeroVelocity.cpp):

```bash
cd ~/ros2_jazzy
colcon build --packages-select ov_msckf --symlink-install
source install/setup.bash
```

---

## 13. Step 10 — Verify Calibration Quality

Run all three tests before declaring calibration complete.

### 13.1 Test 1 — Static Drift Test (Must Pass)

```bash
# Launch full VIO stack
# Place drone on flat surface, completely still

# Watch position for 60 seconds
ros2 topic echo /ov_msckf/odomimu --field pose.pose.position

# Log values at t=0s and t=60s
# Calculate displacement: sqrt(Δx² + Δy² + Δz²)
```

| Result | Interpretation |
|--------|---------------|
| Drift < 2cm in 60s | Excellent calibration. Proceed to flight test. |
| Drift 2–5cm in 60s | Acceptable. Monitor during flight. |
| Drift 5–15cm in 60s | Poor. Re-check timeshift value. Possibly re-record IMU-cam bag. |
| Drift > 15cm in 60s | Failed. Full recalibration required. |

### 13.2 Test 2 — Static Velocity Test (Must Pass)

```bash
# Drone completely still on flat surface
# Watch velocity output for 30 seconds

ros2 topic echo /ov_msckf/odomimu --field twist.twist.linear

# Log maximum value seen on any axis
```

| Result | Interpretation |
|--------|---------------|
| All axes < 0.03 m/s | Excellent. Safe to arm after 15s ZUPT. |
| Any axis 0.03–0.08 m/s | Marginal. Wait longer (20s) before arming. |
| Any axis > 0.08 m/s | Failed. Do not arm. Investigate VIO init. |

### 13.3 Test 3 — Return-to-Start Test (Should Pass)

```
Physical test:

1. Place drone on flat surface with textured mat underneath
2. Launch VIO, wait for HEALTHY status
3. Mark the drone position with tape
4. Slowly pick up drone and carry it:
   - 1.0m to the left
   - 1.0m forward
   - 1.0m to the right  
   - 1.0m back to start
   (approximately 1m × 1m square path, total 4m)
5. Place drone back at marked position

Read OpenVINS position estimate.
It should show approximately [0, 0, 0] — back at origin.
```

| Result | Interpretation |
|--------|---------------|
| Error < 5cm | Excellent scale and extrinsics. |
| Error 5–15cm | Acceptable. Stereo baseline might be slightly off. |
| Error > 15cm | Poor. Re-run camera calibration. Check board was flat. |
| Systematic error in one axis | T_cam_imu rotation wrong. Re-record IMU-cam bag with more roll/pitch. |

### 13.4 Test 4 — EKF2 Fusion Verification

```bash
# In QGroundControl MAVLink console
commander status
# Must show: "Vision position valid"
# Must NOT show: "Vision position timeout"

listener estimator_aid_src_ev_vel
# fused: True  → EV velocity fusion is active
# fused: False → PX4 is rejecting VIO velocity — check EKF2_EV_CTRL

listener estimator_innovations
# pos_innov should be small: < 0.1 on all axes when hovering
# Large innovations: VIO and PX4 disagree → calibration or parameter issue
```

---

## 14. Common Errors and Fixes

### 14.1 During Board Preparation

```
Problem: Print scale is wrong (tag measures 82mm instead of 88mm)
Fix: In printer settings, set "Page Scaling" to "None" or "Actual size"
     Some printers default to "Fit to page" which shrinks the image

Problem: Board is warped after printing
Fix: Use forex board (PVC foam board) as backing
     Apply adhesive spray and press paper flat before it dries
     Store board lying flat, not standing vertically
```

### 14.2 During Recording

```
Problem: White blobs visible in infra1/infra2 images
Fix: IR projector is still ON
     Verify launch parameter: depth_module.emitter_enabled:=0
     Sometimes the camera needs to be power-cycled to reset the emitter state
     Unplug and replug USB, then relaunch

Problem: IMU rate is 100Hz instead of 200Hz
Fix: Change gyro_fps and accel_fps to 200 in launch
     Also check: some versions of realsense-ros cap at 100Hz on Pi 5 due to USB bandwidth
     Solution: disable color stream to free bandwidth (enable_color:=false)

Problem: Infra images are dropping frames (rate below 14Hz)
Fix: USB bandwidth issue
     Connect D455 directly to Pi 5 USB 3.0 port (blue port)
     Do not use USB hub during calibration
     Disable any other USB devices during recording
```

### 14.3 During Kalibr Processing

```
Problem: "No corners detected in image sequence"
Fix: 
  1. Check --show-extraction output to see what Kalibr sees
  2. Board may be too small for the flight distance
  3. Lighting may have shadows on the board
  4. Try reducing --bag-from-to start time (camera may need more warmup)

Problem: "Optimisation did not converge"
Fix:
  1. Not enough variety in motion — re-record with more extreme angles
  2. Board was partially occluded in many frames
  3. Try increasing --time-calibration flag tolerance

Problem: Reprojection error > 1.0px
Fix:
  1. Board is warped — check with straight edge
  2. Wrong tagSize in april_6x6.yaml — measure printed tag and update
  3. Wrong distortion model — for D455 use pinhole-radtan not equidist

Problem: timeshift > 0.05s
Fix:
  1. Connect D455 directly to USB 3.0, not through hub
  2. On Pi 5, ensure USB port is not throttled
     sudo lsusb -v to check speed (should show 480Mbit SuperSpeed)
  3. Re-record IMU-cam bag with faster, more dynamic motion
```

### 14.4 After Calibration

```
Problem: Static drift still > 5cm after updating configs
Fix:
  1. Verify you updated the LIVE config, not just the backup
     cat ~/ros2_jazzy/src/open_vins/config/rs_d455/kalibr_imucam_chain.yaml
     Check timeshift value matches your calibration output exactly
  
  2. Disable online extrinsics re-estimation (critical)
     In estimator_config.yaml:
     calib_cam_extrinsics: 0   ← must be 0, not 1
     calib_cam_intrinsics: 0   ← must be 0, not 1
     Online re-estimation causes drift as it wanders from calibrated values

  3. Check init_window_time is at least 10.0
     VIO needs sufficient static time to converge IMU bias before estimates are valid

Problem: Works on checkerboard floor but drifts on grass
Fix: Expected behaviour. Grass is an invalid VIO test surface.
     Moving grass blades create phantom features.
     Use textured mat (yoga mat, newspaper, printed A0 sheet) for all testing.
     Competition arena (Mars-simulated) will have textured floor — no issue there.
```

---

## 15. Quick Checklist

Use this before every calibration session.

### Board Preparation
- [ ] Board printed at exact scale (verify with ruler: tag = 88mm)
- [ ] Board mounted on rigid flat forex/PVC surface
- [ ] Board surface is flat (checked with straight edge — no warping)
- [ ] Board mounted on tripod at 1.2m height

### Environment
- [ ] Good even lighting — no shadows on board
- [ ] No flickering lights (avoid fluorescent tubes)
- [ ] Sufficient space (2m radius around tripod)
- [ ] No strong airflow or vibration sources nearby

### Camera Setup
- [ ] D455 connected directly to Pi 5 USB 3.0 port (not hub)
- [ ] Launch script has `depth_module.emitter_enabled:=0`
- [ ] `ros2 topic hz /d455/infra1/image_rect_raw` shows 14.5–15.5 Hz
- [ ] `ros2 topic hz /d455/imu` shows 195–205 Hz
- [ ] `rqt_image_view` shows infra1 with NO white blobs

### Camera Calibration Bag
- [ ] Duration: 240–300 seconds
- [ ] infra1, infra2, and imu all present in bag
- [ ] Covered all 18 positions in motion sequence
- [ ] Bag converted to ROS1 format successfully

### Camera Calibration Results
- [ ] Reprojection error cam0 < 0.30px
- [ ] Reprojection error cam1 < 0.30px
- [ ] Stereo baseline: 94–96mm
- [ ] PDF report shows clean corner detection

### IMU-Cam Calibration Bag
- [ ] Duration: 180–240 seconds
- [ ] infra1 and imu present in bag
- [ ] All 6 DOF excited (X, Y, Z translation + roll, pitch, yaw rotation)
- [ ] Bag converted to ROS1 format successfully

### IMU-Cam Calibration Results
- [ ] timeshift in range -0.030s to +0.030s
- [ ] T_cam_imu rotation matrix diagonal values all > 0.99
- [ ] Translation Z value is physically plausible (should match camera mount distance)

### Config Update
- [ ] kalibr_imucam_chain.yaml updated in live config
- [ ] kalibr_imu_chain.yaml verified
- [ ] `calib_cam_extrinsics: 0` in estimator_config.yaml
- [ ] Backup copies synced to ~/ISRO/
- [ ] Calibration date logged in ~/ISRO/calibration_log.txt

### Post-Calibration Tests
- [ ] Static drift test: < 2cm in 60 seconds
- [ ] Static velocity test: all axes < 0.03 m/s
- [ ] Return-to-start test: < 5cm error after 4m loop
- [ ] EKF2 fusion: `listener estimator_aid_src_ev_vel` shows `fused: True`

---

## 16. Calibration Quality Reference Table

| Parameter | Excellent | Acceptable | Poor — Redo |
|-----------|-----------|------------|-------------|
| Camera reprojection error | < 0.15px | 0.15 – 0.30px | > 0.50px |
| Stereo baseline (D455) | 94.5 – 95.5mm | 94.0 – 96.0mm | Outside 93–97mm |
| timeshift_cam_imu | -0.010s to +0.010s | ±0.030s | Outside ±0.030s |
| Static drift (60s) | < 2cm | 2 – 5cm | > 15cm |
| Static velocity | < 0.03 m/s | 0.03 – 0.08 m/s | > 0.08 m/s |
| Return-to-start error | < 5cm | 5 – 15cm | > 15cm |
| EKF2 innovation (pos) | < 0.05m | 0.05 – 0.10m | > 0.10m |

---

## File Locations Reference

```
~/calibration/
  april_6x6.yaml                   ← April Tag target config
  imu_params.yaml                  ← IMU noise parameters for Kalibr
  launch_for_calibration.sh        ← RealSense launch (projector OFF)
  bags/
    camera_calib.bag               ← Converted camera calibration bag (ROS1)
    imu_cam_calib.bag              ← Converted IMU-cam calibration bag (ROS1)
  results/
    camera_calib-camchain.yaml     ← Camera calibration output (Kalibr format)
    camera_calib-report.pdf        ← Visual quality report
    imu_cam_calib-results-imu.txt  ← IMU-cam results with T_cam_imu and timeshift
    imu_cam_calib-report.pdf       ← Visual quality report

~/ros2_jazzy/src/open_vins/config/rs_d455/   ← LIVE configs (OpenVINS reads these)
  kalibr_imucam_chain.yaml
  kalibr_imu_chain.yaml
  estimator_config.yaml

~/ISRO/                            ← Backup copies
  kalibr_imucam_chain.yaml
  kalibr_imu_chain.yaml
  calibration_log.txt
```

---

*Team aeroKLE — IRoC-U 2026 — KLE Technological University, Hubballi*
*Guide version: April 2026*

#!/usr/bin/env bash
# =============================================================================
# Kalibr Calibration Runner — Intel RealSense D455
# =============================================================================
# Usage:
#   ./run_calibration.sh cam          — camera-only calibration
#   ./run_calibration.sh imu          — camera+IMU calibration
#   ./run_calibration.sh shell        — open interactive shell in container
#   ./run_calibration.sh build        — build the Docker image only
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA_DIR="$SCRIPT_DIR"
DOCKER_DIR="$SCRIPT_DIR/docker"
IMAGE_NAME="kalibr:noetic"

# ── Paths inside the container (mapped from $DATA_DIR → /data) ──────────────
CAM_BAG="/data/bags/cam_calib.bag"
IMU_BAG="/data/bags/imu_calib.bag"
TARGET_YAML="/data/config/aprilgrid.yaml"
IMU_YAML="/data/config/imu.yaml"
CAM_CHAIN_YAML="/data/calibration_results/cam_calib/cam_calib-camchain.yaml"
CAM_OUTPUT_DIR="/data/calibration_results/cam_calib"
IMU_OUTPUT_DIR="/data/calibration_results/imu_calib"

# ── Topics ───────────────────────────────────────────────────────────────────
CAM0_TOPIC="/d455/infra1/image_rect_raw"
CAM1_TOPIC="/d455/infra2/image_rect_raw"
IMU_TOPIC="/d455/imu"

# ── Helpers ──────────────────────────────────────────────────────────────────
build_image() {
    echo "[kalibr] Building Docker image $IMAGE_NAME ..."
    docker build -t "$IMAGE_NAME" "$DOCKER_DIR"
    echo "[kalibr] Build complete."
}

run_in_docker() {
    xhost +local:docker 2>/dev/null || true
    docker run --rm -it \
        -e DISPLAY="${DISPLAY:-:0}" \
        -e KALIBR_MANUAL_FOCAL_LENGTH_INIT=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "$DATA_DIR:/data:rw" \
        --network host \
        "$IMAGE_NAME" \
        /bin/bash -c "source /catkin_ws/devel/setup.bash && $1"
}

# ── Commands ─────────────────────────────────────────────────────────────────
CMD="${1:-help}"

case "$CMD" in
  build)
    build_image
    ;;

  cam)
    # Ensure image exists
    docker image inspect "$IMAGE_NAME" &>/dev/null || build_image
    mkdir -p "$DATA_DIR/calibration_results/cam_calib"
    echo "[kalibr] Running camera calibration ..."
    run_in_docker "
      kalibr_calibrate_cameras \
        --bag $CAM_BAG \
        --topics $CAM0_TOPIC $CAM1_TOPIC \
        --models pinhole-radtan pinhole-radtan \
        --target $TARGET_YAML \
        --output-folder $CAM_OUTPUT_DIR \
        --dont-show-report
    "
    echo "[kalibr] Camera calibration done. Results in: calibration_results/cam_calib/"
    ;;

  imu)
    docker image inspect "$IMAGE_NAME" &>/dev/null || build_image
    if [[ ! -f "$DATA_DIR/calibration_results/cam_calib/cam_calib-camchain.yaml" ]]; then
        echo "[kalibr] ERROR: Run camera calibration first (./run_calibration.sh cam)"
        exit 1
    fi
    mkdir -p "$DATA_DIR/calibration_results/imu_calib"
    echo "[kalibr] Running camera+IMU calibration ..."
    run_in_docker "
      kalibr_calibrate_imu_camera \
        --bag $IMU_BAG \
        --cam $CAM_CHAIN_YAML \
        --imu $IMU_YAML \
        --target $TARGET_YAML \
        --output-folder $IMU_OUTPUT_DIR \
        --dont-show-report
    "
    echo "[kalibr] IMU calibration done. Results in: calibration_results/imu_calib/"
    ;;

  shell)
    docker image inspect "$IMAGE_NAME" &>/dev/null || build_image
    echo "[kalibr] Opening interactive shell ..."
    run_in_docker "bash"
    ;;

  *)
    echo "Usage: $0 {build|cam|imu|shell}"
    echo ""
    echo "  build  — build the Docker image"
    echo "  cam    — run camera-only calibration"
    echo "  imu    — run camera+IMU calibration (requires cam step first)"
    echo "  shell  — open interactive shell inside the container"
    exit 1
    ;;
esac

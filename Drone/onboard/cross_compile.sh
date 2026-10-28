#!/bin/bash
# NIDAR Autonomous Indoor Drone System
# ARM64 (aarch64) Cross-Compilation Script

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build_arm64"
OUTPUT_DIR="${SCRIPT_DIR}/deploy"

echo "=========================================================="
echo " Starting NIDAR ARM64 Companion Computer Cross-Compile"
echo " Target: Linux aarch64 (Raspberry Pi 4/5, Jetson Orin/Nano)"
echo "=========================================================="

# Check for cross compiler
if ! command -v aarch64-linux-gnu-g++ &> /dev/null; then
    echo "[!] Warning: aarch64-linux-gnu-g++ not found in PATH."
    echo "    On Ubuntu/Debian, install with: sudo apt install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu"
fi

mkdir -p "${BUILD_DIR}"
mkdir -p "${OUTPUT_DIR}"

cd "${BUILD_DIR}"

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_TOOLCHAIN_FILE="${SCRIPT_DIR}/toolchain-aarch64.cmake" \
    -DNIDAR_TARGET_ARM=ON

make -j$(nproc 2>/dev/null || echo 4)

echo "Copying production binaries to deploy..."
cp nidar_onboard "${OUTPUT_DIR}/"

echo "Packaging standalone bundle..."
tar -czf "${OUTPUT_DIR}/nidar_onboard_arm64.tar.gz" -C "${OUTPUT_DIR}" nidar_onboard

echo "=========================================================="
echo " Build Complete: ${OUTPUT_DIR}/nidar_onboard_arm64.tar.gz"
echo " Transfer to drone via: scp ${OUTPUT_DIR}/nidar_onboard_arm64.tar.gz pi@nidar-drone.local:~/"
echo "=========================================================="

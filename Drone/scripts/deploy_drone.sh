#!/bin/bash
# NIDAR AirMouse Autonomous System - Companion Computer Deployment Script

set -e

DRONE_HOST="${1:-nidar-drone.local}"
DRONE_USER="${2:-pi}"
INSTALL_DIR="/home/${DRONE_USER}/nidar_system"

echo "=========================================================="
echo " Deploying NIDAR Production Codebase to Drone"
echo " Target: ${DRONE_USER}@${DRONE_HOST}:${INSTALL_DIR}"
echo "=========================================================="

# Build cross-compiled binary if needed
if [ ! -f "onboard/deploy/nidar_onboard" ]; then
    echo "[*] Building ARM64 binary..."
    cd onboard && ./cross_compile.sh && cd ..
fi

echo "[*] Creating remote directory..."
ssh "${DRONE_USER}@${DRONE_HOST}" "mkdir -p ${INSTALL_DIR}/models ${INSTALL_DIR}/bin"

echo "[*] Uploading binaries and models..."
scp onboard/deploy/nidar_onboard "${DRONE_USER}@${DRONE_HOST}:${INSTALL_DIR}/bin/"
scp models/yolov8n.onnx "${DRONE_USER}@${DRONE_HOST}:${INSTALL_DIR}/models/"

echo "[*] Installing systemd autonomy service..."
ssh "${DRONE_USER}@${DRONE_HOST}" "cat << 'EOF' | sudo tee /etc/systemd/system/nidar-autonomy.service
[Unit]
Description=NIDAR AirMouse Fully Autonomous Indoor Search & Rescue Daemon
After=network.target

[Service]
Type=simple
User=${DRONE_USER}
WorkingDirectory=${INSTALL_DIR}
ExecStart=${INSTALL_DIR}/bin/nidar_onboard --lidar /dev/ttyUSB0 --model ${INSTALL_DIR}/models/yolov8n.onnx --fc /dev/ttyS0
Restart=always
RestartSec=1
LimitRTPRIO=99
LimitMEMLOCK=infinity

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable nidar-autonomy.service
"

echo "=========================================================="
echo " Deployment Complete! System starts automatically on boot."
echo " Start manually with: ssh ${DRONE_USER}@${DRONE_HOST} 'sudo systemctl start nidar-autonomy'"
echo "=========================================================="

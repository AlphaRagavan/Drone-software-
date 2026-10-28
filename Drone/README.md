# NIDAR AirMouse Challenge: Autonomous Indoor Search & Rescue System

A complete, production-ready, 100% offline autonomous drone system engineered for GPS-denied indoor search-and-rescue operations in 15m × 15m mazes with 1-meter wide corridors, 8-foot ceilings, and modular 2x2 meter rooms.

---

## 1. System Architecture Overview

```
                                      +---------------------------------------------+
                                      |         Windows 10/11 GCS (Acer Nitro V15)  |
                                      |                                             |
                                      |  +--------------------+  +---------------+  |
                                      |  | Hardware-Acc Video |  | 2D SLAM Canvas|  |
                                      |  +--------------------+  +---------------+  |
                                      |  | Survivor Tag Log   |  | ABORT BUTTON  |  |
                                      |  +--------------------+  +---------------+  |
                                      +--------^-------------------------|----------+
                                               | mDNS Auto-Discovery     |
                                               | 5GHz Isolated Wi-Fi     | UDP Port 9999
                     RTSP:8554 (Video)         | TCP:9001 (Map/Telem)    | (EMERGENCY ABORT)
               +-------------------------------+                         v
               |
+--------------v--------------------------------------------------------------------+
|  ARM Linux Companion Computer (Raspberry Pi 4/5 / Jetson Orin Nano)               |
|                                                                                   |
|  +------------------+   +-------------------+   +------------------------------+  |
|  | RPLIDAR Driver   |-->| 2D Hector SLAM    |-->| Frontier Explorer            |  |
|  | grabScanDataHq() |   | 5cm Grid Matrix   |   | & Corridor A* Path Planner   |  |
|  +------------------+   +-------------------+   +--------------+---------------+  |
|                                                                |                  |
|  +------------------+   +-------------------+                  v                  |
|  | Camera V4L2      |-->| YOLOv8n ONNX      |       +---------------------+       |
|  | 30 FPS Streamer  |   | Survivor Detector |------>| Kinematics Twist    |       |
|  +------------------+   +-------------------+       | Controller & Center |       |
|                                                     +----------+----------+       |
|                                                                |                  |
|  +-------------------------------------------------------------v---------------+  |
|  | 20 Hz Mission State Machine (TAKEOFF -> EXPLORE -> TAG -> EXIT & LAND)      |  |
|  | Enforces 30-Minute Competition Ceiling (Auto-RTL at 27m, Land at 29.5m)    |  |
|  +-------------------------------------------------------------+---------------+  |
|                                                                |                  |
|                                                                v UART 921600 baud |
|                                                     +---------------------+       |
|                                                     | Flight Controller   |       |
|                                                     | (PX4 / ArduPilot)   |       |
+-----------------------------------------------------+---------------------+-------+
```

---

## 2. Six Chronological Development Steps

### Step 1: Build System & Onboard Networking
- **CMake & Cross-Compilation**: `onboard/CMakeLists.txt`, `toolchain-aarch64.cmake`, and `cross_compile.sh` configured to build optimized ARM64 binaries targeting Linux companion computers.
- **mDNS / Bonjour Broadcaster**: Standalone C++ RFC 6762/6763 broadcaster announcing `nidar-drone.local` on the 5GHz link without external avahi dependencies.
- **Three Concurrent Sockets**:
  1. `rtsp://nidar-drone.local:8554/live` (and HTTP MJPEG on port 8080) for real-time video.
  2. TCP Port `9001`: Framed binary telemetry (`0xAA55`) and RLE-compressed 2D map stream.
  3. UDP Port `9999`: Real-time emergency abort listener running on a high-priority thread.

### Step 2: RPLIDAR Integration & Local SLAM
- **RPLIDAR Driver**: Uses Slamtec SDK `grabScanDataHq()` to capture 360° sweeps into `sl_lidar_response_measurement_node_hq_t`, converting 14-bit angle and 2-bit distance to metric Cartesian scans.
- **2D Hector SLAM**:
  - Resolution: 0.05m (5 cm/cell). Dimension: 360 × 360 cells ($18\text{m} \times 18\text{m}$ area).
  - Continuous Gauss-Newton scan-to-map alignment minimizing $1 - M(S_i(p))$.
  - Real-time Bresenham ray-casting updating log-odds occupancy without wheel odometry or GPS.

### Step 3: Frontier Exploration & Kinematics
- **Frontier Exploration**: Identifies boundary cells between known free space and unknown territory, clusters them via BFS, and ranks candidates using a distance/size/heading cost-utility function.
- **Corridor Pathfinding**: 8-connected A* search with 0.25m obstacle inflation, leaving a 0.50m center path through 1m corridors. Line-of-sight shortcutting prunes waypoints into smooth lines.
- **Kinematics & Centering**: Measures left ($d_L$) and right ($d_R$) wall clearance from LIDAR. Proportional controller $v_y = K_p(d_L - d_R)$ keeps drone centered in 1m corridors with active front obstacle deceleration ($< 0.45\text{m}$) and emergency brake ($< 0.25\text{m}$).

### Step 4: AI Survivor Detection & Coordinate Tagging
- **Offline YOLOv8n**: Fully offline ONNX model loaded via OpenCV DNN. Analyzes 640×640 frames for class 0 (`person`) with confidence threshold $\ge 0.60$ and NMS threshold $0.45$.
- **Sensor Fusion**: Correlates bounding box center azimuth $\phi$ with RPLIDAR range returns to calculate world coordinate:
  $$\begin{pmatrix} X_s \\ Y_s \end{pmatrix} = \begin{pmatrix} X_d \\ Y_d \end{pmatrix} + D \begin{pmatrix} \cos(\theta_d + \phi) \\ \sin(\theta_d + \phi) \end{pmatrix}$$
- **Modular 2x2m Room Standards**: Calculates $RoomX = \lfloor X_s / 2.0 \rfloor, RoomY = \lfloor Y_s / 2.0 \rfloor$, formats JSON payload and broadcasts to GCS over TCP.

### Step 5: Autonomy State Machine
- **Main State Machine**: Executes 20 Hz loop in `onboard/src/main.cpp`.
  - `BOOT_AND_INIT` -> `TAKEOFF_AND_ENTER` -> `EXPLORE_MAZE` -> `TAG_SURVIVOR` -> `FIND_EXIT_AND_LAND`.
- **30-Minute Timer Enforcement**:
  - Tracks `std::chrono::steady_clock` from takeoff.
  - At 27 minutes (1620s): Automatically switches to `FIND_EXIT_AND_LAND` with 3-minute safety margin.
  - At 29.5 minutes (1770s): Triggers immediate precision landing to avoid disqualification.

### Step 6: Windows 10/11 Ground Control Station (GCS)
- **C++ (Qt6/OpenCV)**: Full native desktop application in `gcs/windows_cpp_qt/` with custom `QPainter` canvas, multi-panel HUD, and socket managers.
- **Standalone 1-Click Desktop App**: `gcs/windows_app/gcs_app.py` and `run_gcs.bat` tailored for Windows 10/11 on Acer Nitro V15.
- **Features**:
  - Resolves `nidar-drone.local` via mDNS/Bonjour.
  - Video panel rendering live camera feed.
  - Real-time 2D arena canvas displaying SLAM occupancy grid, 2x2m room grid, and drone trail.
  - Instant survivor marker plotting dropping glowing pulsing beacons on tagged room coordinates.
  - Prominent **EMERGENCY ABORT** button blasting UDP datagram bursts to port 9999.

---

## 3. Directory Layout

```
d:/Drone/
├── onboard/
│   ├── CMakeLists.txt                 # Build system with ARM64 cross-compile support
│   ├── Makefile                       # High-level build shortcuts (native/cross/deploy)
│   ├── toolchain-aarch64.cmake        # CMake toolchain for aarch64-linux-gnu
│   ├── cross_compile.sh               # ARM64 cross-compile automation script
│   ├── include/
│   │   ├── networking/                # mDNS, TCP telemetry, UDP abort, RTSP streamer
│   │   ├── lidar/                     # RPLIDAR C++ driver (grabScanDataHq)
│   │   ├── slam/                      # Occupancy grid and Hector SLAM 2D
│   │   ├── navigation/                # Frontier exploration, A* planner, kinematics
│   │   ├── vision/                    # Camera stream and YOLOv8n survivor detector
│   │   ├── autonomy/                  # 20 Hz state machine and 30-minute timer
│   │   └── hal/                       # Flight controller UART interface (MAVLink/MSP)
│   ├── src/                           # Complete C++ implementations
│   └── third_party/rplidar_sdk/       # Slamtec RPLIDAR C++ SDK headers & driver
├── gcs/
│   ├── windows_cpp_qt/                # Windows 10/11 C++ (Qt6/OpenCV) Desktop GCS
│   └── windows_app/                   # Standalone GCS for Acer Nitro V15
│       ├── gcs_app.py                 # Multi-panel UI application
│       └── run_gcs.bat                # 1-click launcher
├── simulation/
│   └── run_indoor_simulation.py       # 15x15m maze simulation server for GCS testing
├── scripts/
│   ├── deploy_drone.sh                # SSH/SCP deployment and systemd service setup
│   └── setup_models.py                # Offline YOLOv8n ONNX setup script
├── models/
│   └── yolov8n.onnx                   # Offline AI model weights
└── README.md
```

---

## 4. Quick Start Guide

### A. Testing on Windows 10/11 (Acer Nitro V15 Laptop)
1. **Launch the Indoor Simulation Server**:
   ```powershell
   python simulation/run_indoor_simulation.py
   ```
2. **Launch the Ground Control Station**:
   Double-click `gcs/windows_app/run_gcs.bat` or run:
   ```powershell
   python gcs/windows_app/gcs_app.py
   ```
3. The GCS automatically connects, displays the live camera stream, renders the 2D arena map with 2x2m room boxes, traces the drone's corridor trajectory, drops glowing survivor markers on tagged coordinates, and allows testing the **Emergency Abort** button.

### B. Compiling and Deploying to ARM Companion Computer
1. **Cross-Compile for ARM64 on Linux / WSL**:
   ```bash
   cd onboard
   ./cross_compile.sh
   ```
2. **Deploy to Drone over 5GHz Wi-Fi**:
   ```bash
   ../scripts/deploy_drone.sh nidar-drone.local pi
   ```
3. **Run Manually on Companion Computer**:
   ```bash
   ./nidar_onboard --lidar /dev/ttyUSB0 --model models/yolov8n.onnx --fc /dev/ttyS0
   ```

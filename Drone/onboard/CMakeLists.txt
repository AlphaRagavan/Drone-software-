cmake_minimum_required(VERSION 3.16)
project(nidar_onboard_system C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Optimization and warning flags
add_compile_options(-O3 -Wall -Wextra -pthread)

# System definitions
add_definitions(-DNIDAR_INDOOR_CHALLENGE)

# Find OpenCV for camera capture and YOLO inference
find_package(OpenCV QUIET COMPONENTS core imgproc dnn videoio highgui)
if(OpenCV_FOUND)
    message(STATUS "[NIDAR] OpenCV ${OpenCV_VERSION} found.")
    include_directories(${OpenCV_INCLUDE_DIRS})
    add_definitions(-DHAVE_OPENCV)
else()
    message(WARNING "[NIDAR] OpenCV not found in system path. Compiling with built-in vision fallbacks.")
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/rplidar_sdk/include
)

# Source files
set(RPLIDAR_SOURCES
    third_party/rplidar_sdk/rplidar_driver_impl.cpp
)

set(NETWORKING_SOURCES
    src/networking/mdns_broadcaster.cpp
    src/networking/tcp_telemetry_server.cpp
    src/networking/udp_abort_listener.cpp
    src/networking/rtsp_streamer.cpp
)

set(LIDAR_SOURCES
    src/lidar/rplidar_driver.cpp
)

set(SLAM_SOURCES
    src/slam/occupancy_grid.cpp
    src/slam/hector_slam_2d.cpp
)

set(NAVIGATION_SOURCES
    src/navigation/frontier_explorer.cpp
    src/navigation/path_planner.cpp
    src/navigation/kinematics_controller.cpp
)

set(VISION_SOURCES
    src/vision/camera_stream.cpp
    src/vision/survivor_detector.cpp
)

set(AUTONOMY_SOURCES
    src/autonomy/mission_state_machine.cpp
    src/hal/flight_controller_interface.cpp
)

# Main executable target
add_executable(nidar_onboard
    src/main.cpp
    ${RPLIDAR_SOURCES}
    ${NETWORKING_SOURCES}
    ${LIDAR_SOURCES}
    ${SLAM_SOURCES}
    ${NAVIGATION_SOURCES}
    ${VISION_SOURCES}
    ${AUTONOMY_SOURCES}
)

# Link libraries
target_link_libraries(nidar_onboard
    pthread
    $<$<BOOL:${UNIX}>:rt>
    $<$<BOOL:${UNIX}>:dl>
    $<$<BOOL:${OpenCV_FOUND}>:${OpenCV_LIBS}>
)

message(STATUS "Configured NIDAR Onboard Autonomous System Target.")

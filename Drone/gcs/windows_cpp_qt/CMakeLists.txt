cmake_minimum_required(VERSION 3.16)
project(NidarGCS_Windows LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Find Qt6 or Qt5
find_package(Qt6 COMPONENTS Widgets Network Multimedia QUIET)
if(NOT Qt6_FOUND)
    find_package(Qt5 COMPONENTS Widgets Network Multimedia REQUIRED)
    set(QT_LIBRARIES Qt5::Widgets Qt5::Network Qt5::Multimedia)
else()
    set(QT_LIBRARIES Qt6::Widgets Qt6::Network Qt6::Multimedia)
endif()

# Find OpenCV for video decoding
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio highgui)

include_directories(
    src
    ${OpenCV_INCLUDE_DIRS}
    ../../onboard/include
)

set(SOURCES
    src/main.cpp
    src/main_window.cpp
    src/map_canvas.cpp
    src/video_widget.cpp
    src/telemetry_client.cpp
    src/bonjour_resolver.cpp
    src/abort_controller.cpp
)

add_executable(NidarGCS WIN32 ${SOURCES})

target_link_libraries(NidarGCS
    ${QT_LIBRARIES}
    ${OpenCV_LIBS}
    ws2_32
    iphlpapi
)

# Enable high DPI support on Windows 10/11
if(MSVC)
    target_compile_options(NidarGCS PRIVATE /W4 /MP)
else()
    target_compile_options(NidarGCS PRIVATE -Wall -Wextra)
endif()

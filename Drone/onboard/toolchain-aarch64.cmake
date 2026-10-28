# CMake Toolchain file for ARM64 (aarch64-linux-gnu) Cross-Compilation
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# Define cross-compiler prefix
set(TOOLCHAIN_PREFIX aarch64-linux-gnu-)
set(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)

# Target sysroot (if configured)
if(DEFINED ENV{ARM64_SYSROOT})
    set(CMAKE_SYSROOT $ENV{ARM64_SYSROOT})
    set(CMAKE_FIND_ROOT_PATH $ENV{ARM64_SYSROOT})
endif()

# Search mode settings
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# ARM-specific architecture and vector optimization flags
set(CMAKE_C_FLAGS_INIT   "-O3 -march=armv8-a+crc -mtune=cortex-a72 -pthread -fPIC")
set(CMAKE_CXX_FLAGS_INIT "-O3 -march=armv8-a+crc -mtune=cortex-a72 -pthread -fPIC")

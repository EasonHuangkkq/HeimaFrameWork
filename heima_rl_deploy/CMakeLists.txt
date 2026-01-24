cmake_minimum_required(VERSION 3.10)
project(HeimaRLDeploy VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/../heimaSDK_copy
    ${PROJECT_SOURCE_DIR}/../loong_ctrl_locomotion-main/third_party/include
    /usr/local/include
    /usr/include
)

# Find libraries
find_library(ETHERCAT_LIB ethercat
    PATHS /usr/local/lib
    NO_DEFAULT_PATH
    REQUIRED
)

find_package(Threads REQUIRED)

# ONNX Runtime library path - detect architecture
if(${CMAKE_CXX_COMPILER} MATCHES "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    message("=== Detected ARM aarch64 architecture ===")
    if(EXISTS "${PROJECT_SOURCE_DIR}/../loong_ctrl_locomotion-main/third_party/lib_lin_a64")
        link_directories(${PROJECT_SOURCE_DIR}/../loong_ctrl_locomotion-main/third_party/lib_lin_a64)
        link_directories(${PROJECT_SOURCE_DIR}/../loong_ctrl_locomotion-main/third_party/lib_lin_a64/onnxruntime)
        set(ONNX_LIB_DIR "${PROJECT_SOURCE_DIR}/../loong_ctrl_locomotion-main/third_party/lib_lin_a64/onnxruntime")
    else()
        message(FATAL_ERROR "ONNX Runtime library for ARM aarch64 not found")
    endif()
else()
    message("=== Detected x86-64 architecture ===")
if(EXISTS "${PROJECT_SOURCE_DIR}/../loong_ctrl_locomotion-main/third_party/lib_lin_x64")
    link_directories(${PROJECT_SOURCE_DIR}/../loong_ctrl_locomotion-main/third_party/lib_lin_x64)
        link_directories(${PROJECT_SOURCE_DIR}/../loong_ctrl_locomotion-main/third_party/lib_lin_x64/onnxruntime)
        set(ONNX_LIB_DIR "${PROJECT_SOURCE_DIR}/../loong_ctrl_locomotion-main/third_party/lib_lin_x64/onnxruntime")
else()
        message(FATAL_ERROR "ONNX Runtime library for x86-64 not found")
    endif()
endif()

# Build driver SDK library
add_library(driver_sdk SHARED
    ../heimaSDK_copy/common.cpp
    ../heimaSDK_copy/config_xml.cpp
    ../heimaSDK_copy/tinyxml2.cpp
    ../heimaSDK_copy/heima_driver_sdk.cpp
    ../heimaSDK_copy/rs232.cpp
    ../heimaSDK_copy/rs485.cpp
    ../heimaSDK_copy/can.cpp
    ../heimaSDK_copy/ecat.cpp
    ../heimaSDK_copy/yesense_imu.cpp
    ../heimaSDK_copy/yesense/src/yesense_decoder.cpp
    ../heimaSDK_copy/yesense/src/yesense_std_out_decoder.cpp
)

target_link_libraries(driver_sdk PUBLIC
    ${ETHERCAT_LIB}
    Threads::Threads
    m
)

# Enable RS232/IMU support
target_compile_definitions(driver_sdk PRIVATE ENABLE_RS232)

# Build ONNX wrapper
add_library(onnx_wrapper STATIC
    onnx_wrapper.cpp
)

target_link_libraries(onnx_wrapper PUBLIC
    onnxruntime
)

target_include_directories(onnx_wrapper PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/../loong_ctrl_locomotion-main/third_party/include
    ${PROJECT_SOURCE_DIR}/../loong_ctrl_locomotion-main/third_party/include/onnxruntime
)

# Build main executable
add_executable(heima_rl_deploy
    main.cpp
)

target_link_libraries(heima_rl_deploy PUBLIC
    driver_sdk
    onnx_wrapper
    onnxruntime
    Threads::Threads
    m
)

# Enable RS232/IMU support
target_compile_definitions(heima_rl_deploy PRIVATE ENABLE_RS232)

# Set RPATH for runtime library loading
# $ORIGIN means the directory where the executable is located
set_target_properties(heima_rl_deploy PROPERTIES
    BUILD_RPATH "$ORIGIN;${ONNX_LIB_DIR}"
    INSTALL_RPATH "$ORIGIN;${ONNX_LIB_DIR}"
    BUILD_WITH_INSTALL_RPATH FALSE
)

# Install
install(TARGETS heima_rl_deploy driver_sdk
    RUNTIME DESTINATION .
    LIBRARY DESTINATION .
)


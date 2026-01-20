cmake_minimum_required(VERSION 3.10)
project(HeimaRLDeploy VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/../heimaSDK
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

# ONNX Runtime library path
if(EXISTS "${PROJECT_SOURCE_DIR}/../loong_ctrl_locomotion-main/third_party/lib_lin_x64")
    link_directories(${PROJECT_SOURCE_DIR}/../loong_ctrl_locomotion-main/third_party/lib_lin_x64)
    set(ONNX_LIB_DIR "${PROJECT_SOURCE_DIR}/../loong_ctrl_locomotion-main/third_party/lib_lin_x64")
elseif(EXISTS "${PROJECT_SOURCE_DIR}/../loong_ctrl_locomotion-main/third_party/lib_lin_a64")
    link_directories(${PROJECT_SOURCE_DIR}/../loong_ctrl_locomotion-main/third_party/lib_lin_a64)
    set(ONNX_LIB_DIR "${PROJECT_SOURCE_DIR}/../loong_ctrl_locomotion-main/third_party/lib_lin_a64")
else()
    message(FATAL_ERROR "ONNX Runtime library not found")
endif()

# Build driver SDK library
add_library(driver_sdk SHARED
    ../heimaSDK/common.cpp
    ../heimaSDK/config_xml.cpp
    ../heimaSDK/tinyxml2.cpp
    ../heimaSDK/heima_driver_sdk.cpp
    ../heimaSDK/rs232.cpp
    ../heimaSDK/rs485.cpp
    ../heimaSDK/can.cpp
    ../heimaSDK/ecat.cpp
)

target_link_libraries(driver_sdk PUBLIC
    ${ETHERCAT_LIB}
    Threads::Threads
    m
)

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

# Set RPATH for runtime library loading
set_target_properties(heima_rl_deploy PROPERTIES
    INSTALL_RPATH "${ONNX_LIB_DIR}"
    BUILD_WITH_INSTALL_RPATH TRUE
)

# Install
install(TARGETS heima_rl_deploy driver_sdk
    RUNTIME DESTINATION .
    LIBRARY DESTINATION .
)


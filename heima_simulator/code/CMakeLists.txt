cmake_minimum_required(VERSION 3.10)
project(HeimaSimDeploy VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Build option: simulator or real robot mode
option(BUILD_REAL_ROBOT "Build with real robot support (requires DriverSDK)" OFF)

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/../test_example
    ${PROJECT_SOURCE_DIR}/../../loong_ctrl_locomotion-main/third_party/include
    /usr/local/include
    /usr/include
)

# Check for DriverSDK if real robot mode is requested
if(BUILD_REAL_ROBOT)
    message(STATUS "=== Building with REAL ROBOT support ===")
    
    # Check if DriverSDK source exists
    if(NOT EXISTS "${PROJECT_SOURCE_DIR}/../../heimaSDK_copy")
        message(FATAL_ERROR "DriverSDK source not found at ../../heimaSDK_copy/. Cannot build real robot mode.")
    endif()
    
    # Find EtherCAT library (required for real robot)
    find_library(ETHERCAT_LIB ethercat
        PATHS /usr/local/lib /usr/lib
        NO_DEFAULT_PATH
    )
    
    if(NOT ETHERCAT_LIB)
        message(WARNING "EtherCAT library not found. Real robot EtherCAT communication may not work.")
        message(WARNING "To install: sudo apt-get install libethercat-dev")
    else()
        message(STATUS "EtherCAT library found: ${ETHERCAT_LIB}")
    endif()
    
    # Add DriverSDK paths
    include_directories(${PROJECT_SOURCE_DIR}/../../heimaSDK_copy)
    add_definitions(-DBUILD_REAL_ROBOT)
    
    message(STATUS "DriverSDK will be built from source")
else()
    message(STATUS "=== Building with SIMULATOR ONLY support ===")
    message(STATUS "To enable real robot support, use: cmake -DBUILD_REAL_ROBOT=ON ..")
endif()

# Find packages
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)

# Find gRPC and Protobuf
find_package(PkgConfig)
if(PkgConfig_FOUND)
    pkg_check_modules(GRPC grpc++)
    pkg_check_modules(PROTOBUF protobuf)
endif()

# Find protobuf library (required)
find_library(PROTOBUF_LIB protobuf
    PATHS /usr/local/lib /usr/lib
)
if(NOT PROTOBUF_LIB)
    message(FATAL_ERROR "Protobuf library not found. Please install libprotobuf-dev")
endif()

# Find gRPC library
find_library(GRPC_CPP_LIB grpc++
    PATHS /usr/local/lib /usr/lib
)
if(NOT GRPC_CPP_LIB)
    message(FATAL_ERROR "gRPC++ library not found. Please install libgrpc++-dev")
endif()

# Set library variables
if(GRPC_FOUND)
    set(GRPC_LIBRARIES ${GRPC_LIBRARIES})
else()
    set(GRPC_LIBRARIES ${GRPC_CPP_LIB})
endif()

if(PROTOBUF_FOUND)
    set(PROTOBUF_LIBRARIES ${PROTOBUF_LIBRARIES})
else()
    set(PROTOBUF_LIBRARIES ${PROTOBUF_LIB})
endif()

message(STATUS "gRPC libraries: ${GRPC_LIBRARIES}")
message(STATUS "Protobuf libraries: ${PROTOBUF_LIBRARIES}")

# ONNX Runtime library path - detect architecture
if(${CMAKE_CXX_COMPILER} MATCHES "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    message(STATUS "=== Detected ARM aarch64 architecture ===")
    if(EXISTS "${PROJECT_SOURCE_DIR}/../../loong_ctrl_locomotion-main/third_party/lib_lin_a64")
        link_directories(${PROJECT_SOURCE_DIR}/../../loong_ctrl_locomotion-main/third_party/lib_lin_a64)
        link_directories(${PROJECT_SOURCE_DIR}/../../loong_ctrl_locomotion-main/third_party/lib_lin_a64/onnxruntime)
        set(ONNX_LIB_DIR "${PROJECT_SOURCE_DIR}/../../loong_ctrl_locomotion-main/third_party/lib_lin_a64/onnxruntime")
    else()
        message(FATAL_ERROR "ONNX Runtime library for ARM aarch64 not found")
    endif()
else()
    message(STATUS "=== Detected x86-64 architecture ===")
    if(EXISTS "${PROJECT_SOURCE_DIR}/../../loong_ctrl_locomotion-main/third_party/lib_lin_x64")
        link_directories(${PROJECT_SOURCE_DIR}/../../loong_ctrl_locomotion-main/third_party/lib_lin_x64)
        link_directories(${PROJECT_SOURCE_DIR}/../../loong_ctrl_locomotion-main/third_party/lib_lin_x64/onnxruntime)
        set(ONNX_LIB_DIR "${PROJECT_SOURCE_DIR}/../../loong_ctrl_locomotion-main/third_party/lib_lin_x64/onnxruntime")
    else()
        message(FATAL_ERROR "ONNX Runtime library for x86-64 not found")
    endif()
endif()

# Build ONNX wrapper library
add_library(onnx_wrapper STATIC
    onnx_wrapper.cpp
)

target_link_libraries(onnx_wrapper PUBLIC
    onnxruntime
)

target_include_directories(onnx_wrapper PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/../../loong_ctrl_locomotion-main/third_party/include
    ${PROJECT_SOURCE_DIR}/../../loong_ctrl_locomotion-main/third_party/include/onnxruntime
)

# Build ankle solver library
add_library(ankle_solver STATIC
    ankle_solver/cpp/ankle_solver.cpp
)

target_link_libraries(ankle_solver PUBLIC
    Eigen3::Eigen
)

target_include_directories(ankle_solver PUBLIC
    ${PROJECT_SOURCE_DIR}
)

# Build driver SDK library if real robot mode is enabled
if(BUILD_REAL_ROBOT)
    message(STATUS "Building DriverSDK from source...")
    
    add_library(driver_sdk SHARED
        ${PROJECT_SOURCE_DIR}/../../heimaSDK_copy/common.cpp
        ${PROJECT_SOURCE_DIR}/../../heimaSDK_copy/config_xml.cpp
        ${PROJECT_SOURCE_DIR}/../../heimaSDK_copy/tinyxml2.cpp
        ${PROJECT_SOURCE_DIR}/../../heimaSDK_copy/heima_driver_sdk.cpp
        ${PROJECT_SOURCE_DIR}/../../heimaSDK_copy/rs232.cpp
        ${PROJECT_SOURCE_DIR}/../../heimaSDK_copy/rs485.cpp
        ${PROJECT_SOURCE_DIR}/../../heimaSDK_copy/can.cpp
        ${PROJECT_SOURCE_DIR}/../../heimaSDK_copy/ecat.cpp
        ${PROJECT_SOURCE_DIR}/../../heimaSDK_copy/yesense_imu.cpp
        ${PROJECT_SOURCE_DIR}/../../heimaSDK_copy/yesense/src/yesense_decoder.cpp
        ${PROJECT_SOURCE_DIR}/../../heimaSDK_copy/yesense/src/yesense_std_out_decoder.cpp
    )
    
    target_link_libraries(driver_sdk PUBLIC
        Threads::Threads
        m
    )
    
    # Link EtherCAT if available
    if(ETHERCAT_LIB)
        target_link_libraries(driver_sdk PUBLIC ${ETHERCAT_LIB})
    endif()
    
    # Enable RS232/IMU support
    target_compile_definitions(driver_sdk PRIVATE ENABLE_RS232)
    
    message(STATUS "DriverSDK library configured")
endif()

# Build robot interface library
set(ROBOT_INTERFACE_SOURCES
    robot_interface.cpp
    robot_interface_sim.cpp
)

# Add real robot interface only if requested
if(BUILD_REAL_ROBOT)
    list(APPEND ROBOT_INTERFACE_SOURCES robot_interface_real.cpp)
    message(STATUS "Including robot_interface_real.cpp")
endif()

add_library(robot_interface STATIC
    ${ROBOT_INTERFACE_SOURCES}
)

target_link_libraries(robot_interface PUBLIC
    ${GRPC_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
)

if(BUILD_REAL_ROBOT)
    target_link_libraries(robot_interface PUBLIC driver_sdk)
endif()

target_include_directories(robot_interface PUBLIC
    ${PROJECT_SOURCE_DIR}
)

# Build main executable (unified: simulator & real robot)
add_executable(heima_sim_deploy
    main.cpp
    ./proto/mujoco_service.pb.cc
    ./proto/mujoco_service.grpc.pb.cc
)

# Link libraries - ensure protobuf is included
target_link_libraries(heima_sim_deploy PUBLIC
    robot_interface
    onnx_wrapper
    ankle_solver
    onnxruntime
    ${GRPC_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    Threads::Threads
    m
)

# Add protobuf include directories if found
if(PROTOBUF_FOUND)
    target_include_directories(heima_sim_deploy PUBLIC ${PROTOBUF_INCLUDE_DIRS})
endif()

# Set RPATH for runtime library loading
set_target_properties(heima_sim_deploy PROPERTIES
    BUILD_RPATH "$ORIGIN;${ONNX_LIB_DIR}"
    INSTALL_RPATH "$ORIGIN;${ONNX_LIB_DIR}"
    BUILD_WITH_INSTALL_RPATH FALSE
)

# Install targets
if(BUILD_REAL_ROBOT)
    install(TARGETS heima_sim_deploy driver_sdk
        RUNTIME DESTINATION .
        LIBRARY DESTINATION .
    )
else()
    install(TARGETS heima_sim_deploy
        RUNTIME DESTINATION .
    )
endif()

cmake_minimum_required(VERSION 3.10)

# 项目名称
project(SingleMotorTest VERSION 1.0 LANGUAGES CXX)

# ==========================================
# 1. 编译器设置
# ==========================================
# 强制使用 C++20 (因为代码里用了 <bit> 等特性)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")  # 匹配你的手动编译命令

# ==========================================
# 2. 查找依赖库
# ==========================================
# 包含头文件路径
include_directories(
    ${PROJECT_SOURCE_DIR}
    /usr/local/include     # <--- 优先搜索这里 (对应 /usr/local/lib)
    /usr/include
)

# 【关键修改】强制只在 /usr/local/lib 查找 ethercat
# NO_DEFAULT_PATH 会禁止 cmake 去系统默认路径和其他路径找
find_library(ETHERCAT_LIB ethercat
    PATHS /usr/local/lib
    NO_DEFAULT_PATH
    REQUIRED
)

find_package(Threads REQUIRED) # 查找 pthread

# 打印一下找到的路径，确保是你想要的 /usr/local/lib/libethercat.so
message(STATUS "Found EtherCAT Lib: ${ETHERCAT_LIB}")

# ==========================================
# 3. 生成 SDK 动态库 (libdriver_sdk.so)
# ==========================================
# 将 tinyxml2.cpp 直接加入编译列表
add_library(driver_sdk SHARED
    common.cpp
    config_xml.cpp
    tinyxml2.cpp       # <--- 源码直接编译，不需要找系统库
)

# 链接依赖库
target_link_libraries(driver_sdk PUBLIC
    ${ETHERCAT_LIB}    # 链接刚才强制指定的 ethercat 库
    Threads::Threads   # pthread
    m                  # math
)

# ==========================================
# 4. 生成可执行文件
# ==========================================
if(EXISTS "${PROJECT_SOURCE_DIR}/main.cpp")
    add_executable(run_motor main.cpp)
    target_link_libraries(run_motor PUBLIC driver_sdk)
endif()

# Leg smoke test executable
add_executable(leg_smoke_test
    leg_smoke_test.cpp
    heima_driver_sdk.cpp
    ecat.cpp
    rs485.cpp
    can.cpp
)
target_link_libraries(leg_smoke_test PUBLIC
    driver_sdk
    ${ETHERCAT_LIB}
    Threads::Threads
    m
)

# Arm smoke test executable
add_executable(arm_smoke_test
    arm_smoke_test.cpp
    heima_driver_sdk.cpp
    ecat.cpp
    rs485.cpp
    can.cpp
)
target_link_libraries(arm_smoke_test PUBLIC
    driver_sdk
    ${ETHERCAT_LIB}
    Threads::Threads
    m
)

# Whole body smoke test executable (33 motors with optional IMU)
add_executable(whole_body_smoke_test
    whole_body_smoke_test.cpp
    heima_driver_sdk.cpp
    ecat.cpp
    rs485.cpp
    can.cpp
    rs232.cpp
    yesense_imu.cpp
    yesense/src/yesense_decoder.cpp
    yesense/src/yesense_std_out_decoder.cpp
)
target_compile_definitions(whole_body_smoke_test PRIVATE ENABLE_RS232)
target_link_libraries(whole_body_smoke_test PUBLIC
    driver_sdk
    ${ETHERCAT_LIB}
    Threads::Threads
    m
)

# Whole body PVT test executable
add_executable(whole_body_pvt_test
    whole_body_pvt_test.cpp
    heima_driver_sdk.cpp
    ecat.cpp
    rs485.cpp
    can.cpp
    rs232.cpp
    yesense_imu.cpp
    yesense/src/yesense_decoder.cpp
    yesense/src/yesense_std_out_decoder.cpp
)
target_compile_definitions(whole_body_pvt_test PRIVATE ENABLE_RS232)
target_link_libraries(whole_body_pvt_test PUBLIC
    driver_sdk
    ${ETHERCAT_LIB}
    Threads::Threads
    m
)

# Ankle Hardware PVT test executable
add_executable(test_ankle_hardware
    test_ankle_hardware.cpp
    ../heima_simulator/code/ankle_solver/cpp/ankle_solver.cpp
    heima_driver_sdk.cpp
    ecat.cpp
    rs485.cpp
    can.cpp
    rs232.cpp
    yesense_imu.cpp
    yesense/src/yesense_decoder.cpp
    yesense/src/yesense_std_out_decoder.cpp
)
target_include_directories(test_ankle_hardware PRIVATE
    ../heima_simulator/code/ankle_solver/cpp
    ../loong_utility-main/third_party/include/eigen3
)
target_compile_definitions(test_ankle_hardware PRIVATE ENABLE_RS232)
target_link_libraries(test_ankle_hardware PUBLIC
    driver_sdk
    ${ETHERCAT_LIB}
    Threads::Threads
    m
)
# Python/ctypes-friendly shared library (C ABI)
add_library(heima_sdk_c SHARED
    heima_sdk_c_api.cpp
    heima_driver_sdk.cpp
    ecat.cpp
    rs485.cpp
    can.cpp
    rs232.cpp
    yesense_imu.cpp
    yesense/src/yesense_decoder.cpp
    yesense/src/yesense_std_out_decoder.cpp
    common.cpp
    config_xml.cpp
    tinyxml2.cpp
)
target_compile_definitions(heima_sdk_c PRIVATE ENABLE_RS232)
target_link_libraries(heima_sdk_c PUBLIC
    ${ETHERCAT_LIB}
    Threads::Threads
    m
)

# loong_base-main compatible SDK (exports namespace DriverSDK)
add_library(loong_driver_sdk SHARED
    heima_driver_sdk.cpp
    ecat.cpp
    rs485.cpp
    can.cpp
    rs232.cpp
    yesense_imu.cpp
    yesense/src/yesense_decoder.cpp
    yesense/src/yesense_std_out_decoder.cpp
    common.cpp
    config_xml.cpp
    tinyxml2.cpp
)
target_compile_definitions(loong_driver_sdk PRIVATE ENABLE_RS232)
target_link_libraries(loong_driver_sdk PUBLIC
    ${ETHERCAT_LIB}
    Threads::Threads
    m
)

# ==========================================
# 5. 安装配置 (保持与原脚本结构一致)
# ==========================================
set(INSTALL_TARGETS
    driver_sdk
    leg_smoke_test
    arm_smoke_test
    whole_body_smoke_test
    heima_sdk_c
    loong_driver_sdk  # loong_base-main 需要的 SDK 库
)
if(TARGET run_motor)
    list(APPEND INSTALL_TARGETS run_motor)
endif()

install(TARGETS ${INSTALL_TARGETS}
    RUNTIME DESTINATION .
    LIBRARY DESTINATION .
)

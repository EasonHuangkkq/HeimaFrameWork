cmake_minimum_required(VERSION 3.10)

# 项目名称
project(HeimaWebControl VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# ==========================================
# 查找依赖库
# ==========================================
set(HEIMA_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)

include_directories(
    ${HEIMA_SDK_DIR}
    /usr/local/include
    /usr/include
)

find_library(ETHERCAT_LIB ethercat
    PATHS /usr/local/lib
    NO_DEFAULT_PATH
    REQUIRED
)

find_package(Threads REQUIRED)

# ==========================================
# 生成 C++ 驱动
# ==========================================
add_executable(whole_body_web_control
    whole_body_web_control.cpp
    ${HEIMA_SDK_DIR}/heima_driver_sdk.cpp
    ${HEIMA_SDK_DIR}/ecat.cpp
    ${HEIMA_SDK_DIR}/rs485.cpp
    ${HEIMA_SDK_DIR}/can.cpp
    ${HEIMA_SDK_DIR}/rs232.cpp
    ${HEIMA_SDK_DIR}/yesense_imu.cpp
    ${HEIMA_SDK_DIR}/yesense/src/yesense_decoder.cpp
    ${HEIMA_SDK_DIR}/yesense/src/yesense_std_out_decoder.cpp
    ${HEIMA_SDK_DIR}/common.cpp
    ${HEIMA_SDK_DIR}/config_xml.cpp
    ${HEIMA_SDK_DIR}/tinyxml2.cpp
)

target_compile_definitions(whole_body_web_control PRIVATE ENABLE_RS232)

target_link_libraries(whole_body_web_control PUBLIC
    ${ETHERCAT_LIB}
    Threads::Threads
    m
)

# ==========================================
# 安装目标到当前文件夹
# ==========================================
install(TARGETS whole_body_web_control
    RUNTIME DESTINATION .
)

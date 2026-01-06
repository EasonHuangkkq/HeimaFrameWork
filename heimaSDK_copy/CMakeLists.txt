cmake_minimum_required(VERSION 3.10)

# 项目名称
project(SingleMotorTest VERSION 1.0 LANGUAGES CXX)

# ==========================================
# 1. 编译器设置
# ==========================================
# 强制使用 C++20 (因为代码里用了 <bit> 等特性)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

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
# 4. 生成可执行文件 (run_motor)
# ==========================================
add_executable(run_motor main.cpp)

# 链接上面生成的 SDK
target_link_libraries(run_motor PUBLIC driver_sdk)

# ==========================================
# 5. 安装配置 (保持与原脚本结构一致)
# ==========================================
install(TARGETS driver_sdk run_motor
    RUNTIME DESTINATION .
    LIBRARY DESTINATION .
)
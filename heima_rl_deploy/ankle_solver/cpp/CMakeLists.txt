cmake_minimum_required(VERSION 3.10)
project(AnkleSolver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(${EIGEN3_INCLUDE_DIR})

# Source files
set(SOURCES
    ankle_solver.cpp
    main.cpp
)

# Create executable
add_executable(ankle_solver ${SOURCES})

# Link libraries
target_link_libraries(ankle_solver Eigen3::Eigen)

# Set output directory
set_target_properties(ankle_solver PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)


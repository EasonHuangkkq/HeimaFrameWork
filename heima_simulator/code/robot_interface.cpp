/*
 * Robot Interface - Implementation
 */

#include "robot_interface.h"
#include "robot_interface_sim.h"

#ifdef BUILD_REAL_ROBOT
#include "robot_interface_real.h"
#endif

#include <iostream>

std::unique_ptr<RobotInterface> createRobotInterface(
    const std::string& mode,
    const std::string& config)
{
    if (mode == "sim" || mode == "simulator") {
        std::cout << "Creating simulator interface (gRPC)" << std::endl;
        std::string grpc_server = config.empty() ? "localhost:50051" : config;
        return std::make_unique<SimulatorInterface>(grpc_server);
    }
    else if (mode == "real" || mode == "robot") {
#ifdef BUILD_REAL_ROBOT
        std::cout << "Creating real robot interface (SDK)" << std::endl;
        std::string config_file = config.empty() ? "config.xml" : config;
        return std::make_unique<RealRobotInterface>(config_file);
#else
        std::cerr << "Error: Real robot mode not available!" << std::endl;
        std::cerr << "This binary was built with simulator-only support." << std::endl;
        std::cerr << "To enable real robot mode, rebuild with: cmake -DBUILD_REAL_ROBOT=ON .." << std::endl;
        return nullptr;
#endif
    }
    else {
        std::cerr << "Unknown mode: " << mode << ". Use 'sim' or 'real'" << std::endl;
        return nullptr;
    }
}


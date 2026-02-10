/*
 * Simulator Robot Interface - Implementation
 */

#include "robot_interface_sim.h"
#include <iostream>

bool SimulatorInterface::init() {
    std::cout << "Connecting to gRPC server: " << grpc_server_ << std::endl;
    
    auto channel = grpc::CreateChannel(grpc_server_, grpc::InsecureChannelCredentials());
    stub_ = MuJoCoService::NewStub(channel);
    
    // Test connection
    std::vector<float> test_rpy, test_ang_vel, test_joint_pos, test_joint_vel;
    if (!fetchObs(test_rpy, test_ang_vel, test_joint_pos, test_joint_vel)) {
        std::cerr << "Failed to connect to gRPC server" << std::endl;
        return false;
    }
    
    std::cout << "Connected to simulator successfully!" << std::endl;
    return true;
}

bool SimulatorInterface::fetchObs(
    std::vector<float>& rpy,
    std::vector<float>& base_ang_vel,
    std::vector<float>& joint_pos,
    std::vector<float>& joint_vel)
{
    FetchObsRequest request;
    FetchObsResponse response;
    ClientContext context;
    
    Status status = stub_->FetchObs(&context, request, &response);
    
    if (status.ok()) {
        rpy.assign(response.rpy().begin(), response.rpy().end());
        base_ang_vel.assign(response.base_ang_vel().begin(), response.base_ang_vel().end());
        joint_pos.assign(response.joint_pos().begin(), response.joint_pos().end());
        joint_vel.assign(response.joint_vel().begin(), response.joint_vel().end());
        return true;
    } else {
        std::cerr << "FetchObs failed: " << status.error_message() << std::endl;
        return false;
    }
}

bool SimulatorInterface::writeTorque(const std::vector<float>& torques) {
    WriteTorqueRequest request;
    WriteTorqueResponse response;
    ClientContext context;
    
    for (float t : torques) {
        request.add_torques(t);
    }
    
    Status status = stub_->WriteTorque(&context, request, &response);
    
    if (status.ok() && response.success()) {
        return true;
    } else {
        std::cerr << "WriteTorque failed: " << status.error_message() << std::endl;
        return false;
    }
}

void SimulatorInterface::shutdown() {
    std::cout << "Shutting down simulator interface..." << std::endl;
    // Send zero torques
    std::vector<float> zero_torques(12, 0.0f);
    writeTorque(zero_torques);
}


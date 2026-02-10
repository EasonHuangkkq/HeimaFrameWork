/*
 * Minimal C++ gRPC client example
 * Calls _fetch_obs() and _write_torque() from Python MuJoCo server
 */

#include <iostream>
#include <vector>
#include <grpcpp/grpcpp.h>
#include "mujoco_service.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using mujoco::MuJoCoService;
using mujoco::FetchObsRequest;
using mujoco::FetchObsResponse;
using mujoco::WriteTorqueRequest;
using mujoco::WriteTorqueResponse;

class MuJoCoClient {
public:
    MuJoCoClient(std::shared_ptr<Channel> channel)
        : stub_(MuJoCoService::NewStub(channel)) {}

    // Fetch observation (calls _fetch_obs from Python)
    bool FetchObs(std::vector<float>& base_ang_vel,
                   std::vector<float>& joint_pos,
                   std::vector<float>& joint_vel) {
        FetchObsRequest request;
        FetchObsResponse response;
        ClientContext context;

        Status status = stub_->FetchObs(&context, request, &response);

        if (status.ok()) {
            base_ang_vel.assign(response.base_ang_vel().begin(), 
                               response.base_ang_vel().end());
            joint_pos.assign(response.joint_pos().begin(), 
                            response.joint_pos().end());
            joint_vel.assign(response.joint_vel().begin(), 
                            response.joint_vel().end());
            return true;
        } else {
            std::cerr << "FetchObs failed: " << status.error_message() << std::endl;
            return false;
        }
    }

    // Write torque (calls _write_torque from Python)
    bool WriteTorque(const std::vector<float>& torques) {
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

private:
    std::unique_ptr<MuJoCoService::Stub> stub_;
};

int main() {
    // Connect to Python gRPC server
    std::string server_address = "localhost:50051";
    auto channel = grpc::CreateChannel(server_address, 
                                      grpc::InsecureChannelCredentials());
    MuJoCoClient client(channel);

    std::cout << "=== Minimal C++ gRPC Client ===" << std::endl;
    std::cout << "Connected to server at " << server_address << std::endl;

    // Example: Write some torques
    std::vector<float> torques = {100.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f,
                                  70.0f, 80.0f, 90.0f, 100.0f, 110.0f, 120.0f};
    
    std::cout << "\n1. Writing torques..." << std::endl;

    if (client.WriteTorque(torques)) {
        std::cout << "   SUCCESS" << std::endl;
    } else {
        std::cout << "   FAILED" << std::endl;
        return 1;
    }

    // Example: Fetch observation
    std::vector<float> base_ang_vel, joint_pos, joint_vel;
    
    std::cout << "\n2. Fetching observation..." << std::endl;
    if (client.FetchObs(base_ang_vel, joint_pos, joint_vel)) {
        std::cout << "   SUCCESS" << std::endl;
        std::cout << "   base_ang_vel: [" 
                  << base_ang_vel[0] << ", " 
                  << base_ang_vel[1] << ", " 
                  << base_ang_vel[2] << "]" << std::endl;
        std::cout << "   joint_pos size: " 
        << joint_pos[0] << ", " 
        << joint_pos[1] << ", " 
        << joint_pos[2] << ", " 
        << joint_pos[3] << ", " 
        << joint_pos[4] << ", " 
        << joint_pos[5] << ", " 
        << joint_pos[6] << ", " 
        << joint_pos[7] << ", " 
        << joint_pos[8] << ", " 
        << joint_pos[9] << ", " 
        << joint_pos[10] << ", " 
        << joint_pos[11] << ", " 
        << joint_pos[12] << std::endl;
        std::cout << "   joint_vel: [" 
                  << joint_vel[0] << ", " 
                  << joint_vel[1] << ", " 
                  << joint_vel[2] << ", " 
                  << joint_vel[3] << ", " 
                  << joint_vel[4] << ", " 
                  << joint_vel[5] << ", " 
                  << joint_vel[6] << ", " 
                  << joint_vel[7] << ", " 
                  << joint_vel[8] << ", " 
                  << joint_vel[9] << ", " 
                  << joint_vel[10] << ", " 
                  << joint_vel[11] << std::endl;
    } else {
        std::cout << "   FAILED" << std::endl;
        return 1;
    }

    std::cout << "\n=== Done ===" << std::endl;
    return 0;
}


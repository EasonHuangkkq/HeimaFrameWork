/*
 * Simulator Robot Interface - gRPC-based MuJoCo simulator interface
 */

#ifndef ROBOT_INTERFACE_SIM_H
#define ROBOT_INTERFACE_SIM_H

#include "robot_interface.h"
#include <grpcpp/grpcpp.h>
#include "proto/mujoco_service.grpc.pb.h"
#include <memory>

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using mujoco::MuJoCoService;
using mujoco::FetchObsRequest;
using mujoco::FetchObsResponse;
using mujoco::WriteTorqueRequest;
using mujoco::WriteTorqueResponse;

class SimulatorInterface : public RobotInterface {
public:
    SimulatorInterface(const std::string& grpc_server = "localhost:50051")
        : grpc_server_(grpc_server) {}
    
    bool init() override;
    
    bool fetchObs(
        std::vector<float>& rpy,
        std::vector<float>& base_ang_vel,
        std::vector<float>& joint_pos,
        std::vector<float>& joint_vel) override;
    
    bool writeTorque(const std::vector<float>& torques) override;
    
    void shutdown() override;
    
    int getMotorCount() const override { return 12; }
    
private:
    std::string grpc_server_;
    std::unique_ptr<MuJoCoService::Stub> stub_;
};

#endif // ROBOT_INTERFACE_SIM_H


/*
 * Real Robot Interface - SDK-based real robot interface
 */

#ifndef ROBOT_INTERFACE_REAL_H
#define ROBOT_INTERFACE_REAL_H

#include "robot_interface.h"
#include "heima_driver_sdk.h"
#include <vector>

class RealRobotInterface : public RobotInterface {
public:
    RealRobotInterface(const std::string& config_file = "config.xml")
        : config_file_(config_file) {}
    
    bool init() override;
    
    bool fetchObs(
        std::vector<float>& rpy,
        std::vector<float>& base_ang_vel,
        std::vector<float>& joint_pos,
        std::vector<float>& joint_vel) override;
    
    bool writeTorque(const std::vector<float>& torques) override;
    
    void shutdown() override;
    
    int getMotorCount() const override;
    
private:
    std::string config_file_;
    DriverSDK::DriverSDK* sdk_ = nullptr;
    int motor_count_ = 0;
    std::vector<DriverSDK::motorActualStruct> motor_states_;
    std::vector<DriverSDK::motorTargetStruct> motor_commands_;
    DriverSDK::imuStruct imu_data_;
};

#endif // ROBOT_INTERFACE_REAL_H


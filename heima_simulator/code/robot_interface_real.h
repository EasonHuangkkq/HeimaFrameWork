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
    // opMode: 5=PVT, 8=CSP, 10=CST
    RealRobotInterface(const std::string& config_file = "config.xml",
                       const std::vector<unsigned short>& maxCurrent = {},
                       int ecatCpu = -1,
                       int opMode = 10)
        : config_file_(config_file), max_current_(maxCurrent), ecat_cpu_(ecatCpu), opMode_(opMode) {}
    
    bool init() override;
    
    bool fetchObs(
        std::vector<float>& rpy,
        std::vector<float>& base_ang_vel,
        std::vector<float>& joint_pos,
        std::vector<float>& joint_vel) override;
    
    bool writeTorque(const std::vector<float>& torques) override;
    
    bool writePosition(const std::vector<float>& positions) override;
    
    bool writePVT(const std::vector<float>& pos,
                  const std::vector<float>& vel,
                  const std::vector<float>& tor,
                  const std::vector<float>& kp,
                  const std::vector<float>& kd) override;
    
    void shutdown() override;
    
    int getMotorCount() const override;
    
private:
    std::string config_file_;
    std::vector<unsigned short> max_current_;
    int ecat_cpu_ = -1;
    int opMode_ = 10;  // 5=PVT, 8=CSP, 10=CST
    DriverSDK::DriverSDK* sdk_ = nullptr;
    int motor_count_ = 0;
    std::vector<DriverSDK::motorActualStruct> motor_states_;
    std::vector<DriverSDK::motorTargetStruct> motor_commands_;
    DriverSDK::imuStruct imu_data_;
    float initial_yaw_ = 0.0f;
    bool initial_yaw_set_ = false;
};

#endif // ROBOT_INTERFACE_REAL_H


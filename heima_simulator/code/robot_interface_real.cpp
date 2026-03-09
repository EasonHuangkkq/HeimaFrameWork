/*
 * Real Robot Interface - Implementation
 */

#include "robot_interface_real.h"
#include <iostream>
#include <unistd.h>
#include <algorithm>

bool RealRobotInterface::init() {
    std::cout << "Initializing real robot SDK..." << std::endl;
    std::cout << "Config file: " << config_file_ << std::endl;
    
    sdk_ = &DriverSDK::DriverSDK::instance();
    
    // Set EtherCAT thread CPU affinity BEFORE init()
    if(ecat_cpu_ >= 0){
        sdk_->setCPU(static_cast<unsigned short>(ecat_cpu_));
        std::cout << "Set EtherCAT thread to CPU " << ecat_cpu_ << std::endl;
    }
    
    // Set per-motor maxCurrent BEFORE init() — SDK defaults all to 1000 if not set
    if(!max_current_.empty()){
        sdk_->setMaxCurr(max_current_);
        std::cout << "Set per-motor maxCurrent (" << max_current_.size() << " motors)" << std::endl;
    }
    
    sdk_->init(config_file_.c_str());
    
    motor_count_ = sdk_->getTotalMotorNr();
    std::cout << "Total motors: " << motor_count_ << std::endl;
    
    if (motor_count_ < 12) {
        std::cerr << "Error: Need at least 12 motors for leg control" << std::endl;
        return false;
    }
    
    motor_states_.resize(motor_count_);
    motor_commands_.resize(motor_count_);
    
    // Set operating mode: 5=PVT, 8=CSP, 10=CST
    std::vector<char> modes(motor_count_, static_cast<char>(opMode_));
    sdk_->setMode(modes);
    const char* modeName = (opMode_ == 5) ? "PVT" : (opMode_ == 8) ? "CSP" : "CST";
    std::cout << "Operating mode: " << modeName << " (" << opMode_ << ")" << std::endl;
    
    // Enable all motors (PVT: use kp=0,kd=0 during OP wait for safe approach)
    for(int i = 0; i < motor_count_; ++i){
        motor_commands_[i].enabled = 1;
        motor_commands_[i].pos = 0.0f;
        motor_commands_[i].vel = 0.0f;
        motor_commands_[i].tor = 0.0f;
        motor_commands_[i].kp = 0.0f;
        motor_commands_[i].kd = 0.0f;
    }
    
    // Wait for motors to reach operational state
    std::cout << "Waiting for motors to be operational..." << std::endl;
    const int max_wait_ticks = 10000;
    int wait_ticks = 0;
    bool all_operational = false;
    
    while(!all_operational && wait_ticks < max_wait_ticks){
        // Read actual states first
        sdk_->getMotorActual(motor_states_);
        
        // Set target position to actual position (match fillHoldTargets pattern)
        for(int i = 0; i < motor_count_; ++i){
            motor_commands_[i].pos = motor_states_[i].pos;
            motor_commands_[i].vel = 0.0f;
            motor_commands_[i].tor = 0.0f;
            motor_commands_[i].kp = 0.0f;
            motor_commands_[i].kd = 0.0f;
            motor_commands_[i].enabled = 1;
        }
        sdk_->setMotorTarget(motor_commands_);
        
        all_operational = true;
        for(int i = 0; i < 15 && i < motor_count_; ++i){
            if((motor_states_[i].statusWord & 0x007f) != 0x0037){
                all_operational = false;
                break;
            }
        }
        
        if(wait_ticks % 500 == 0){
            std::cout << "Waiting... (" << wait_ticks << "/" << max_wait_ticks << ")" << std::endl;
        }
        
        wait_ticks++;
        usleep(1000);  // 1ms
    }
    
    if(!all_operational){
        std::cerr << "Warning: Not all motors reached operational state" << std::endl;
    }else{
        std::cout << "All motors operational!" << std::endl;
    }
    
    return true;
}

bool RealRobotInterface::fetchObs(
    std::vector<float>& rpy,
    std::vector<float>& base_ang_vel,
    std::vector<float>& joint_pos,
    std::vector<float>& joint_vel)
{
    // Get motor states and IMU data
    sdk_->getMotorActual(motor_states_);
    sdk_->getIMU(imu_data_);
    
    // Fill RPY
    rpy.resize(3);
    rpy[0] = imu_data_.rpy[0];  // Roll
    rpy[1] = imu_data_.rpy[1];  // Pitch
    rpy[2] = imu_data_.rpy[2];  // Yaw
    
    // Set initial yaw offset on first reading
    if (!initial_yaw_set_) {
        initial_yaw_ = rpy[2];
        initial_yaw_set_ = true;
        std::cout << "Initial yaw set to: " << initial_yaw_ << " rad (" 
                  << (initial_yaw_ * 180.0f / 3.14159265359f) << " deg)" << std::endl;
    }
    
    // Adjust yaw relative to initial
    rpy[2] -= initial_yaw_;
    
    // Fill base angular velocity
    base_ang_vel.resize(3);
    base_ang_vel[0] = imu_data_.gyr[0];
    base_ang_vel[1] = imu_data_.gyr[1];
    base_ang_vel[2] = imu_data_.gyr[2];
    
    // Fill joint positions and velocities (12 legs + 3 waist = 15 motors)
    int num_joints = std::min(15, motor_count_);
    joint_pos.resize(num_joints);
    joint_vel.resize(num_joints);
    for (int i = 0; i < num_joints; ++i) {
        joint_pos[i] = motor_states_[i].pos;
        joint_vel[i] = motor_states_[i].vel;
    }
    
    // If we have fewer than 15 motors, pad with zeros for waist motors
    if (num_joints < 15) {
        joint_pos.resize(15, 0.0f);
        joint_vel.resize(15, 0.0f);
    }
    
    return true;
}

bool RealRobotInterface::writeTorque(const std::vector<float>& torques) {
    if (torques.size() < 12) {
        std::cerr << "Error: Expected at least 12 torques, got " << torques.size() << std::endl;
        return false;
    }
    
    // Set torque commands for motors (legs + waist)
    int num_motors = std::min(static_cast<int>(torques.size()), motor_count_);
    for (int i = 0; i < num_motors; ++i) {
        motor_commands_[i].tor = torques[i];
        motor_commands_[i].pos = 0.0f;  // Not used in torque mode
        motor_commands_[i].vel = 0.0f;  // Not used in torque mode
        motor_commands_[i].enabled = 1;
    }
    
    // Send commands
    sdk_->setMotorTarget(motor_commands_);
    return true;
}

bool RealRobotInterface::writePVT(const std::vector<float>& pos,
                                  const std::vector<float>& vel,
                                  const std::vector<float>& tor,
                                  const std::vector<float>& kp,
                                  const std::vector<float>& kd){
    int n = std::min({static_cast<int>(pos.size()),
                     static_cast<int>(vel.size()),
                     static_cast<int>(tor.size()),
                     static_cast<int>(kp.size()),
                     static_cast<int>(kd.size()),
                     motor_count_});
    if(n < 12){
        std::cerr << "Error: Expected at least 12 PVT targets, got " << n << std::endl;
        return false;
    }
    for(int i = 0; i < n; ++i){
        motor_commands_[i].pos = pos[i];
        motor_commands_[i].vel = vel[i];
        motor_commands_[i].tor = tor[i];
        motor_commands_[i].kp  = kp[i];
        motor_commands_[i].kd  = kd[i];
        motor_commands_[i].enabled = 1;
    }
    sdk_->setMotorTarget(motor_commands_);
    return true;
}

bool RealRobotInterface::writePosition(const std::vector<float>& positions){
    if(positions.size() < 12){
        std::cerr << "Error: Expected at least 12 positions, got " << positions.size() << std::endl;
        return false;
    }
    
    int num_motors = std::min(static_cast<int>(positions.size()), motor_count_);
    for(int i=0; i<num_motors; ++i){
        motor_commands_[i].pos = positions[i];
        motor_commands_[i].vel = 0.0f;
        motor_commands_[i].tor = 0.0f;
        motor_commands_[i].enabled = 1;
    }
    
    sdk_->setMotorTarget(motor_commands_);
    return true;
}

void RealRobotInterface::shutdown(){
    std::cout << "Shutting down real robot interface..." << std::endl;
    
    // Read current positions before disabling
    sdk_->getMotorActual(motor_states_);
    
    // Disable all motors (zero kp/kd/tor for PVT safety, hold current position)
    for(int i = 0; i < motor_count_; ++i){
        motor_commands_[i].enabled = 0;
        motor_commands_[i].pos = motor_states_[i].pos;  // Hold current position
        motor_commands_[i].vel = 0.0f;
        motor_commands_[i].tor = 0.0f;
        motor_commands_[i].kp = 0.0f;
        motor_commands_[i].kd = 0.0f;
    }
    
    for(int i = 0; i < 100; ++i){
        sdk_->setMotorTarget(motor_commands_);
        usleep(1000);
    }
}

int RealRobotInterface::getMotorCount() const {
    return motor_count_;
}


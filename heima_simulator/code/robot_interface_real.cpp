/*
 * Real Robot Interface - Implementation
 */

#include "robot_interface_real.h"
#include <iostream>
#include <unistd.h>

bool RealRobotInterface::init() {
    std::cout << "Initializing real robot SDK..." << std::endl;
    std::cout << "Config file: " << config_file_ << std::endl;
    
    sdk_ = &DriverSDK::DriverSDK::instance();
    sdk_->init(config_file_.c_str());
    
    motor_count_ = sdk_->getTotalMotorNr();
    std::cout << "Total motors: " << motor_count_ << std::endl;
    
    if (motor_count_ < 12) {
        std::cerr << "Error: Need at least 12 motors for leg control" << std::endl;
        return false;
    }
    
    motor_states_.resize(motor_count_);
    motor_commands_.resize(motor_count_);
    
    // Set operating mode to CST (Cyclic Synchronous Torque) for torque control
    std::vector<char> modes(motor_count_, 10); // Mode 10 = CST
    sdk_->setMode(modes);
    
    // Enable all motors
    for (int i = 0; i < motor_count_; ++i) {
        motor_commands_[i].enabled = 1;
        motor_commands_[i].pos = 0.0f;
        motor_commands_[i].vel = 0.0f;
        motor_commands_[i].tor = 0.0f;
    }
    
    // Wait for motors to reach operational state
    std::cout << "Waiting for motors to be operational..." << std::endl;
    const int max_wait_ticks = 10000;
    int wait_ticks = 0;
    bool all_operational = false;
    
    while (!all_operational && wait_ticks < max_wait_ticks) {
        sdk_->getMotorActual(motor_states_);
        
        all_operational = true;
        for (int i = 0; i < 15 && i < motor_count_; ++i) {
            if ((motor_states_[i].statusWord & 0x007f) != 0x0031) {
                all_operational = false;
                break;
            }
        }
        
        if (wait_ticks % 500 == 0) {
            std::cout << "Waiting... (" << wait_ticks << "/" << max_wait_ticks << ")" << std::endl;
        }
        
        wait_ticks++;
        usleep(1000);  // 1ms
    }
    
    if (!all_operational) {
        std::cerr << "Warning: Not all motors reached operational state" << std::endl;
    } else {
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
    
    // Fill base angular velocity
    base_ang_vel.resize(3);
    base_ang_vel[0] = imu_data_.gyr[0];
    base_ang_vel[1] = imu_data_.gyr[1];
    base_ang_vel[2] = imu_data_.gyr[2];
    
    // Fill joint positions and velocities (first 12 motors = legs)
    joint_pos.resize(12);
    joint_vel.resize(12);
    for (int i = 0; i < 12; ++i) {
        joint_pos[i] = motor_states_[i].pos;
        joint_vel[i] = motor_states_[i].vel;
    }
    
    return true;
}

bool RealRobotInterface::writeTorque(const std::vector<float>& torques) {
    if (torques.size() != 12) {
        std::cerr << "Error: Expected 12 torques, got " << torques.size() << std::endl;
        return false;
    }
    
    // Set torque commands for first 12 motors (legs)
    for (int i = 0; i < 12; ++i) {
        motor_commands_[i].tor = torques[i];
        motor_commands_[i].pos = 0.0f;  // Not used in torque mode
        motor_commands_[i].vel = 0.0f;  // Not used in torque mode
        motor_commands_[i].enabled = 1;
    }
    
    // Send commands
    sdk_->setMotorTarget(motor_commands_);
    return true;
}

void RealRobotInterface::shutdown() {
    std::cout << "Shutting down real robot interface..." << std::endl;
    
    // Disable all motors
    for (int i = 0; i < motor_count_; ++i) {
        motor_commands_[i].enabled = 0;
        motor_commands_[i].pos = motor_states_[i].pos;  // Hold current position
        motor_commands_[i].vel = 0.0f;
        motor_commands_[i].tor = 0.0f;
    }
    
    for (int i = 0; i < 100; ++i) {
        sdk_->setMotorTarget(motor_commands_);
        usleep(1000);
    }
}

int RealRobotInterface::getMotorCount() const {
    return motor_count_;
}


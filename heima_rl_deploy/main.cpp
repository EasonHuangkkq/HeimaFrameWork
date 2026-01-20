/*
 * Heima RL Deployment Program
 * 
 * Deploys a trained neural network policy from legged_gym (heima_noarm)
 * to control the robot using the Heima SDK.
 */

#include "heima_driver_sdk.h"
#include "onnx_wrapper.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <chrono>

// Signal handling
volatile sig_atomic_t g_stop = 0;
void handleSignal(int) {
    g_stop = 1;
}

// Observation scaling factors (from training config)
const float LIN_VEL_SCALE = 2.0f;
const float ANG_VEL_SCALE = 0.25f;
const float DOF_POS_SCALE = 1.0f;
const float DOF_VEL_SCALE = 0.05f;
const float CMD_SCALE[3] = {2.0f, 2.0f, 0.25f};

// Default joint angles (from training config)
const float DEFAULT_JOINT_ANGLES[12] = {
    0.0f, 0.0f, 0.2f, -0.43f, 0.22f, 0.0f,  // Right leg
    0.0f, 0.0f, 0.2f, -0.43f, 0.22f, 0.0f   // Left leg
};

// Action scale (from training config)
const float ACTION_SCALE = 1.0f;

// PD gains (from training config)
const float PD_KP[12] = {
    300.0f, 300.0f, 300.0f, 400.0f, 120.0f, 120.0f,  // Right leg
    300.0f, 300.0f, 300.0f, 400.0f, 120.0f, 120.0f   // Left leg
};

const float PD_KD[12] = {
    1.0f, 1.0f, 1.0f, 4.0f, 1.0f, 1.0f,  // Right leg
    1.0f, 1.0f, 1.0f, 4.0f, 1.0f, 1.0f   // Left leg
};

// Compute rotation matrix from RPY angles
void rpyToRotationMatrix(float roll, float pitch, float yaw, float R[9]) {
    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float cy = cosf(yaw);
    float sy = sinf(yaw);
    
    R[0] = cy * cp;
    R[1] = cy * sp * sr - sy * cr;
    R[2] = cy * sp * cr + sy * sr;
    R[3] = sy * cp;
    R[4] = sy * sp * sr + cy * cr;
    R[5] = sy * sp * cr - cy * sr;
    R[6] = -sp;
    R[7] = cp * sr;
    R[8] = cp * cr;
}

// Rotate vector by inverse of rotation matrix (transpose)
void rotateInverse(const float R[9], const float v[3], float result[3]) {
    result[0] = R[0] * v[0] + R[3] * v[1] + R[6] * v[2];
    result[1] = R[1] * v[0] + R[4] * v[1] + R[7] * v[2];
    result[2] = R[2] * v[0] + R[5] * v[1] + R[8] * v[2];
}

// Build observation vector (48 dimensions)
void buildObservation(
    const DriverSDK::imuStruct& imu,
    const std::vector<DriverSDK::motorActualStruct>& motor_states,
    const float commands[3],  // [vx, vy, wz]
    const std::vector<float>& previous_actions,
    std::vector<float>& obs
) {
    obs.resize(48);
    int idx = 0;
    
    // Note: We need base linear velocity, but SDK doesn't provide it directly
    // For now, we'll estimate it or set to zero
    // In a real deployment, you might need odometry or velocity estimation
    float base_lin_vel[3] = {0.0f, 0.0f, 0.0f};  // TODO: Estimate from odometry
    
    // 1. Base linear velocity (scaled) - 3 dims
    obs[idx++] = base_lin_vel[0] * LIN_VEL_SCALE;
    obs[idx++] = base_lin_vel[1] * LIN_VEL_SCALE;
    obs[idx++] = base_lin_vel[2] * LIN_VEL_SCALE;
    
    // 2. Base angular velocity (scaled) - 3 dims
    obs[idx++] = imu.gyr[0] * ANG_VEL_SCALE;
    obs[idx++] = imu.gyr[1] * ANG_VEL_SCALE;
    obs[idx++] = imu.gyr[2] * ANG_VEL_SCALE;
    
    // 3. Projected gravity - 3 dims
    // Compute rotation matrix from RPY
    float R[9];
    rpyToRotationMatrix(imu.rpy[0], imu.rpy[1], imu.rpy[2], R);
    
    // Gravity vector in world frame [0, 0, -1]
    float gravity_world[3] = {0.0f, 0.0f, -1.0f};
    float projected_gravity[3];
    rotateInverse(R, gravity_world, projected_gravity);
    
    obs[idx++] = projected_gravity[0];
    obs[idx++] = projected_gravity[1];
    obs[idx++] = projected_gravity[2];
    
    // 4. Commands (scaled) - 3 dims
    obs[idx++] = commands[0] * CMD_SCALE[0];
    obs[idx++] = commands[1] * CMD_SCALE[1];
    obs[idx++] = commands[2] * CMD_SCALE[2];
    
    // 5. DOF positions (relative to default, scaled) - 12 dims
    // Assuming motors 0-11 are leg motors (adjust based on your config)
    for (int i = 0; i < 12; ++i) {
        float pos_rel = motor_states[i].pos - DEFAULT_JOINT_ANGLES[i];
        obs[idx++] = pos_rel * DOF_POS_SCALE;
    }
    
    // 6. DOF velocities (scaled) - 12 dims
    for (int i = 0; i < 12; ++i) {
        obs[idx++] = motor_states[i].vel * DOF_VEL_SCALE;
    }
    
    // 7. Previous actions - 12 dims
    for (int i = 0; i < 12; ++i) {
        obs[idx++] = (i < previous_actions.size()) ? previous_actions[i] : 0.0f;
    }
}

int main(int argc, char** argv) {
    // Parse arguments
    std::string config_file = "config.xml";
    std::string onnx_file = "policy_heima_noarm.onnx";
    float cmd_vx = 0.0f;
    float cmd_vy = 0.0f;
    float cmd_wz = 0.0f;
    
    if (argc > 1) {
        onnx_file = argv[1];
    }
    if (argc > 2) {
        config_file = argv[2];
    }
    if (argc > 3) {
        cmd_vx = std::stof(argv[3]);
    }
    if (argc > 4) {
        cmd_vy = std::stof(argv[4]);
    }
    if (argc > 5) {
        cmd_wz = std::stof(argv[5]);
    }
    
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    
    std::cout << "=== Heima RL Deployment ===" << std::endl;
    std::cout << "Config file: " << config_file << std::endl;
    std::cout << "ONNX model: " << onnx_file << std::endl;
    std::cout << "Commands: vx=" << cmd_vx << ", vy=" << cmd_vy << ", wz=" << cmd_wz << std::endl;
    
    // Initialize SDK
    DriverSDK::DriverSDK& sdk = DriverSDK::DriverSDK::instance();
    std::cout << "\nInitializing SDK..." << std::endl;
    sdk.init(config_file.c_str());
    
    int motor_count = sdk.getTotalMotorNr();
    std::cout << "Total motors: " << motor_count << std::endl;
    
    if (motor_count < 12) {
        std::cerr << "Error: Need at least 12 motors for leg control" << std::endl;
        return 1;
    }
    
    // Set operating mode (CSP - Cyclic Synchronous Position)
    std::vector<char> modes(motor_count, 8);
    sdk.setMode(modes);
    
    // Initialize ONNX model
    std::cout << "\nLoading ONNX model..." << std::endl;
    OnnxWrapper onnx;
    if (!onnx.init(onnx_file)) {
        std::cerr << "Failed to load ONNX model: " << onnx_file << std::endl;
        return 1;
    }
    
    if (onnx.getInputSize() != 48) {
        std::cerr << "Error: Model expects input size 48, got " << onnx.getInputSize() << std::endl;
        return 1;
    }
    
    if (onnx.getOutputSize() != 12) {
        std::cerr << "Error: Model expects output size 12, got " << onnx.getOutputSize() << std::endl;
        return 1;
    }
    
    // Prepare data structures
    std::vector<DriverSDK::motorTargetStruct> motor_commands(motor_count);
    std::vector<DriverSDK::motorActualStruct> motor_states(motor_count);
    DriverSDK::imuStruct imu_data;
    
    std::vector<float> observation(48);
    std::vector<float> actions(12, 0.0f);
    std::vector<float> previous_actions(12, 0.0f);
    
    float commands[3] = {cmd_vx, cmd_vy, cmd_wz};
    
    // Enable all motors
    std::cout << "\nEnabling motors..." << std::endl;
    for (int i = 0; i < motor_count; ++i) {
        motor_commands[i].enabled = 1;
        motor_commands[i].pos = DEFAULT_JOINT_ANGLES[i % 12];
        motor_commands[i].vel = 0.0f;
        motor_commands[i].tor = 0.0f;
    }
    
    // Wait for motors to reach operational state
    const int max_wait_ticks = 5000;
    int wait_ticks = 0;
    bool all_operational = false;
    
    while (!g_stop && !all_operational && wait_ticks < max_wait_ticks) {
        // sdk.setMotorTarget(motor_commands);
        sdk.getMotorActual(motor_states);
        // sdk.advance();
        
        all_operational = true;
        for (int i = 0; i < 12; ++i) {
            if ((motor_states[i].statusWord & 0x007f) != 0x0037) {
                all_operational = false;
                break;
            }
        }
        
        if (wait_ticks % 500 == 0) {
            std::cout << "Waiting for motors to be operational... (" << wait_ticks << "/" << max_wait_ticks << ")" << std::endl;
        }
        
        wait_ticks++;
        usleep(1000);  // 1ms
    }
    
    if (!all_operational) {
        std::cerr << "Warning: Not all motors reached operational state" << std::endl;
    } else {
        std::cout << "All motors operational!" << std::endl;
    }
    
    // Main control loop
    // Policy runs at 50Hz (matching training), EtherCAT communication at 1kHz
    std::cout << "\nStarting control loop..." << std::endl;
    std::cout << "Policy frequency: 50 Hz (20ms)" << std::endl;
    std::cout << "EtherCAT frequency: 1 kHz (1ms)" << std::endl;
    std::cout << "Press Ctrl+C to stop\n" << std::endl;
    
    auto loop_start = std::chrono::steady_clock::now();
    int loop_count = 0;
    int policy_count = 0;
    
    // Policy update timing (50 Hz = 20ms)
    const int POLICY_PERIOD_US = 20000;  // 20ms in microseconds
    auto last_policy_update = std::chrono::steady_clock::now();
    
    while (!g_stop) {
        auto cycle_start = std::chrono::steady_clock::now();
        
        // Read sensors (every 1ms for high-frequency feedback)
        sdk.getMotorActual(motor_states);
        sdk.getIMU(imu_data);
        
        // Check if it's time to update policy (50 Hz)
        auto now = std::chrono::steady_clock::now();
        auto time_since_policy = std::chrono::duration_cast<std::chrono::microseconds>(
            now - last_policy_update).count();
        
        if (time_since_policy >= POLICY_PERIOD_US) {
            // Build observation
            buildObservation(imu_data, motor_states, commands, previous_actions, observation);
            
            // Run neural network inference (50 Hz)
            if (!onnx.run(observation, actions)) {
                std::cerr << "Inference failed!" << std::endl;
                break;
            }
            
            // Update previous actions
            previous_actions = actions;
            
            last_policy_update = now;
            policy_count++;
        }
        
        // Convert actions to joint targets (use latest actions from policy)
        // Action is relative to default pose, scaled by ACTION_SCALE
        for (int i = 0; i < 12; ++i) {
            float target_pos = DEFAULT_JOINT_ANGLES[i] + actions[i] * ACTION_SCALE;
            motor_commands[i].pos = target_pos;
            motor_commands[i].vel = 0.0f;  // Velocity feedforward can be added here
            motor_commands[i].tor = 0.0f;  // Torque feedforward can be added here
            motor_commands[i].enabled = 1;
        }
        
        // Send commands (every 1ms for high-frequency control)
        // sdk.setMotorTarget(motor_commands);  // COMMENTED OUT: Disabled motor command sending
        // sdk.advance();
        
        // Timing and statistics
        loop_count++;
        if (loop_count % 1000 == 0) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - loop_start).count();
            float avg_freq = 1000.0f * loop_count / elapsed;
            float policy_freq = 1000.0f * policy_count / elapsed;
            
            std::cout << "Loop: " << loop_count 
                      << " | EtherCAT freq: " << avg_freq << " Hz"
                      << " | Policy freq: " << policy_freq << " Hz"
                      << " | Roll: " << imu_data.rpy[0]
                      << " | Pitch: " << imu_data.rpy[1] << std::endl;
        }
        
        // Maintain 1kHz loop rate (EtherCAT communication)
        auto cycle_end = std::chrono::steady_clock::now();
        auto cycle_time = std::chrono::duration_cast<std::chrono::microseconds>(
            cycle_end - cycle_start).count();
        
        int sleep_us = 1000 - cycle_time;  // 1ms = 1000us
        if (sleep_us > 0) {
            usleep(sleep_us);
        }
    }
    
    // Graceful shutdown
    std::cout << "\nShutting down..." << std::endl;
    for (int i = 0; i < motor_count; ++i) {
        motor_commands[i].enabled = 0;
        motor_commands[i].pos = motor_states[i].pos;  // Hold current position
        motor_commands[i].vel = 0.0f;
        motor_commands[i].tor = 0.0f;
    }
    
    for (int i = 0; i < 100; ++i) {
        // sdk.setMotorTarget(motor_commands);
        // sdk.advance();
        usleep(1000);
    }
    
    std::cout << "Done." << std::endl;
    return 0;
}


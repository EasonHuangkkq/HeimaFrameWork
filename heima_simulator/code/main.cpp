/*
 * Heima RL Deployment Program (Unified: Simulator & Real Robot)
 * 
 * Deploys a trained neural network policy from legged_gym (heima_noarm)
 * to control the robot using either:
 *   - Simulator mode: gRPC-based MuJoCo simulator
 *   - Real robot mode: Heima SDK
 */

#include "robot_interface.h"
#include "onnx_wrapper.h"
#include "ankle_solver/cpp/ankle_solver.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <chrono>
#include <fstream>
#include <iomanip>

// Signal handling
volatile sig_atomic_t g_stop = 0;
void handleSignal(int) {
    g_stop = 1;
}

// Ankle solver configuration
const bool USE_ANKLE_SOLVER = false;  // Set to true to enable ankle kinematics correction

// Apply ankle solver to transform desired ankle pitch/roll into actual motor angles
void applyAnkleSolver(float target_pos[15], AnkleSolver& ankle_solver) {
    if (!USE_ANKLE_SOLVER) return;
    
    // Left leg ankle (indices 4=pitch, 5=roll)
    double left_ankle_pitch = static_cast<double>(target_pos[4]);
    double left_ankle_roll = static_cast<double>(target_pos[5]);
    auto left_ankle_angles = ankle_solver.solve(-left_ankle_pitch, -left_ankle_roll);
    target_pos[4] = left_ankle_angles.second;  // theta_f -> pitch motor
    target_pos[5] = left_ankle_angles.first;   // theta_e -> roll motor
    
    // Right leg ankle (indices 10=pitch, 11=roll)
    double right_ankle_pitch = static_cast<double>(target_pos[10]);
    double right_ankle_roll = static_cast<double>(target_pos[11]);
    auto right_ankle_angles = ankle_solver.solve(-right_ankle_pitch, -right_ankle_roll);
    target_pos[10] = right_ankle_angles.second; // theta_f -> pitch motor
    target_pos[11] = right_ankle_angles.first;  // theta_e -> roll motor
}

// Apply inverse ankle solver to convert motor angles back to ankle pitch/roll for observation
void applyAnkleSolverInverse(std::vector<float>& joint_pos, AnkleSolver& ankle_solver) {
    if (!USE_ANKLE_SOLVER) return;
    
    // Left leg ankle (indices 4=pitch motor, 5=roll motor)
    double left_theta_e = static_cast<double>(joint_pos[5]);  // roll motor angle
    double left_theta_f = static_cast<double>(joint_pos[4]);  // pitch motor angle
    auto left_ankle_angles = ankle_solver.solve_inverse(left_theta_e, left_theta_f);
    joint_pos[4] = -left_ankle_angles.first;   // pitch angle
    joint_pos[5] = -left_ankle_angles.second;  // roll angle
    
    // Right leg ankle (indices 10=pitch motor, 11=roll motor)
    double right_theta_e = static_cast<double>(joint_pos[11]);  // roll motor angle
    double right_theta_f = static_cast<double>(joint_pos[10]);  // pitch motor angle
    auto right_ankle_angles = ankle_solver.solve_inverse(right_theta_e, right_theta_f);
    joint_pos[10] = -right_ankle_angles.first;   // pitch angle
    joint_pos[11] = -right_ankle_angles.second;  // roll angle
}

// Observation scaling factors (from training config)
const float LIN_VEL_SCALE = 2.0f;
const float ANG_VEL_SCALE = 0.25f;
const float DOF_POS_SCALE = 1.0f;
const float DOF_VEL_SCALE = 0.05f;
const float CMD_SCALE[3] = {2.0f, 2.0f, 0.25f};

// Default joint angles (from training config)
// const float DEFAULT_JOINT_ANGLES[12] = {
//     0.0f, 0.0f, 0.2f, -0.43f, 0.22f, 0.0f,  // Right leg
//     0.0f, 0.0f, 0.2f, -0.43f, 0.22f, 0.0f   // Left leg
// };

// TESTING
const float DEFAULT_JOINT_ANGLES[15] = {
    0.0f, 0.0f, 0.2f, -0.43f, 0.22f, 0.0f,  // Left leg
    0.0f, 0.0f, 0.2f, -0.43f, 0.22f, 0.0f,   // Right leg
    0.0f, 0.0f, 0.0f
};

// Action scale (from training config)
const float ACTION_SCALE = 0.25f;

// PD gains (from training config)
// const float PD_KP[12] = {
//     300.0f, 300.0f, 300.0f, 400.0f, 120.0f, 120.0f,  // Right leg
//     300.0f, 300.0f, 300.0f, 400.0f, 120.0f, 120.0f   // Left leg
// };

// const float PD_KD[12] = {
//     1.0f, 1.0f, 1.0f, 4.0f, 1.0f, 1.0f,  // Right leg
//     1.0f, 1.0f, 1.0f, 4.0f, 1.0f, 1.0f,   // Left leg
//     1.0f, 1.0f, 1.0f
// };


// TESTING
const float PD_KP[15] = {
    150.0f, 150.0f, 150.0f, 200.0f, 60.0f, 60.0f,  // Left leg
    150.0f, 150.0f, 150.0f, 200.0f, 60.0f, 60.0f,   // Right leg
    200.0f, 200.0f, 200.0f
};
// const float PD_KP[15] = {
//     30.0f, 30.0f, 30.0f, 40.0f, 20.0f, 20.0f,  // Left leg
//     30.0f, 30.0f, 30.0f, 40.0f, 20.0f, 20.0f,   // Right leg
//     8.0f, 8.0f, 8.0f
// };

const float PD_KD[15] = {
    1.0f, 1.0f, 1.0f, 4.0f, 1.0f, 1.0f,  // Left leg
    1.0f, 1.0f, 1.0f, 4.0f, 1.0f, 1.0f,   // Right leg
    1.0f, 1.0f, 1.0f
};

// Maximum torque limits for each motor (N·m)
// const float MAX_TORQUE_LIMIT[15] = {
//     30.0f, 20.0f, 30.0f, 30.0f, 30.0f, 30.0f,  // Left leg
//     30.0f, 20.0f, 30.0f, 30.0f, 30.0f, 30.0f,   // Right leg
//     50.0f, 50.0f, 50.0f
// };
const float MAX_TORQUE_LIMIT[15] = {
    3000.0f, 3000.0f, 3000.0f, 3000.0f, 3000.0f, 3000.0f,  // Left leg
    3000.0f, 3000.0f, 3000.0f, 3000.0f, 3000.0f, 3000.0f,   // Right leg
    400.0f, 400.0f, 400.0f
};

// PD Control function
float pdControl(float targetPos, float actualPos, float targetVel, float actualVel, float kp, float kd) {
    float posError = targetPos - actualPos;
    float velError = targetVel - actualVel;
    float torque = kp * posError + kd * velError;
    return torque;
}

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

// Build observation vector (45 dimensions)
void buildObservation(
    const std::vector<float>& rpy,
    const std::vector<float>& base_ang_vel,
    const std::vector<float>& joint_pos,
    const std::vector<float>& joint_vel,
    const float commands[3],  // [vx, vy, wz]
    const std::vector<float>& previous_actions,
    std::vector<float>& obs
) {
    obs.resize(45);
    int idx = 0;
    
    // 1. Base angular velocity (scaled) - 3 dims
    obs[idx++] = base_ang_vel[0] * ANG_VEL_SCALE;
    obs[idx++] = base_ang_vel[1] * ANG_VEL_SCALE;
    obs[idx++] = base_ang_vel[2] * ANG_VEL_SCALE;
    
    // 2. Projected gravity - 3 dims
    // Compute rotation matrix from RPY
    float R[9];
    rpyToRotationMatrix(rpy[0], rpy[1], rpy[2], R);
    
    // Gravity vector in world frame [0, 0, -1]
    float gravity_world[3] = {0.0f, 0.0f, -1.0f};
    float projected_gravity[3];
    rotateInverse(R, gravity_world, projected_gravity);
    
    obs[idx++] = projected_gravity[0];
    obs[idx++] = projected_gravity[1];
    obs[idx++] = projected_gravity[2];
    
    // 3. Commands (scaled) - 3 dims
    obs[idx++] = commands[0] * CMD_SCALE[0];
    obs[idx++] = commands[1] * CMD_SCALE[1];
    obs[idx++] = commands[2] * CMD_SCALE[2];
    
    // 4. DOF positions (relative to default, scaled) - 12 dims
    for (int i = 0; i < 12; ++i) {
        float pos_rel = joint_pos[i] - DEFAULT_JOINT_ANGLES[i];
        obs[idx++] = pos_rel * DOF_POS_SCALE;
    }
    
    // 5. DOF velocities (scaled) - 12 dims
    for (int i = 0; i < 12; ++i) {
        obs[idx++] = joint_vel[i] * DOF_VEL_SCALE;
    }
    
    // 6. Previous actions - 12 dims
    for (int i = 0; i < 12; ++i) {
        obs[idx++] = (i < previous_actions.size()) ? previous_actions[i] : 0.0f;
    }
}

int main(int argc, char** argv) {
    // Parse arguments
    std::string mode = "sim";  // "sim" or "real"
    std::string onnx_file = "policy_heima_noarm.onnx";
    std::string config = "";  // gRPC server for sim, config.xml for real
    float cmd_vx = 0.0f;
    float cmd_vy = 0.0f;
    float cmd_wz = 0.0f;
    
    // Parse command line arguments
    // Usage: ./main [mode] [onnx_file] [config] [vx] [vy] [wz]
    //   mode: "sim" (default) or "real"
    //   config: for sim = "localhost:50051" (default), for real = "config.xml" (default)
    if (argc > 1) {
        mode = argv[1];
    }
    if (argc > 2) {
        onnx_file = argv[2];
    }
    if (argc > 3) {
        config = argv[3];
    }
    if (argc > 4) {
        cmd_vx = std::stof(argv[4]);
    }
    if (argc > 5) {
        cmd_vy = std::stof(argv[5]);
    }
    if (argc > 6) {
        cmd_wz = std::stof(argv[6]);
    }
    
    // Set default config based on mode
    if (config.empty()) {
        if (mode == "sim" || mode == "simulator") {
            config = "localhost:50051";
        } else {
            config = "config.xml";
        }
    }
    
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    
    std::cout << "=== Heima RL Deployment ===" << std::endl;
    std::cout << "Mode: " << mode << std::endl;
    std::cout << "ONNX model: " << onnx_file << std::endl;
    std::cout << "Config: " << config << std::endl;
    std::cout << "Commands: vx=" << cmd_vx << ", vy=" << cmd_vy << ", wz=" << cmd_wz << std::endl;
    
    // Initialize robot interface
    auto robot = createRobotInterface(mode, config);
    if (!robot) {
        std::cerr << "Failed to create robot interface" << std::endl;
        return 1;
    }
    
    if (!robot->init()) {
        std::cerr << "Failed to initialize robot interface" << std::endl;
        return 1;
    }
    
    // Initialize ankle solver
    AnkleSolver ankle_solver;
    if (USE_ANKLE_SOLVER) {
        std::cout << "\nAnkle solver: ENABLED (will apply kinematics correction)" << std::endl;
    } else {
        std::cout << "\nAnkle solver: DISABLED (direct motor control)" << std::endl;
    }
    
    // Initialize ONNX model
    std::cout << "\nLoading ONNX model..." << std::endl;
    OnnxWrapper onnx;
    if (!onnx.init(onnx_file)) {
        std::cerr << "Failed to load ONNX model: " << onnx_file << std::endl;
        return 1;
    }
    
    if (onnx.getInputSize() != 45) {
        std::cerr << "Error: Model expects input size 45, got " << onnx.getInputSize() << std::endl;
        return 1;
    }
    
    if (onnx.getOutputSize() != 12) {
        std::cerr << "Error: Model expects output size 12, got " << onnx.getOutputSize() << std::endl;
        return 1;
    }
    
    // Prepare data structures
    std::vector<float> rpy(3, 0.0f);
    std::vector<float> base_ang_vel(3, 0.0f);
    std::vector<float> joint_pos(12, 0.0f);
    std::vector<float> joint_vel(12, 0.0f);
    
    std::vector<float> observation(45);
    std::vector<float> actions(12, 0.0f);
    std::vector<float> previous_actions(12, 0.0f);
    
    float commands[3] = {cmd_vx, cmd_vy, cmd_wz};

    // Open CSV file for logging observations
    std::ofstream csv_file("observations_log.csv");
    if (!csv_file.is_open()) {
        std::cerr << "Warning: Failed to open CSV file for logging" << std::endl;
    } else {
        // Write CSV header
        csv_file << "timestamp_ms,";
        csv_file << "roll,pitch,yaw,";
        csv_file << "ang_vel_x,ang_vel_y,ang_vel_z,";
        for (int i = 0; i < 12; ++i) {
            csv_file << "joint_pos_" << i << ",";
        }
        for (int i = 0; i < 12; ++i) {
            csv_file << "joint_vel_" << i << ",";
        }
        csv_file << "cmd_vx,cmd_vy,cmd_wz,";
        for (int i = 0; i < 12; ++i) {
            csv_file << "action_" << i << ",";
        }
        for (int i = 0; i < 45; ++i) {
            csv_file << "obs_" << i;
            if (i < 44) csv_file << ",";
        }
        csv_file << "\n";
        csv_file << std::fixed << std::setprecision(6);
        std::cout << "CSV logging enabled: observations_log.csv" << std::endl;
    }

    // Stage 2: Move to default position using PD control (REAL ROBOT ONLY)
    if (mode == "real" || mode == "robot") {
        std::cout << "\nStage 2: Moving to default position..." << std::endl;
        
        // Fetch initial state
        if (!robot->fetchObs(rpy, base_ang_vel, joint_pos, joint_vel)) {
            std::cerr << "Failed to fetch initial state" << std::endl;
            return 1;
        }
        
        // Store initial positions for interpolation
        std::vector<float> initial_positions = joint_pos;
        
        const int interpolation_ticks = 5000;  // 5 seconds to move to default
        const int hold_ticks = 10000;          // 10 seconds hold at default
        int stage2_ticks = 0;
        
        while (!g_stop && stage2_ticks < (interpolation_ticks + hold_ticks)) {
            // Fetch current state
            if (!robot->fetchObs(rpy, base_ang_vel, joint_pos, joint_vel)) {
                std::cerr << "Failed to fetch state during initialization" << std::endl;
                return 1;
            }
            
            // Calculate interpolation factor (0 to 1)
            float alpha = 1.0f;
            if (stage2_ticks < interpolation_ticks) {
                alpha = static_cast<float>(stage2_ticks) / static_cast<float>(interpolation_ticks);
            }
            
            // Compute torques for each joint
            std::vector<float> torques(12, 0.0f);
            for (int i = 0; i < 12; ++i) {
                // Interpolate target position
                float target = initial_positions[i] * (1.0f - alpha) + DEFAULT_JOINT_ANGLES[i] * alpha;
                
                // PD control
                float torque = pdControl(target, joint_pos[i], 0.0f, joint_vel[i], PD_KP[i], PD_KD[i]);
                
                // Clip torque
                if (torque > MAX_TORQUE_LIMIT[i]) torque = MAX_TORQUE_LIMIT[i];
                if (torque < -MAX_TORQUE_LIMIT[i]) torque = -MAX_TORQUE_LIMIT[i];
                
                torques[i] = torque;
            }
            
            // Send torques
            robot->writeTorque(torques);
            
            if (stage2_ticks % 500 == 0) {
                std::cout << "Moving to default... (" << stage2_ticks 
                          << "/" << (interpolation_ticks + hold_ticks) << ")" << std::endl;
            }
            
            stage2_ticks++;
            usleep(1000);  // 1ms
        }
        
        std::cout << "Default position reached and stabilized!" << std::endl;
    } else {
        std::cout << "\nSimulator mode: Skipping default position initialization" << std::endl;
    }

    // Main control loop
    // Policy runs at 50Hz (matching training), communication at 1kHz
    std::cout << "\nStarting control loop..." << std::endl;
    std::cout << "Policy frequency: 50 Hz (20ms)" << std::endl;
    std::cout << "Communication frequency: 1 kHz (1ms)" << std::endl;
    std::cout << "Press Ctrl+C to stop\n" << std::endl;
    
    auto loop_start = std::chrono::steady_clock::now();
    int loop_count = 0;
    int policy_count = 0;
    
    // Policy update timing (50 Hz = 20ms)
    const int POLICY_PERIOD_US = 20000;  // 20ms in microseconds
    auto last_policy_update = std::chrono::steady_clock::now();
    
    // PD control update timing (200 Hz = 5ms)
    const int PD_UPDATE_PERIOD_MS = 5;  // 5ms = 5 iterations at 1kHz
    int pd_update_counter = 0;
    
    while (!g_stop) {
        auto cycle_start = std::chrono::steady_clock::now();
        
        // Read sensors (every 1ms for high-frequency feedback)
        if (!robot->fetchObs(rpy, base_ang_vel, joint_pos, joint_vel)) {
            std::cerr << "Failed to fetch observations" << std::endl;
            break;
        }
        
        // Check if it's time to update policy (50 Hz)
        auto now = std::chrono::steady_clock::now();
        auto time_since_policy = std::chrono::duration_cast<std::chrono::microseconds>(
            now - last_policy_update).count();
        
        if (time_since_policy >= POLICY_PERIOD_US) {
            // Apply inverse ankle solver to observations (convert motor angles back to virtual ankle angles)
            std::vector<float> corrected_joint_pos = joint_pos;
            applyAnkleSolverInverse(corrected_joint_pos, ankle_solver);
            
            // Build observation with corrected joint positions
            buildObservation(rpy, base_ang_vel, corrected_joint_pos, joint_vel, commands, previous_actions, observation);
            
            // Log observations to CSV before inference (use corrected positions)
            if (csv_file.is_open()) {
                auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - loop_start).count();
                
                csv_file << elapsed_ms << ",";
                csv_file << rpy[0] << "," << rpy[1] << "," << rpy[2] << ",";
                csv_file << base_ang_vel[0] << "," << base_ang_vel[1] << "," << base_ang_vel[2] << ",";
                for (int i = 0; i < 12; ++i) {
                    csv_file << corrected_joint_pos[i] << ",";  // Log corrected positions
                }
                for (int i = 0; i < 12; ++i) {
                    csv_file << joint_vel[i] << ",";
                }
                csv_file << commands[0] << "," << commands[1] << "," << commands[2] << ",";
                for (int i = 0; i < 12; ++i) {
                    csv_file << previous_actions[i] << ",";
                }
                for (int i = 0; i < 45; ++i) {
                    csv_file << observation[i];
                    if (i < 44) csv_file << ",";
                }
                csv_file << "\n";
            }
            
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
        
        // Recalculate PD control every 5ms (200 Hz)
        if (pd_update_counter == 0) {
            // Convert actions to joint targets and compute PD torque commands
            // Action is relative to default pose, scaled by ACTION_SCALE
            float target_pos[15] = {0.0f};
            for (int i = 0; i < 15; ++i) {
                target_pos[i] = DEFAULT_JOINT_ANGLES[i];
                if (i < 12) {
                    target_pos[i] = DEFAULT_JOINT_ANGLES[i] + actions[i] * ACTION_SCALE;
                }
            }

            // Apply ankle solver transformation if enabled
            applyAnkleSolver(target_pos, ankle_solver);

            // Prepare torque commands for 12 leg joints
            std::vector<float> torques(12, 0.0f);
            
            for (int i = 0; i < 12; ++i) {
                float actual_pos = joint_pos[i];
                float target_vel = 0.0f;
                float actual_vel = joint_vel[i];
                
                // Compute PD torque command
                float torqueCmd = pdControl(target_pos[i], actual_pos, target_vel, actual_vel, PD_KP[i], PD_KD[i]);
                
                // Safety clip: limit torque command to prevent excessive values
                if (torqueCmd > MAX_TORQUE_LIMIT[i]) {
                    torqueCmd = MAX_TORQUE_LIMIT[i];
                    std::cerr << "WARNING: Torque clipped for motor " << i 
                              << " (clipped to: " << torqueCmd << " N·m)" << std::endl;
                } else if (torqueCmd < -MAX_TORQUE_LIMIT[i]) {
                    torqueCmd = -MAX_TORQUE_LIMIT[i];
                    std::cerr << "WARNING: Torque clipped for motor " << i 
                              << " (clipped to: " << torqueCmd << " N·m)" << std::endl;
                }
                
                torques[i] = torqueCmd;
            }
            
            // Send torques to robot
            robot->writeTorque(torques);
        }
        
        // Update counter for PD control recalculation (every 5ms)
        pd_update_counter++;
        if (pd_update_counter >= PD_UPDATE_PERIOD_MS) {
            pd_update_counter = 0;
        }
        
        // Timing and statistics
        loop_count++;
        if (loop_count % 100 == 0) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - loop_start).count();
            float avg_freq = 1000.0f * loop_count / elapsed;
            float policy_freq = 1000.0f * policy_count / elapsed;
            
            std::cout << "Loop: " << loop_count 
                      << " | Comm freq: " << avg_freq << " Hz"
                      << " | Policy freq: " << policy_freq << " Hz"
                      << " | Roll: " << rpy[0]
                      << " | Pitch: " << rpy[1]
                      << " | Yaw: " << rpy[2] 
                      << std::endl;
        }
        
        // Maintain 1kHz loop rate (gRPC communication)
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
    robot->shutdown();
    
    // Close CSV file
    if (csv_file.is_open()) {
        csv_file.close();
        std::cout << "CSV log saved: observations_log.csv (" << policy_count << " samples @ 50Hz)" << std::endl;
    }
    
    std::cout << "Done." << std::endl;
    return 0;
}


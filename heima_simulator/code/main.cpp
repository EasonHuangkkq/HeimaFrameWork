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
#include <termios.h>
#include <fcntl.h>
#include <thread>
#include <mutex>
#include <yaml-cpp/yaml.h>
#include <string>
#include <sched.h>
#include <pthread.h>

// Signal handling
volatile sig_atomic_t g_stop = 0;
void handleSignal(int) {
    g_stop = 1;
}

// Keyboard input handling
struct termios old_termios;
bool keyboard_enabled = false;

void setupKeyboard() {
    struct termios new_termios;
    tcgetattr(STDIN_FILENO, &old_termios);
    new_termios = old_termios;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    new_termios.c_cc[VMIN] = 0;
    new_termios.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    keyboard_enabled = true;
}

void restoreKeyboard() {
    if (keyboard_enabled) {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_termios);
        keyboard_enabled = false;
    }
}

// Command velocity with mutex protection
std::mutex cmd_mutex;
float cmd_vx = 0.0f;
float cmd_vy = 0.0f;
float cmd_wz = 0.0f;
const float CMD_VEL_FORWARD_FIXED = 0.5f;  // Fixed velocity value
const float CMD_VEL_BACKWARD_FIXED = 0.5f;  // Fixed velocity value
const float CMD_VEL_LEFT_FIXED = 0.3f;  // Fixed velocity value
const float CMD_VEL_RIGHT_FIXED = 0.3f;  // Fixed velocity value
const float CMD_VEL_ROTATE_LEFT_FIXED = 0.9f;  // Fixed velocity value
const float CMD_VEL_ROTATE_RIGHT_FIXED = 0.9f;  // Fixed velocity value
const float CMD_VEL_STOP_FIXED = 0.0f;  // Fixed velocity value

void updateCommandsFromKeyboard() {
    char c;
    
    // Read all available characters
    while (read(STDIN_FILENO, &c, 1) > 0) {
        std::lock_guard<std::mutex> lock(cmd_mutex);
        
        switch (c) {
            case 'w':
            case 'W':
                cmd_vx = CMD_VEL_FORWARD_FIXED;  // Forward
                break;
            case 's':
            case 'S':
                cmd_vx = -CMD_VEL_BACKWARD_FIXED;  // Backward
                break;
            default:
                break;
        }
        switch (c) {
            case 'a':
            case 'A':
                cmd_vy = CMD_VEL_LEFT_FIXED;  // Left
                break;
            case 'd':
            case 'D':
                cmd_vy = -CMD_VEL_RIGHT_FIXED;  // Right
                break;
            default:
                break;
        }
        switch (c) {
            case 'q':
            case 'Q':
                cmd_wz = CMD_VEL_ROTATE_LEFT_FIXED;  // Rotate left
                break;
            case 'e':
            case 'E':
                cmd_wz = -CMD_VEL_ROTATE_RIGHT_FIXED;  // Rotate right
                break;
            default:
                break;
            
        }

        if (c != 'w' && c != 's' && c != 'a' && c != 'd' && c != 'q' && c != 'e' 
            && c != 'W' && c != 'S' && c != 'A' && c != 'D' && c != 'Q' && c != 'E') {
            cmd_vx = 0.0f;
            cmd_vy = 0.0f;
            cmd_wz = 0.0f;
        }
    }
}

// Ankle solver configuration
bool USE_ANKLE_SOLVER = true;  // Set to true to enable ankle kinematics correction

// Apply ankle solver to transform desired ankle pitch/roll into actual motor angles
void applyAnkleSolver(float target_pos[13], AnkleSolver& ankle_solver) {
    if (!USE_ANKLE_SOLVER) return;
    
    // right leg ankle (indices 4=pitch, 5=roll)
    double right_ankle_pitch = static_cast<double>(target_pos[4]);
    double right_ankle_roll = static_cast<double>(target_pos[5]);
    auto right_ankle_angles = ankle_solver.solve(-right_ankle_pitch, -right_ankle_roll);
    target_pos[4] = right_ankle_angles.second;  // theta_f -> pitch motor
    target_pos[5] = right_ankle_angles.first;   // theta_e -> roll motor
    
    // left leg ankle (indices 10=pitch, 11=roll)
    double left_ankle_pitch = static_cast<double>(target_pos[10]);
    double left_ankle_roll = static_cast<double>(target_pos[11]);
    auto left_ankle_angles = ankle_solver.solve(-left_ankle_pitch, -left_ankle_roll);
    target_pos[10] = left_ankle_angles.second; // theta_f -> pitch motor
    target_pos[11] = left_ankle_angles.first;  // theta_e -> roll motor
}

// Apply inverse ankle solver to convert motor angles back to ankle pitch/roll for observation
void applyAnkleSolverInverse(std::vector<float>& joint_pos, AnkleSolver& ankle_solver) {
    if (!USE_ANKLE_SOLVER) return;
    
    // right leg ankle (indices 4=pitch motor, 5=roll motor)
    double right_theta_e = static_cast<double>(joint_pos[4]);  // roll motor angle
    double right_theta_f = static_cast<double>(joint_pos[5]);  // pitch motor angle
    auto right_ankle_angles = ankle_solver.solve_inverse(right_theta_e, right_theta_f);
    joint_pos[4] = -right_ankle_angles.first;   // pitch angle
    joint_pos[5] = -right_ankle_angles.second;  // roll angle
    
    // left leg ankle (indices 10=pitch motor, 11=roll motor)
    double left_theta_e = static_cast<double>(joint_pos[10]);  // roll motor angle
    double left_theta_f = static_cast<double>(joint_pos[11]);  // pitch motor angle
    auto left_ankle_angles = ankle_solver.solve_inverse(left_theta_e, left_theta_f);
    joint_pos[10] = -left_ankle_angles.first;   // pitch angle
    joint_pos[11] = -left_ankle_angles.second;  // roll angle
}

// Observation scaling factors (from training config, loaded from YAML)
float LIN_VEL_SCALE = 2.0f;
float ANG_VEL_SCALE = 0.25f;
float DOF_POS_SCALE = 1.0f;
float DOF_VEL_SCALE = 0.05f;
float CMD_SCALE[3] = {2.0f, 2.0f, 0.25f};
float CMD_FILTER_ALPHA = 0.05f;

// Default joint angles, action scales, and PD gains (loaded from config file)
// Order: Left leg [hip_roll, hip_yaw, hip_pitch, knee, ankle_pitch, ankle_roll]
//        Right leg [hip_roll, hip_yaw, hip_pitch, knee, ankle_pitch, ankle_roll]
//        Waist [3 motors]
float DEFAULT_JOINT_ANGLES[13] = {
    0.0f, 0.0f, -0.25f, -0.5f, 0.25f, 0.0f,  // Left leg: roll, yaw, pitch, knee, ankle_pitch, ankle_roll
    0.0f, 0.0f, -0.25f, -0.5f, 0.25f, 0.0f,   // Right leg: roll, yaw, pitch, knee, ankle_pitch, ankle_roll
    0.0f  // Waist motors (default values, will be loaded from config)
};

float ACTION_SCALE[13] = {
    1.9910780472403882f, 1.9910780472403882f, 1.1352349630969132f, 1.9910780472403882f, 0.8940104439540857f, 0.8940104439540857f,  // Left leg
    1.9910780472403882f, 1.9910780472403882f, 1.1352349630969132f, 1.9910780472403882f, 0.8940104439540857f, 0.8940104439540857f,  // Right leg
    1.0f  // Waist motors (default values, will be loaded from config)
};

float PD_KP[13] = {
    40.179f, 40.179f, 99.098f, 40.179f, 16.778f, 16.778f,  // Left leg: roll, yaw, pitch, knee, ankle_pitch, ankle_roll
    40.179f, 40.179f, 99.098f, 40.179f, 16.778f, 16.778f,   // Right leg: roll, yaw, pitch, knee, ankle_pitch, ankle_roll
    200.0f  // Waist motors (default values, will be loaded from config)
};

float PD_KD[13] = {
    2.558f, 2.558f, 6.309f, 2.558f, 1.068f, 1.068f,  // Left leg: roll, yaw, pitch, knee, ankle_pitch, ankle_roll
    2.558f, 2.558f, 6.309f, 2.558f, 1.068f, 1.068f,   // Right leg: roll, yaw, pitch, knee, ankle_pitch, ankle_roll
    1.0f  // Waist motors (default values, will be loaded from config)
};

// Deploy-side safety limits
float ACTION_CLIP = 1.0f;
float TARGET_POS_MIN[13] = {
    -2.0f, -2.0f, -2.0f, -2.0f, -2.0f, -2.0f,
    -2.0f, -2.0f, -2.0f, -2.0f, -2.0f, -2.0f,
    -2.0f
};
float TARGET_POS_MAX[13] = {
    2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f,
    2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f,
    2.0f
};
float PVT_MAX_VEL[13] = {
    10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f,
    10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f,
    10.0f
};

float clampFloat(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

float sanitizeFinite(float value, float fallback) {
    if (!std::isfinite(value)) {
        return fallback;
    }
    return value;
}

float stepTowardLimited(float current, float target, float max_step) {
    if (!std::isfinite(current)) {
        current = 0.0f;
    }
    if (!std::isfinite(target)) {
        return current;
    }
    if (!std::isfinite(max_step)) {
        return current;
    }
    if (max_step <= 0.0f) {
        return target;
    }
    float error = target - current;
    if (error > max_step) {
        return current + max_step;
    }
    if (error < -max_step) {
        return current - max_step;
    }
    return target;
}

bool loadFloatArrayOrScalar(const YAML::Node& node, float target[13], const std::string& config_name) {
    if (!node) {
        return false;
    }

    if (node.IsScalar()) {
        float value = node.as<float>();
        for (int i = 0; i < 13; ++i) {
            target[i] = value;
        }
        std::cout << "[Config] Loaded " << config_name << " = " << value << " (applied to all 13 joints)" << std::endl;
        return true;
    }

    if (node.IsSequence() && node.size() == 13) {
        for (size_t i = 0; i < 13; ++i) {
            target[i] = node[i].as<float>();
        }
        std::cout << "[Config] Loaded " << config_name << " from YAML sequence" << std::endl;
        return true;
    }

    std::cerr << "[Config] Warning: " << config_name << " must be a scalar or 13-element sequence. Using previous values." << std::endl;
    return false;
}

void normalizeSafetyConfig() {
    if (ACTION_CLIP < 0.0f) {
        ACTION_CLIP = -ACTION_CLIP;
    }
    for (int i = 0; i < 13; ++i) {
        if (PVT_MAX_VEL[i] < 0.0f) {
            PVT_MAX_VEL[i] = -PVT_MAX_VEL[i];
        }
        if (TARGET_POS_MIN[i] > TARGET_POS_MAX[i]) {
            float tmp = TARGET_POS_MIN[i];
            TARGET_POS_MIN[i] = TARGET_POS_MAX[i];
            TARGET_POS_MAX[i] = tmp;
        }
    }
}

void clipActionsInPlace(std::vector<float>& actions) {
    for (size_t i = 0; i < actions.size() && i < 12; ++i) {
        actions[i] = sanitizeFinite(actions[i], 0.0f);
        actions[i] = clampFloat(actions[i], -ACTION_CLIP, ACTION_CLIP);
    }
}

void clampTargetPositions(float target_pos[13]) {
    for (int i = 0; i < 13; ++i) {
        target_pos[i] = sanitizeFinite(target_pos[i], DEFAULT_JOINT_ANGLES[i]);
        target_pos[i] = clampFloat(target_pos[i], TARGET_POS_MIN[i], TARGET_POS_MAX[i]);
    }
}

void buildSafeTargetPositions(const std::vector<float>& actions, float target_pos[13]) {
    for (int i = 0; i < 13; ++i) {
        target_pos[i] = DEFAULT_JOINT_ANGLES[i];
        if (i < 12 && i < static_cast<int>(actions.size())) {
            target_pos[i] += actions[i] * ACTION_SCALE[i];
        }
    }
    clampTargetPositions(target_pos);
}

// Thread config — default values, can be overridden by YAML
int ThreadEcatCpu     = 1;   // EtherCAT SDK thread CPU
int ThreadControlCpu  = 2;   // Main control loop CPU
int ThreadControlPri  = 88;  // SCHED_FIFO priority (1-99)


// Load configuration from YAML file
bool loadConfigFromYAML(const std::string& config_path) {
    // Check if file exists
    std::ifstream file(config_path);
    if (!file.good()) {
        std::cerr << "[Config] Warning: Config file not found: " << config_path << std::endl;
        std::cerr << "[Config] Using default hardcoded values." << std::endl;
        return false;
    }
    file.close();
    
    try {
        YAML::Node config = YAML::LoadFile(config_path);
        
        // Load PD gains
        if (config["pd_gains"]) {
            if (config["pd_gains"]["kp"]) {
                auto kp_node = config["pd_gains"]["kp"];
                if (kp_node.size() == 13) {
                    for (size_t i = 0; i < 13; ++i) {
                        PD_KP[i] = kp_node[i].as<float>();
                    }
                    std::cout << "[Config] Loaded PD_KP from " << config_path << std::endl;
                } else {
                    std::cerr << "[Config] Warning: pd_gains.kp has " << kp_node.size() 
                              << " elements, expected 13. Using default values." << std::endl;
                }
            }
            
            if (config["pd_gains"]["kd"]) {
                auto kd_node = config["pd_gains"]["kd"];
                if (kd_node.size() == 13) {
                    for (size_t i = 0; i < 13; ++i) {
                        PD_KD[i] = kd_node[i].as<float>();
                    }
                    std::cout << "[Config] Loaded PD_KD from " << config_path << std::endl;
                } else {
                    std::cerr << "[Config] Warning: pd_gains.kd has " << kd_node.size() 
                              << " elements, expected 13. Using default values." << std::endl;
                }
            }
        }
        
        // Load default joint angles
        if (config["default_joint_angles"]) {
            auto angles_node = config["default_joint_angles"];
            if (angles_node.size() == 13) {
                for (size_t i = 0; i < 13; ++i) {
                    DEFAULT_JOINT_ANGLES[i] = angles_node[i].as<float>();
                }
                std::cout << "[Config] Loaded DEFAULT_JOINT_ANGLES from " << config_path << std::endl;
            } else {
                std::cerr << "[Config] Warning: default_joint_angles has " << angles_node.size() 
                          << " elements, expected 13. Using default values." << std::endl;
            }
        }
        
        // Load action scales
        if (config["action_scale"]) {
            auto scale_node = config["action_scale"];
            if (scale_node.size() == 13) {
                for (size_t i = 0; i < 13; ++i) {
                    ACTION_SCALE[i] = scale_node[i].as<float>();
                }
                std::cout << "[Config] Loaded ACTION_SCALE from " << config_path << std::endl;
            } else {
                std::cerr << "[Config] Warning: action_scale has " << scale_node.size() 
                          << " elements, expected 13. Using default values." << std::endl;
            }
        }
        
        if (config["safety"]) {
            auto safety_node = config["safety"];

            if (safety_node["action_clip"]) {
                ACTION_CLIP = safety_node["action_clip"].as<float>();
                std::cout << "[Config] Loaded ACTION_CLIP = " << ACTION_CLIP << std::endl;
            }

            loadFloatArrayOrScalar(safety_node["target_pos_min"], TARGET_POS_MIN, "safety.target_pos_min");
            loadFloatArrayOrScalar(safety_node["target_pos_max"], TARGET_POS_MAX, "safety.target_pos_max");
            loadFloatArrayOrScalar(safety_node["pvt_max_vel"], PVT_MAX_VEL, "safety.pvt_max_vel");
        }
        normalizeSafetyConfig();

        // Load observation scales
        if (config["observation_scales"]) {
            auto obs_scales = config["observation_scales"];
            
            if (obs_scales["lin_vel_scale"]) {
                LIN_VEL_SCALE = obs_scales["lin_vel_scale"].as<float>();
                std::cout << "[Config] Loaded LIN_VEL_SCALE = " << LIN_VEL_SCALE << std::endl;
            }
            
            if (obs_scales["ang_vel_scale"]) {
                ANG_VEL_SCALE = obs_scales["ang_vel_scale"].as<float>();
                std::cout << "[Config] Loaded ANG_VEL_SCALE = " << ANG_VEL_SCALE << std::endl;
            }
            
            if (obs_scales["dof_pos_scale"]) {
                DOF_POS_SCALE = obs_scales["dof_pos_scale"].as<float>();
                std::cout << "[Config] Loaded DOF_POS_SCALE = " << DOF_POS_SCALE << std::endl;
            }
            
            if (obs_scales["dof_vel_scale"]) {
                DOF_VEL_SCALE = obs_scales["dof_vel_scale"].as<float>();
                std::cout << "[Config] Loaded DOF_VEL_SCALE = " << DOF_VEL_SCALE << std::endl;
            }
            
            if (obs_scales["cmd_scale"]) {
                auto cmd_scale_node = obs_scales["cmd_scale"];
                if (cmd_scale_node.size() >= 3) {
                    CMD_SCALE[0] = cmd_scale_node[0].as<float>();
                    CMD_SCALE[1] = cmd_scale_node[1].as<float>();
                    CMD_SCALE[2] = cmd_scale_node[2].as<float>();
                    std::cout << "[Config] Loaded CMD_SCALE = [" 
                              << CMD_SCALE[0] << ", " << CMD_SCALE[1] << ", " << CMD_SCALE[2] << "]" << std::endl;
                } else {
                    std::cerr << "[Config] Warning: cmd_scale has " << cmd_scale_node.size() 
                              << " elements, expected 3. Using default values." << std::endl;
                }
            }
        }

        if (config["observation_filter"] && config["observation_filter"]["cmd_alpha"]) {
            CMD_FILTER_ALPHA = config["observation_filter"]["cmd_alpha"].as<float>();
            if (CMD_FILTER_ALPHA < 0.0f) {
                CMD_FILTER_ALPHA = 0.0f;
            } else if (CMD_FILTER_ALPHA > 1.0f) {
                CMD_FILTER_ALPHA = 1.0f;
            }
            std::cout << "[Config] Loaded CMD_FILTER_ALPHA = " << CMD_FILTER_ALPHA << std::endl;
        }
        

        // Load thread configuration
        if(config["thread"]){
            auto th = config["thread"];
            if(th["ecat_cpu"])     ThreadEcatCpu    = th["ecat_cpu"].as<int>();
            if(th["control_cpu"])  ThreadControlCpu = th["control_cpu"].as<int>();
            if(th["control_priority"]) ThreadControlPri = th["control_priority"].as<int>();
            std::cout << "[Config] Thread: ecat_cpu=" << ThreadEcatCpu
                      << ", control_cpu=" << ThreadControlCpu
                      << ", priority=" << ThreadControlPri << std::endl;
        }

        // Load ankle solver config
        if(config["use_ankle_solver"]){
            USE_ANKLE_SOLVER = config["use_ankle_solver"].as<bool>();
            std::cout << "[Config] Loaded USE_ANKLE_SOLVER = " << (USE_ANKLE_SOLVER ? "true" : "false") << std::endl;
        }
        
        return true;
    } catch (const YAML::Exception& e) {
        std::cerr << "[Config] Error loading YAML file " << config_path << ": " << e.what() << std::endl;
        std::cerr << "[Config] Using default hardcoded values." << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "[Config] Error: " << e.what() << std::endl;
        std::cerr << "[Config] Using default hardcoded values." << std::endl;
        return false;
    }
}

// Maximum torque limits for each motor (N·m) - aligned with heima_noarm_config.py
const float MAX_TORQUE_LIMIT[13] = {
    120.0f, 80.0f, 120.0f, 130.0f, 120.0f, 120.0f,  // Left leg: hip_roll, hip_yaw, hip_pitch, knee, ankle_pitch, ankle_roll
    120.0f, 80.0f, 120.0f, 130.0f, 120.0f, 120.0f,  // Right leg: hip_roll, hip_yaw, hip_pitch, knee, ankle_pitch, ankle_roll
    130.0f  // Waist motors (if present)
};
// const float MAX_TORQUE_LIMIT[13] = {
//     3000.0f, 3000.0f, 3000.0f, 3000.0f, 3000.0f, 3000.0f,  // Left leg
//     3000.0f, 3000.0f, 3000.0f, 3000.0f, 3000.0f, 3000.0f,   // Right leg
//     400.0f, 400.0f, 400.0f
// };

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
        
    // 1. Commands (scaled) - 3 dims
    obs[idx++] = commands[0] * CMD_SCALE[0];
    obs[idx++] = commands[1] * CMD_SCALE[1];
    obs[idx++] = commands[2] * CMD_SCALE[2];

    // 2. Base angular velocity (scaled) - 3 dims
    obs[idx++] = base_ang_vel[0] * ANG_VEL_SCALE;
    obs[idx++] = base_ang_vel[1] * ANG_VEL_SCALE;
    obs[idx++] = base_ang_vel[2] * ANG_VEL_SCALE;
    
    // 3. Projected gravity - 3 dims
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
    std::string config_yaml = "checkpoints/v1/config.yaml";  // YAML config file path
    float cmd_vx_arg = 0.0f;
    float cmd_vy_arg = 0.0f;
    float cmd_wz_arg = 0.0f;
    
    // Parse command line arguments
    // Usage: ./main [mode] [onnx_file] [config] [config_yaml] [vx] [vy] [wz]
    //   mode: "sim" (default) or "real"
    //   config: for sim = "localhost:50051" (default), for real = "config.xml" (default)
    //   config_yaml: path to YAML config file (default: "checkpoints/v1/config.yaml")
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
        config_yaml = argv[4];
    }
    if (argc > 5) {
        cmd_vx_arg = std::stof(argv[5]);
    }
    if (argc > 6) {
        cmd_vy_arg = std::stof(argv[6]);
    }
    if (argc > 7) {
        cmd_wz_arg = std::stof(argv[7]);
    }
    
    // Set default config based on mode
    if (config.empty()) {
        if (mode == "sim" || mode == "simulator") {
            config = "localhost:50051";
        } else {
            config = "config.xml";
        }
    }
    
    // Load configuration from YAML file
    std::cout << "=== Loading Configuration ===" << std::endl;
    std::cout << "Config file: " << config_yaml << std::endl;
    if (!loadConfigFromYAML(config_yaml)) {
        std::cout << "[Config] Using default hardcoded values." << std::endl;
    }
    std::cout << "============================\n" << std::endl;
    
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    
    // Setup keyboard input
    setupKeyboard();
    
    std::cout << "=== Heima RL Deployment ===" << std::endl;
    std::cout << "Mode: " << mode << std::endl;
    std::cout << "ONNX model: " << onnx_file << std::endl;
    std::cout << "Config: " << config << std::endl;
    std::cout << "\n=== Keyboard Controls ===" << std::endl;
    std::cout << "W/S: Forward/Backward (vx = ±" << CMD_VEL_FORWARD_FIXED << ")" << std::endl;
    std::cout << "A/D: Left/Right (vy = ±" << CMD_VEL_LEFT_FIXED << ")" << std::endl;
    std::cout << "Q/E: Rotate Left/Right (wz = ±" << CMD_VEL_ROTATE_LEFT_FIXED << ")" << std::endl;
    std::cout << "Space: Stop all commands" << std::endl;
    std::cout << "Enter: Print current commands" << std::endl;
    std::cout << "Ctrl+C: Exit" << std::endl;
    std::cout << "========================\n" << std::endl;
    
    // Initialize robot interface (pass ecatCpu for SDK thread binding, opMode=5 for PVT)
    int opMode = (mode == "real" || mode == "robot") ? 5 : 10;  // PVT for real, CST for sim
    auto robot = createRobotInterface(mode, config, std::vector<unsigned short>(), ThreadEcatCpu, opMode);
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
    std::vector<float> joint_pos(13, 0.0f);  // 12 legs + 3 waist
    std::vector<float> joint_vel(13, 0.0f);  // 12 legs + 3 waist
    
    std::vector<float> observation(45);
    std::vector<float> actions(12, 0.0f);
    std::vector<float> previous_actions(12, 0.0f);
    std::vector<float> current_torques(12, 0.0f);  // Store current torque commands for logging
    
    // Initialize commands from command line arguments
    {
        std::lock_guard<std::mutex> lock(cmd_mutex);
        cmd_vx = cmd_vx_arg;  // Use command line values if provided
        cmd_vy = cmd_vy_arg;
        cmd_wz = cmd_wz_arg;
    }
    float commands[3] = {0.0f, 0.0f, 0.0f};
    float filtered_commands[3] = {cmd_vx_arg, cmd_vy_arg, cmd_wz_arg};

    // Open CSV file for logging observations
    std::ofstream csv_file("observations_log.csv");
    std::ofstream obs_log_file("observations.csv");
    obs_log_file << std::fixed << std::setprecision(6);
    std::ofstream actions_log_file("actions.csv");
    actions_log_file << std::fixed << std::setprecision(6);
    
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
        for (int i = 0; i < 12; ++i) {
            csv_file << "torque_" << i << ",";
        }
        for (int i = 0; i < 45; ++i) {
            csv_file << "obs_" << i;
            if (i < 44) csv_file << ",";
        }
        csv_file << "\n";
        csv_file << std::fixed << std::setprecision(6);
        std::cout << "CSV logging enabled: observations_log.csv" << std::endl;
    }

    // Stage 2: Move to default position using PVT (REAL ROBOT ONLY)
    if(mode == "real" || mode == "robot"){
        std::cout << "\nStage 2: Moving to default position (PVT mode)..." << std::endl;
        
        // Fetch initial state
        if(!robot->fetchObs(rpy, base_ang_vel, joint_pos, joint_vel)){
            std::cerr << "Failed to fetch initial state" << std::endl;
            return 1;
        }
        applyAnkleSolverInverse(joint_pos, ankle_solver);
        
        // Store initial positions for interpolation
        std::vector<float> initial_positions = joint_pos;
        std::vector<float> stage2_command_positions = initial_positions;
        
        const int interpolation_ticks = 5000;  // 5 seconds to move to default
        const int hold_ticks = 10000;          // 10 seconds hold at default
        int stage2_ticks = 0;
        
        // Prepare PVT vectors (13 motors)
        std::vector<float> tgtPos(13, 0.0f);
        std::vector<float> tgtVel(13, 0.0f);
        std::vector<float> tgtTor(13, 0.0f);
        std::vector<float> tgtKp(PD_KP, PD_KP +13);
        std::vector<float> tgtKd(PD_KD, PD_KD +13);
        
        while(!g_stop && stage2_ticks < (interpolation_ticks + hold_ticks)){
            // Fetch current state
            if(!robot->fetchObs(rpy, base_ang_vel, joint_pos, joint_vel)){
                std::cerr << "Failed to fetch state during initialization" << std::endl;
                return 1;
            }
            
            // Calculate interpolation factor (0 to 1)
            float alpha = 1.0f;
            if(stage2_ticks < interpolation_ticks){
                alpha = static_cast<float>(stage2_ticks) /static_cast<float>(interpolation_ticks);
            }
            
            // Interpolate target positions
            for(int i = 0; i < 13; ++i){
                if(i < 12){
                    tgtPos[i] = initial_positions[i]*(1.0f -alpha) +DEFAULT_JOINT_ANGLES[i]*alpha;
                }else{
                    tgtPos[i] = DEFAULT_JOINT_ANGLES[i];
                }
            }

            for(int i = 0; i < 13; ++i){
                tgtPos[i] = clampFloat(tgtPos[i], TARGET_POS_MIN[i], TARGET_POS_MAX[i]);
                float max_step = PVT_MAX_VEL[i] * 0.001f;
                stage2_command_positions[i] = stepTowardLimited(stage2_command_positions[i], tgtPos[i], max_step);
                tgtPos[i] = stage2_command_positions[i];
            }
            
            // Apply ankle solver transformation if enabled
            // (tgtPos is used as float[13] via data())
            float target_pos_arr[13];
            for(int i = 0; i < 13; ++i) target_pos_arr[i] = tgtPos[i];
            applyAnkleSolver(target_pos_arr, ankle_solver);
            for(int i = 0; i < 13; ++i) tgtPos[i] = target_pos_arr[i];
            
            // Send PVT command: driver does kp*(pos-actual) + kd*(0-vel) + 0
            robot->writePVT(tgtPos, tgtVel, tgtTor, tgtKp, tgtKd);
            
            if(stage2_ticks % 500 == 0){
                std::cout << "Moving to default... (" << stage2_ticks 
                          << "/" << (interpolation_ticks + hold_ticks) << ")" << std::endl;
            }
            
            stage2_ticks++;
            usleep(1000);  // 1ms
        }
        
        std::cout << "Default position reached and stabilized!" << std::endl;
    }else{
        std::cout << "\nSimulator mode: Skipping default position initialization" << std::endl;
    }

    // Main control loop
    // Policy runs at 50Hz (matching training), communication at 1kHz
    std::cout << "\nStarting control loop..." << std::endl;
    std::cout << "Policy frequency: 50 Hz (20ms)" << std::endl;
    std::cout << "Communication frequency: 1 kHz (1ms)" << std::endl;
    std::cout << "Press Ctrl+C to stop\n" << std::endl;
    
    // Set main control loop CPU affinity and real-time priority
    if(mode == "real" || mode == "robot"){
        // CPU affinity
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(ThreadControlCpu, &cpuset);
        if(sched_setaffinity(0, sizeof(cpuset), &cpuset) == 0){
            std::cout << "[Thread] Control loop bound to CPU " << ThreadControlCpu << std::endl;
        }else{
            std::cerr << "[Thread] Warning: Failed to set CPU affinity (need root?)" << std::endl;
        }
        
        // Real-time priority (SCHED_FIFO)
        struct sched_param param;
        param.sched_priority = ThreadControlPri;
        if(sched_setscheduler(0, SCHED_FIFO, &param) == 0){
            std::cout << "[Thread] Control loop set to SCHED_FIFO priority " << ThreadControlPri << std::endl;
        }else{
            std::cerr << "[Thread] Warning: Failed to set RT priority (need root?)" << std::endl;
        }
    }
    
    auto loop_start = std::chrono::steady_clock::now();
    int loop_count = 0;
    int policy_count = 0;
    
    // Policy update timing (50 Hz = 20ms)
    const int POLICY_PERIOD_US = 20000;  // 20ms in microseconds
    const auto policy_period = std::chrono::microseconds(POLICY_PERIOD_US);
    auto last_policy_update = std::chrono::steady_clock::now();
    
    // PD control update timing — only used for sim/CST mode (200 Hz = 5ms)
    const int PD_UPDATE_PERIOD_MS = 5;  // 5ms = 5 iterations at 1kHz
    int pd_update_counter = 0;
    
    // PVT mode vectors (reusable, 13 motors)
    std::vector<float> pvtPosTarget(13, 0.0f);   // Step target from 50Hz RL action
    std::vector<float> pvtPosPrevious(13, 0.0f); // Previous target for interpolation
    std::vector<float> pvtPosInterpolated(13, 0.0f); // Smoothed target applied to hardware
    std::vector<float> pvtVel(13, 0.0f);
    std::vector<float> pvtTor(13, 0.0f);
    std::vector<float> pvtKp(PD_KP, PD_KP +13);
    std::vector<float> pvtKd(PD_KD, PD_KD +13);
    
    // Fetch state once to prime the filters and targets
    if (!robot->fetchObs(rpy, base_ang_vel, joint_pos, joint_vel)) {
        std::cerr << "Failed to fetch initial observations" << std::endl;
        return 1;
    }
    
    // Initialize interpolated and previous POS to current actual joint state to avoid immediate jump
    for(int i = 0; i < 13; ++i) {
        pvtPosInterpolated[i] = joint_pos[i];
        pvtPosPrevious[i] = joint_pos[i];
        pvtPosTarget[i] = joint_pos[i];
    }

    while (!g_stop) {
        auto cycle_start = std::chrono::steady_clock::now();
        
        // Update commands from keyboard input
        updateCommandsFromKeyboard();
        
        // Get current commands (thread-safe)
        {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            commands[0] = cmd_vx;
            commands[1] = cmd_vy;
            commands[2] = cmd_wz;
        }
        
        // Read sensors (every 1ms for high-frequency feedback)
        if (!robot->fetchObs(rpy, base_ang_vel, joint_pos, joint_vel)) {
            std::cerr << "Failed to fetch observations" << std::endl;
            break;
        }

        
        // Check if it's time to update policy (50 Hz)
        auto now = std::chrono::steady_clock::now();
        auto time_since_policy = std::chrono::duration_cast<std::chrono::microseconds>(
            now - last_policy_update).count();
        bool policy_due = time_since_policy >= POLICY_PERIOD_US;
        
        if (policy_due) {
            for (int i = 0; i < 3; ++i) {
                filtered_commands[i] = (1.0f - CMD_FILTER_ALPHA) * filtered_commands[i] + CMD_FILTER_ALPHA * commands[i];
            }

            // Apply inverse ankle solver to observations (convert motor angles back to virtual ankle angles)
            std::vector<float> corrected_joint_pos = joint_pos;
            applyAnkleSolverInverse(corrected_joint_pos, ankle_solver);
            
            // Build observation with corrected joint positions
            buildObservation(rpy, base_ang_vel, corrected_joint_pos, joint_vel, filtered_commands, previous_actions, observation);
            
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
                csv_file << filtered_commands[0] << "," << filtered_commands[1] << "," << filtered_commands[2] << ",";
                for (int i = 0; i < 12; ++i) {
                    csv_file << previous_actions[i] << ",";
                }
                for (int i = 0; i < 12; ++i) {
                    csv_file << current_torques[i] << ",";
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

            clipActionsInPlace(actions);

            if (obs_log_file.is_open()) {
                for (int i = 0; i < 45; ++i) {
                    obs_log_file << observation[i];
                    if (i < 44) obs_log_file << ",";
                }
                obs_log_file << "\n";
            }

            if (actions_log_file.is_open()) {
                for (int i = 0; i < 12; ++i) {
                    actions_log_file << actions[i];
                    if (i < 11) actions_log_file << ",";
                }
                actions_log_file << "\n";
            }
            
            // Update previous actions
            previous_actions = actions;

            while (time_since_policy >= POLICY_PERIOD_US) {
                last_policy_update += policy_period;
                time_since_policy -= POLICY_PERIOD_US;
            }
            policy_count++;
        }
        
        // --- PVT mode (real robot): send targets directly to driver ---
        if(mode == "real" || mode == "robot"){
            // Calculate interpolation factor (0.0 to 1.0)
            auto time_since_update_us = std::chrono::duration_cast<std::chrono::microseconds>(
                now - last_policy_update).count();
            float interpolation_alpha = static_cast<float>(time_since_update_us) / POLICY_PERIOD_US;
            if (interpolation_alpha > 1.0f) interpolation_alpha = 1.0f;
            
            // At the exact moment the policy updates (time_since_policy >= POLICY_PERIOD_US block above),
            // we compute new pvtPosTarget. We must save the old target to pvtPosPrevious to interpolate.
            if(policy_due) {
                pvtPosPrevious = pvtPosInterpolated;  // Start from actual reached position for zero-jump continuity
                
                // Compute new target positions from actions (updated at 50Hz)
                float target_pos_arr[13];
                buildSafeTargetPositions(actions, target_pos_arr);
                
                // Apply ankle solver transformation if enabled
                applyAnkleSolver(target_pos_arr, ankle_solver);
                for(int i = 0; i < 13; ++i) pvtPosTarget[i] = target_pos_arr[i];
                
                // Reset alpha for the very first frame of the new update
                interpolation_alpha = 0.0f;
            }

            // Apply Linear Interpolation between previous target and new target
            for(int i = 0; i < 13; ++i){
                float interpolated_target = pvtPosPrevious[i] * (1.0f - interpolation_alpha) + pvtPosTarget[i] * interpolation_alpha;
                float max_step = PVT_MAX_VEL[i] * 0.001f;
                pvtPosInterpolated[i] = stepTowardLimited(pvtPosInterpolated[i], interpolated_target, max_step);
            }
            
            // Store zeros for torque logging (driver computes actual torques internally)
            for(int i = 0; i < 12; ++i){
                current_torques[i] = 0.0f;
            }
            
            // Send PVT command every cycle — driver does PD internally:
            // torque = kp*(pos-actPos) + kd*(vel-actVel) + tor
            robot->writePVT(pvtPosInterpolated, pvtVel, pvtTor, pvtKp, pvtKd);
        }else{
            // --- Sim mode (CST): retain original local PD torque control ---
            // Recalculate PD control every 5ms (200 Hz)
            if(pd_update_counter == 0){
                // Convert actions to joint targets and compute PD torque commands
                float target_pos[13] = {0.0f};
                buildSafeTargetPositions(actions, target_pos);

                // Apply ankle solver transformation if enabled
                applyAnkleSolver(target_pos, ankle_solver);

                // Prepare torque commands for 12 leg joints + 3 waist motors
                std::vector<float> torques(13, 0.0f);
                
                // Leg motors (0-11)
                for(int i = 0; i < 12; ++i){
                    float actual_pos = joint_pos[i];
                    float target_vel = 0.0f;
                    float actual_vel = joint_vel[i];
                    
                    // Compute PD torque command
                    float torqueCmd = pdControl(target_pos[i], actual_pos, target_vel, actual_vel, PD_KP[i], PD_KD[i]);
                    
                    // Safety clip
                    if(torqueCmd > MAX_TORQUE_LIMIT[i]){
                        torqueCmd = MAX_TORQUE_LIMIT[i];
                    }else if(torqueCmd < -MAX_TORQUE_LIMIT[i]){
                        torqueCmd = -MAX_TORQUE_LIMIT[i];
                    }
                    
                    torques[i] = torqueCmd;
                }
                
                // Waist motors (12-14)
                for(int i = 12; i < 13; ++i){
                    float target = DEFAULT_JOINT_ANGLES[i];
                    float actual = joint_pos[i];
                    float actual_vel = joint_vel[i];
                    float torqueCmd = pdControl(target, actual, 0.0f, actual_vel, PD_KP[i], PD_KD[i]);
                    
                    if(torqueCmd > MAX_TORQUE_LIMIT[i]){
                        torqueCmd = MAX_TORQUE_LIMIT[i];
                    }else if(torqueCmd < -MAX_TORQUE_LIMIT[i]){
                        torqueCmd = -MAX_TORQUE_LIMIT[i];
                    }
                    
                    torques[i] = torqueCmd;
                }
                
                // Store torques for logging
                for(int i = 0; i < 12; ++i){
                    current_torques[i] = torques[i];
                }
                
                // Send torques to robot
                robot->writeTorque(torques);
            }
            
            // Update counter for PD control recalculation (every 5ms)
            pd_update_counter++;
            if(pd_update_counter >= PD_UPDATE_PERIOD_MS){
                pd_update_counter = 0;
            }
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
    restoreKeyboard();
    robot->shutdown();
    
    // Close CSV file
    if (csv_file.is_open()) {
        csv_file.close();
        std::cout << "CSV log saved: observations_log.csv (" << policy_count << " samples @ 50Hz)" << std::endl;
    }

    if (obs_log_file.is_open()) {
        obs_log_file.close();
        std::cout << "Observations log saved: observations.csv (" << policy_count << " samples @ 50Hz)" << std::endl;
    }
    if (actions_log_file.is_open()) {
        actions_log_file.close();
        std::cout << "Actions log saved: actions.csv (" << policy_count << " samples @ 50Hz)" << std::endl;
    }
    
    std::cout << "Done." << std::endl;
    return 0;
}

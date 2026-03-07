/*
 * Heima RL Deployment — CSP Position Mode
 * 
 * Same as main.cpp but uses CSP (Cyclic Synchronous Position) mode.
 * RL inference outputs target positions → sent directly to driver,
 * driver's internal position loop handles tracking.
 * No software PD control needed.
 *
 * Usage: sudo ./heima_sim_deploy_csp real onnx_file config.xml config.yaml [vx vy wz]
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
void handleSignal(int){
    g_stop = 1;
}

// Keyboard input handling
struct termios old_termios;
bool keyboard_enabled = false;

void setupKeyboard(){
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

void restoreKeyboard(){
    if(keyboard_enabled){
        tcsetattr(STDIN_FILENO, TCSANOW, &old_termios);
    }
}

// Speed command constants
const float CMD_VEL_FORWARD_FIXED = 0.3f;
const float CMD_VEL_LEFT_FIXED = 0.2f;
const float CMD_VEL_ROTATE_LEFT_FIXED = 0.3f;

// Thread-safe command variables
std::mutex cmd_mutex;
float cmd_vx = 0.0f, cmd_vy = 0.0f, cmd_wz = 0.0f;

void updateCommandsFromKeyboard(){
    char ch;
    while(read(STDIN_FILENO, &ch, 1) > 0){
        std::lock_guard<std::mutex> lock(cmd_mutex);
        switch(ch){
            case 'w': case 'W': cmd_vx = CMD_VEL_FORWARD_FIXED; break;
            case 's': case 'S': cmd_vx = -CMD_VEL_FORWARD_FIXED; break;
            case 'a': case 'A': cmd_vy = CMD_VEL_LEFT_FIXED; break;
            case 'd': case 'D': cmd_vy = -CMD_VEL_LEFT_FIXED; break;
            case 'q': case 'Q': cmd_wz = CMD_VEL_ROTATE_LEFT_FIXED; break;
            case 'e': case 'E': cmd_wz = -CMD_VEL_ROTATE_LEFT_FIXED; break;
            case ' ': cmd_vx = 0; cmd_vy = 0; cmd_wz = 0; break;
        }
    }
}

// Observation builder (same as main.cpp)
void buildObservation(
    const std::vector<float>& rpy,
    const std::vector<float>& base_ang_vel,
    const std::vector<float>& joint_pos,
    const std::vector<float>& joint_vel,
    const float commands[3],
    const std::vector<float>& previous_actions,
    std::vector<float>& observation,
    const float* DEFAULT_JOINT_ANGLES)
{
    // [0-2] commands
    observation[0] = commands[0]; // vx
    observation[1] = commands[1]; // vy
    observation[2] = commands[2]; // wz

    // [3-5] angular velocity
    observation[3] = base_ang_vel[0];
    observation[4] = base_ang_vel[1];
    observation[5] = base_ang_vel[2];

    // [6-8] projected gravity (from RPY)
    float roll = rpy[0], pitch = rpy[1];
    observation[6] = -std::sin(pitch);
    observation[7] = std::sin(roll) * std::cos(pitch);
    observation[8] = std::cos(roll) * std::cos(pitch);

    // [9-20] joint positions - default angles (12 joints)
    for(int i=0; i<12; ++i){
        observation[9+i] = joint_pos[i] - DEFAULT_JOINT_ANGLES[i];
    }

    // [21-32] joint velocities (12 joints)
    for(int i=0; i<12; ++i){
        observation[21+i] = joint_vel[i];
    }

    // [33-44] previous actions
    for(int i=0; i<12; ++i){
        observation[33+i] = previous_actions[i];
    }
}

// Ankle solver helpers (same as main.cpp)
bool USE_ANKLE_SOLVER = true;

void applyAnkleSolver(float* target_pos, AnkleSolver& solver){
    if(!USE_ANKLE_SOLVER) return;
    // Left ankle (indices 4,5)
    auto [l_motor_p, l_motor_r] = solver.solve(target_pos[4], target_pos[5]);
    target_pos[4] = static_cast<float>(l_motor_p);
    target_pos[5] = static_cast<float>(l_motor_r);
    // Right ankle (indices 10,11)
    auto [r_motor_p, r_motor_r] = solver.solve(target_pos[10], target_pos[11]);
    target_pos[10] = static_cast<float>(r_motor_p);
    target_pos[11] = static_cast<float>(r_motor_r);
}

void applyAnkleSolverInverse(std::vector<float>& joint_pos, AnkleSolver& solver){
    if(!USE_ANKLE_SOLVER) return;
    auto [l_virt_p, l_virt_r] = solver.inverse(joint_pos[4], joint_pos[5]);
    joint_pos[4] = static_cast<float>(l_virt_p);
    joint_pos[5] = static_cast<float>(l_virt_r);
    auto [r_virt_p, r_virt_r] = solver.inverse(joint_pos[10], joint_pos[11]);
    joint_pos[10] = static_cast<float>(r_virt_p);
    joint_pos[11] = static_cast<float>(r_virt_r);
}

// ============= Configurable parameters =============

float DEFAULT_JOINT_ANGLES[15] = {
    0.0f, 0.0f, -0.44f, 0.88f, -0.44f, 0.0f,   // L: hip_roll, hip_yaw, hip_pitch, knee, ankle_pitch, ankle_roll
    0.0f, 0.0f, -0.44f, 0.88f, -0.44f, 0.0f,   // R
    0.0f, 0.0f, 0.0f                              // Waist
};

float ACTION_SCALE[12] = {
    0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f,
    0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f
};

// Thread config
int ThreadEcatCpu    = 1;
int ThreadControlCpu = 2;
int ThreadControlPri = 88;

// Per-motor maxCurrent
unsigned short MAX_CURRENT_SDK[15] = {
    1000, 1000, 1000, 1000, 1000, 1000,
    1000, 1000, 1000, 1000, 1000, 1000,
    1000, 1000, 1000
};

// ============= YAML config loader =============

bool loadConfigFromYAML(const std::string& config_path){
    try{
        auto config = YAML::LoadFile(config_path);
        std::cout << "[Config] Loading CSP config from: " << config_path << std::endl;

        if(config["default_joint_angles"]){
            auto angles = config["default_joint_angles"];
            for(int i=0; i<std::min(15, (int)angles.size()); ++i){
                DEFAULT_JOINT_ANGLES[i] = angles[i].template as<float>();
            }
        }
        if(config["action_scale"]){
            auto scales = config["action_scale"];
            for(int i=0; i<std::min(12, (int)scales.size()); ++i){
                ACTION_SCALE[i] = scales[i].template as<float>();
            }
        }
        if(config["max_current"]){
            auto mc = config["max_current"];
            for(int i=0; i<std::min(15, (int)mc.size()); ++i){
                MAX_CURRENT_SDK[i] = mc[i].template as<unsigned short>();
            }
        }
        if(config["thread"]){
            auto th = config["thread"];
            if(th["ecat_cpu"])     ThreadEcatCpu    = th["ecat_cpu"].as<int>();
            if(th["control_cpu"])  ThreadControlCpu = th["control_cpu"].as<int>();
            if(th["control_priority"]) ThreadControlPri = th["control_priority"].as<int>();
        }
        if(config["use_ankle_solver"]){
            USE_ANKLE_SOLVER = config["use_ankle_solver"].as<bool>();
        }

        return true;
    }catch(const YAML::Exception& e){
        std::cerr << "[Config] Error loading YAML: " << e.what() << std::endl;
        return false;
    }
}

// ============= MAIN =============

int main(int argc, char** argv){
    // Parse arguments (same order as main.cpp)
    std::string mode = "real";  // CSP only makes sense for real robot
    std::string onnx_file = "policy.onnx";
    std::string config = "";
    std::string config_yaml = "checkpoints/v1/config.yaml";
    float cmd_vx_arg = 0.0f, cmd_vy_arg = 0.0f, cmd_wz_arg = 0.0f;

    if(argc > 1) mode = argv[1];
    if(argc > 2) onnx_file = argv[2];
    if(argc > 3) config = argv[3];
    if(argc > 4) config_yaml = argv[4];
    if(argc > 5) cmd_vx_arg = std::stof(argv[5]);
    if(argc > 6) cmd_vy_arg = std::stof(argv[6]);
    if(argc > 7) cmd_wz_arg = std::stof(argv[7]);

    if(config.empty()){
        config = (mode == "real" || mode == "robot") ? "config.xml" : "localhost:50051";
    }

    // Load YAML config
    loadConfigFromYAML(config_yaml);

    // Print config summary
    std::cout << "\n=== Heima RL Deployment (CSP Position Mode) ===" << std::endl;
    std::cout << "Mode: " << mode << " (CSP — driver position loop)" << std::endl;
    std::cout << "ONNX model: " << onnx_file << std::endl;
    std::cout << "Config: " << config << std::endl;
    std::cout << "Ankle solver: " << (USE_ANKLE_SOLVER ? "ENABLED" : "DISABLED") << std::endl;

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    setupKeyboard();

    std::cout << "\n=== Keyboard Controls ===" << std::endl;
    std::cout << "W/S: Forward/Backward | A/D: Left/Right | Q/E: Rotate | Space: Stop" << std::endl;

    // Create robot interface with CSP mode (useCSP=true)
    std::vector<unsigned short> maxCurrentVec(MAX_CURRENT_SDK, MAX_CURRENT_SDK + 15);
    auto robot = createRobotInterface(mode, config, maxCurrentVec, ThreadEcatCpu, true);
    if(!robot){
        std::cerr << "Failed to create robot interface" << std::endl;
        return 1;
    }
    if(!robot->init()){
        std::cerr << "Failed to initialize robot interface" << std::endl;
        return 1;
    }

    // Initialize ankle solver
    AnkleSolver ankle_solver;

    // Initialize ONNX model
    std::cout << "\nLoading ONNX model..." << std::endl;
    OnnxWrapper onnx;
    if(!onnx.init(onnx_file)){
        std::cerr << "Failed to load ONNX model: " << onnx_file << std::endl;
        return 1;
    }
    if(onnx.getInputSize() != 45 || onnx.getOutputSize() != 12){
        std::cerr << "Error: Model must have input=45, output=12" << std::endl;
        return 1;
    }

    // Prepare data structures
    std::vector<float> rpy(3, 0.0f);
    std::vector<float> base_ang_vel(3, 0.0f);
    std::vector<float> joint_pos(15, 0.0f);
    std::vector<float> joint_vel(15, 0.0f);
    std::vector<float> observation(45);
    std::vector<float> actions(12, 0.0f);
    std::vector<float> previous_actions(12, 0.0f);
    float commands[3] = {cmd_vx_arg, cmd_vy_arg, cmd_wz_arg};

    {
        std::lock_guard<std::mutex> lock(cmd_mutex);
        cmd_vx = cmd_vx_arg;
        cmd_vy = cmd_vy_arg;
        cmd_wz = cmd_wz_arg;
    }

    // Open log files
    std::ofstream obs_log("observations_csp.csv");
    std::ofstream act_log("actions_csp.csv");
    obs_log << std::fixed << std::setprecision(6);
    act_log << std::fixed << std::setprecision(6);

    // ========== Stage 2: Move to default position (real robot only) ==========
    if(mode == "real" || mode == "robot"){
        std::cout << "\nStage 2: Moving to default position (CSP)..." << std::endl;

        if(!robot->fetchObs(rpy, base_ang_vel, joint_pos, joint_vel)){
            std::cerr << "Failed to fetch initial state" << std::endl;
            return 1;
        }
        applyAnkleSolverInverse(joint_pos, ankle_solver);

        std::vector<float> initial_positions = joint_pos;
        const int interpolation_ticks = 5000;  // 5s
        const int hold_ticks = 10000;          // 10s
        int tick = 0;

        while(!g_stop && tick < (interpolation_ticks + hold_ticks)){
            robot->fetchObs(rpy, base_ang_vel, joint_pos, joint_vel);

            float alpha = (tick < interpolation_ticks)
                ? static_cast<float>(tick) / interpolation_ticks
                : 1.0f;

            // Interpolate to default position
            std::vector<float> target(15);
            for(int i=0; i<15; ++i){
                if(i < 12){
                    target[i] = initial_positions[i]*(1.0f - alpha) + DEFAULT_JOINT_ANGLES[i]*alpha;
                }else{
                    target[i] = DEFAULT_JOINT_ANGLES[i];
                }
            }

            // Apply ankle solver
            float target_arr[15];
            for(int i=0; i<15; ++i) target_arr[i] = target[i];
            applyAnkleSolver(target_arr, ankle_solver);
            for(int i=0; i<15; ++i) target[i] = target_arr[i];

            // Send position directly (CSP)
            robot->writePosition(target);

            if(tick % 500 == 0){
                std::cout << "Moving to default... (" << tick
                          << "/" << (interpolation_ticks + hold_ticks) << ")" << std::endl;
            }

            tick++;
            usleep(1000);  // 1ms
        }
        std::cout << "Default position reached!" << std::endl;
    }

    // ========== Set RT thread (real robot only) ==========
    if(mode == "real" || mode == "robot"){
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(ThreadControlCpu, &cpuset);
        if(sched_setaffinity(0, sizeof(cpuset), &cpuset) == 0){
            std::cout << "[Thread] Bound to CPU " << ThreadControlCpu << std::endl;
        }else{
            std::cerr << "[Thread] Warning: CPU affinity failed (need root?)" << std::endl;
        }

        struct sched_param param;
        param.sched_priority = ThreadControlPri;
        if(sched_setscheduler(0, SCHED_FIFO, &param) == 0){
            std::cout << "[Thread] SCHED_FIFO priority " << ThreadControlPri << std::endl;
        }else{
            std::cerr << "[Thread] Warning: RT priority failed (need root?)" << std::endl;
        }
    }

    // ========== Main control loop ==========
    std::cout << "\nStarting CSP control loop..." << std::endl;
    std::cout << "Policy: 50 Hz | Position update: 50 Hz (same freq, no PD needed)" << std::endl;
    std::cout << "Press Ctrl+C to stop\n" << std::endl;

    auto loop_start = std::chrono::steady_clock::now();
    int loop_count = 0;
    int policy_count = 0;

    const int POLICY_PERIOD_US = 20000;  // 20ms = 50Hz
    auto last_policy_update = std::chrono::steady_clock::now();

    // Current target positions (start at default)
    std::vector<float> current_target(15, 0.0f);
    for(int i=0; i<15; ++i) current_target[i] = DEFAULT_JOINT_ANGLES[i];

    while(!g_stop){
        auto cycle_start = std::chrono::steady_clock::now();

        updateCommandsFromKeyboard();
        {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            commands[0] = cmd_vx;
            commands[1] = cmd_vy;
            commands[2] = cmd_wz;
        }

        // Read sensors
        if(!robot->fetchObs(rpy, base_ang_vel, joint_pos, joint_vel)){
            std::cerr << "Failed to fetch observations" << std::endl;
            break;
        }

        // Check if it's time to update policy (50 Hz)
        auto now = std::chrono::steady_clock::now();
        auto time_since_policy = std::chrono::duration_cast<std::chrono::microseconds>(
            now - last_policy_update).count();

        if(time_since_policy >= POLICY_PERIOD_US){
            // Apply inverse ankle solver to observations
            std::vector<float> corrected_pos = joint_pos;
            applyAnkleSolverInverse(corrected_pos, ankle_solver);

            // Build observation (45-dim)
            buildObservation(rpy, base_ang_vel, corrected_pos, joint_vel,
                           commands, previous_actions, observation, DEFAULT_JOINT_ANGLES);

            // Run RL inference
            if(!onnx.run(observation, actions)){
                std::cerr << "Inference failed!" << std::endl;
                break;
            }

            // Log
            if(obs_log.is_open()){
                for(int i=0; i<45; ++i){
                    obs_log << observation[i];
                    if(i < 44) obs_log << ",";
                }
                obs_log << "\n";
            }
            if(act_log.is_open()){
                for(int i=0; i<12; ++i){
                    act_log << actions[i];
                    if(i < 11) act_log << ",";
                }
                act_log << "\n";
            }

            // Compute target positions: DEFAULT + actions * ACTION_SCALE
            float target_pos[15];
            for(int i=0; i<15; ++i){
                target_pos[i] = DEFAULT_JOINT_ANGLES[i];
                if(i < 12){
                    target_pos[i] += actions[i] * ACTION_SCALE[i];
                }
            }

            // Apply ankle solver
            applyAnkleSolver(target_pos, ankle_solver);

            // Update current target
            for(int i=0; i<15; ++i) current_target[i] = target_pos[i];

            previous_actions = actions;
            last_policy_update = now;
            policy_count++;
        }

        // Send current target position to driver (CSP) every 1ms
        robot->writePosition(current_target);

        // Statistics
        loop_count++;
        if(loop_count % 100 == 0){
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - loop_start).count();
            float avg_freq = 1000.0f * loop_count / elapsed;
            float policy_freq = 1000.0f * policy_count / elapsed;

            std::cout << "Loop: " << loop_count
                      << " | Comm: " << avg_freq << " Hz"
                      << " | Policy: " << policy_freq << " Hz"
                      << " | R:" << rpy[0] << " P:" << rpy[1] << " Y:" << rpy[2]
                      << std::endl;
        }

        // Maintain 1kHz loop
        auto cycle_end = std::chrono::steady_clock::now();
        auto cycle_us = std::chrono::duration_cast<std::chrono::microseconds>(
            cycle_end - cycle_start).count();
        int sleep_us = 1000 - cycle_us;
        if(sleep_us > 0) usleep(sleep_us);
    }

    // ========== Shutdown ==========
    std::cout << "\nShutting down..." << std::endl;
    restoreKeyboard();
    robot->shutdown();

    if(obs_log.is_open()) obs_log.close();
    if(act_log.is_open()) act_log.close();

    std::cout << "Done. (" << policy_count << " policy steps)" << std::endl;
    return 0;
}

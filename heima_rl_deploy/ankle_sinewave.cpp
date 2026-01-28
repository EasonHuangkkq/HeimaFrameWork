/*
 * ankle_sinewave.cpp
 * 
 * Program to control ankle motors with sine wave pitch/roll commands
 * 
 * Usage: sudo ./ankle_sinewave [leg] [config.xml] [pitch_amplitude] [roll_amplitude] [frequency]
 *   leg: which leg to control (0=right, 1=left, default: 0)
 *   config.xml: path to config file (default: ../heimaSDK_copy/config.xml)
 *   pitch_amplitude: amplitude in radians for pitch sine wave (default: 0.2)
 *   roll_amplitude: amplitude in radians for roll sine wave (default: 0.2)
 *   frequency: frequency in Hz for both sine waves (default: 0.1)
 * 
 * This program:
 * 1. Initializes the ankle solver
 * 2. Generates sine wave pitch/roll commands
 * 3. Uses the solver to compute motor angles from pitch/roll
 * 4. Controls ankle motors using PD control (CST mode)
 */

#include "heima_driver_sdk.h"
#include "ankle_solver/cpp/ankle_solver.h"
#include <iostream>
#include <vector>
#include <csignal>
#include <unistd.h>
#include <cmath>
#include <chrono>

volatile sig_atomic_t g_stop = 0;
void handleSignal(int) {
    g_stop = 1;
}

// PD gains for ankle motors
const float PD_KP_PITCH = 50.0f;
const float PD_KD_PITCH = 1.0f;
const float PD_KP_ROLL = 50.0f;
const float PD_KD_ROLL = 1.0f;

// Maximum torque limits (N·m)
const float MAX_TORQUE_LIMIT = 40.0f;

// PD Control function
float pdControl(float targetPos, float actualPos, float targetVel, float actualVel, float kp, float kd) {
    float posError = targetPos - actualPos;
    float velError = targetVel - actualVel;
    float torque = kp * posError + kd * velError;
    return torque;
}

int main(int argc, char** argv) {
    // Parse arguments
    int leg = 0;  // 0 = right leg, 1 = left leg
    if (argc > 1) {
        leg = std::stoi(argv[1]);
        if (leg != 0 && leg != 1) {
            std::cerr << "Error: leg must be 0 (right) or 1 (left)" << std::endl;
            return 1;
        }
    }
    
    std::string configPath = "../heimaSDK_copy/config.xml";
    if (argc > 2) {
        configPath = argv[2];
    }
    
    float pitch_amplitude = 0.4f;  // radians (~11.5 degrees)
    if (argc > 3) {
        pitch_amplitude = std::stof(argv[3]);
    }
    
    float roll_amplitude = 0.4f;  // radians (~11.5 degrees)
    if (argc > 4) {
        roll_amplitude = std::stof(argv[4]);
    }
    
    float frequency = 3.0f;  // Hz
    if (argc > 5) {
        frequency = std::stof(argv[5]);
    }
    
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    
    std::cout << "=== Ankle Sine Wave Control ===" << std::endl;
    std::cout << "Config: " << configPath << std::endl;
    std::cout << "Leg: " << (leg == 0 ? "Right" : "Left") << std::endl;
    std::cout << "Pitch amplitude: " << pitch_amplitude << " rad (" << (pitch_amplitude * 180.0f / M_PI) << " deg)" << std::endl;
    std::cout << "Roll amplitude: " << roll_amplitude << " rad (" << (roll_amplitude * 180.0f / M_PI) << " deg)" << std::endl;
    std::cout << "Frequency: " << frequency << " Hz" << std::endl;
    
    // Initialize ankle solver
    AnkleSolver ankle_solver;
    std::cout << "\nAnkle solver initialized" << std::endl;
    
    // Initialize SDK
    DriverSDK::DriverSDK& sdk = DriverSDK::DriverSDK::instance();
    std::cout << "\nInitializing SDK..." << std::endl;
    sdk.init(configPath.c_str());
    
    const int motorCount = sdk.getTotalMotorNr();
    std::cout << "Total motors: " << motorCount << std::endl;
    
    // Ankle motor indices: 
    // Right leg: motors 4 (ankle pitch), 5 (ankle roll)
    // Left leg: motors 10 (ankle pitch), 11 (ankle roll)
    int ankle_pitch_idx = leg == 0 ? 4 : 10;
    int ankle_roll_idx = leg == 0 ? 5 : 11;
    
    if (ankle_pitch_idx >= motorCount || ankle_roll_idx >= motorCount) {
        std::cerr << "Error: Ankle motor indices out of range" << std::endl;
        return 1;
    }
    
    std::cout << "Ankle pitch motor index: " << ankle_pitch_idx << std::endl;
    std::cout << "Ankle roll motor index: " << ankle_roll_idx << std::endl;
    
    // Set operating mode: 10 = CST (Cyclic Synchronous Torque) for PD control
    std::vector<char> modes(motorCount, 10);
    sdk.setMode(modes);
    
    // Prepare motor data structures
    std::vector<DriverSDK::motorTargetStruct> targets(motorCount);
    std::vector<DriverSDK::motorActualStruct> actuals(motorCount);
    
    // Initialize all motors (disabled initially)
    for (int i = 0; i < motorCount; ++i) {
        targets[i].pos = 0.0f;
        targets[i].vel = 0.0f;
        targets[i].tor = 0.0f;
        targets[i].enabled = 0;
    }
    
    // Wait for ankle motors to reach operational state
    std::cout << "\nWaiting for ankle motors to be operational..." << std::endl;
    const int maxWaitTicks = 10000;
    int waitTicks = 0;
    bool motorsOperational = false;
    
    while (!g_stop && !motorsOperational && waitTicks < maxWaitTicks) {
        sdk.getMotorActual(actuals);
        
        // Check if both ankle motors are operational
        bool pitch_operational = (actuals[ankle_pitch_idx].statusWord & 0x007f) == 0x0031;
        bool roll_operational = (actuals[ankle_roll_idx].statusWord & 0x007f) == 0x0031;
        
        if (pitch_operational && roll_operational) {
            motorsOperational = true;
            std::cout << "Ankle motors are operational!" << std::endl;
        }
        
        if (waitTicks % 500 == 0) {
            std::cout << "  Pitch motor status: 0x" << std::hex << actuals[ankle_pitch_idx].statusWord 
                      << std::dec << " | Roll motor status: 0x" << std::hex 
                      << actuals[ankle_roll_idx].statusWord << std::dec << " (waiting...)" << std::endl;
        }
        
        waitTicks++;
        usleep(1000); // 1ms
    }
    
    if (!motorsOperational) {
        std::cerr << "Warning: Ankle motors did not reach operational state" << std::endl;
    }
    
    // Enable ankle motors
    std::cout << "\nEnabling ankle motors..." << std::endl;
    targets[ankle_pitch_idx].enabled = 1;
    targets[ankle_roll_idx].enabled = 1;
    
    // Control loop: sine wave pitch/roll commands
    std::cout << "\nStarting sine wave control loop (Ctrl+C to stop)..." << std::endl;
    
    const float dt = 0.001f;  // 1ms = 1kHz
    float t = 0.0f;
    
    int loopCount = 0;
    auto loop_start = std::chrono::steady_clock::now();
    
    while (!g_stop) {
        auto cycle_start = std::chrono::steady_clock::now();
        
        // Generate sine wave pitch/roll commands
        double pitch = pitch_amplitude * std::sin(2.0 * M_PI * frequency * t);
        // double pitch = 0.0;
        // double roll = roll_amplitude * std::sin(2.0 * M_PI * frequency * t);
        double roll = 0.0;
        
        // Solve for motor angles using ankle solver
        // Note: solver expects negative pitch/roll based on main.cpp usage
        auto ankle_angles = ankle_solver.solve(-pitch, -roll);
        double theta_e = ankle_angles.second;  // corresponds to pitch motor
        double theta_f = ankle_angles.first;   // corresponds to roll motor
        
        // Read current motor states
        sdk.getMotorActual(actuals);
        
        float actual_pitch_pos = actuals[ankle_pitch_idx].pos;
        float actual_roll_pos = actuals[ankle_roll_idx].pos;
        float actual_pitch_vel = actuals[ankle_pitch_idx].vel;
        float actual_roll_vel = actuals[ankle_roll_idx].vel;
        
        // Compute PD torque commands
        float target_pitch_vel = 0.0f;  // Target velocity (could compute from derivative)
        float target_roll_vel = 0.0f;
        
        float pitch_torque = pdControl(
            static_cast<float>(theta_e), 
            actual_pitch_pos, 
            target_pitch_vel, 
            actual_pitch_vel, 
            PD_KP_PITCH, 
            PD_KD_PITCH
        );
        
        float roll_torque = pdControl(
            static_cast<float>(theta_f), 
            actual_roll_pos, 
            target_roll_vel, 
            actual_roll_vel, 
            PD_KP_ROLL, 
            PD_KD_ROLL
        );
        
        // Safety clip: limit torque commands
        if (pitch_torque > MAX_TORQUE_LIMIT) {
            pitch_torque = MAX_TORQUE_LIMIT;
        } else if (pitch_torque < -MAX_TORQUE_LIMIT) {
            pitch_torque = -MAX_TORQUE_LIMIT;
        }
        
        if (roll_torque > MAX_TORQUE_LIMIT) {
            roll_torque = MAX_TORQUE_LIMIT;
        } else if (roll_torque < -MAX_TORQUE_LIMIT) {
            roll_torque = -MAX_TORQUE_LIMIT;
        }
        
        // Set motor commands (CST mode - torque control)
        targets[ankle_pitch_idx].tor = pitch_torque;
        targets[ankle_pitch_idx].pos = 0.0f;  // Not used in torque mode
        targets[ankle_pitch_idx].vel = 0.0f;  // Not used in torque mode
        targets[ankle_pitch_idx].enabled = 1;
        
        targets[ankle_roll_idx].tor = roll_torque;
        targets[ankle_roll_idx].pos = 0.0f;  // Not used in torque mode
        targets[ankle_roll_idx].vel = 0.0f;  // Not used in torque mode
        targets[ankle_roll_idx].enabled = 1;
        
        // Send commands
        sdk.setMotorTarget(targets);
        
        // Print status every second (1000 loops at 1kHz)
        if (loopCount % 1000 == 0) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - loop_start).count();
            float avg_freq = 1000.0f * loopCount / (elapsed > 0 ? elapsed : 1);
            
            std::cout << std::fixed;
            std::cout.precision(4);
            std::cout << "t=" << t << "s | "
                      << "pitch_cmd=" << pitch << " rad | roll_cmd=" << roll << " rad | "
                      << "theta_e=" << theta_e << " rad | theta_f=" << theta_f << " rad | "
                      << "actual_pitch=" << actual_pitch_pos << " rad | "
                      << "actual_roll=" << actual_roll_pos << " rad | "
                      << "pitch_torque=" << pitch_torque << " N·m | "
                      << "roll_torque=" << roll_torque << " N·m | "
                      << "freq=" << avg_freq << " Hz" << std::endl;
        }
        
        t += dt;
        loopCount++;
        
        // Maintain 1kHz loop rate
        auto cycle_end = std::chrono::steady_clock::now();
        auto cycle_time = std::chrono::duration_cast<std::chrono::microseconds>(
            cycle_end - cycle_start).count();
        
        int sleep_us = 1000 - cycle_time;  // 1ms = 1000us
        if (sleep_us > 0) {
            usleep(sleep_us);
        }
    }
    
    // Graceful shutdown: disable motors and hold position
    std::cout << "\nShutting down..." << std::endl;
    sdk.getMotorActual(actuals);
    
    targets[ankle_pitch_idx].enabled = 0;
    targets[ankle_roll_idx].enabled = 0;
    targets[ankle_pitch_idx].pos = actuals[ankle_pitch_idx].pos; // Hold current position
    targets[ankle_roll_idx].pos = actuals[ankle_roll_idx].pos;   // Hold current position
    targets[ankle_pitch_idx].tor = 0.0f;
    targets[ankle_roll_idx].tor = 0.0f;
    
    for (int i = 0; i < 100; ++i) {
        sdk.setMotorTarget(targets);
        usleep(1000);
    }
    
    std::cout << "Done." << std::endl;
    return 0;
}

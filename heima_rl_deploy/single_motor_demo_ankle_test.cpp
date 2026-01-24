/*
 * single_motor_demo_ankle_test.cpp
 * 
 * Ankle test program using ankle solver to control ankle motors
 * 
 * Compile command:
   g++ -std=c++20 -O2 -pthread \
       single_motor_demo_ankle_test.cpp \
       ./ankle_solver/cpp/ankle_solver.cpp \
       ../heimaSDK_copy/heima_driver_sdk.cpp \
       ../heimaSDK_copy/ecat.cpp \
       ../heimaSDK_copy/config_xml.cpp \
       ../heimaSDK_copy/common.cpp \
       ../heimaSDK_copy/tinyxml2.cpp \
       ../heimaSDK_copy/rs232.cpp \
       ../heimaSDK_copy/rs485.cpp \
       ../heimaSDK_copy/can.cpp \
       -I. -I../heimaSDK_copy -I./ankle_solver/cpp \
       -I/usr/local/include -I/usr/include/eigen3 \
       -L/usr/local/lib -lethercat -lm \
       -o build/single_motor_demo_ankle_test
 
 * Usage: sudo ./single_motor_demo_ankle_test [leg] [config.xml]
 *   leg: which leg to control (0=right, 1=left, default: 0)
 *   config.xml: path to config file (default: ../heimaSDK_copy/config.xml)
 * 
 * This program:
 * 1. Initializes the ankle solver with kinematic parameters
 * 2. Controls ankle motors based on desired pitch/roll angles
 * 3. Uses the solver to compute motor angles (theta_f, theta_e) from pitch/roll
 */

#include "heima_driver_sdk.h"
#include "./ankle_solver/cpp/ankle_solver.h"
#include <iostream>
#include <vector>
#include <csignal>
#include <unistd.h>
#include <cmath>

volatile sig_atomic_t g_stop = 0;
void handleSignal(int) {
    g_stop = 1;
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
    
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    
    std::cout << "=== Ankle Control Test ===" << std::endl;
    std::cout << "Config: " << configPath << std::endl;
    std::cout << "Leg: " << (leg == 0 ? "Right" : "Left") << std::endl;
    
    // Initialize ankle solver with kinematic parameters
    // These values match the ones in ankle_solver/cpp/main.cpp
    double h1 = 358.74 - (-25.06);
    double h2 = 263.51 - (-25.06);
    double m = 316.55;
    double n = 232.27;
    double r_E = std::sqrt(std::pow(-69 - (-22), 2) + std::pow(263.51 - 232.27, 2));
    double r_F = std::sqrt(std::pow(-69 - (-22), 2) + std::pow(358.74 - 316.55, 2));
    double l1 = 69 - 22;
    double d1 = 35;
    
    AnkleSolver ankle_solver(l1, d1, m, n, h1, h2, r_E, r_F);
    std::cout << "Ankle solver initialized" << std::endl;
    std::cout << "  h1=" << h1 << ", h2=" << h2 << std::endl;
    std::cout << "  m=" << m << ", n=" << n << std::endl;
    std::cout << "  r_E=" << r_E << ", r_F=" << r_F << std::endl;
    std::cout << "  l1=" << l1 << ", d1=" << d1 << std::endl;
    
    // Initialize SDK
    DriverSDK::DriverSDK& sdk = DriverSDK::DriverSDK::instance();
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
    
    // Set operating mode: 8 = CSP (Cyclic Synchronous Position)
    std::vector<char> modes(motorCount, 8);
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
    const int maxWaitTicks = 5000;
    int waitTicks = 0;
    bool motorsOperational = false;
    
    while (!g_stop && !motorsOperational && waitTicks < maxWaitTicks) {
        sdk.getMotorActual(actuals);
        
        // Check if both ankle motors are operational
        bool pitch_operational = (actuals[ankle_pitch_idx].statusWord & 0x007f) == 0x0037;
        bool roll_operational = (actuals[ankle_roll_idx].statusWord & 0x007f) == 0x0037;
        
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
    
    // Control loop: test ankle with sine wave pitch/roll commands
    std::cout << "\nStarting control loop (Ctrl+C to stop)..." << std::endl;
    std::cout << "Ankle will follow sine wave pitch/roll trajectory" << std::endl;
    
    const float pitch_amplitude = 0.2f;  // radians (~11.5 degrees)
    const float roll_amplitude = 0.2f;    // radians (~11.5 degrees)
    const float pitch_frequency = 0.1f;   // Hz
    const float roll_frequency = 0.1f;   // Hz
    const float dt = 0.001f;              // 1ms = 1kHz
    float t = 0.0f;
    
    int loopCount = 0;
    while (!g_stop) {
        // Generate sine wave pitch/roll commands
        // double pitch = pitch_amplitude * std::sin(2.0 * M_PI * pitch_frequency * t);
        // double roll = roll_amplitude * std::sin(2.0 * M_PI * roll_frequency * t);
        double pitch = 0.1f;
        double roll = 0.1f;
        
        // Solve for motor angles using ankle solver
        auto [theta_f, theta_e] = ankle_solver.solve(pitch, roll);
        std::cout << "theta_f" << theta_f << "; theta_e" << theta_e << std::endl;
        
        // Set motor targets
        // Note: theta_f and theta_e are the solver outputs, map them to pitch/roll motors
        // Based on the solver, theta_f corresponds to one motor and theta_e to the other
        targets[ankle_pitch_idx].pos = static_cast<float>(theta_e);  // theta_e for pitch motor
        targets[ankle_roll_idx].pos = static_cast<float>(theta_f);   // theta_f for roll motor
        targets[ankle_pitch_idx].vel = 0.0f;
        targets[ankle_roll_idx].vel = 0.0f;
        targets[ankle_pitch_idx].tor = 0.0f;
        targets[ankle_roll_idx].tor = 0.0f;
        targets[ankle_pitch_idx].enabled = 1;
        targets[ankle_roll_idx].enabled = 1;
        
        // Send command and read feedback
        // sdk.setMotorTarget(targets);
        sdk.getMotorActual(actuals);
        
        // Print status every second (1000 loops at 1kHz)
        if (loopCount % 1000 == 0) {
            std::cout << std::fixed;
            std::cout.precision(4);
            std::cout << "t=" << t << "s | "
                      << "pitch=" << pitch << " rad | roll=" << roll << " rad | "
                      << "theta_e=" << theta_e << " rad | theta_f=" << theta_f << " rad | "
                      << "actual_pitch=" << actuals[ankle_pitch_idx].pos << " rad | "
                      << "actual_roll=" << actuals[ankle_roll_idx].pos << " rad" << std::endl;
        }
        
        t += dt;
        loopCount++;
        usleep(1000); // 1ms
    }
    
    // Graceful shutdown: disable motors and hold position
    std::cout << "\nShutting down..." << std::endl;
    targets[ankle_pitch_idx].enabled = 0;
    targets[ankle_roll_idx].enabled = 0;
    targets[ankle_pitch_idx].pos = actuals[ankle_pitch_idx].pos; // Hold current position
    targets[ankle_roll_idx].pos = actuals[ankle_roll_idx].pos;   // Hold current position
    
    for (int i = 0; i < 100; ++i) {
        // sdk.setMotorTarget(targets);
        usleep(1000);
    }
    
    std::cout << "Done." << std::endl;
    return 0;
}

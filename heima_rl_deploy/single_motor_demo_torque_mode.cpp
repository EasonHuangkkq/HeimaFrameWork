/*
 * single_motor_demo_torque_mode.cpp
 * 
 * Brief program to control one motor using torque control (CST mode)
 * 
 * Usage: sudo ./single_motor_demo_torque_mode [motor_index] [target_current_A] [max_current_A] [config.xml]
 *   motor_index:     which motor to control (0-based, default: 0)
 *   target_current_A: target current in Amperes (default: 0.5)
 *   max_current_A:    maximum current limit in Amperes (default: 10.0)
 *   config.xml:      path to config file (default: config.xml)
 */

#include "heima_driver_sdk.h"
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

int main(int argc, char** argv) {
    // Parse arguments
    int motorIndex = 0;
    if (argc > 1) {
        motorIndex = std::stoi(argv[1]);
    }
    
    float targetCurrentA = 0.5f;
    if (argc > 2) {
        targetCurrentA = std::stof(argv[2]);
    }
    
    float maxCurrentA = 10.0f;
    if (argc > 3) {
        maxCurrentA = std::stof(argv[3]);
    }
    
    std::string configPath = "config.xml";
    if (argc > 4) {
        configPath = argv[4];
    }
    
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    
    std::cout << "=== Single Motor Torque Control Demo ===" << std::endl;
    std::cout << "Config: " << configPath << std::endl;
    std::cout << "Motor index: " << motorIndex << std::endl;
    std::cout << "Target current: " << targetCurrentA << " A" << std::endl;
    std::cout << "Max current: " << maxCurrentA << " A" << std::endl;
    
    // Initialize SDK
    DriverSDK::DriverSDK& sdk = DriverSDK::DriverSDK::instance();
    sdk.init(configPath.c_str());
    
    const int motorCount = sdk.getTotalMotorNr();
    std::cout << "Total motors: " << motorCount << std::endl;
    
    if (motorIndex < 0 || motorIndex >= motorCount) {
        std::cerr << "Error: Motor index " << motorIndex 
                  << " out of range [0, " << motorCount << ")" << std::endl;
        return 1;
    }
    
    // Set operating mode: 10 = CST (Cyclic Synchronous Torque)
    std::vector<char> modes(motorCount, 10);
    sdk.setMode(modes);
    
    // Set maximum current limit (in 0.01A units: 100 = 1A)
    // const unsigned short maxCurrentRaw = static_cast<unsigned short>(std::lround(maxCurrentA * 100.0f));
    // sdk.setMaxCurr(std::vector<unsigned short>(motorCount, maxCurrentRaw));
    // std::cout << "Max current set to: " << maxCurrentA << " A (raw: " << maxCurrentRaw << ")" << std::endl;
    
    // Prepare motor data structures
    std::vector<DriverSDK::motorTargetStruct> targets(motorCount);
    std::vector<DriverSDK::motorActualStruct> actuals(motorCount);
    
    // Initialize all motors (enabled, zero torque initially)
    for (int i = 0; i < motorCount; ++i) {
        targets[i].pos = 0.0f;
        targets[i].vel = 0.0f;
        targets[i].tor = 0.0f;
        targets[i].enabled = 1;
    }
    
    // Wait for motor to reach operational state
    std::cout << "\nWaiting for motor " << motorIndex << " to be operational..." << std::endl;
    const int maxWaitTicks = 10000; // ~10s at 1kHz
    int waitTicks = 0;
    bool motorOperational = false;
    
    while (!g_stop && !motorOperational && waitTicks < maxWaitTicks) {
        // sdk.setMotorTarget(targets);
        sdk.getMotorActual(actuals);
        
        // Check if motor is in operational state (statusWord bits 0-6 == 0x37)
        if ((actuals[motorIndex].statusWord & 0x007f) == 0x0037) {
            motorOperational = true;
            std::cout << "Motor " << motorIndex << " is operational!" << std::endl;
        }
        
        if (waitTicks % 1000 == 0) {
            std::cout << "  Status: 0x" << std::hex << actuals[motorIndex].statusWord 
                      << std::dec << " (waiting...)" << std::endl;
        }
        
        waitTicks++;
        usleep(1000); // 1ms
    }
    
    if (!motorOperational) {
        std::cerr << "Warning: Motor did not reach operational state" << std::endl;
    }
    
    // Control loop: sine wave torque command
    std::cout << "\nStarting torque control loop (Ctrl+C to stop)..." << std::endl;
    std::cout << "Motor " << motorIndex << " will follow a sine wave torque command" << std::endl;
    
    const float frequency = 0.1f;   // Hz
    const float dt = 0.001f;        // 1ms = 1kHz
    auto t0 = std::chrono::steady_clock::now();
    
    int loopCount = 0;
    while (!g_stop) {
        // Generate sine wave torque command (in Amperes for MT_Device + CST mode)
        auto now = std::chrono::steady_clock::now();
        float t = std::chrono::duration<float>(now - t0).count();
        float targetTorque = targetCurrentA * std::sin(2.0f * M_PI * frequency * t);
        
        // Clamp torque to max current limit
        if (targetTorque > maxCurrentA) targetTorque = maxCurrentA;
        if (targetTorque < -maxCurrentA) targetTorque = -maxCurrentA;
        
        // Set torque command (in Amperes for CST mode)
        targets[motorIndex].tor = -10.0f;
        // targets[motorIndex].tor = targetTorque;
        targets[motorIndex].pos = 0.0f;  // Not used in torque mode
        targets[motorIndex].vel = 0.0f;  // Not used in torque mode
        targets[motorIndex].enabled = 1;
        
        // Send command and read feedback
        sdk.setMotorTarget(targets);
        sdk.getMotorActual(actuals);
        // sdk.advance();
        
        // Print status every second (1000 loops at 1kHz)
        if (loopCount % 1000 == 0) {
            std::cout << "t=" << t << "s | "
                      << "target_tor=" << targetTorque << " A | "
                      << "actual_tor=" << actuals[motorIndex].tor << " Nm | "
                      << "pos=" << actuals[motorIndex].pos << " rad | "
                      << "vel=" << actuals[motorIndex].vel << " rad/s | "
                      << "status=0x" << std::hex << actuals[motorIndex].statusWord 
                      << std::dec << std::endl;
        }
        
        loopCount++;
        usleep(1000); // 1ms
    }
    
    // Graceful shutdown: set torque to zero and disable motor
    std::cout << "\nShutting down..." << std::endl;
    targets[motorIndex].tor = 0.0f;
    targets[motorIndex].vel = 0.0f;
    targets[motorIndex].pos = 0.0f;
    targets[motorIndex].enabled = 0;
    
    for (int i = 0; i < 200; ++i) {
        // sdk.setMotorTarget(targets);
        usleep(1000);
    }
    
    std::cout << "Done." << std::endl;
    return 0;
}

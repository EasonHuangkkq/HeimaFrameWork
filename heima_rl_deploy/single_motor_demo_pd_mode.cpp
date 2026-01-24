/*
 * single_motor_demo_pd_mode.cpp
 * 
 * Brief program to control one motor using PD (Proportional-Derivative) control
 * 
 * Usage: sudo ./single_motor_demo_pd_mode [motor_index] [kp] [kd] [config.xml]
 *   motor_index: which motor to control (0-based, default: 0)
 *   kp:           proportional gain (default: 300.0)
 *   kd:           derivative gain (default: 1.0)
 *   config.xml:  path to config file (default: config.xml)
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

// PD Control function
// Computes torque command from position error and velocity error
float pdControl(float targetPos, float actualPos, float targetVel, float actualVel, float kp, float kd) {
    float posError = targetPos - actualPos;
    float velError = targetVel - actualVel;
    float torque = kp * posError + kd * velError;
    return torque;
}

int main(int argc, char** argv) {
    // Parse arguments
    int motorIndex = 0;
    if (argc > 1) {
        motorIndex = std::stoi(argv[1]);
    }
    
    float kp = 300.0f;
    if (argc > 2) {
        kp = std::stof(argv[2]);
    }
    
    float kd = 1.0f;
    if (argc > 3) {
        kd = std::stof(argv[3]);
    }
    
    std::string configPath = "config.xml";
    if (argc > 4) {
        configPath = argv[4];
    }
    
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    
    std::cout << "=== Single Motor PD Control Demo ===" << std::endl;
    std::cout << "Config: " << configPath << std::endl;
    std::cout << "Motor index: " << motorIndex << std::endl;
    std::cout << "Kp: " << kp << std::endl;
    std::cout << "Kd: " << kd << std::endl;
    
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
    
    // Set operating mode: 8 = CSP (Cyclic Synchronous Position)
    // For PD control, we can use CSP mode and compute torque ourselves,
    // or use mode 10 (CST) and send computed torque
    std::vector<char> modes(motorCount, 10);  // CST mode for torque control
    sdk.setMode(modes);
    
    // Set maximum current limit (in 0.01A units: 100 = 1A)
    // const unsigned short maxCurrentRaw = static_cast<unsigned short>(std::lround(maxCurrentA * 100.0f));
    // sdk.setMaxCurr(std::vector<unsigned short>(motorCount, maxCurrentRaw));
    // std::cout << "Max current set to: " << maxCurrentA << " A (raw: " << maxCurrentRaw << ")" << std::endl;
    
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
    
    // Enable the target motor
    std::cout << "\nEnabling motor " << motorIndex << "..." << std::endl;
    targets[motorIndex].enabled = 1;
    
    // Control loop: PD control with sine wave position target
    std::cout << "\nStarting PD control loop (Ctrl+C to stop)..." << std::endl;
    std::cout << "Motor " << motorIndex << " will track a sine wave position using PD control" << std::endl;
    
    const float amplitude = 0.5f;  // radians
    const float frequency = 0.1f;   // Hz
    const float dt = 0.001f;        // 1ms = 1kHz
    auto t0 = std::chrono::steady_clock::now();
    
    // Store previous position for velocity estimation (if needed)
    float prevPos = 0.0f;
    bool firstLoop = true;
    
    int loopCount = 0;
    while (!g_stop) {
        // Get current time
        auto now = std::chrono::steady_clock::now();
        float t = std::chrono::duration<float>(now - t0).count();
        
        // Generate sine wave position target
        // float targetPos = amplitude * std::sin(2.0f * M_PI * frequency * t);
        // float targetVel = 2.0f * M_PI * frequency * amplitude * std::cos(2.0f * M_PI * frequency * t);
        float targetPos = 0.5f;
        float targetVel = 0.0f;
        
        // Read current motor state
        sdk.getMotorActual(actuals);
        
        float actualPos = actuals[motorIndex].pos;
        float actualVel = actuals[motorIndex].vel;
        
        // Compute PD torque command
        float torqueCmd = pdControl(targetPos, actualPos, targetVel, actualVel, kp, kd);
        
        // Clamp torque to reasonable limits (adjust based on your motor)
        const float maxTorque = 50.0f;  // Nm (adjust as needed)
        if (torqueCmd > maxTorque) torqueCmd = maxTorque;
        if (torqueCmd < -maxTorque) torqueCmd = -maxTorque;
        
        // Set torque command (in Amperes for CST mode with MT_Device)
        // For other devices, torque is in Nm and SDK will convert
        targets[motorIndex].tor = torqueCmd;
        targets[motorIndex].pos = targetPos;  // Store target for reference, but not used
        targets[motorIndex].vel = targetVel;   // Store target velocity for reference, but not used
        // targets[motorIndex].kp = kp;          // Store PD gains
        // targets[motorIndex].kd = kd;
        targets[motorIndex].enabled = 1;
        
        // Send command
        sdk.setMotorTarget(targets);
        
        // Print status every second (1000 loops at 1kHz)
        if (loopCount % 1000 == 0) {
            float posError = targetPos - actualPos;
            float velError = targetVel - actualVel;
            std::cout << "t=" << t << "s | "
                      << "target_pos=" << targetPos << " rad | "
                      << "actual_pos=" << actualPos << " rad | "
                      << "pos_err=" << posError << " rad | "
                      << "vel_err=" << velError << " rad/s | "
                      << "torque_cmd=" << torqueCmd << " Nm | "
                      << "actual_tor=" << actuals[motorIndex].tor << " Nm | "
                      << "status=0x" << std::hex << actuals[motorIndex].statusWord 
                      << std::dec << std::endl;
        }
        
        prevPos = actualPos;
        firstLoop = false;
        loopCount++;
        usleep(1000); // 1ms
    }
    
    // Graceful shutdown: set torque to zero and disable motor
    std::cout << "\nShutting down..." << std::endl;
    targets[motorIndex].tor = 0.0f;
    targets[motorIndex].vel = 0.0f;
    targets[motorIndex].pos = actuals[motorIndex].pos; // Hold current position
    targets[motorIndex].enabled = 0;
    
    for (int i = 0; i < 200; ++i) {
        // sdk.setMotorTarget(targets);
        usleep(1000);
    }
    
    std::cout << "Done." << std::endl;
    return 0;
}

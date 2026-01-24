/*
 * single_motor_demo.cpp
 * 

 g++ -std=c++20 -O2 -pthread   single_motor_demo_pos_mode.cpp   ../heimaSDK_copy/heima_driver_sdk.cpp   ../heimaSDK_copy/ecat.cpp   ../heimaSDK_copy/config_xml.cpp   ../heimaSDK_copy/common.cpp   ../heimaSDK_copy/tinyxml2.cpp   ../heimaSDK_copy/rs232.cpp   ../heimaSDK_copy/rs485.cpp   ../heimaSDK_copy/can.cpp   -I.   -I../heimaSDK_copy   -I/usr/local/include   -L/usr/local/lib   -lethercat   -lm   -o build/single_motor_demo_pos_mode
 * Brief program to control one motor using Heima SDK
 * 
 * Usage: sudo ./single_motor_demo [motor_index] [config.xml]
 *   motor_index: which motor to control (0-based, default: 0)
 *   config.xml:  path to config file (default: config.xml)
 */

 #include "heima_driver_sdk.h"
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
     int motorIndex = 0;
     if (argc > 1) {
         motorIndex = std::stoi(argv[1]);
     }
     
     std::string configPath = "config.xml";
     if (argc > 2) {
         configPath = argv[2];
     }
     
     std::signal(SIGINT, handleSignal);
     std::signal(SIGTERM, handleSignal);
     
     std::cout << "=== Single Motor Control Demo ===" << std::endl;
     std::cout << "Config: " << configPath << std::endl;
     std::cout << "Motor index: " << motorIndex << std::endl;
     
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
     
     // Wait for motor to reach operational state
     std::cout << "\nWaiting for motor " << motorIndex << " to be operational..." << std::endl;
     const int maxWaitTicks = 5000;
     int waitTicks = 0;
     bool motorOperational = false;
     
     while (!g_stop && !motorOperational && waitTicks < maxWaitTicks) {
        //  sdk.setMotorTarget(targets);
         sdk.getMotorActual(actuals);
        //  sdk.advance();
         
         // Check if motor is in operational state (statusWord bits 0-6 == 0x37)
         if ((actuals[motorIndex].statusWord & 0x007f) == 0x0037) {
             motorOperational = true;
             std::cout << "Motor " << motorIndex << " is operational!" << std::endl;
         }
         
         if (waitTicks % 500 == 0) {
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
     targets[motorIndex].pos = 0.0f; // Start at zero position
     
     // Control loop: simple sine wave position command
     std::cout << "\nStarting control loop (Ctrl+C to stop)..." << std::endl;
     std::cout << "Motor " << motorIndex << " will follow a sine wave trajectory" << std::endl;
     
     const float amplitude = 0.5f;  // radians
     const float frequency = 0.1f;   // Hz
     const float dt = 0.001f;        // 1ms = 1kHz
     float t = 0.0f;
     
     int loopCount = 0;
     while (!g_stop) {
        // Generate sine wave position command (range: -5 to -4 radians)
        float targetPos = 2.5f + 0.5f * std::sin(2.0f * M_PI * frequency * t);
         targets[motorIndex].pos = targetPos;
         targets[motorIndex].vel = 0.0f;
         targets[motorIndex].tor = 0.0f;
         targets[motorIndex].enabled = 1;
         
         // Send command and read feedback
        //  sdk.setMotorTarget(targets);
         sdk.getMotorActual(actuals);
        //  sdk.advance();
         
         // Print status every second (1000 loops at 1kHz)
         if (loopCount % 1000 == 0) {
             std::cout << "t=" << t << "s | "
                       << "target=" << targetPos << " rad | "
                       << "actual=" << actuals[motorIndex].pos << " rad | "
                       << "vel=" << actuals[motorIndex].vel << " rad/s | "
                       << "status=0x" << std::hex << actuals[motorIndex].statusWord 
                       << std::dec << std::endl;
         }
         
         t += dt;
         loopCount++;
         usleep(1000); // 1ms
     }
     
     // Graceful shutdown: disable motor and hold position
     std::cout << "\nShutting down..." << std::endl;
     targets[motorIndex].enabled = 0;
     targets[motorIndex].pos = actuals[motorIndex].pos; // Hold current position
     
     for (int i = 0; i < 100; ++i) {
        //  sdk.setMotorTarget(targets);
        //  sdk.advance();
         usleep(1000);
     }
     
     std::cout << "Done." << std::endl;
     return 0;
 }
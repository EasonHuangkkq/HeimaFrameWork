/*
 * IMU test utility for Heima SDK (YeSense via RS232).
 *
 * This program only initializes the SDK, reads IMU, and prints it.
 * It does NOT send motor commands.
 *
 * Usage:
 *   ./imu_test [config.xml] [rate_hz=100] [duration_s=0]
 *     duration_s=0 means run forever until Ctrl+C
 *
 * Compile command (from heima_rl_deploy/):
 *   mkdir -p build && \
   g++ -std=c++20 -O2 -pthread -DENABLE_RS232 \
     imu_test.cpp \
     ../heimaSDK_copy/common.cpp \
     ../heimaSDK_copy/config_xml.cpp \
     ../heimaSDK_copy/tinyxml2.cpp \
     ../heimaSDK_copy/heima_driver_sdk.cpp \
     ../heimaSDK_copy/rs232.cpp \
     ../heimaSDK_copy/rs485.cpp \
     ../heimaSDK_copy/can.cpp \
     ../heimaSDK_copy/ecat.cpp \
     ../heimaSDK_copy/yesense_imu.cpp \
     ../heimaSDK_copy/yesense/src/yesense_decoder.cpp \
     ../heimaSDK_copy/yesense/src/yesense_std_out_decoder.cpp \
     -I. -I../heimaSDK_copy -I../heimaSDK_copy/yesense/src \
     -I/usr/local/include -L/usr/local/lib -lethercat -lm \
     -o build/imu_test
 */

#include "heima_driver_sdk.h"
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <thread>

static volatile sig_atomic_t g_stop = 0;
static void handleSignal(int) { g_stop = 1; }

int main(int argc, char** argv) {
    std::string config_file = "config.xml";
    int rate_hz = 100;
    int duration_s = 0;

    if (argc > 1) config_file = argv[1];
    if (argc > 2) rate_hz = std::max(1, std::atoi(argv[2]));
    if (argc > 3) duration_s = std::max(0, std::atoi(argv[3]));

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    std::cout << "=== IMU Test ===" << std::endl;
    std::cout << "Config: " << config_file << std::endl;
    std::cout << "Rate: " << rate_hz << " Hz" << std::endl;
    if (duration_s > 0) std::cout << "Duration: " << duration_s << " s" << std::endl;
    else std::cout << "Duration: forever (Ctrl+C to stop)" << std::endl;

    DriverSDK::DriverSDK& sdk = DriverSDK::DriverSDK::instance();
    std::cout << "Initializing SDK..." << std::endl;
    sdk.init(config_file.c_str());

    DriverSDK::imuStruct imu{};
    const auto period = std::chrono::microseconds(1000000 / rate_hz);
    const auto t0 = std::chrono::steady_clock::now();
    auto next_tick = t0;
    int tick = 0;

    while (!g_stop) {
        if (duration_s > 0) {
            auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - t0).count();
            if (elapsed_s >= duration_s) break;
        }

        sdk.getIMU(imu);

        // Print at the chosen rate; keep it readable.
        std::cout << std::fixed << std::setprecision(4)
                  << "t=" << tick
                  << " rpy=(" << imu.rpy[0] << ", " << imu.rpy[1] << ", " << imu.rpy[2] << ")"
                  << " gyr=(" << imu.gyr[0] << ", " << imu.gyr[1] << ", " << imu.gyr[2] << ")"
                  << " acc=(" << imu.acc[0] << ", " << imu.acc[1] << ", " << imu.acc[2] << ")"
                  << std::endl;

        tick++;
        next_tick += period;
        std::this_thread::sleep_until(next_tick);
    }

    std::cout << "Done." << std::endl;
    return 0;
}

